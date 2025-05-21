#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from rclpy.duration import Duration
import sensor_msgs_py.point_cloud2 as pc2
import time

try:
    from scipy.spatial.transform import Rotation as R_scipy
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("scipy 라이브러리를 찾을 수 없습니다. 쿼터니언 변환 시 수동 계산합니다. 'pip install scipy'로 설치 가능합니다.")

class AccumulatedPointcloud(Node):
    def __init__(self):
        super().__init__('accumulated_pointcloud')
        self.declare_parameter('input_topic', 'pc_3d')
        self.declare_parameter('output_topic', 'accumulated_points')
        self.declare_parameter('max_points_accumulated', 100000)
        self.declare_parameter('downsample_grid_size', 0.05)
        self.declare_parameter('use_tf', True)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('target_frame', 'map')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_points_accumulated = self.get_parameter('max_points_accumulated').value
        self.downsample_grid_size = self.get_parameter('downsample_grid_size').value
        self.use_tf = self.get_parameter('use_tf').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_tf = None
        self.last_tf_stamp = None

        # *** numpy array 누적 (속도 개선) ***
        self.all_points_transformed = np.empty((0, 4), dtype=np.float32)

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pc_callback,
            10
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        if self.publish_rate_hz > 0:
            self.timer_period = 1.0 / self.publish_rate_hz
            self.timer = self.create_timer(self.timer_period, self.publish_accumulated_cloud)

        self.get_logger().info(
            f"누적 PointCloud 노드 시작됨.\n"
            f"  - 입력 PointCloud2 토픽: '{self.input_topic}'\n"
            f"  - 출력 누적 PointCloud2 토픽: '{self.output_topic}'\n"
            f"  - 목표 TF 프레임: '{self.target_frame}'\n"
            f"  - 최대 누적 포인트 수: {self.max_points_accumulated}\n"
            f"  - 다운샘플링 그리드 크기: {self.downsample_grid_size if self.downsample_grid_size > 0 else '비활성화'}"
        )

    def pc_callback(self, point_cloud_msg: PointCloud2):
        import time
        t0 = time.time()

        # 1. PointCloud2 → np.array 변환 (struct/tuple/list 모두 robust)
        try:
            points_iter = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            points_list = list(points_iter)
            if not points_list:
                return
            # 일반적 경우: tuple/list형 데이터
            if isinstance(points_list[0], (list, tuple, np.ndarray)):
                pts = np.array(points_list, dtype=np.float32)
            else:
                # structured array (필드명이 존재할 때)
                arr = np.array(points_list)
                pts = np.stack([arr['x'], arr['y'], arr['z'], arr['intensity']], axis=-1).astype(np.float32)
        except Exception as e:
            self.get_logger().error(f"PointCloud2 메시지 읽기 실패: {e}")
            return
        t1 = time.time()

        # 2. TF lookup (다중 fallback + 마지막 성공 TF까지 활용)
        transform_stamped = None
        tf_success = False
        if self.use_tf:
            lookup_stamp = point_cloud_msg.header.stamp
            need_lookup = True
            # 최근 TF 캐싱 활용
            if hasattr(self, 'last_tf') and hasattr(self, 'last_tf_stamp'):
                if self.last_tf is not None and self.last_tf_stamp is not None:
                    sec_diff = abs(lookup_stamp.sec - self.last_tf_stamp.sec) + abs(lookup_stamp.nanosec - self.last_tf_stamp.nanosec) * 1e-9
                    if sec_diff < 0.05:
                        transform_stamped = self.last_tf
                        tf_success = True
                        need_lookup = False
            # 기본 TF lookup 시도
            if need_lookup:
                try:
                    transform_stamped = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        point_cloud_msg.header.frame_id,
                        lookup_stamp,
                        timeout=Duration(seconds=0.2)
                    )
                    self.last_tf = transform_stamped
                    self.last_tf_stamp = lookup_stamp
                    tf_success = True
                except Exception as e:
                    self.get_logger().warn(f"[TF lookup 실패, 최신 TF fallback 시도] {e}")
                    # 최신 TF로 재시도
                    try:
                        latest_time = self.tf_buffer.get_latest_common_time(self.target_frame, point_cloud_msg.header.frame_id)
                        transform_stamped = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            point_cloud_msg.header.frame_id,
                            latest_time,
                            timeout=Duration(seconds=0.1)
                        )
                        self.get_logger().warn("get_latest_common_time으로 fallback 성공")
                        tf_success = True
                    except Exception as e2:
                        self.get_logger().warn(f"[최신 TF fallback도 실패, 마지막 성공 TF 시도] {e2}")
                        if hasattr(self, 'last_success_tf') and self.last_success_tf is not None:
                            transform_stamped = self.last_success_tf
                            tf_success = True
                        else:
                            self.get_logger().error("TF 완전 실패 - 해당 프레임 skip")
                            return
        t2 = time.time()

        # 3. transform 적용
        if self.use_tf and transform_stamped is not None:
            trans_matrix = self.compute_transform_matrix_from_stamped_transform(transform_stamped.transform)
            xyz = pts[:, :3]
            ones = np.ones((xyz.shape[0], 1), dtype=np.float32)
            xyz1 = np.hstack((xyz, ones))
            t_xyz = (xyz1 @ trans_matrix.T)[:, :3]
            pts[:, :3] = t_xyz
            # 성공한 transform은 계속 기억
            self.last_success_tf = transform_stamped

        # 4. numpy array 누적
        if not hasattr(self, 'all_points_transformed') or self.all_points_transformed is None or len(self.all_points_transformed) == 0:
            self.all_points_transformed = pts
        else:
            self.all_points_transformed = np.vstack((self.all_points_transformed, pts))
        # 5. 최대 크기 유지
        if self.all_points_transformed.shape[0] > self.max_points_accumulated:
            self.all_points_transformed = self.all_points_transformed[-self.max_points_accumulated:, :]

        t3 = time.time()
        # 디버그 로그(필요시 해제)
        # self.get_logger().info(f"cb:read={t1-t0:.3f}s tf={t2-t1:.3f}s proc={t3-t2:.3f}s all={t3-t0:.3f}s")

    def compute_transform_matrix_from_stamped_transform(self, transform_msg):
        trans = transform_msg.translation
        rot_q = transform_msg.rotation
        if SCIPY_AVAILABLE:
            r_matrix = R_scipy.from_quat([rot_q.x, rot_q.y, rot_q.z, rot_q.w]).as_matrix()
        else:
            x, y, z, w = rot_q.x, rot_q.y, rot_q.z, rot_q.w
            xx, xy, xz, xw = x*x, x*y, x*z, x*w
            yy, yz, yw = y*y, y*z, y*w
            zz, zw = z*z, z*w
            r_matrix = np.array([
                [1 - 2*(yy + zz),     2*(xy - zw),     2*(xz + yw)],
                [    2*(xy + zw), 1 - 2*(xx + zz),     2*(yz - xw)],
                [    2*(xz - yw),     2*(yz + xw), 1 - 2*(xx + yy)]
            ], dtype=np.float32)
        tfm = np.eye(4, dtype=np.float32)
        tfm[:3, :3] = r_matrix
        tfm[:3, 3] = [trans.x, trans.y, trans.z]
        return tfm

    def downsample_points(self, arr):
        if arr.shape[0] == 0 or self.downsample_grid_size <= 0:
            return arr
        # *** numpy 기반 다운샘플 (빠름, 그리드 cell별 마지막점 유지) ***
        idx = np.floor(arr[:, :3] / self.downsample_grid_size).astype(np.int32)
        _, uniq_idx = np.unique(idx, axis=0, return_index=True)
        return arr[uniq_idx]

    def publish_accumulated_cloud(self):
        if self.all_points_transformed.shape[0] == 0:
            return
        t0 = time.time()
        pts = self.downsample_points(self.all_points_transformed)
        t1 = time.time()

        if pts.shape[0] == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        try:
            cloud_msg = pc2.create_cloud(header, fields, pts.tolist())
            self.pub.publish(cloud_msg)
            # self.get_logger().info(f"누적 PointCloud 발행: {len(pts)}점 (downsample:{self.downsample_grid_size})")
        except Exception as e:
            self.get_logger().error(f"누적 PointCloud2 생성/발행 실패: {e}")

        t2 = time.time()
        # 디버그용 타이밍 로그
        # self.get_logger().info(f"pub:downsample={t1-t0:.3f}s pub={t2-t1:.3f}s")

def main(args=None):
    rclpy.init(args=args)
    node = AccumulatedPointcloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 수신, 노드 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
