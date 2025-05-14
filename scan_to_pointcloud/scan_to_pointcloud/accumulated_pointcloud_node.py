#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/accumulated_pointcloud_node.py
# 수정일: 2025-05-14

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
# import struct # sensor_msgs_py.point_cloud2 사용 시 직접 필요 없음
import tf2_ros
from rclpy.time import Time # 사용하지 않지만, 명시적 Time 객체 생성 시 필요할 수 있음
from rclpy.duration import Duration
# from visualization_msgs.msg import Marker, MarkerArray # 현재 미사용
# from std_msgs.msg import ColorRGBA # 현재 미사용
# from geometry_msgs.msg import Point # 현재 미사용
# from tf2_ros import TransformBroadcaster # 현재 미사용
import sensor_msgs_py.point_cloud2 as pc2 # 포인트 클라우드 생성/읽기 표준 라이브러리
try:
    from scipy.spatial.transform import Rotation as R_scipy
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("scipy 라이브러리를 찾을 수 없습니다. 쿼터니언 변환 시 수동 계산합니다. 'pip install scipy'로 설치 가능합니다.")


class AccumulatedPointcloud(Node):
    def __init__(self):
        super().__init__('accumulated_pointcloud')
        
        # 파라미터 선언
        self.declare_parameter('input_topic', 'pc_3d')           # 구독할 PointCloud2 토픽
        self.declare_parameter('output_topic', 'accumulated_points') # 발행할 누적 PointCloud2 토픽
        self.declare_parameter('max_points_accumulated', 100000) # 누적할 최대 포인트 수
        self.declare_parameter('downsample_grid_size', 0.05)   # 다운샘플링 시 사용할 그리드 크기 (0 이하면 비활성화)
        self.declare_parameter('use_tf', True)                   # TF 변환 사용 여부
        self.declare_parameter('publish_rate_hz', 1.0)           # 누적된 포인트 클라우드 발행 주기 (Hz)
        self.declare_parameter('target_frame', 'map')            # 포인트 클라우드를 변환할 목표 TF 프레임

        # 파라미터 가져오기
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_points_accumulated = self.get_parameter('max_points_accumulated').value
        self.downsample_grid_size = self.get_parameter('downsample_grid_size').value
        self.use_tf = self.get_parameter('use_tf').value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.target_frame = self.get_parameter('target_frame').value

        # TF 버퍼 및 리스너 설정
        # ****** TF 데이터 보관 시간을 30초로 늘림 (Extrapolation 오류 완화 목적) ******
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0)) 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 누적된 포인트 (NumPy 배열로 관리하는 것이 더 효율적일 수 있으나, 우선 리스트로 유지)
        # 각 요소는 [x, y, z, intensity] 형태의 리스트 또는 튜플
        self.all_points_transformed = [] 
        
        # 구독자 (입력 PointCloud2)
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pc_callback,
            10  # QoS depth
        )
        
        # 발행자 (누적된 PointCloud2)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        # 주기적 발행을 위한 타이머
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
        points_to_add_with_intensity = [] # 이번 콜백에서 추가할 포인트들 ([x,y,z,intensity] 리스트)

        # sensor_msgs_py.point_cloud2를 사용하여 포인트 읽기 (x, y, z, intensity 필드 가정)
        # generator를 list로 변환
        try:
            # field_names를 명시하여 원하는 데이터만 추출하고 순서 보장
            original_points_generator = pc2.read_points(point_cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            original_points_list = list(original_points_generator) 
        except Exception as e:
            self.get_logger().error(f"入力PointCloud2 메시지 읽기 실패 ({point_cloud_msg.header.frame_id}): {e}", throttle_duration_sec=5.0)
            return

        if not original_points_list:
            # self.get_logger().debug("빈 PointCloud2 메시지 수신, 처리할 포인트 없음.")
            return

        if self.use_tf:
            try:
                # 입력 포인트 클라우드의 타임스탬프를 사용하여 해당 시점의 TF를 조회
                transform_stamped = self.tf_buffer.lookup_transform(
                    self.target_frame,                      # 누적할 목표 프레임 (예: 'map')
                    point_cloud_msg.header.frame_id,      # 원본 포인트 클라우드의 프레임 (예: 'laser')
                    point_cloud_msg.header.stamp,           # 원본 포인트 클라우드의 타임스탬프 *** 중요 ***
                    timeout=Duration(seconds=1.0)           # 타임아웃 (너무 길면 시스템 지연 유발 가능)
                )
                
                # NumPy 배열로 변환 (N, 4 형태: x, y, z, intensity)
                original_points_np = np.array(original_points_list, dtype=np.float32)
                
                # 변환 행렬 계산
                trans_matrix = self.compute_transform_matrix_from_stamped_transform(transform_stamped.transform)

                # 포인트 변환 (x,y,z 부분만)
                xyz_points_original = original_points_np[:, :3] # (N, 3)
                # 동차 좌표계로 변환 (N, 4)
                ones_column = np.ones((xyz_points_original.shape[0], 1), dtype=np.float32)
                points_homogeneous = np.hstack((xyz_points_original, ones_column)) 
                
                # 변환 행렬 적용: P' = T * P (여기서 P는 열벡터로 가정, NumPy에서는 (N,4) @ (4,4) 로 처리)
                # trans_matrix는 (4,4) 형태이므로, points_homogeneous (N,4)와 곱하려면 P_transformed = points_homogeneous @ trans_matrix.T
                # 또는 P_transformed_T = trans_matrix @ points_homogeneous.T  => P_transformed = P_transformed_T.T
                transformed_points_homogeneous = points_homogeneous @ trans_matrix.T # (N,4)
                
                # 변환된 xyz와 원래 intensity 결합
                transformed_xyz = transformed_points_homogeneous[:, :3]
                intensities_column = original_points_np[:, 3].reshape(-1, 1) # (N,1)
                
                # 최종 추가할 포인트들 (리스트의 리스트 형태)
                points_to_add_with_intensity = np.hstack((transformed_xyz, intensities_column)).tolist()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f"TF 조회 실패: '{point_cloud_msg.header.frame_id}' -> '{self.target_frame}' "
                    f"(시간: {point_cloud_msg.header.stamp.sec}.{point_cloud_msg.header.stamp.nanosec}): {e}",
                    throttle_duration_sec=5.0 # 동일 경고 반복 줄이기
                )
                return # TF 변환 실패 시 이 메시지는 누적하지 않음
        else: # TF 미사용 (주로 디버깅용, 원본 프레임 그대로 누적)
            points_to_add_with_intensity = original_points_list

        if not points_to_add_with_intensity:
            return

        self.all_points_transformed.extend(points_to_add_with_intensity)
        
        # 너무 많은 포인트가 쌓이는 것을 방지 (오래된 데이터부터 제거)
        current_num_points = len(self.all_points_transformed)
        if current_num_points > self.max_points_accumulated:
            num_to_remove = current_num_points - self.max_points_accumulated
            self.all_points_transformed = self.all_points_transformed[num_to_remove:]
            # self.get_logger().debug(f"최대 누적 포인트 수 초과. 오래된 포인트 {num_to_remove}개 제거. 현재: {len(self.all_points_transformed)}")


    def compute_transform_matrix_from_stamped_transform(self, transform_msg):
        """geometry_msgs/Transform 메시지로부터 4x4 NumPy 변환 행렬 계산"""
        trans = transform_msg.translation
        rot_q = transform_msg.rotation # geometry_msgs/Quaternion
        
        if SCIPY_AVAILABLE:
            # scipy 사용 시: 쿼터니언 -> 회전 행렬
            r_matrix = R_scipy.from_quat([rot_q.x, rot_q.y, rot_q.z, rot_q.w]).as_matrix()
        else:
            # scipy 없을 경우 수동 계산 (쿼터니언 -> 회전 행렬)
            x, y, z, w = rot_q.x, rot_q.y, rot_q.z, rot_q.w
            xx, xy, xz, xw = x*x, x*y, x*z, x*w
            yy, yz, yw = y*y, y*z, y*w
            zz, zw = z*z, z*w
            r_matrix = np.array([
                [1 - 2*(yy + zz),     2*(xy - zw),     2*(xz + yw)],
                [    2*(xy + zw), 1 - 2*(xx + zz),     2*(yz - xw)],
                [    2*(xz - yw),     2*(yz + xw), 1 - 2*(xx + yy)]
            ], dtype=np.float32)

        # 4x4 변환 행렬 생성
        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = r_matrix
        transform_matrix[:3, 3] = [trans.x, trans.y, trans.z]
        return transform_matrix

    def downsample_points(self, points_list_of_lists):
        """그리드 기반 다운샘플링. points_list_of_lists는 [[x,y,z,intensity], ...] 형태"""
        if not points_list_of_lists or self.downsample_grid_size <= 0:
            return points_list_of_lists
        
        grid = {} # 그리드 셀 인덱스를 키로, 해당 셀의 포인트 ([x,y,z,intensity])를 값으로 저장
        
        for point_with_intensity in points_list_of_lists:
            # point_with_intensity는 [x,y,z,intensity] 형태
            grid_key = (
                int(point_with_intensity[0] / self.downsample_grid_size),
                int(point_with_intensity[1] / self.downsample_grid_size),
                int(point_with_intensity[2] / self.downsample_grid_size)
            )
            # 각 셀에 마지막으로 들어온 포인트 저장 (또는 평균, 첫 포인트 등 전략 선택 가능)
            grid[grid_key] = point_with_intensity 
            
        return list(grid.values())

    def publish_accumulated_cloud(self):
        if not self.all_points_transformed:
            # self.get_logger().debug("누적된 포인트가 없어 발행하지 않습니다.")
            return

        # 발행 전 최종 다운샘플링
        points_to_publish = self.downsample_points(self.all_points_transformed)
        
        if not points_to_publish:
            # self.get_logger().debug("다운샘플링 후 포인트가 없어 발행하지 않습니다.")
            return

        # PointCloud2 메시지 헤더 설정
        header = Header()
        header.stamp = self.get_clock().now().to_msg() # 발행 시점의 현재 시간
        header.frame_id = self.target_frame # 누적된 포인트 클라우드의 프레임 ID (예: 'map')
        
        # PointCloud2 필드 정의 (x, y, z, intensity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        try:
            # points_to_publish는 [x,y,z,intensity] 형태의 리스트들의 리스트
            cloud_msg = pc2.create_cloud(header, fields, points_to_publish)
            self.pub.publish(cloud_msg)
            # self.get_logger().info(f"누적 PointCloud 발행: {len(points_to_publish)} 포인트 ({self.output_topic})")
        except Exception as e:
            self.get_logger().error(f"누적 PointCloud2 생성 또는 발행 실패: {e}", throttle_duration_sec=5.0)

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