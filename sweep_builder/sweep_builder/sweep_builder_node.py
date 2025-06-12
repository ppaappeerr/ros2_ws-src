# sweep_builder/sweep_builder_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from builtin_interfaces.msg import Time

class SweepBuilder(Node):
    def __init__(self):
        super().__init__('sweep_builder')
        # 파라미터
        self.declare_parameter('accum_duration', 0.2)   # [s] 누적 시간창
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('quadrant', 'all')  # 여기에 파라미터 선언 추가
        
        self.accum_dur   = self.get_parameter('accum_duration').value
        self.min_range   = self.get_parameter('min_range').value
        self.max_range   = self.get_parameter('max_range').value

        # 버퍼
        self.points = []           # (x,y,z)
        self.start_time: Time = None

        # 구독자
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        # '/imu', '/imu_raw', '/imu_filtered' 중 하나 선택

        # 발행자
        self.pc_pub = self.create_publisher(PointCloud2, '/sweep_pc', 10)

        # 최근 IMU 각도
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    # ---------- 콜백 ----------
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.get_logger().info(
            f"[IMU] roll={self.roll:.3f}, pitch={self.pitch:.3f}, yaw={self.yaw:.3f}")

    def scan_cb(self, msg: LaserScan):
        # 스캔 각도 배열
        angles = np.linspace(
            msg.angle_min, msg.angle_max,
            num=len(msg.ranges), endpoint=False
        )
        ranges = np.array(msg.ranges, dtype=np.float32)

        # 범위 필터
        valid = np.logical_and(ranges > self.min_range,
                               ranges < self.max_range)
        angles = angles[valid]
        ranges = ranges[valid]

        # 2D 극좌표 → 2D Cartesian (LiDAR 프레임)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        # -> 3D 회전 (IMU pitch/roll 적용)
        # yaw(수평 회전)은 사용자가 회전하면 스캔이 겹치므로 제거
        # R = R_roll * R_pitch
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)

        rot_mat = np.array([[cp, sr*sp, cr*sp],
                            [0.0,  cr,   -sr   ],
                            [-sp, sr*cp, cr*cp]])

        pts = np.vstack((xs, ys, zs))
        pts_rot = rot_mat @ pts        # 3xN

        # 사분면 필터링 (올바른 방식으로 적용)
        quadrant = self.get_parameter('quadrant').get_parameter_value().string_value
        if quadrant == 'front':
            pts_rot = pts_rot[:, pts_rot[0, :] > 0]
        elif quadrant == 'rear':
            pts_rot = pts_rot[:, pts_rot[0, :] < 0]

        # 버퍼 적재
        now = msg.header.stamp
        if self.start_time is None:
            self.start_time = now
        self.points.extend(pts_rot.T.tolist())

        # 누적 기간 초과 시 PointCloud2 발행
        elapsed = (now.sec - self.start_time.sec) + \
                  (now.nanosec - self.start_time.nanosec) * 1e-9
        if elapsed >= self.accum_dur:
            self.publish_pc(now)
            # 버퍼 초기화
            self.points.clear()
            self.start_time = None

    # ---------- PointCloud2 작성 ----------

    def publish_pc(self, stamp):
        if not self.points:
            return

        header = Header()
        header.stamp = stamp
        header.frame_id = 'laser'   # RViz에서 이 프레임 기준으로 표시됨

        # PointField(x,y,z) only
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc2_msg = pc2.create_cloud(header, fields, self.points)
        self.pc_pub.publish(pc2_msg)

def main():
    rclpy.init()
    node = SweepBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
