import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
from tf_transformations import euler_from_quaternion

class GroundAlignedScanNode(Node):
    def __init__(self):
        super().__init__('ground_aligned_scan_node')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_flat', 10)

        self.roll = 0.0
        self.pitch = 0.0
        self.latest_scan = None

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        self.roll, self.pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, scan_msg):
        self.latest_scan = scan_msg
        self.publish_flat_scan()

    def publish_flat_scan(self):
        if self.latest_scan is None:
            return

        angles = np.linspace(self.latest_scan.angle_min,
                             self.latest_scan.angle_max,
                             len(self.latest_scan.ranges),
                             endpoint=False)
        ranges = np.array(self.latest_scan.ranges)

        # 원래 LaserScan을 Cartesian으로 변환 (LiDAR frame)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        # Roll/Pitch로 회전 보정 (IMU 기반)
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)

        rotation_matrix = np.array([
            [cp, sr*sp, cr*sp],
            [0, cr, -sr],
            [-sp, sr*cp, cr*cp]
        ])

        points = np.vstack((xs, ys, zs))
        flat_points = rotation_matrix @ points

        # 평면(Z=0)에 투영하여 다시 2D LaserScan으로 변환
        flat_ranges = np.sqrt(flat_points[0, :]**2 + flat_points[1, :]**2)

        # 변환된 데이터를 LaserScan 메시지로 발행
        flat_scan = LaserScan()
        flat_scan.header = self.latest_scan.header
        flat_scan.angle_min = self.latest_scan.angle_min
        flat_scan.angle_max = self.latest_scan.angle_max
        flat_scan.angle_increment = self.latest_scan.angle_increment
        flat_scan.time_increment = self.latest_scan.time_increment
        flat_scan.scan_time = self.latest_scan.scan_time
        flat_scan.range_min = self.latest_scan.range_min
        flat_scan.range_max = self.latest_scan.range_max
        flat_scan.ranges = flat_ranges.tolist()

        self.scan_pub.publish(flat_scan)

def main(args=None):
    rclpy.init(args=args)
    node = GroundAlignedScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
