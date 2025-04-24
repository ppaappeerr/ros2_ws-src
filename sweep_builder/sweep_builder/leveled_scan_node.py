# leveled_scan_node.py  (패키지: sweep_builder)
import rclpy, numpy as np, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
class LevelScan(Node):
    def __init__(self):
        super().__init__('level_scan')
        self.roll = self.pitch = 0.0
        self.declare_parameter('in_scan',  '/scan')
        self.declare_parameter('out_scan', '/scan_leveled')
        self.sub_scan = self.create_subscription(
            LaserScan, self.get_parameter('in_scan').value, self.scan_cb, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu_filtered', self.imu_cb, 50)
        self.pub = self.create_publisher(LaserScan,
                                         self.get_parameter('out_scan').value, 10)

    def imu_cb(self, m):
        from tf_transformations import euler_from_quaternion
        self.roll, self.pitch, _ = euler_from_quaternion([m.orientation.x,
                                                          m.orientation.y,
                                                          m.orientation.z,
                                                          m.orientation.w])

    def scan_cb(self, m):
        out = LaserScan()
        out.header = m.header
        out.header.frame_id = 'laser'  # 중요
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        factor = math.cos(self.roll) * math.cos(self.pitch)
        out.ranges = [r * factor for r in m.ranges]
        out.intensities = m.intensities
        self.pub.publish(out)

def main():
    rclpy.init(); rclpy.spin(LevelScan()); rclpy.shutdown()
if __name__ == '__main__':
    main()
