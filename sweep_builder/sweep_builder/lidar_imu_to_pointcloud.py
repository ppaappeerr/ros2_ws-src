import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from tf_transformations import quaternion_matrix

class LidarImuToPointcloud(Node):
    def __init__(self):
        super().__init__('lidar_imu_to_pointcloud')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 50)
        self.pc_pub = self.create_publisher(PointCloud2, '/points_3d', 10)
        self.orientation_quat = [0, 0, 0, 1]

    def imu_callback(self, imu_msg):
        q = imu_msg.orientation
        self.orientation_quat = [q.x, q.y, q.z, q.w]

    def scan_callback(self, scan_msg):
        points = []
        angle = scan_msg.angle_min

        R = quaternion_matrix(self.orientation_quat)[:3, :3]

        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                z = 0.0
                pt = np.array([x, y, z])
                pt_rot = R @ pt
                points.append(pt_rot)
            angle += scan_msg.angle_increment

        pc_msg = point_cloud2.create_cloud_xyz32(scan_msg.header, points)
        self.pc_pub.publish(pc_msg)

def main():
    rclpy.init()
    node = LidarImuToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
