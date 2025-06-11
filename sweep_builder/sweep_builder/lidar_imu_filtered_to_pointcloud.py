import rclpy, numpy as np, tf_transformations
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node

class LidarImuPC(Node):
    def __init__(self):
        super().__init__('lidar_imu_pc')
        self.create_subscription(LaserScan,'/scan',self.scan_cb,10)
        self.create_subscription(Imu,'/imu/data',self.imu_cb,50)
        self.pub = self.create_publisher(PointCloud2,'/points_3d',10)
        self.q_imu = [0,0,0,1]

    def imu_cb(self,msg:Imu):
        self.q_imu = [msg.orientation.x,msg.orientation.y,
                      msg.orientation.z,msg.orientation.w]

    def scan_cb(self,scan:LaserScan):
        R_imu = tf_transformations.quaternion_matrix(self.q_imu)[:3,:3]
        pts, ang = [], scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * np.cos(ang)
                y = r * np.sin(ang)
                pt = np.array([x, y, 0.0], dtype=np.float32)
                pt_rot = R_imu @ pt  # IMU 회전만 1회 적용
                pts.append(pt_rot)
            ang += scan.angle_increment
        cloud = point_cloud2.create_cloud_xyz32(scan.header, pts)
        self.pub.publish(cloud)

def main():
    rclpy.init(); rclpy.spin(LidarImuPC()); rclpy.shutdown()
