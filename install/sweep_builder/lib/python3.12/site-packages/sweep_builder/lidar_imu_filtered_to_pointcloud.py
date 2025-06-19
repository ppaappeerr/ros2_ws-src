import rclpy, numpy as np, tf_transformations, struct
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointField

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

    def create_pointcloud2(self, points_with_time):
        # PointCloud2 메시지 생성
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = 'base_link'
        
        # *** 올바른 필드 구조 설정 ***
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),  # 추가
            PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1),  # 수정
        ]
        
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 24  # 4*4 + 8 = 24 bytes per point
        cloud_msg.row_step = cloud_msg.point_step * len(points_with_time)
        cloud_msg.height = 1
        cloud_msg.width = len(points_with_time)
        
        # 데이터 패킹
        data = []
        for point in points_with_time:
            x, y, z, timestamp = point
            # x, y, z, intensity, timestamp 순서로 패킹
            data.extend(struct.pack('ffffd', x, y, z, 1.0, timestamp))  # intensity=1.0으로 고정
        
        cloud_msg.data = bytes(data)
        return cloud_msg

def main():
    rclpy.init(); rclpy.spin(LidarImuPC()); rclpy.shutdown()
