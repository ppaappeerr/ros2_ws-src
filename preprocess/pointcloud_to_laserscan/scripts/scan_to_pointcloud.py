import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        
        # IMU 및 라이다 subscription
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # 포인트 클라우드 publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'points2', 10)
        
        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # IMU 데이터 저장
        self.last_imu = None
    
    def imu_callback(self, msg):
        self.last_imu = msg
    
    def scan_callback(self, msg):
        if self.last_imu is None:
            return
        
        # IMU 방향에서 롤, 피치 추출
        q = self.last_imu.orientation
        roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # 2D 레이저 스캔을 3D 포인트 클라우드로 변환
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            
            # 각도 계산
            angle = msg.angle_min + i * msg.angle_increment
            
            # 기본 2D 좌표 (x, y를 계산)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # IMU의 롤과 피치를 적용하여 Z 축 계산
            # 간단한 근사를 위해 평면 방정식 사용
            z = x * math.sin(pitch) + y * math.sin(roll)
            
            # 점의 품질을 반영하기 위해 강도 추가 (있는 경우)
            intensity = 255  # 기본값
            if len(msg.intensities) > i:
                intensity = int(msg.intensities[i])
            
            # XYZRGB 포인트 생성 (RGB는 강도에 기반)
            r = g = b = intensity
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            points.append([x, y, z, rgb])
        
        # 포인트 클라우드 생성 및 게시
        if points:
            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.UINT32, count=1)
            ]
            
            cloud_msg = pc2.create_cloud(msg.header, fields, points)
            cloud_msg.header.frame_id = 'laser'
            self.cloud_pub.publish(cloud_msg)
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        쿼터니언에서 오일러 각(roll, pitch, yaw) 계산
        """
        # 롤 (X 축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # 피치 (Y 축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # sinp 부호에 따라 90도
        else:
            pitch = math.asin(sinp)
        
        # 요 (Z 축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main():
    rclpy.init()
    node = ScanToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()