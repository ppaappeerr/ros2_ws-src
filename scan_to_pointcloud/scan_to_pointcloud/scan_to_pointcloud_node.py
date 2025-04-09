import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import sensor_msgs.msg as sensor_msgs

# PointField 클래스 정의
class PointField:
    INT8    = 1
    UINT8   = 2
    INT16   = 3
    UINT16  = 4
    INT32   = 5
    UINT32  = 6
    FLOAT32 = 7
    FLOAT64 = 8

# 수동 point_cloud2 함수 구현
def create_cloud(header, fields, points):
    """
    간단한 포인트 클라우드 메시지 생성 함수
    """
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    
    # 필드 설정
    cloud_msg.fields = []
    for f in fields:
        field = sensor_msgs.PointField()
        field.name = f.name
        field.offset = f.offset
        field.datatype = f.datatype
        field.count = f.count
        cloud_msg.fields.append(field)
    
    # 포인트 데이터 설정
    if len(points) > 0:
        cloud_msg.height = 1
        cloud_msg.width = len(points)
    else:
        cloud_msg.height = 0
        cloud_msg.width = 0
    
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 4 bytes per float * 4 values (x,y,z,rgb)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    
    # 바이트 배열로 변환
    cloud_msg.data = bytearray()
    for p in points:
        cloud_msg.data.extend(struct.pack('ffff', p[0], p[1], p[2], p[3]))
    
    cloud_msg.is_dense = True
    
    return cloud_msg

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
        
        self.get_logger().info("Scan to PointCloud converter initialized!")
    
    def imu_callback(self, msg):
        self.last_imu = msg
    
    def scan_callback(self, msg):
        if self.last_imu is None:
            self.get_logger().warn("No IMU data yet")
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
            intensity = 255.0  # 기본값
            if hasattr(msg, 'intensities') and len(msg.intensities) > i:
                intensity = float(msg.intensities[i])
            
            # XYZRGB 포인트 생성
            points.append([x, y, z, intensity])
        
        # 포인트 클라우드 생성 및 게시
        if points:
            fields = [
                # X 필드
                sensor_msgs.PointField(
                    name='x',
                    offset=0,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                # Y 필드
                sensor_msgs.PointField(
                    name='y',
                    offset=4,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                # Z 필드
                sensor_msgs.PointField(
                    name='z',
                    offset=8,
                    datatype=PointField.FLOAT32,
                    count=1
                ),
                # Intensity 필드
                sensor_msgs.PointField(
                    name='intensity',
                    offset=12,
                    datatype=PointField.FLOAT32,
                    count=1
                )
            ]
            
            cloud_msg = create_cloud(msg.header, fields, points)
            cloud_msg.header.frame_id = 'laser'
            self.cloud_pub.publish(cloud_msg)
            self.get_logger().info(f"Published pointcloud with {len(points)} points")
    
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

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()