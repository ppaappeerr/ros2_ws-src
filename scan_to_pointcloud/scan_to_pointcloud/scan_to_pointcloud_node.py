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
        
        # 매개변수 추가 - 숫자로 시작하지 않는 토픽 이름 사용
        self.declare_parameter('output_topic', 'pc_3d')
        self.output_topic = self.get_parameter('output_topic').value
        
        # 포인트 클라우드 publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_topic, 10)
        
        # IMU 및 라이다 subscription
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
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
        
        # 현재 TF 확인
        try:
            # base_link를 기준으로 laser의 위치 확인
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'laser',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            # 변환을 찾을 수 없으면 기본 프레임 사용
        
        # 2D 레이저 스캔을 IMU 방향을 고려한 3D 포인트 클라우드로 변환
        points = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
                
            # 각도 계산
            angle = msg.angle_min + i * msg.angle_increment
            
            # 기본 2D 좌표 (x, y를 계산)
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # IMU의 롤과 피치를 이용한 개선된 3D 변환
            q = self.last_imu.orientation
            R = self.quaternion_to_rotation_matrix([q.x, q.y, q.z, q.w])
            
            # 레이저 평면상의 점
            point_local = np.array([x, y, 0.0])
            
            # IMU 방향 적용 - 회전 행렬을 통한 변환
            point_rotated = np.dot(R, point_local)
            
            # 변환된 3D 좌표
            x_3d, y_3d, z_3d = point_rotated
            
            # 강도 정보 추가
            intensity = 100.0
            if hasattr(msg, 'intensities') and i < len(msg.intensities):
                intensity = float(msg.intensities[i])
            
            # 포인트 추가
            points.append([x_3d, y_3d, z_3d, intensity])
        
        # 포인트클라우드 생성 및 발행
        if points:
            # 필드 설정
            fields = [
                sensor_msgs.PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 헤더 설정 - 프레임 ID를 'base_link'로 고정 (항상 같은 프레임 사용)
            header = msg.header
            header.frame_id = 'base_link'  # 이전: 'laser'
            
            cloud_msg = create_cloud(header, fields, points)
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
    
    def quaternion_to_rotation_matrix(self, q):
        """
        쿼터니언을 회전 행렬로 변환
        """
        x, y, z, w = q
        R = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])
        return R

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()