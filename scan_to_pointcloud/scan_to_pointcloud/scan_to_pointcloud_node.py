import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np
import math
import struct
from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs
import tf2_ros

class PointField:
    INT8    = 1
    UINT8   = 2
    INT16   = 3
    UINT16  = 4
    INT32   = 5
    UINT32  = 6
    FLOAT32 = 7
    FLOAT64 = 8

def create_cloud(header, fields, points):
    """간단한 포인트 클라우드 메시지 생성 함수"""
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    
    cloud_msg.fields = []
    for f in fields:
        field = sensor_msgs.PointField()
        field.name = f.name
        field.offset = f.offset
        field.datatype = f.datatype
        field.count = f.count
        cloud_msg.fields.append(field)
    
    if len(points) > 0:
        cloud_msg.height = 1
        cloud_msg.width = len(points)
    else:
        cloud_msg.height = 0
        cloud_msg.width = 0
    
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 4 bytes per float * 4 values (x,y,z,rgb)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    
    cloud_msg.data = bytearray()
    for p in points:
        cloud_msg.data.extend(struct.pack('ffff', p[0], p[1], p[2], p[3]))
    
    cloud_msg.is_dense = True
    
    return cloud_msg

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        
        # 파라미터 선언
        self.declare_parameter('input_scan_topic', 'scan')
        self.declare_parameter('output_cloud_topic', 'points2')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('min_height', -0.1)  # Z축 최소값 (m)
        self.declare_parameter('max_height', 0.1)   # Z축 최대값 (m)
        self.declare_parameter('height_increment', 0.01) # Z축 증가량 (m)
        
        # 파라미터 로드
        input_topic = self.get_parameter('input_scan_topic').value
        output_topic = self.get_parameter('output_cloud_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.height_increment = self.get_parameter('height_increment').value
        
        # 구독 및 발행
        self.scan_sub = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, output_topic, 10)
            
        # TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"ScanToPointcloud: Converting {input_topic} to {output_topic}")
    
    def scan_callback(self, msg: LaserScan):
        """라이다 스캔을 포인트 클라우드로 변환"""
        points = []
        num_heights = int((self.max_height - self.min_height) / self.height_increment) + 1
        
        # 각 레이저 스캔 포인트에 대해
        for i, r in enumerate(msg.ranges):
            # 유효한 거리 범위 내의 포인트만 처리
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 각도 계산
            angle = msg.angle_min + i * msg.angle_increment
            
            # 2D 좌표 계산
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            # 여러 높이에 포인트 생성하여 3D로 확장
            for h_idx in range(num_heights):
                z = self.min_height + h_idx * self.height_increment
                
                # 강도값 (있으면 사용, 없으면 기본값)
                intensity = 100.0
                if hasattr(msg, 'intensities') and len(msg.intensities) > i:
                    intensity = float(msg.intensities[i])
                
                # 거리에 따라 강도 감쇠
                intensity_scaled = intensity * (1.0 - (r / msg.range_max * 0.7))
                
                # 3D 포인트 추가
                points.append([x, y, z, intensity_scaled])
        
        # 포인트 클라우드 메시지 생성
        if points:
            fields = [
                sensor_msgs.PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 헤더 설정
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = self.laser_frame
            
            # 포인트 클라우드 생성 및 발행
            cloud_msg = create_cloud(header, fields, points)
            self.cloud_pub.publish(cloud_msg)
            
            # 로깅 (1초에 한 번 정도)
            now = self.get_clock().now()
            if int(now.nanoseconds / 1e9) % 1 == 0:
                self.get_logger().info(f"Published point cloud with {len(points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()