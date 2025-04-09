#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
import struct

class SimplePointField:
    FLOAT32 = 7

def create_simple_cloud(header, points):
    """매우 간단한 포인트 클라우드 생성"""
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    
    # 필드 정의 - x, y, z만 사용
    fields = []
    
    # X 필드
    x_field = sensor_msgs.PointField()
    x_field.name = 'x'
    x_field.offset = 0
    x_field.datatype = SimplePointField.FLOAT32
    x_field.count = 1
    fields.append(x_field)
    
    # Y 필드
    y_field = sensor_msgs.PointField()
    y_field.name = 'y'
    y_field.offset = 4
    y_field.datatype = SimplePointField.FLOAT32
    y_field.count = 1
    fields.append(y_field)
    
    # Z 필드
    z_field = sensor_msgs.PointField()
    z_field.name = 'z'
    z_field.offset = 8
    z_field.datatype = SimplePointField.FLOAT32
    z_field.count = 1
    fields.append(z_field)
    
    cloud_msg.fields = fields
    
    # 포인트 데이터 설정
    if points:
        cloud_msg.height = 1
        cloud_msg.width = len(points)
    else:
        cloud_msg.height = 0
        cloud_msg.width = 0
    
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 12  # 3 floats * 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    
    # 데이터 패킹
    cloud_msg.data = bytearray()
    for p in points:
        cloud_msg.data.extend(struct.pack('fff', p[0], p[1], p[2]))
    
    cloud_msg.is_dense = True
    
    return cloud_msg

class SimpleAccumulator(Node):
    def __init__(self):
        super().__init__('simple_accumulator')
        
        # 레이저 스캔 구독
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # 누적 포인트 클라우드 발행
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'simple_cloud', 10)
        
        # 데이터 저장
        self.points = []
        self.max_points = 20000
        
        # 정기적 발행
        self.timer = self.create_timer(0.5, self.publish_cloud)
        
        self.get_logger().info("Simple accumulator started!")
    
    def scan_callback(self, msg):
        """레이저 스캔 데이터 처리"""
        # 현재 스캔에서의 포인트 추출
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 각도 계산
            angle = msg.angle_min + i * msg.angle_increment
            
            # 극좌표 -> 직교좌표 변환
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.05 * math.sin(angle * 5)  # 시각화를 위한 인위적인 z값
            
            # 포인트 추가
            self.points.append([x, y, z])
            
            # 최대 포인트 수 제한
            if len(self.points) > self.max_points:
                self.points.pop(0)  # 가장 오래된 포인트 제거
    
    def publish_cloud(self):
        """누적된 포인트 클라우드 발행"""
        if not self.points:
            return
        
        # 헤더 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'laser'  # 레이저 프레임 사용
        
        # 포인트 클라우드 생성 및 발행
        cloud_msg = create_simple_cloud(header, self.points)
        self.cloud_pub.publish(cloud_msg)
        
        self.get_logger().info(f"Published cloud with {len(self.points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()