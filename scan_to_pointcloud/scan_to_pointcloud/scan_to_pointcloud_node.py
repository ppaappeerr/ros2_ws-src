#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/scan_to_pointcloud_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import math
import struct
import tf2_ros
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy  # QoS import 추가

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        
        # 파라미터
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'pc_3d')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('output_frame', '')  # 빈 문자열이면 input_frame과 동일
        
        # 파라미터 가져오기
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.output_frame = self.get_parameter('output_frame').value or self.frame_id
        
        # QoS 설정 (Reliable)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # BEST_EFFORT -> RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # Depth는 유지하거나 늘릴 수 있음
            durability=DurabilityPolicy.VOLATILE  # VOLATILE 유지
        )

        # 포인트 클라우드 발행자 (수정된 QoS 적용)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile)  # qos_profile 적용

        # 스캔 구독 (기존 QoS 유지 또는 BEST_EFFORT 사용 가능)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, self.input_topic, self.scan_callback, scan_qos)  # scan_qos 적용

        # TF 버퍼
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"스캔→포인트클라우드 변환 노드 시작: {self.input_topic} → {self.output_topic}")
    
    def scan_callback(self, msg):
        # 2D 스캔 데이터를 3D 포인트 클라우드로 변환
        points = []
        
        # 각 스캔 포인트를 순회
        for i, r in enumerate(msg.ranges):
            # 유효하지 않은 범위 건너뛰기
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 2D 좌표 계산 (레이저 프레임 기준)
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0  # 레이저 평면 상에서는 z=0
            
            # 강도 (선택적)
            intensity = 100.0
            if hasattr(msg, 'intensities') and i < len(msg.intensities):
                if np.isfinite(msg.intensities[i]):
                    intensity = float(msg.intensities[i])
            
            # 포인트 추가 (x, y, z, intensity)
            points.append([x, y, z, intensity])
        
        # 포인트 클라우드 메시지 생성
        if points:
            # 필드 정의
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 헤더
            header = msg.header
            header.frame_id = self.output_frame  # 중요: 명시적으로 출력 프레임 설정
            
            # PointCloud2 메시지 생성 및 발행
            cloud_msg = self.create_cloud_msg(header, fields, points)
            self.cloud_pub.publish(cloud_msg)
    
    def create_cloud_msg(self, header, fields, points):
        """PointCloud2 메시지 생성"""
        msg = PointCloud2()
        msg.header = header
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 x float32
        msg.row_step = msg.point_step * msg.width
        
        # 포인트 데이터를 바이트 배열로 변환
        buf = bytearray()
        for p in points:
            buf.extend(struct.pack('ffff', *p))
        msg.data = bytes(buf)
        
        msg.is_dense = True
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    finally:
        # --- 수정: 종료 처리 로직 개선 ---
        if node and rclpy.ok(): # 노드가 존재하고 rclpy 컨텍스트가 유효할 때만 destroy
            node.destroy_node()
        if rclpy.ok(): # rclpy 컨텍스트가 유효할 때만 shutdown
             rclpy.shutdown()
        # ---------------------

if __name__ == '__main__':
    main()