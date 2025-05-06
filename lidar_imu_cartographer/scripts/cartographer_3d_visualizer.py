#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/scripts/cartographer_3d_visualizer.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import struct
import numpy as np

class Cartographer3DVisualizer(Node):
    def __init__(self):
        super().__init__('cartographer_3d_visualizer')
        
        # 누적 포인트클라우드와 서브맵 구독
        self.submap_sub = self.create_subscription(
            MarkerArray, '/submaps', self.submap_callback, 10)
        self.points_sub = self.create_subscription(
            PointCloud2, '/accumulated_points', self.points_callback, 10)
        
        # 3D 맵 포인트클라우드 발행
        self.map_3d_pub = self.create_publisher(
            PointCloud2, '/map_3d_cloud', 10)
            
        # 디버그 마커 발행
        self.debug_marker_pub = self.create_publisher(
            MarkerArray, '/map_3d_markers', 10)
            
        # 맵 데이터 저장
        self.submap_points = []
        self.last_update_time = self.get_clock().now()
        
        # 주기적 업데이트
        self.timer = self.create_timer(2.0, self.publish_3d_map)
        
        self.get_logger().info("3D 맵 시각화 노드 시작")
        
    def submap_callback(self, msg):
        """서브맵 마커 처리"""
        self.get_logger().info(f"서브맵 마커 수신: {len(msg.markers)}개")
        
        # 마커 분석 및 추출 (필요 시)
        for i, marker in enumerate(msg.markers):
            if i < 3:  # 일부 마커만 로깅
                self.get_logger().info(f"마커 {i}: 타입={marker.type}, 포인트 수={len(marker.points)}")
                
        # 디버그용 마커 재발행
        self.debug_marker_pub.publish(msg)
        
    def points_callback(self, msg):
        """누적 포인트클라우드 처리"""
        # 로그 빈도 제한
        now = self.get_clock().now()
        if (now - self.last_update_time).nanoseconds / 1e9 > 5.0:
            self.get_logger().info(f"포인트클라우드 수신: {msg.width * msg.height}개 포인트")
            self.last_update_time = now
            
        # 3D 맵 데이터로 재발행
        self.publish_enhanced_cloud(msg)
        
    def publish_enhanced_cloud(self, cloud_msg):
        """포인트클라우드를 향상된 형태로 재발행"""
        # 헤더 복사하되 프레임은 map으로
        enhanced_msg = PointCloud2()
        enhanced_msg.header = cloud_msg.header
        enhanced_msg.header.frame_id = 'map'
        
        # 나머지 필드 복사
        enhanced_msg.height = cloud_msg.height
        enhanced_msg.width = cloud_msg.width
        enhanced_msg.fields = cloud_msg.fields
        enhanced_msg.is_bigendian = cloud_msg.is_bigendian
        enhanced_msg.point_step = cloud_msg.point_step
        enhanced_msg.row_step = cloud_msg.row_step
        enhanced_msg.data = cloud_msg.data
        enhanced_msg.is_dense = cloud_msg.is_dense
        
        # 개선된 3D 맵으로 발행
        self.map_3d_pub.publish(enhanced_msg)
        
    def publish_3d_map(self):
        """3D 맵 데이터 발행"""
        self.get_logger().info("3D 맵 발행 중...")
        
        # 맵 마커 생성
        marker_array = MarkerArray()
        
        # 기본 월드 좌표축 마커
        axis_marker = Marker()
        axis_marker.header.frame_id = 'map'
        axis_marker.header.stamp = self.get_clock().now().to_msg()
        axis_marker.ns = 'map_3d_coords'
        axis_marker.id = 0
        axis_marker.type = Marker.LINE_LIST
        axis_marker.action = Marker.ADD
        axis_marker.scale.x = 0.05  # 선 두께
        
        # X축 (빨강), Y축 (녹색), Z축 (파랑)
        axis_marker.points = [
            # X축
            Point(x=0, y=0, z=0),
            Point(x=1, y=0, z=0),
            # Y축
            Point(x=0, y=0, z=0),
            Point(x=0, y=1, z=0),
            # Z축
            Point(x=0, y=0, z=0),
            Point(x=0, y=0, z=1)
        ]
        
        axis_marker.colors = [
            # X축 색상
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
            # Y축 색상
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            # Z축 색상
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        ]
        
        marker_array.markers.append(axis_marker)
        self.debug_marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Cartographer3DVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()