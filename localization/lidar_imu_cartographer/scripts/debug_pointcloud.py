#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/scripts/debug_pointcloud.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import subprocess
import time

class PointCloudDebugger(Node):
    def __init__(self):
        super().__init__('pointcloud_debugger')
        
        # 토픽 구독
        self.pc_sub = self.create_subscription(
            PointCloud2, 'accumulated_points', 
            self.pointcloud_callback, 10)
        
        # 상태 변수
        self.last_pc_time = None
        self.pc_count = 0
        self.timestamp = None
        self.frame_id = None
        
        # 타이머 설정
        self.timer = self.create_timer(5.0, self.check_pointcloud_status)
        
        self.get_logger().info("포인트 클라우드 디버거 시작")
    
    def pointcloud_callback(self, msg):
        """포인트 클라우드 데이터 수신 시 호출"""
        self.last_pc_time = self.get_clock().now()
        self.pc_count += 1
        self.timestamp = msg.header.stamp
        self.frame_id = msg.header.frame_id
        
        # 처음 몇 번만 상세 정보 출력
        if self.pc_count < 5 or self.pc_count % 10 == 0:
            self.get_logger().info(
                f"포인트 클라우드 수신 #{self.pc_count}: "
                f"크기={len(msg.data)} 바이트, "
                f"프레임={msg.header.frame_id}, "
                f"포인트 수={msg.width * msg.height}"
            )
    
    def check_pointcloud_status(self):
        """포인트 클라우드 상태 확인"""
        now = self.get_clock().now()
        
        # 시스템 상태 출력
        self.get_logger().info("===== 포인트 클라우드 상태 =====")
        
        # 마지막 수신 시간 출력
        if self.last_pc_time:
            diff = (now - self.last_pc_time).nanoseconds / 1e9
            self.get_logger().info(f"마지막 포인트 클라우드: {diff:.1f}초 전")
            if self.frame_id:
                self.get_logger().info(f"프레임 ID: {self.frame_id}")
            if self.timestamp:
                self.get_logger().info(f"타임스탬프: {self.timestamp.sec}.{self.timestamp.nanosec}")
        else:
            self.get_logger().warn("포인트 클라우드가 아직 수신되지 않음")
        
        # 누적 카운트 출력
        self.get_logger().info(f"총 수신 횟수: {self.pc_count}")
        
        # 활성 토픽 확인
        try:
            output = subprocess.check_output(
                "ros2 topic list -t | grep -E 'point|cloud|scan'", 
                shell=True
            ).decode('utf-8')
            self.get_logger().info(f"관련 토픽:\n{output}")
        except:
            self.get_logger().warn("토픽 목록 확인 실패")
        
        # TF 상태 확인
        try:
            tf_output = subprocess.check_output(
                "ros2 topic echo /tf --max-messages 1 2>/dev/null", shell=True
            ).decode('utf-8')
            self.get_logger().info(f"TF 상태: {len(tf_output.strip()) > 0}")
        except:
            self.get_logger().warn("TF 상태 확인 실패")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()