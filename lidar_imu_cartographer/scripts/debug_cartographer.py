#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/scripts/debug_cartographer.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import subprocess
import time
import sys

class CartographerDebugger(Node):
    def __init__(self):
        super().__init__('cartographer_debugger')
        
        # 카르토그래퍼 노드 실행 확인
        self.get_logger().info("카르토그래퍼 노드 상태 확인 중...")
        self.check_cartographer_nodes()
        
        # accumulated_points 토픽 구독
        self.sub_points = self.create_subscription(
            PointCloud2, 
            '/accumulated_points', 
            self.accumulated_points_callback, 
            10)
        
        # 점유 그리드 맵 토픽 모니터링
        self.sub_map = self.create_subscription(
            String, 
            '/map', 
            self.map_callback, 
            10)
        
        # 정기적인 시스템 상태 점검
        self.timer = self.create_timer(5.0, self.check_system_status)
        
        # accumulated_points 데이터 카운터
        self.points_count = 0
        self.last_points_size = 0
        self.last_points_time = None
    
    def check_cartographer_nodes(self):
        """카르토그래퍼 관련 노드 실행 상태 확인"""
        try:
            output = subprocess.check_output(['ros2', 'node', 'list']).decode('utf-8')
            nodes = output.strip().split('\n')
            
            # 카르토그래퍼 노드 확인
            carto_nodes = [n for n in nodes if 'cartographer' in n]
            
            if not carto_nodes:
                self.get_logger().error("카르토그래퍼 노드가 실행중이지 않습니다!")
            else:
                self.get_logger().info(f"실행 중인 카르토그래퍼 노드: {', '.join(carto_nodes)}")
                
                # 각 노드의 파라미터 확인
                for node in carto_nodes:
                    try:
                        params = subprocess.check_output(['ros2', 'param', 'list', node]).decode('utf-8')
                        self.get_logger().info(f"노드 {node}의 파라미터: \n{params}")
                    except:
                        self.get_logger().warning(f"노드 {node}의 파라미터를 가져올 수 없습니다.")
            
            # TF 토픽이 제대로 발행되고 있는지 확인
            self.check_tf_status()
            
        except Exception as e:
            self.get_logger().error(f"노드 목록 확인 오류: {e}")
    
    def check_tf_status(self):
        """TF 토픽 상태 확인"""
        try:
            cmd = "ros2 topic echo /tf --once"
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT)
            self.get_logger().info("TF 토픽이 발행되고 있습니다.")
        except:
            self.get_logger().error("TF 토픽 발행 문제가 있습니다!")
    
    def check_cartographer_logs(self):
        """카르토그래퍼 로그 확인 개선 버전"""
        try:
            cmd = "ros2 topic echo /rosout --field level --field name --field message --once --no-daemon 2>/dev/null | grep -E 'cartographer|error|warn'"
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.PIPE, timeout=2)
            return output.decode('utf-8')
        except subprocess.TimeoutExpired:
            return "로그 확인 제한 시간 초과"
        except Exception as e:
            return f"로그 확인 오류: {e}"
    
    def accumulated_points_callback(self, msg):
        """accumulated_points 토픽 콜백"""
        self.points_count += 1
        now = self.get_clock().now()
        
        # 새로운 포인트 메시지가 있을 때만 로그
        point_size = msg.width * msg.height
        
        if self.last_points_time is None or \
           (now - self.last_points_time).nanoseconds / 1e9 > 5.0:
            self.get_logger().info(
                f"accumulated_points: {point_size}개 포인트, "
                f"프레임: {msg.header.frame_id}, "
                f"타임스탬프: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}"
            )
            self.last_points_time = now
            self.last_points_size = point_size
    
    def map_callback(self, msg):
        """map 토픽 콜백"""
        self.get_logger().info(f"Map 데이터 수신! (크기: {len(str(msg))} 바이트)")
    
    def check_system_status(self):
        """시스템 상태 주기적 체크"""
        # 실행 중인 모든 노드 및 토픽 정보 출력
        self.get_logger().info("=== 시스템 상태 점검 ===")
        
        # accumulated_points 토픽 상태
        self.get_logger().info(f"accumulated_points 메시지 카운트: {self.points_count}")
        
        # 카르토그래퍼 로그 확인 (가능한 경우)
        try:
            # 최근 카르토그래퍼 로그 확인
            log_output = self.check_cartographer_logs()
            self.get_logger().info(f"카르토그래퍼 로그: {log_output}")
        except:
            pass
        
        # 토픽 정보 출력
        try:
            topics_cmd = "ros2 topic list -t | grep -E '(point|map|tf)'"
            topics = subprocess.check_output(topics_cmd, shell=True).decode('utf-8')
            self.get_logger().info(f"관련 토픽들:\n{topics}")
        except:
            self.get_logger().warning("토픽 정보를 가져올 수 없습니다.")

def main(args=None):
    rclpy.init(args=args)
    node = CartographerDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()