#!/usr/bin/env python3
"""
HeightMap 2.5D vs FTG-3D 알고리즘 성능 비교 스크립트

실시간으로 두 알고리즘의 성능을 비교하고 분석합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud2
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
import threading

class AlgorithmComparator(Node):
    def __init__(self):
        super().__init__('algorithm_comparator')
        
        # 데이터 저장용 큐 (최근 100개 데이터)
        self.heightmap_angles = deque(maxlen=100)
        self.ftg3d_angles = deque(maxlen=100)
        self.timestamps = deque(maxlen=100)
        
        # 통계 데이터
        self.stats = {
            'heightmap': {'total_commands': 0, 'avg_deviation': 0.0, 'response_times': []},
            'ftg3d': {'total_commands': 0, 'avg_deviation': 0.0, 'response_times': []},
            'comparison': {'similarity': 0.0, 'correlation': 0.0}
        }
        
        # 구독자 설정
        self.heightmap_sub = self.create_subscription(
            Vector3Stamped, '/safe_path_vector_heightmap',
            self.heightmap_callback, 10)
        
        self.ftg3d_sub = self.create_subscription(
            Vector3Stamped, '/safe_path_vector_ftg3d',
            self.ftg3d_callback, 10)
        
        # 타이머 설정 (1초마다 분석)
        self.analysis_timer = self.create_timer(1.0, self.analyze_performance)
        
        self.get_logger().info("Algorithm Comparator started")
    
    def heightmap_callback(self, msg):
        angle = np.arctan2(msg.vector.y, msg.vector.x)
        current_time = time.time()
        
        self.heightmap_angles.append(angle)
        self.timestamps.append(current_time)
        self.stats['heightmap']['total_commands'] += 1
    
    def ftg3d_callback(self, msg):
        angle = np.arctan2(msg.vector.y, msg.vector.x)
        current_time = time.time()
        
        self.ftg3d_angles.append(angle)
        self.stats['ftg3d']['total_commands'] += 1
    
    def analyze_performance(self):
        if len(self.heightmap_angles) < 10 or len(self.ftg3d_angles) < 10:
            return
        
        # 각도 차이 분석
        heightmap_data = np.array(list(self.heightmap_angles))
        ftg3d_data = np.array(list(self.ftg3d_angles))
        
        # 길이 맞추기 (더 짧은 것에 맞춤)
        min_len = min(len(heightmap_data), len(ftg3d_data))
        heightmap_data = heightmap_data[-min_len:]
        ftg3d_data = ftg3d_data[-min_len:]
        
        # 통계 계산
        angle_diff = np.abs(heightmap_data - ftg3d_data)
        similarity = 1.0 - np.mean(angle_diff) / np.pi  # 0~1 사이 값
        correlation = np.corrcoef(heightmap_data, ftg3d_data)[0, 1] if min_len > 1 else 0.0
        
        # 안정성 분석 (각도 변화율)
        heightmap_stability = self.calculate_stability(heightmap_data)
        ftg3d_stability = self.calculate_stability(ftg3d_data)
        
        # 통계 업데이트
        self.stats['comparison']['similarity'] = similarity
        self.stats['comparison']['correlation'] = correlation
        self.stats['heightmap']['stability'] = heightmap_stability
        self.stats['ftg3d']['stability'] = ftg3d_stability
        
        # 로그 출력
        self.get_logger().info(
            f"Performance Comparison:\n"
            f"  HeightMap 2.5D: {self.stats['heightmap']['total_commands']} commands, "
            f"stability: {heightmap_stability:.3f}\n"
            f"  FTG-3D: {self.stats['ftg3d']['total_commands']} commands, "
            f"stability: {ftg3d_stability:.3f}\n"
            f"  Similarity: {similarity:.3f}, Correlation: {correlation:.3f}"
        )
    
    def calculate_stability(self, angles):
        """각도 변화율로 안정성 계산 (낮을수록 안정함)"""
        if len(angles) < 2:
            return 0.0
        
        angle_changes = np.diff(angles)
        # 각도 차이를 -π ~ π 범위로 정규화
        angle_changes = np.arctan2(np.sin(angle_changes), np.cos(angle_changes))
        return np.std(angle_changes)  # 표준편차로 안정성 측정
    
    def save_results(self, filename="algorithm_comparison.json"):
        """결과를 JSON 파일로 저장"""
        results = {
            'timestamp': time.time(),
            'stats': self.stats,
            'recent_data': {
                'heightmap_angles': list(self.heightmap_angles),
                'ftg3d_angles': list(self.ftg3d_angles),
                'timestamps': list(self.timestamps)
            }
        }
        
        with open(filename, 'w') as f:
            import json
            json.dump(results, f, indent=2)
        
        self.get_logger().info(f"Results saved to {filename}")

def main():
    rclpy.init()
    
    comparator = AlgorithmComparator()
    
    try:
        # 백그라운드에서 분석 실행
        rclpy.spin(comparator)
    except KeyboardInterrupt:
        # Ctrl+C로 종료시 결과 저장
        comparator.save_results()
        comparator.get_logger().info("Comparison completed and saved")
    finally:
        comparator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
