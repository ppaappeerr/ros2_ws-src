#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_stable.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 센서 및 고정 TF 노드 - static_transform만 발행
    sensor_nodes = [
        Node(package='sllidar_ros2', executable='sllidar_node', 
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': '/dev/ttyUSB0',
                 'serial_baudrate': 115200,
                 'frame_id': 'laser',
                 'angle_compensate': True
             }]),
        Node(package='mpu6050_py', executable='mpu6050_node', name='mpu6050_node'),
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']),
    ]
    
    # 지연 시작 노드들 (센서 안정화 후)
    delayed_nodes = [
        # 포인트 클라우드 변환
        Node(package='scan_to_pointcloud', executable='scan_to_pointcloud',
             parameters=[{'input_topic': 'scan', 'output_topic': 'pc_3d'}]),
        
        # 포인트 클라우드 누적
        Node(package='scan_to_pointcloud', executable='accumulated_pointcloud',
             parameters=[{
                 'use_tf': True, 
                 'max_points': 50000, 
                 'publish_rate': 5.0,
                 'use_imu_tf_only': False,  # 중요: 이 설정이 false여야 cartographer tf와 충돌 안 함
                 'disable_position_tracking': True  # 위치는 카르토그래퍼가 관리
             }]),
        
        # 카르토그래퍼 노드 - TF 생성 책임
        Node(package='cartographer_ros', executable='cartographer_node',
             arguments=['-configuration_directory', config_dir, 
                       '-configuration_basename', 'cartographer_3d.lua'],
             remappings=[('points2', 'accumulated_points')]),
        
        # 맵 생성 노드
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node')
    ]
    
    # 가장 늦게 시작하는 imu_tf_publisher (추가 지연)
    tf_publisher_node = Node(
        package='lidar_imu_fusion', 
        executable='imu_tf_publisher',
        parameters=[{
            'parent_frame': 'map', 
            'child_frame': 'base_link',
            'publish_rate': 10.0,
            'maintain_orientation': False,  # 중요: 오뚝이 효과 제거
            'invert_roll': False, 
            'invert_pitch': False, 
            'invert_yaw': False
        }]
    )
    
    return LaunchDescription([
        LogInfo(msg="안정적인 3D 카르토그래퍼 SLAM 시작 - TF 충돌 해결"),
        *sensor_nodes,
        TimerAction(period=2.0, actions=delayed_nodes),  # 2초 대기 후 실행
        TimerAction(period=6.0, actions=[tf_publisher_node])  # 카르토그래퍼 준비 후 실행
    ])