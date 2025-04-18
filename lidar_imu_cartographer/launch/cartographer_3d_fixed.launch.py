#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 센서 및 고정 TF 노드
    sensor_nodes = [
        # 라이다 설정 명시 추가
        Node(package='sllidar_ros2', executable='sllidar_node', 
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': '/dev/ttyUSB0',
                 'serial_baudrate': 115200,
                 'frame_id': 'laser',
                 'angle_compensate': True
             }]),
        Node(package='mpu6050_py', executable='mpu6050_node', 
             parameters=[{'frame_id': 'imu_link'}]),
        
        # 고정 TF 설정
        Node(package='tf2_ros', executable='static_transform_publisher', 
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']),
    ]
    
    # 지연 시작 노드들
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
                 # 주요 변경점: 카르토그래퍼가 TF만 처리하도록 설정
                 'use_imu_tf_only': True,  
                 'disable_position_tracking': True  
             }]),
        
        # 카르토그래퍼 노드
        Node(package='cartographer_ros', executable='cartographer_node',
             arguments=['-configuration_directory', config_dir, 
                       '-configuration_basename', 'cartographer_3d.lua'],
             remappings=[('points2', 'accumulated_points')]),
        
        # 맵 생성 노드
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node'),
        
        # RViz 추가
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', '/opt/ros/jazzy/share/cartographer_ros/configuration_files/demo_3d.rviz'])
    ]
    
    return LaunchDescription([
        LogInfo(msg="TF 충돌 해결된 3D 카르토그래퍼 SLAM"),
        *sensor_nodes,
        TimerAction(period=2.0, actions=delayed_nodes)  # 2초 대기 후 실행
    ])