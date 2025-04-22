#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 기본 카르토그래퍼 설정 사용
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    
    # 노드 목록
    nodes = []
    
    # 1. 센서 노드들
    nodes.append(
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser'
            }]
        )
    )
    
    nodes.append(
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{'frame_id': 'base_link'}]  # 중요: IMU 프레임을 base_link로 변경
        )
    )
    
    # 2. TF 설정 - 중요 변경: imu_link를 base_link와 동일하게 설정
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    )
    
    # imu_link와 base_link를 오프셋 없이 일치시킴 (중요 수정)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 3. 포인트클라우드 변환 및 누적 (15초 후 시작)
    pc_nodes = []
    
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{'input_topic': 'scan', 'output_topic': 'pc_3d'}]
        )
    )
    
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True, 
                'max_points': 50000,
                'point_skip': 1  # 품질 향상을 위해 스킵 없음
            }]
        )
    )
    
    # 4. Cartographer 노드 (IMU 프레임 설정 수정)
    carto_nodes = []
    
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', 
                os.path.join(cartographer_ros_dir, 'configuration_files'),
                '-configuration_basename', 'backpack_3d.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),
                ('imu', 'imu')
            ]
        )
    )
    
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # 5. RViz 추가
    carto_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(cartographer_ros_dir, 'configuration_files', 'demo_3d.rviz')]
        )
    )
    
    # 6. 순차적 실행 (충분한 지연으로 초기화 안정화)
    nodes.append(TimerAction(period=5.0, actions=pc_nodes))  # 5초 후 포인트클라우드 처리
    nodes.append(TimerAction(period=10.0, actions=carto_nodes))  # 10초 후 Cartographer
    
    return LaunchDescription(nodes)