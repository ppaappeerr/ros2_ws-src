#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 기본 카르토그래퍼 설정 사용
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    
    # 기본 노드 (센서 + TF)
    nodes = []
    
    # 센서 노드들
    nodes.append(
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'frame_id': 'laser'}]
        )
    )
    
    nodes.append(
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{'frame_id': 'imu_link'}]
        )
    )
    
    # 기본 TF 설정
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
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 1단계 - 포인트 클라우드
    step1_nodes = []
    
    step1_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{'input_topic': 'scan', 'output_topic': 'pc_3d'}]
        )
    )
    
    step1_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{'use_tf': True, 'max_points': 50000}]
        )
    )
    
    # 2단계 - 카르토그래퍼 (공식 예제 사용)
    step2_nodes = []
    
    step2_nodes.append(
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
    
    step2_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # 3단계 - IMU TF
    step3_nodes = []
    
    step3_nodes.append(
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            parameters=[{'maintain_orientation': False}]
        )
    )
    
    # 타이머 추가
    nodes.append(TimerAction(period=2.0, actions=step1_nodes))
    nodes.append(TimerAction(period=5.0, actions=step2_nodes))
    nodes.append(TimerAction(period=10.0, actions=step3_nodes))
    
    return LaunchDescription(nodes)