#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    nodes = []
    
    # 0. 라이다 권한 설정
    nodes.append(
        ExecuteProcess(
            cmd=['sudo', 'chmod', 'a+rw', '/dev/ttyUSB0'],
            output='screen'
        )
    )
    
    # 1. 고정 TF 설정 (기본 프레임 구조)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 2. 센서 노드
    nodes.append(
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True
            }]
        )
    )
    
    nodes.append(
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_rate': 50.0
            }]
        )
    )
    
    # 3. IMU TF 발행 (핵심: map → base_link 변환)
    nodes.append(
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            parameters=[{
                'parent_frame': 'map',
                'child_frame': 'base_link',
                'publish_rate': 20.0,
                'stabilization_time': 2.0,
                'maintain_orientation': False,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False
            }]
        )
    )
    
    # 4. 지연 시작 노드들 (안정화 후)
    delayed_nodes = []
    
    # 4.1 스캔 → 포인트 클라우드
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'output_frame': 'laser'  # 출력 프레임 명시적 설정
            }]
        )
    )
    
    # 4.2 포인트 클라우드 누적
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,  # 중요: TF 변환 활성화
                'max_points': 100000,
                'grid_size': 0.02,  # 적당한 해상도
                'publish_rate': 10.0
            }]
        )
    )
    
    # 4.3 카르토그래퍼 (가장 마지막에 시작)
    carto_nodes = []
    
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_3d_fixed.lua'
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
    
    # RViz 추가
    carto_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/opt/ros/jazzy/share/cartographer_ros/configuration_files/demo_3d.rviz']
        )
    )
    
    # 순차적 실행
    nodes.append(TimerAction(period=3.0, actions=delayed_nodes))  # 3초 후 포인트클라우드 처리
    nodes.append(TimerAction(period=6.0, actions=carto_nodes))    # 6초 후 Cartographer
    
    return LaunchDescription(nodes)