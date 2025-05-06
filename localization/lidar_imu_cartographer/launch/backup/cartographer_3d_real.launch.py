#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_real.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 설정
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 노드 목록
    nodes = []
    
    # USB 권한 설정
    nodes.append(
        ExecuteProcess(
            cmd=['sudo', 'chmod', 'a+rw', '/dev/ttyUSB0'],
            output='screen'
        )
    )
    
    # 1. TF 트리 설정 (고정)
    nodes.append(LogInfo(msg="실제 IMU 움직임 기반 3D SLAM 시작..."))
    
    # world -> map
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    )
    
    # map -> odom (초기 상태)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    )
    
    # base_link -> laser (센서 위치)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    )
    
    # base_link -> imu_link (중요: IMU와 tracking_frame 일치)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 2. 센서 노드들
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
                'angle_compensate': True,
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
                'publish_rate': 50.0,
                'use_calibration': True
            }]
        )
    )
    
    # 3. IMU TF 발행 노드 (IMU 데이터 기반 움직임)
    nodes.append(
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            parameters=[{
                'parent_frame': 'odom',  # 중요: map이 아닌 odom에서 base_link로 변환
                'child_frame': 'base_link',
                'publish_rate': 50.0,
                'stabilization_time': 1.0,
                'maintain_orientation': False,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False
            }]
        )
    )
    
    # 4. 포인트 클라우드 변환 및 누적 노드
    pc_nodes = []
    
    # 스캔 -> 3D 포인트
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
            }]
        )
    )
    
    # 포인트 누적
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 10000,
                'grid_size': 0.05,
                'publish_rate': 10.0
            }]
        )
    )
    
    # 5. Cartographer 노드들
    carto_nodes = []
    
    # SLAM 노드
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_3d_real.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),
                ('imu', 'imu')
            ]
        )
    )
    
    # 점유 그리드 노드
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # RViz
    carto_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('cartographer_ros'), 
                                          'configuration_files', 'demo_3d.rviz')]
        )
    )
    
    # 순차적 실행
    nodes.append(TimerAction(period=1.0, actions=pc_nodes))   # 1초 후 포인트클라우드 처리
    nodes.append(TimerAction(period=3.0, actions=carto_nodes)) # 3초 후 Cartographer
    
    return LaunchDescription(nodes)