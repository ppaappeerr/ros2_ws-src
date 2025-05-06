#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 노드 목록
    nodes = []
    
    # 1. 센서 노드
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
    
    # 2. TF 설정
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
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 3. IMU TF 발행 노드
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
                'maintain_orientation': False,  # 오뚝이 효과 제거
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False
            }]
        )
    )
    
    # 이 노드들은 지연 실행됨 (TF 설정 후에 시작)
    delayed_nodes = []
    
    # 4. 스캔-포인트클라우드 변환
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_tf': True,  # TF 변환 활성화
                'use_height_coloring': True,
                'add_z_variation': True,  # Z축 변화 추가 (새 파라미터)
                'z_variation_mode': 'tilt',  # 'tilt', 'cone' 또는 'grid'
                'z_variation_factor': 0.15   # Z축 변화 강도
            }]
        )
    )
    
    # 5. 누적 포인트클라우드
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 80000,  # 더 많은 포인트 허용
                'grid_size': 0.01,
                'publish_rate': 10.0,  # 더 빠른 갱신
                'use_imu_tf_only': False,  # IMU와 Cartographer TF 모두 사용
                'disable_position_tracking': False  # 위치 추적 활성화
            }]
        )
    )
    
    # 6. Cartographer 노드
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_3d.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),
                ('imu', 'imu')
            ]
        )
    )
    
    # 7. 점유 그리드 노드
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # 8. RViz
    delayed_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')]
        )
    )
    
    # TF 설정 후 5초 대기한 다음 나머지 노드 시작
    nodes.append(
        TimerAction(
            period=5.0,
            actions=delayed_nodes
        )
    )
    
    return LaunchDescription(nodes)