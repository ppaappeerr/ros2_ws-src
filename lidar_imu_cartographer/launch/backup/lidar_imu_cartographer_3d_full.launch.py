#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/lidar_imu_cartographer_3d_full.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')
    
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
                'inverted': False,
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
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link'],
        )
    )
    
    # 3. IMU TF 발행
    nodes.append(
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            parameters=[{
                'parent_frame': 'map',
                'child_frame': 'base_link',
                'publish_rate': 20.0,
                'stabilization_time': 1.0,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False,
                'maintain_orientation': False
            }]
        )
    )
    
    # 지연 시작 노드
    delayed_nodes = []
    
    # 4. 포인트클라우드 변환
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud_node',  # 중요: 올바른 실행 파일 이름으로 변경
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_height_coloring': True,
                'output_frame': 'laser',
                'use_tf': False
            }]
        )
    )
    
    # 5. 포인트클라우드 누적
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 150000,
                'grid_size': 0.01,
                'publish_rate': 20.0,
                'point_skip': 1,
                'use_imu_tf_only': True,
                'disable_position_tracking': True,
            }]
        )
    )
    
    # 6. Cartographer 3D
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
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
    
    # 7. 2D 점유 그리드 노드
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',  # 정확한 실행 파일 이름
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
            arguments=['-d', rviz_config],
            output='screen'
        )
    )
    
    # 지연 노드 타이머 추가
    nodes.append(
        TimerAction(
            period=2.0,
            actions=delayed_nodes
        )
    )
    
    # 실행 메시지
    nodes.append(LogInfo(msg="3D SLAM 시작: 카르토그래퍼와 누적 포인트클라우드 사용"))
    
    return LaunchDescription(nodes)