#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_solution.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 1단계: 센서와 TF 설정 노드
    nodes = []
    
    # 세부 로그 출력
    nodes.append(LogInfo(msg="Cartographer 3D 솔루션 실행 - TF 및 시간 충돌 해결"))
    
    # 1.1 정적 TF 노드들 - 기본 TF 트리 구조 설정
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    ))
    
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    ))
    
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    ))
    
    # 1.2 센서 노드들
    nodes.append(Node(
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
    ))
    
    nodes.append(Node(
        package='mpu6050_py',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate': 50.0
        }]
    ))
    
    # 2단계: 지연 시작 노드들 (센서/TF 초기화 후)
    # - 이 단계에서는 포인트 클라우드 변환과 누적 노드를 시작합니다.
    stage1_nodes = []
    
    # 2.1 2D 스캔 -> 3D 포인트 클라우드 변환
    stage1_nodes.append(Node(
        package='scan_to_pointcloud',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        parameters=[{
            'input_topic': 'scan',
            'output_topic': 'pc_3d',
            'frame_id': 'laser',
            'use_height_coloring': True,
            'output_frame': 'laser',
            'use_tf': False  # TF 변환 없이 직접 변환 (중요)
        }]
    ))
    
    # 2.2 포인트 클라우드 누적
    stage1_nodes.append(Node(
        package='scan_to_pointcloud',
        executable='accumulated_pointcloud',
        name='accumulated_pointcloud',
        parameters=[{
            'use_tf': True,
            'max_points': 50000,
            'grid_size': 0.01,
            'publish_rate': 5.0,
            'use_imu_tf_only': True,  # IMU TF만 사용
            'disable_position_tracking': True,  # 위치 추적 비활성화
        }]
    ))
    
    # 3단계: Cartographer 및 관련 노드
    # - 누적된 포인트클라우드가 준비된 후에 Cartographer 노드를 시작합니다.
    stage2_nodes = []
    
    # 3.1 Cartographer SLAM 노드
    stage2_nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_3d_solution.lua'
        ],
        remappings=[
            ('points2', 'accumulated_points'),  # 누적 포인트 클라우드 사용
            ('imu', 'imu')
        ]
    ))
    
    # 3.2 점유 그리드 노드
    stage2_nodes.append(Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': False, 'resolution': 0.05}]
    ))
    
    # 3.3 RViz 시각화
    rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')
    if not os.path.exists(rviz_config):
        # 기본 설정 사용
        rviz_config = os.path.join(
            get_package_share_directory('cartographer_ros'),
            'configuration_files', 
            'demo_3d.rviz'
        )
        
    stage2_nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    ))
    
    # 4단계: IMU TF 발행 노드 (Cartographer 초기화 이후에 시작)
    # - Cartographer가 초기화된 후에 시작하여 TF 충돌 방지
    stage3_nodes = []
    
    stage3_nodes.append(Node(
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
            'invert_yaw': False,
        }]
    ))
    
    # 단계별 시작 타이머 추가
    nodes.append(TimerAction(period=2.0, actions=stage1_nodes))  # 2초 후 포인트클라우드 처리
    nodes.append(TimerAction(period=5.0, actions=stage2_nodes))  # 5초 후 Cartographer
    nodes.append(TimerAction(period=8.0, actions=stage3_nodes))  # 8초 후 IMU TF 발행
    
    return LaunchDescription(nodes)