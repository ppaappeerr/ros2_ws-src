#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    
    # 설정 파일 경로
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    
    # 파라미터 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    nodes = []
    
    # 1. 센서 준비 노드들 (TF 설정 노드는 먼저 시작)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
            output='screen'
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
            output='screen'
        )
    )
    
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link'],
            output='screen'
        )
    )
    
    # 2. 센서 노드들 (LiDAR + IMU)
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
            }],
            output='screen'
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
            }],
            output='screen'
        )
    )
    
    # 3. IMU TF 발행자 (베이스 요소)
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
            }],
            output='screen'
        )
    )
    
    # 4. 나머지 노드들은 타이머로 지연 시작 (TF 설정 완료 후)
    delayed_nodes = []
    
    # 4.1 스캔 -> 포인트클라우드 변환
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_height_coloring': True,
                'output_frame': 'laser',
                'use_tf': False
            }],
            output='screen'
        )
    )
    
    # 4.2 포인트클라우드 누적
    delayed_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 100000,
                'grid_size': 0.01,
                'publish_rate': 5.0,
                'point_skip': 2,
                'use_imu_tf_only': True,
                'disable_position_tracking': True,
            }],
            output='screen'
        )
    )
    
    # 4.3 카르토그래퍼 노드
    # 중요: 여기서 맵 프레임과 포인트클라우드 토픽 설정을 정확히 지정
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', 'cartographer_3d_fixed.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),
                ('imu', 'imu'),
                ('odom', 'odom')
            ]
        )
    )
    
    # 4.4 점유 그리드 노드
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
            }]
        )
    )
    
    # 4.5 RViz 노드
    delayed_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'cartographer_3d_fixed.rviz')],
            output='screen'
        )
    )
    
    # 타이머로 지연 시작
    nodes.append(
        TimerAction(
            period=3.0,  # 3초 후 시작 (안정적인 TF 설정을 위해)
            actions=delayed_nodes
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='시뮬레이션 시간 사용 여부'),
        *nodes
    ])