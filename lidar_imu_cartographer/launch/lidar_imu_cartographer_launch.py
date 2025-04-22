#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # 파라미터 가져오기
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 기본 노드 목록 (TF 및 센서)
    nodes = [
        LogInfo(msg=f"모드: {'3D' if slam_mode.lower() == '3d' else '2D'} SLAM"),
        
        # 1. SLLiDAR 노드
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
        ),
        
        # 2. MPU6050 IMU 노드
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{'frame_id': 'imu_link'}]
        ),
        
        # 3. 기본 TF 설정 (고정 변환)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link']
        ),
        
        # 4. IMU TF 발행 노드 - 중요: base_link 이동/회전 제공
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            parameters=[{
                'parent_frame': 'map',
                'child_frame': 'base_link',
                'publish_rate': 20.0,
                'stabilization_time': 2.0,
                'maintain_orientation': False,  # 중요: 오뚝이 효과 없애기
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False,
            }]
        )
    ]
    
    # 모드별 지연 실행 노드 (TF 설정 후 시작)
    delayed_nodes = []
    
    # 3D 모드 노드
    if slam_mode.lower() == '3d':
        # 포인트 클라우드 변환
        delayed_nodes.append(
            Node(
                package='scan_to_pointcloud',
                executable='scan_to_pointcloud',
                name='scan_to_pointcloud',
                parameters=[{
                    'input_topic': 'scan',
                    'output_topic': 'pc_3d',
                    'frame_id': 'laser',
                    'use_tf': False,  # TF 변환 사용 안 함 (제자리 변환)
                    'use_height_coloring': True
                }]
            )
        )
        
        # 포인트 클라우드 누적
        delayed_nodes.append(
            Node(
                package='scan_to_pointcloud',
                executable='accumulated_pointcloud',
                name='accumulated_pointcloud',
                parameters=[{
                    'use_tf': True,  # TF 변환 사용
                    'max_points': 100000,  # 최대 포인트 수
                    'grid_size': 0.01,  # 그리드 크기
                    'publish_rate': 10.0,  # 발행 주기
                    'use_imu_tf_only': True,  # IMU TF만 사용
                    'disable_position_tracking': True  # 위치 추적 사용 안 함
                }]
            )
        )
        
        # Cartographer 3D 설정
        config_basename = 'cartographer_3d.lua'
        input_topic = 'accumulated_points'
    else:
        # 2D 모드
        config_basename = 'cartographer_2d.lua'
        input_topic = 'scan'
    
    # Cartographer 노드 (2D/3D 공통)
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename
            ],
            remappings=[
                ('points2', input_topic),  # input_topic에 따라 자동 설정
                ('scan', 'scan'),
                ('imu', 'imu')
            ]
        )
    )
    
    # 점유 그리드 노드 (2D/3D 공통)
    delayed_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # RViz 설정 (모드별)
    if use_rviz:
        rviz_config = os.path.join(
            pkg_share, 
            'rviz', 
            'cartographer_3d.rviz' if slam_mode.lower() == '3d' else 'cartographer_2d.rviz'
        )
        
        # RViz 설정 파일이 없으면 기본 설정 사용
        if not os.path.exists(rviz_config):
            cartographer_share = get_package_share_directory('cartographer_ros')
            rviz_config = os.path.join(
                cartographer_share, 
                'configuration_files',
                'demo_3d.rviz' if slam_mode.lower() == '3d' else 'demo_2d.rviz'
            )
        
        delayed_nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
        )
    
    # 지연 실행 (5초 대기 후 시작)
    nodes.append(
        TimerAction(
            period=5.0,  # 5초 지연으로 충분한 데이터 축적 기다림
            actions=delayed_nodes
        )
    )
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # 파라미터 선언
        DeclareLaunchArgument(
            'slam_mode',
            default_value='2d',
            description='SLAM 모드 (2d 또는 3d)'
        ),
        DeclareLaunchArgument(
            'use_rviz', 
            default_value='true',
            description='RViz 실행 여부'
        ),
        # 런치 설정 실행
        OpaqueFunction(function=launch_setup)
    ])