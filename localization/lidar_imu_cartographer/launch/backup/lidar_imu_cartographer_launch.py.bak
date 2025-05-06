#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/lidar_imu_cartographer_launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    """런치 설정 함수"""
    # 파라미터 가져오기
    slam_mode = LaunchConfiguration('slam_mode').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    
    # 패키지 경로
    carto_pkg_dir = get_package_share_directory('cartographer_ros')
    lidar_imu_carto_dir = get_package_share_directory('lidar_imu_cartographer')

    # 노드 목록
    nodes = []

    # 1단계: 센서 (LiDAR + IMU) 실행
    # combined_sensors_launch.py에서는 world->map, base_link->laser, base_link->imu_link TF 발행
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
    
    # 기본 TF 설정
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

    # 2단계: imu_tf_publisher 실행 - map→base_link TF 발행 (IMU 기반)
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

    # 지연 후 실행할 노드들 (TF 설정 완료 후)
    additional_nodes = []
    
    # 3D 모드인 경우 포인트클라우드 변환 추가
    if slam_mode.lower() == '3d':
        # 3단계: scan_to_pointcloud 실행 (2D→3D 변환)
        scan_to_pointcloud_params = {
            'input_topic': 'scan',
            'output_topic': 'pc_3d',
            'frame_id': 'laser',
            'use_height_coloring': True,
            'output_frame': 'laser',  # laser 프레임 기준으로 고정
            'use_tf': False  # TF 변환 사용하지 않음
        }
        additional_nodes.append(
            Node(
                package='scan_to_pointcloud',
                executable='scan_to_pointcloud',
                name='scan_to_pointcloud',
                parameters=[scan_to_pointcloud_params],
                output='screen'
            )
        )

        # 4단계: accumulated_pointcloud 실행 (3D 포인트 누적)
        accumulated_pointcloud_params = {
            'use_tf': True,
            'max_points': 100000,
            'grid_size': 0.01,
            'publish_rate': 10.0,  # 더 빠른 발행
            'point_skip': 1,  # 더 많은 포인트 사용
            'use_imu_tf_only': True,
            'disable_position_tracking': True,
        }
        additional_nodes.append(
            Node(
                package='scan_to_pointcloud',
                executable='accumulated_pointcloud',
                name='accumulated_pointcloud',
                parameters=[accumulated_pointcloud_params],
                output='screen'
            )
        )
    
    # Cartographer 설정
    config_dir = os.path.join(lidar_imu_carto_dir, 'config')
    
    # 모드별 설정
    if slam_mode.lower() == '3d':
        # 3D 카트그래퍼 설정 개선
        config_basename = 'cartographer_3d_fixed.lua'  # 새로운 설정 파일 사용
        input_topic = 'accumulated_points'
        additional_nodes.append(LogInfo(msg="3D SLAM 모드로 실행 중 (입력: accumulated_points)"))
    else:
        # 2D 카트그래퍼 설정
        config_basename = 'cartographer_2d.lua'
        input_topic = 'scan'
        additional_nodes.append(LogInfo(msg="2D SLAM 모드로 실행 중 (입력: scan)"))
        
    # 카트그래퍼 노드 파라미터 개선
    cartographer_params = {
        'use_sim_time': False
    }
    
    if slam_mode.lower() == '3d':
        # 3D 모드에서 추가 파라미터
        cartographer_params.update({
            'publish_to_tf': False,  # TF 발행 비활성화
            'resolution': 0.05,
            'max_submaps_to_keep': 5
        })
    
    # 카트그래퍼 노드
    additional_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[cartographer_params],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename
            ],
            remappings=[
                ('points2', input_topic),
                ('scan', 'scan'),
                ('imu', 'imu')
            ]
        )
    )

    # 점유 그리드 노드
    additional_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05
            }],
            output='screen'
        )
    )
    
    # 지연 후 실행 (센서 및 TF 설정이 완료된 후)
    nodes.append(
        TimerAction(
            period=2.0,  # 2초 후 실행
            actions=additional_nodes
        )
    )
    
    # RViz 실행 (옵션)
    if use_rviz:
        # 모드별 RViz 설정 선택
        if slam_mode.lower() == '3d':
            rviz_config = os.path.join(lidar_imu_carto_dir, 'rviz', 'cartographer_3d.rviz')
            if not os.path.exists(rviz_config):
                rviz_config = ""  # 파일이 없으면 빈 문자열 (기본 설정 사용)
        else:
            rviz_config = os.path.join(lidar_imu_carto_dir, 'rviz', 'cartographer_2d.rviz')
            if not os.path.exists(rviz_config):
                rviz_config = ""  # 파일이 없으면 빈 문자열 (기본 설정 사용)
        
        # RViz 노드 추가
        rviz_args = ['-d', rviz_config] if rviz_config else []
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=rviz_args,
                output='screen'
            )
        )
        
    return nodes

def generate_launch_description():
    """런치 설명 생성"""
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