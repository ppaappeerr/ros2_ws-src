#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_complete.launch.py
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
    
    # 0. 장치 권한 부여 (중요)
    nodes.append(
        ExecuteProcess(
            cmd=['sudo', 'chmod', 'a+rw', '/dev/ttyUSB0'],
            output='screen'
        )
    )
    
    # 1. 중요한 TF 트리 설정 (최우선 실행)
    # world -> map -> odom -> base_link -> [laser, imu_link]
    nodes.append(LogInfo(msg="Cartographer 3D SLAM 실행 중..."))
    
    # world -> map
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    )
    
    # map -> odom (중요: Cartographer가 이 프레임에 대해 추론)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    )
    
    # odom -> base_link (중요: 초기 설정, 나중에 동적으로 변함)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
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
    # Cartographer 3D 요구사항: IMU 프레임은 tracking_frame과 정확히 같은 위치에 있어야 함
    # 따라서 여기서는 오프셋을 0으로 설정
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 2. 센서 노드들 (SLLiDAR + MPU6050)
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
                'scan_mode': 'Sensitivity'
            }]
        )
    )
    
    nodes.append(
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'frame_id': 'imu_link',  # imu_link로 설정 (tracking_frame과 일치)
                'publish_rate': 100.0,    # 높은 발행 빈도
                'use_calibration': True   # 보정 사용
            }]
        )
    )
    
    # 포인트 클라우드 변환 및 누적 노드 (1초 지연)
    pc_nodes = []
    
    # 3. 2D 스캔을 3D 포인트 클라우드로 변환 (개선된 버전)
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_tf': False,  # TF 사용 안 함 (변환 단순화)
                'z_max_variation': 0.2,   # Z축 변화량
                'pattern_scale': 5.0,     # 패턴 스케일
                'pattern_type': 'sine'    # 3D 패턴 타입
            }]
        )
    )
    
    # 4. 포인트 클라우드 누적 노드 (3D 매핑에 중요)
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 80000,       # 충분한 포인트 수
                'grid_size': 0.05,         # 적절한 그리드 크기
                'publish_rate': 10.0,      # 발행 빈도
                'use_imu_tf_only': False,  # IMU와 Cartographer TF 모두 활용
                'preserve_height_info': True  # 높이 정보 보존 (중요)
            }]
        )
    )
    
    # Cartographer 및 관련 노드 (3초 지연)
    carto_nodes = []
    
    # 5. 카르토그래퍼 노드 (3D SLAM 수행)
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_3d_final.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),  # 누적 포인트클라우드 사용
                ('imu', 'imu')  # IMU 토픽
            ]
        )
    )
    
    # 6. 점유 그리드 노드 (2D 맵 생성)
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # 7. RViz 시각화
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    rviz_config = os.path.join(cartographer_ros_dir, 'configuration_files', 'demo_3d.rviz')
    
    carto_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    )
    
    # 단계별 실행 (안정적인 초기화를 위해)
    nodes.append(TimerAction(period=1.0, actions=pc_nodes))   # 1초 후 포인트 클라우드 노드 실행
    nodes.append(TimerAction(period=3.0, actions=carto_nodes)) # 3초 후 Cartographer 노드 실행
    
    return LaunchDescription(nodes)