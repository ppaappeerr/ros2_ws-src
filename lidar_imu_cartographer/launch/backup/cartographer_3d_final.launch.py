#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로 설정
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    
    # 노드 목록
    nodes = []
    
    # 1. 디바이스 권한 설정 - 중요: 라이다 연결 문제 해결
    nodes.append(
        ExecuteProcess(
            cmd=['sudo', 'chmod', '666', '/dev/ttyUSB0'],
            output='screen'
        )
    )
    
    # 2. TF 트리 설정 - 중요: 프레임 간 관계 명확히 정의
    nodes.append(LogInfo(msg="Cartographer 3D 매핑 시작 - 수정된 TF 트리"))
    
    # world -> map (정적)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
        )
    )
    
    # map -> odom (필수: Cartographer가 이 프레임을 찾음)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    )
    
    # odom -> base_link (동적 TF를 초기에 설정)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        )
    )
    
    # base_link -> laser (라이다 위치)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        )
    )
    
    # base_link -> imu_link (중요: 둘을 완전히 같게 설정)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )
    )
    
    # 3. 센서 노드 - IMU 프레임을 base_link로 변경
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
                'frame_id': 'base_link', # 중요: IMU 프레임을 base_link로 출력
                'publish_rate': 50.0
            }]
        )
    )
    
    # 포인트클라우드 처리 노드 - 지연 시작
    pc_nodes = []
    
    # 4. 2D 스캔 → 3D 포인트클라우드 변환
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_tf': False
            }]
        )
    )
    
    # 5. 3D 포인트클라우드 누적기
    pc_nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{
                'use_tf': True,
                'max_points': 20000,  # 적은 수의 포인트로 시작
                'grid_size': 0.05,    # 해상도 조정
                'publish_rate': 5.0   # 발행 속도 조정
            }]
        )
    )
    
    # Cartographer 노드 - 마지막 시작
    carto_nodes = []
    
    # 설정 파일 경로 - 기본 예제 파일 사용
    cartographer_ros_dir = get_package_share_directory('cartographer_ros')
    config_file = os.path.join(cartographer_ros_dir, 'configuration_files', 'backpack_3d.lua')
    
    # 6. Cartographer 노드
    carto_nodes.append(
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
    
    # 7. 점유 그리드 노드
    carto_nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}]
        )
    )
    
    # 8. RViz 노드 추가
    carto_nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(cartographer_ros_dir, 'configuration_files', 'demo_3d.rviz')]
        )
    )
    
    # 9. 순차적 실행 설정
    nodes.append(TimerAction(period=2.0, actions=pc_nodes))     # 2초 후 포인트클라우드 처리 시작
    nodes.append(TimerAction(period=5.0, actions=carto_nodes))  # 5초 후 Cartographer 시작
    
    return LaunchDescription(nodes)