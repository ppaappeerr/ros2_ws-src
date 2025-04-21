#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_fixed.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # 센서 및 고정 TF 노드 (고유 이름 추가)
    sensor_nodes = [
        # 라이다 설정 명시 추가
        Node(package='sllidar_ros2', executable='sllidar_node', 
             name='sllidar_node_for_3d',  # 고유 이름 추가
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': '/dev/ttyUSB0',
                 'serial_baudrate': 115200,
                 'frame_id': 'laser',
                 'angle_compensate': True
             }]),
             
        # IMU 노드 (방향 계산 활성화)
        Node(package='mpu6050_py', executable='mpu6050_node', 
             name='mpu6050_node_with_orientation',  # 고유 이름 추가
             parameters=[{
                 'frame_id': 'imu_link',
                 'compute_orientation': True,  # 방향 계산 활성화!
                 'orientation_gain': 0.05,     # 필터 게인
                 'gravity_filter': 0.01        # 중력 필터
             }]),
        
        # 고정 TF 설정 (고유 이름 필수!)
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_world_to_map',  # 고유 이름!
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_laser',  # 고유 이름!
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_imu',  # 고유 이름!
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']),
             
        # IMU TF 발행자 (map->base_link 관계 유지)
        Node(package='lidar_imu_fusion', executable='imu_tf_publisher', 
             name='imu_tf_publisher_for_3d',  # 고유 이름 추가
             parameters=[{
                 'parent_frame': 'map',
                 'child_frame': 'base_link',
                 'publish_rate': 20.0,  # 적절한 빈도
                 'maintain_orientation': False,  # 오뚝이 효과 제거
                 'invert_roll': False,
                 'invert_pitch': False, 
                 'invert_yaw': False
             }]),
    ]
    
    # 지연 시작 노드들 (센서 안정화 후)
    delayed_nodes = [
        # 포인트 클라우드 변환 (1초 후 시작)
        TimerAction(period=1.0, actions=[
            Node(package='scan_to_pointcloud', executable='scan_to_pointcloud',
                 name='scan_to_pointcloud_for_3d',  # 고유 이름 추가
                 parameters=[{'input_topic': 'scan', 'output_topic': 'pc_3d'}])
        ]),
        
        # 포인트 클라우드 누적 (2초 후 시작)
        TimerAction(period=2.0, actions=[
            Node(package='scan_to_pointcloud', executable='accumulated_pointcloud',
                 name='accumulated_pointcloud_for_3d',  # 고유 이름 추가
                 parameters=[{
                     'use_tf': True, 
                     'max_points': 50000, 
                     'publish_rate': 10.0,  # 발행 빈도 증가
                     'use_imu_tf_only': True,  # imu_tf_publisher만 사용
                     'disable_position_tracking': True  # 위치는 카르토그래퍼가 관리
                 }])
        ]),
        
        # 카르토그래퍼 노드 (5초 후 시작, 센서가 충분히 안정화된 후)
        TimerAction(period=5.0, actions=[
            Node(package='cartographer_ros', executable='cartographer_node',
                 name='cartographer_node_for_3d',  # 고유 이름 추가
                 parameters=[{'use_sim_time': False}],
                 arguments=['-configuration_directory', config_dir, 
                           '-configuration_basename', 'cartographer_3d.lua'],
                 remappings=[('points2', 'accumulated_points')]),
                 
            # 점유 그리드 노드
            Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                 name='occupancy_grid_node_for_3d',  # 고유 이름 추가
                 parameters=[{'use_sim_time': False, 'resolution': 0.05}]),
                 
            # RViz2 (시각화)
            Node(package='rviz2', executable='rviz2',
                 name='rviz2_for_3d_slam',  # 고유 이름 추가
                 arguments=['-d', '/opt/ros/jazzy/share/cartographer_ros/configuration_files/demo_3d.rviz'])
        ])
    ]
    
    return LaunchDescription([
        LogInfo(msg="3D 카르토그래퍼 SLAM 시작 - 방향 데이터 포함"),
        *sensor_nodes,  # 센서 노드 먼저 시작
        *delayed_nodes  # 지연 시작 노드들
    ])