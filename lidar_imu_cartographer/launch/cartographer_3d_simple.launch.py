#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_simple.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    # Cartographer 노드
    cartographer_node = Node(
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
    
    # 점유 그리드 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False, 'resolution': 0.05}]
    )
    
    return LaunchDescription([
        LogInfo(msg="3D Cartographer와 점유 그리드 노드만 실행합니다."),
        cartographer_node,
        occupancy_grid_node
    ])