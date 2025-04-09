#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ###################################
    # 1) Static TF: base_link -> laser
    ###################################
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        output='screen',
        arguments=[
            '0','0','0',   # x,y,z
            '0','0','0','1',  # qx,qy,qz,qw
            'base_link',
            'laser'
        ]
    )

    ###################################
    # 2) cartographer_node (SLAM 본체)
    ###################################
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        arguments=[
            # cartographer config 경로
            '-configuration_directory', '/home/p/ros2_ws/src/cartographer_ros/cartographer_ros/configuration_files',
            '-configuration_basename', 'rplidar_a2m8.lua'
        ],
        remappings=[
            # 라이다 스캔 토픽이 /scan 이라면 그대로.
            # 만약 /rplidar/scan 이면 ('/scan', '/rplidar/scan') 식으로 수정
            ('scan', 'scan'),
        ],
    )

    ###################################
    # 3) occupancy_grid_node (map 퍼블리시)
    ###################################
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            # occupancy_grid_node 파라미터
            'resolution': 0.05,         # grid 해상도(m 단위)
            'publish_period_sec': 1.0,  # /map 퍼블리시 주기
            'use_sim_time': False
        }],
        remappings=[
            # Cartographer 내부 submap → /map 으로 만듦
            ('map', 'map'),
            ('map_metadata', 'map_metadata'),
        ]
    )

    return LaunchDescription([
        static_tf_node,
        cartographer_node,
        occupancy_grid_node
    ])

