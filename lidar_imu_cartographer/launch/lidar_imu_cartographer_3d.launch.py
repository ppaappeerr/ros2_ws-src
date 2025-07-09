#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')

    sensor_nodes = [
        Node(package='sllidar_ros2', executable='sllidar_node',
             name='sllidar_node',
             parameters=[{
                 'channel_type': 'serial',
                 'serial_port': '/dev/ttyUSB0',
                 'serial_baudrate': 115200,
                 'frame_id': 'laser',
                 'angle_compensate': True
             }]),
        Node(package='mpu6050_py', executable='mpu6050_node',
             name='mpu6050_node',
             parameters=[{
                 'frame_id': 'imu_link',
                 'publish_rate': 50.0
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_world_to_map',
             arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_laser',
             arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_base_to_imu',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']),
        Node(package='lidar_imu_fusion', executable='imu_tf_publisher',
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
             }]),
    ]

    delayed_nodes = [
        TimerAction(period=1.0, actions=[
            Node(package='scan_to_pointcloud', executable='scan_to_pointcloud',
                 name='scan_to_pointcloud',
                 parameters=[{'input_topic': 'scan', 'output_topic': 'pc_3d'}])
        ]),
        TimerAction(period=2.0, actions=[
            Node(package='scan_to_pointcloud', executable='accumulated_pointcloud',
                 name='accumulated_pointcloud',
                 parameters=[{
                     'use_tf': True,
                     'max_points': 50000,
                     'publish_rate': 10.0,
                     'use_imu_tf_only': True,
                     'disable_position_tracking': True
                 }])
        ]),
        TimerAction(period=5.0, actions=[
            Node(package='cartographer_ros', executable='cartographer_node',
                 name='cartographer_node',
                 parameters=[{'use_sim_time': False}],
                 arguments=['-configuration_directory', config_dir,
                            '-configuration_basename', 'cartographer_3d.lua'],
                 remappings=[('points2', 'accumulated_points')]),
            Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                 name='occupancy_grid_node',
                 parameters=[{'use_sim_time': False, 'resolution': 0.05}]),
            Node(package='rviz2', executable='rviz2',
                 name='rviz2',
                 arguments=['-d', rviz_config])
        ])
    ]

    return LaunchDescription([
        LogInfo(msg="3D SLAM 시작"),
        *sensor_nodes,
        *delayed_nodes
    ])