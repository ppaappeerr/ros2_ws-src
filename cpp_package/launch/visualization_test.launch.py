#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Package directory
    pkg_share = FindPackageShare('cpp_package')
    
    # RViz config path
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'launch', 'visualization_config.rviz'
    ])
    
    # LiDAR driver node (슬라이더 라이다)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 256000},
            {'frame_id': 'laser'},
            {'angle_compensate': True},
            {'scan_mode': 'Standard'},
            {'auto_standby': True},
        ],
        output='screen'
    )
    
    # HeightMap 2.5D planner
    heightmap_node = Node(
        package='cpp_package',
        executable='heightmap_planner_node',
        name='heightmap_planner_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('input_cloud', '/downsampled_cloud'),
            ('safe_path_vector', '/safe_path_vector_heightmap'),
        ],
        output='screen'
    )
    
    # FTG-3D planner
    ftg_3d_node = Node(
        package='cpp_package',
        executable='ftg_3d_node',
        name='ftg_3d_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'front_view_only': True},
            {'corridor_width': 0.4},  # 40cm corridor
            {'max_range': 3.0},       # 3m max range
            {'min_gap_width': 5.0},   # 5 degrees minimum gap
            {'safety_margin': 0.1},   # 10cm safety margin
            {'num_sectors': 36},      # 36 sectors (5-degree resolution)
        ],
        remappings=[
            ('input_cloud', '/downsampled_cloud'),
            ('safe_path_vector', '/safe_path_vector_ftg'),
        ],
        output='screen'
    )
    
    # Point cloud downsampler
    downsample_node = Node(
        package='pcl_ros',
        executable='voxel_grid',
        name='downsampler',
        parameters=[
            {'leaf_size': 0.05},  # 5cm voxel grid
        ],
        remappings=[
            ('input', '/scan_cloud'),  # From scan_to_cloud_node
            ('output', '/downsampled_cloud'),
        ],
        output='screen'
    )
    
    # LaserScan to PointCloud2 converter
    scan_to_cloud_node = Node(
        package='laser_geometry',
        executable='laser_scan_to_point_cloud_node',
        name='scan_to_cloud_converter',
        parameters=[
            {'target_frame': 'laser'},
            {'use_inf': True},
        ],
        remappings=[
            ('scan_in', '/scan'),
            ('cloud_out', '/scan_cloud'),
        ],
        output='screen'
    )
    
    # Transform publisher (static transform from base_link to laser)
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', '1', 'base_link', 'laser'],
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        
        # Core nodes
        lidar_node,
        tf_publisher,
        scan_to_cloud_node,
        downsample_node,
        
        # Algorithm nodes
        heightmap_node,
        ftg_3d_node,
        
        # Visualization
        rviz_node,
    ])
