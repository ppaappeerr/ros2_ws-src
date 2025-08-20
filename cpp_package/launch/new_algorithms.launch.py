#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for testing new path planning algorithms:
    - Pipeline P4: HeightMap 2.5D Planner (음의 장애물 감지)
    - Pipeline P5: Follow-the-Gap 3D
    """
    
    # Launch arguments
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='heightmap',
        choices=['heightmap', 'ftg3d', 'both'],
        description='Which pipeline to launch: heightmap, ftg3d, or both'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    # Get launch configurations
    pipeline = LaunchConfiguration('pipeline')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Common preprocessing nodes (shared by both pipelines)
    sensor_fusion_node = Node(
        package='optical_cane_rpi',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        output='screen'
    )
    
    point_cloud_sweeper_node = Node(
        package='cpp_package',
        executable='point_cloud_sweeper_cpp_node',
        name='point_cloud_sweeper_cpp_node',
        output='screen'
    )
    
    voxel_filter_node = Node(
        package='cpp_package',
        executable='voxel_grid_filter_node',
        name='voxel_grid_filter_node',
        parameters=[{
            'front_view_only': True,
            'leaf_size': 0.04
        }],
        output='screen'
    )
    
    # HeightMap 2.5D Planner
    heightmap_planner_node = Node(
        package='cpp_package',
        executable='heightmap_planner_node',
        name='heightmap_planner_node',
        parameters=[{
            'front_view_only': True,
            'grid_resolution': 0.1,
            'max_range': 5.0,
            'ground_height_tolerance': 0.15,
            'drop_threshold': 0.3,
            'obstacle_height_threshold': 0.2
        }],
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('pipeline').matches('heightmap').or_(
                LaunchConfiguration('pipeline').matches('both')
            )
        )
    )
    
    # Follow-the-Gap 3D Planner  
    ftg_3d_planner_node = Node(
        package='cpp_package',
        executable='ftg_3d_node', 
        name='ftg_3d_node',
        parameters=[{
            'front_view_only': True,
            'corridor_width': 0.4,
            'max_range': 5.0,
            'min_gap_width': 15.0,
            'safety_margin': 0.1,
            'num_sectors': 36
        }],
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('pipeline').matches('ftg3d').or_(
                LaunchConfiguration('pipeline').matches('both')
            )
        )
    )
    
    # RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory('optical_cane_rpi'),
        'rviz',
        'new_algorithms.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        pipeline_arg,
        use_rviz_arg,
        
        # Common preprocessing pipeline
        GroupAction([
            sensor_fusion_node,
            point_cloud_sweeper_node,
            voxel_filter_node
        ]),
        
        # Algorithm-specific nodes
        heightmap_planner_node,
        ftg_3d_planner_node,
        
        # Visualization
        rviz_node
    ])
