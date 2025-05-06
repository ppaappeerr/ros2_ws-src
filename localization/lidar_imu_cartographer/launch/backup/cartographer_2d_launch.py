#### python
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_2d_launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'config', 'cartographer_2d.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        # 2D Cartographer 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node_2d',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu'),
                ('map', 'map')  # map 토픽 그대로 사용
            ]
        ),
        # Occupancy Grid
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node_2d',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': 0.05,
            }],
            remappings=[
                ('map', 'map')
            ]
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_2d',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )
    ])