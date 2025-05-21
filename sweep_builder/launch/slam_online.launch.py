import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_config = os.path.join(
        '/home/p/ros2_ws/src/sweep_builder/config', 'slam_online.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # or 'async_slam_toolbox_node'
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        )
    ])
