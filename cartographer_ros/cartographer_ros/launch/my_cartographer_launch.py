from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # *** 올바른 경로로 수정 ***
    cartographer_config_dir = '/home/p/ros2_ws/src/cartographer_ros/cartographer_ros/configuration_files/'
    cartographer_config_file = 'my_2d_slam.lua'

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_file
            ],
            remappings=[('/scan', '/scan_flat')]  # 평면화된 스캔을 입력
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05']
        ),
    ])
