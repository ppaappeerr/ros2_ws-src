# slam_online.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    slam_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/slam_online.yaml')
    ekf_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/ekf_local.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_yaml, {
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan_leveled',
                'autostart': True,  # 이거 안 해주면 아무것도 안함
                'publish_tf': True,
                'resolution': 0.05
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=[ekf_yaml]
        )
    ])