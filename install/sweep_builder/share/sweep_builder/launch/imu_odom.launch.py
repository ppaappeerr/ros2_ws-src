# imu_odom.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ekf_config = os.path.join(
        os.getcwd(), 'src/sweep_builder/config/ekf_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen'
        )
    ])
