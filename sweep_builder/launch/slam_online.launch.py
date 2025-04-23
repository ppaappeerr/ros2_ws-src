# slam_online.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    slam_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/slam_online.yaml')
    ekf_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/ekf_local.yaml')

    return LaunchDescription([

        # ⚙️ 1. Leveled Scan Node (보정된 /scan_leveled 발행)
        Node(
            package='sweep_builder',
            executable='leveled_scan_node',
            name='leveled_scan_node',
            output='screen',
            parameters=[{
                'in_scan': '/scan',
                'out_scan': '/scan_leveled'
            }]
        ),

        # 🧭 2. EKF 필터 노드
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf_yaml]
        ),

        # 🗺️ 3. slam_toolbox (1초 딜레이)
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_yaml]
                )
            ]
        ),

        # 🔄 4. lifecycle_manager (slam_toolbox 자동 활성화)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 5.0,
                'node_names': ['slam_toolbox']
            }]
        )
    ])
