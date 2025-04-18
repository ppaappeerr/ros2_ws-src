#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_debug.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    config_dir = os.path.join(pkg_share, 'config')
    
    return LaunchDescription([
        # 카르토그래퍼 노드만 실행하여 디버깅
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_3d.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),
                ('imu', 'imu')
            ],
            prefix=['xterm -e gdb -ex run --args'],  # 디버깅용 GDB 실행
        ),
        # 로그 확인용 명령
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/rosout', '--field', 'level', '--field', 'name', '--once', '--no-daemon'],
            output='screen'
        )
    ])
