from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('lidar_icp_odometry')
    return LaunchDescription([
        # 1. Z-clip + Voxel
        Node(
            package='lidar_icp_odometry',
            executable='voxel_z_clip',
            name='voxel_z_clip',
            parameters=[os.path.join(pkg, 'config', 'voxel.yaml')],
            output='screen'),

        # 2. ICP Odom
        Node(
            package='lidar_icp_odometry',
            executable='icp_odom_node',
            name='icp_odom_node',
            output='screen'),

        # 3. EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')],
            output='screen'),
    ])
