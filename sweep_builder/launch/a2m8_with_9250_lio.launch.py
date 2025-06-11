from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sllidar_ros2', executable='sllidar_node', output='screen'),
        Node(package='mpu9250_py', executable='mpu9250_node', output='screen'),
        Node(package='sweep_builder', executable='lidar_imu_to_pointcloud.py', output='screen'),
        Node(package='sweep_builder', executable='icp_3d_odom.py', output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=['config/ekf_params.yaml'], output='screen')
    ])
