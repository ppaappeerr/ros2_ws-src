from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package='sllidar_ros2', executable='sllidar_node', name='lidar'),
        Node(package='mpu9250_driver', executable='mpu9250_filtered', name='imu'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.71','0','0','0','base_link','laser'])
    ])
