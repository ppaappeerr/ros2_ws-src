from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='zebedee_lite', executable='imu_preprocess_node', name='imu_pre'),
        Node(package='zebedee_lite', executable='deskew_scan_node',  name='deskew_cloud'),
        Node(package='zebedee_lite', executable='icp_odometry_node', name='icp_odom'),
        Node(package='zebedee_lite', executable='odom_tf_broadcaster', name='odom_tf'),
        # Node(package='robot_localization', executable='ekf_node', name='ekf', parameters=['config/ekf_lidar_imu.yaml']),
    ])
