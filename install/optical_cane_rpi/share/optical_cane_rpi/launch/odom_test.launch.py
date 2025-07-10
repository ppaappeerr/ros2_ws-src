import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_optical_cane_rpi = get_package_share_directory('optical_cane_rpi')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # EKF 설정 파일 경로
    ekf_config_path = os.path.join(pkg_optical_cane_rpi, 'config', 'ekf.yaml')

    # 기본 센서 launch 파일 포함 (LIDAR, IMU, static TFs)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_optical_cane_rpi, 'launch', 'optical_cane.launch.py')
        )
    )

    return LaunchDescription([
        sensors_launch,

        # 1. EKF 노드 실행
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')]
        ),
        
        # 2. 새로 만든 IMU Odometry 노드 실행
        Node(
            package='cpp_package',
            executable='imu_odometry_node',
            name='imu_odometry_node',
            output='screen'
        ),

        # 3. 포인트 클라우드 처리 노드들
        Node(
            package='optical_cane_rpi',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen'
        ),
        Node(
            package='cpp_package',
            executable='point_cloud_sweeper_cpp_node',
            name='point_cloud_sweeper_cpp_node',
            output='screen'
        ),
        Node(
            package='cpp_package',
            executable='height_ground_detector_node',
            name='height_ground_detector_node',
            output='screen'
        ),

        # 4. RViz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_optical_cane_rpi, 'rviz', 'odom_test.rviz')],
            output='screen'
        )
    ])