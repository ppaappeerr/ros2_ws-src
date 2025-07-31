from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LiDAR + IMU 기존 드라이버
        Node(
            package='sllidar_ros2', 
            executable='sllidar_node', 
            name='lidar',
            output='screen'
        ),
        Node(
            package='mpu9250', 
            executable='mpu9250_filtered', 
            name='imu',
            output='screen'
        ),

        # 기존 sweep_builder (3D 포인트 생성)
        Node(
            package='sweep_builder',
            executable='sweep_builder_node',
            name='sweep_builder',
            output='screen'
        ),

        # ① 전방 필터
        Node(
            package='cpp_package', 
            executable='front_cloud_filter_node', 
            name='front_filter',
            parameters=[{
                'input_topic': '/sweep_cloud',
                'output_topic': '/front_cloud',
                'field_of_view_deg': 120.0,
                'max_distance': 3.0,
                'voxel_leaf_size': 0.05
            }],
            output='screen'
        ),

        # ② 장애물 평가
        Node(
            package='cpp_package', 
            executable='obstacle_evaluator_node', 
            name='obst_eval',
            parameters=[{
                'center_width_deg': 30.0,
                'danger_dist': 1.2,
                'warn_dist': 2.0
            }],
            output='screen'
        ),

        # ③ 의사결정
        Node(
            package='cpp_package', 
            executable='navigation_hint_node', 
            name='nav_hint',
            output='screen'
        ),

        # ④ 햅틱 드라이버 (C++ 버전)
        Node(
            package='cpp_package', 
            executable='haptic_driver_node',  # 🔥 .py 제거
            name='haptic',
            output='screen'
        ),
    ])
