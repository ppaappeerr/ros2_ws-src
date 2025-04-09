# sllidar_a2m8_cartographer.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Cartographer 설정
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files')
    configuration_basename = 'sllidar_a2m8_2d.lua'  # 위에서 생성한 설정 파일

    # TF 변환 설정
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser']
    )

    # SLLIDAR 노드
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True
        }]
    )

    # MPU6050 IMU 노드
    imu_node = Node(
        package='mpu6050_py',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{'frame_id': 'imu_link'}]
    )

    # Cartographer 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', cartographer_config_dir,
                  '-configuration_basename', configuration_basename]
    )

    # Occupancy grid 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'resolution': 0.05}]
    )

    # RViz2 노드
    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files', 'demo_2d.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        static_tf_base_to_imu,
        static_tf_base_to_laser,
        sllidar_node,
        imu_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])