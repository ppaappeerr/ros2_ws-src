# sllidar_a2m8_3d_cartographer.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Cartographer 설정
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files')
    configuration_basename = 'sllidar_a2m8_3d.lua'  # 3D 설정 파일
    
    # 기본 static TF 및 센서 노드들
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

    imu_node = Node(
        package='mpu6050_py',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{'frame_id': 'imu_link'}]
    )
    
    # 스캔을 포인트 클라우드로 변환하는 노드
    scan_to_pointcloud_node = Node(
        package='scan_to_pointcloud',  # 패키지 이름 확인 필요
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        output='screen'
    )
    
    # Cartographer 3D 노드
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=['-configuration_directory', cartographer_config_dir,
                  '-configuration_basename', configuration_basename],
        remappings=[('points2', 'points2')]  # 포인트 클라우드 토픽 remap
    )
    
    # 점유 그리드 노드
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False, 'resolution': 0.05}]
    )
    
    # RViz2 노드 (3D 시각화 설정)
    rviz_config_dir = os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files', 'demo_3d.rviz')
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
        scan_to_pointcloud_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])