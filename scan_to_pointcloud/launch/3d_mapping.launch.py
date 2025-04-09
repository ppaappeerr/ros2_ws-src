import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 파라미터 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 패키지 경로
    scan_to_pointcloud_dir = get_package_share_directory('scan_to_pointcloud')
    
    # TF 설정 노드들
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
        arguments=['0', '0', '0.1', '0', '0', '0', '1', 'base_link', 'laser']
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
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate': 100.0,
            'gravity_compensation': True
        }]
    )
    
    # IMU TF 발행 노드
    imu_tf_publisher = Node(
        package='lidar_imu_fusion',
        executable='imu_tf_publisher',
        name='imu_tf_publisher',
        parameters=[{
            'frame_id': 'map',
            'child_frame_id': 'base_link',
            'use_imu_orientation': True,
            'position_drift_correction': 0.999,
            'velocity_drift_correction': 0.98
        }]
    )
    
    # 라이다 스캔 -> 포인트 클라우드 변환 노드
    scan_to_pointcloud_node = Node(
        package='scan_to_pointcloud',
        executable='scan_to_pointcloud',
        name='scan_to_pointcloud',
        parameters=[{
            'input_scan_topic': 'scan',
            'output_cloud_topic': 'points2',
            'laser_frame': 'laser',
            'min_height': -0.05,
            'max_height': 0.05,
            'height_increment': 0.01
        }]
    )
    
    # 포인트 클라우드 누적 노드
    pointcloud_accumulator_node = Node(
        package='scan_to_pointcloud',
        executable='pointcloud_accumulator',
        name='pointcloud_accumulator',
        parameters=[{
            'max_points': 100000,
            'grid_size': 0.05,
            'reset_time': 60.0,
            'global_frame': 'map',
            'input_cloud_topic': 'points2'
        }]
    )
    
    # RViz2 노드
    rviz_config_path = os.path.join(scan_to_pointcloud_dir, 'config', '3d_mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time'),
        static_tf_base_to_imu,
        static_tf_base_to_laser,
        sllidar_node,
        imu_node,
        imu_tf_publisher,
        scan_to_pointcloud_node,
        pointcloud_accumulator_node,
        rviz_node
    ])