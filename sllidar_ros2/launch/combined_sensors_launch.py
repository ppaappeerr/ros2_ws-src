# combined_sensors_launch.py 파일 생성
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLLIDAR 설정
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # TF 정의: base_link -> imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link']
    )

    # TF 정의: base_link -> laser
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
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen'
    )

    # MPU6050 IMU 노드
    imu_node = Node(
        package='mpu6050_py',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{
            'frame_id': 'imu_link'  # IMU frame_id를 imu_link로 설정
        }],
        output='screen'
    )

    # RViz2 실행
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_with_imu.rviz')  # 새 RViz 설정 파일
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        static_tf_base_to_imu,
        static_tf_base_to_laser,
        sllidar_node,
        imu_node,
        rviz_node
    ])