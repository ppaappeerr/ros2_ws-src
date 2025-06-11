from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLLIDAR 설정 (기존 동일)
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    # base_link → laser (정확한 장착 위치로 수정 필요)
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )

    # base_link → imu_link (정확한 장착 위치로 수정 필요)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0.0', '0.0', '-0.071', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
        output='screen'
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
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    # MPU9250 IMU 노드 (mpu9250)
    imu_node = Node(
        package='mpu9250',
        executable='mpu9250',   # 실제 실행 파일명(스크립트명) 확인
        name='mpu9250_node',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_rate': 100.0,
            'use_calibration': True,
            'use_complementary_filter': True,
            'complementary_alpha': 0.98
        }],
        output='screen'
    )

    # RViz 설정 (기존 rviz 파일 경로 확인)
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'rviz',
        'sllidar_with_imu.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('channel_type', default_value='serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),

        static_tf_base_to_laser,
        static_tf_base_to_imu,
        sllidar_node,
        imu_node,
        rviz_node
    ])
