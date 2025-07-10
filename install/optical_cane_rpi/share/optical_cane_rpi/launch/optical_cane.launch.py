import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLLIDAR 설정
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    # 보정 파일 경로 (동적으로 설정)
    calib_path = os.path.expanduser('~/ros2_ws/src/calib/mpu9250_calib.json')

    # TF 설정 - 기본 프레임 변환
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'imu_link'],
        output='screen'
    )

    static_tf_imu_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu_to_laser',
        arguments=['0.0', '0.0', '0.071', '0', '0.0', '0.0', 'base_link', 'laser'],
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

    # MPU9250 IMU 노드
    imu_node = Node(
        package='mpu9250',
        executable='mpu9250_filtered',
        name='mpu9250_filtered_node',
        parameters=[{
            'calibration_path': calib_path,
            'frame_id': 'imu_link',
            'publish_rate': 100.0
        }],
        output='screen'
    )

    # RViz 설정 경로 - 소스 디렉토리 우선 사용
    source_rviz_path = '/home/p/ros2_ws/src/optical_cane_rpi/rviz/optical_cane.rviz'
    
    # 소스 디렉토리에 파일이 있으면 그것을 사용, 없으면 install 디렉토리 사용
    if os.path.exists(source_rviz_path):
        rviz_config_dir = source_rviz_path
    else:
        rviz_config_dir = os.path.join(
            get_package_share_directory('optical_cane_rpi'),
            'rviz',
            'optical_cane.rviz')

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

        static_tf_base_to_imu,
        static_tf_imu_to_laser,
        sllidar_node,
        imu_node,
        rviz_node
    ])
