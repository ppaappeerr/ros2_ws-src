import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # SLLIDAR 설정
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')

    # 보정 파일 경로
    calib_path = os.path.expanduser('~/ros2_ws/src/calib/mpu9250_calib.json')

    # 사용할 패키지들의 공유 디렉토리 경로 정의
    fast_lio_pkg_dir = get_package_share_directory('fast_lio')
    sllidar_ros2_pkg_dir = get_package_share_directory('sllidar_ros2')
    sweep_builder_pkg_dir = get_package_share_directory('sweep_builder')

    # FAST-LIO 설정 파일 경로
    fast_lio_params_file = os.path.join(fast_lio_pkg_dir, 'config', 'my_wearable_lio.yaml')

    # TF 설정 (a2m8 런치파일과 동일하게)
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
        arguments=['0.0', '0.0', '0.071', '0', '0.0', '0.0', 'imu_link', 'laser'],
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

    # MPU9250 IMU 노드 (올바른 패키지 이름과 실행파일)
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

    # 포인트 클라우드 전처리기 노드
    pointcloud_node = Node(
        package='sweep_builder',
        executable='lidar_imu_filtered_to_pointcloud',
        name='lidar_imu_to_pointcloud',
        output='screen'
    )

    # FAST-LIO 노드 (실제 실행파일 이름으로 수정)
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',  # 또는 실제 확인된 실행파일 이름
        name='fast_lio_node',
        parameters=[fast_lio_params_file],
        output='screen'
    )

    # RViz 설정
    rviz_config_dir = os.path.join(
        sllidar_ros2_pkg_dir,
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

        static_tf_base_to_imu,
        static_tf_imu_to_laser,
        sllidar_node,
        imu_node,
        pointcloud_node,
        fast_lio_node,
        rviz_node
    ])