import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLLIDAR C1 설정
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')  # C1은 115200 baud
    frame_id = LaunchConfiguration('frame_id', default='laser')
    use_rviz = LaunchConfiguration('use_rviz', default='false')  # RViz 실행 여부 인자 추가

    calib_path = os.path.expanduser('~/ros2_ws/src/calib/mpu9250_calib.json')

    # IMU는 LiDAR 위 2cm에 위치 (0.02m) - 현재 하드웨어 구성
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )

    static_tf_laser_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_to_imu',
        arguments=['0.0', '0.0', '0.02', '0', '0.0', '0.0', 'laser', 'imu_link'],
        output='screen'
    )

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
            'scan_mode': 'Sensitivity'  # C1용 향상된 감도 스캔 모드
        }],
        output='screen'
    )

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

    source_rviz_path = '/home/p/ros2_ws/src/optical_cane_rpi/rviz/optical_cane.rviz'
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

    # LaunchDescription에 RViz2 노드 조건부 추가
    launch_nodes = [
        DeclareLaunchArgument('channel_type', default_value='serial'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        static_tf_base_to_laser,
        static_tf_laser_to_imu,
        sllidar_node,
        imu_node
    ]

    from launch.conditions import IfCondition
    from launch.actions import GroupAction

    # use_rviz이 true일 때만 rviz_node 추가
    launch_nodes.append(
        GroupAction([rviz_node], condition=IfCondition(use_rviz))
    )

    return LaunchDescription(launch_nodes)
