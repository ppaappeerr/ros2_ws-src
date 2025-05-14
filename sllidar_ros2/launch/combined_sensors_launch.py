# combined_sensors_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # SLLIDAR 설정 (기존과 동일)
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') # 기본값 슬래시도 A2M8은 115200, A3은 256000
    frame_id = LaunchConfiguration('frame_id', default='laser') # sllidar_node가 발행하는 LaserScan의 frame_id
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity') # RPLIDAR A2M8의 일반적인 모드

    # TF 프레임 설정
    # world -> map (고정 TF)
    static_tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'], # x, y, z, yaw, pitch, roll, parent, child
        output='screen'
    )

    # map -> odom (고정 TF - SLAM 부재 시)
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], # x, y, z, yaw, pitch, roll, parent, child
        output='screen'
    )

    # base_link -> laser (센서 정적 TF)
    # 사용자의 실제 LiDAR 장착 위치에 맞게 x, y, z, roll, pitch, yaw 값을 수정해야 합니다.
    # 예: base_link에서 x축으로 0.1m 앞에, z축으로 0.05m 위에 장착된 경우
    # arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser']
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        # 아래 값은 예시이며, 실제 하드웨어 구성에 맞게 정확히 측정/설정해야 합니다.
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )

    # base_link -> imu_link (센서 정적 TF)
    # 사용자의 실제 IMU 장착 위치에 맞게 x, y, z, roll, pitch, yaw 값을 수정해야 합니다.
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        # 아래 값은 예시이며, 실제 하드웨어 구성에 맞게 정확히 측정/설정해야 합니다.
        arguments=['0.0', '0.0', '-0.02', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
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
            'frame_id': frame_id, # 'laser'로 설정됨
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode
        }],
        output='screen'
    )

    # MPU6050 IMU 노드
    imu_node = Node(
        package='mpu6050_py', # 패키지 이름이 맞는지 확인 필요
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{
            'frame_id': 'imu_link', # mpu6050_node가 발행하는 IMU 메시지의 frame_id
            'publish_rate': 100.0, # EKF 주파수(20Hz)보다 충분히 높게 설정
            'use_complementary_filter': True
        }],
        output='screen'
    )

    # RViz 설정 (기존과 동일)
    rviz_config_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'), # sllidar_ros2 패키지에 rviz 파일이 있다면
        'rviz',
        'sllidar_with_imu.rviz') # rviz 파일 이름

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value='serial',
            description='Specifies channel type of lidar'),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Specifies serial port of lidar'),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200', # RPLIDAR A2M8은 115200
            description='Specifies serial baudrate of lidar'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Specifies frame_id of lidar'),
        DeclareLaunchArgument(
            'inverted',
            default_value='false',
            description='Specifies whether to invert lidar data'),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value='true',
            description='Specifies whether to enable angle compensation for lidar'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value='Sensitivity',
            description='Specifies scan mode of lidar'),

        static_tf_world_to_map,
        static_tf_map_to_odom, # 추가된 부분
        static_tf_base_to_laser,
        static_tf_base_to_imu,
        sllidar_node,
        imu_node,
        rviz_node
    ])