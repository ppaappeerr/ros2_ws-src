from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TF 정의: base_link -> imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', '1', 'base_link', 'imu_link'],
        output='screen'
    )

    # TF 정의: base_link -> laser
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
        output='screen'
    )

    # world -> map 변환 추가
    static_tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'map'],
        output='screen'
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
            'angle_compensate': True,
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
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        static_tf_world_to_map,
        static_tf_base_to_laser,
        static_tf_base_to_imu,
        sllidar_node,
        imu_node,
        rviz_node
    ])