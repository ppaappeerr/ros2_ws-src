import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    cartographer_share = get_package_share_directory('cartographer_ros')

    # 설정 파일 경로
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    # 3D Lua 파일 사용 확인
    cartographer_config_basename = 'cartographer_3d_scan_imu.lua'

    # RViz 설정 파일
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz') # 3D RViz 설정

    # 파라미터 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_complementary_filter = LaunchConfiguration('use_complementary_filter', default='true')

    # 사용할 IMU 토픽 결정
    imu_topic_for_cartographer = PythonExpression([
        "'/imu' if '", use_complementary_filter, "' == 'true' else '/imu_raw'"
    ])

    nodes = [
        LogInfo(msg=f"Cartographer 3D SLAM 시작 (LiDAR Scan + IMU) using {cartographer_config_basename}"),
        LogInfo(msg=PythonExpression([
            "'Using Complementary Filter: ' + ('true' if '", use_complementary_filter, "' == 'true' else 'false')"
        ])),
        LogInfo(msg=PythonExpression([
            "'IMU Topic for Cartographer: ' + ('/imu' if '", use_complementary_filter, "' == 'true' else '/imu_raw')"
        ])),
    ]

    # 1단계: TF 트리 설정 (static transforms)
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'], # 실제 로봇에 맞게 조정
            output='screen'
        )
    )

    # 위치 (Translation)
    imu_offset_x = 0.0
    imu_offset_y = 0.0
    imu_offset_z = -0.019

    # 방향 (Rotation) - X, Y, Z 축 일치하므로 모두 0
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0

    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                str(imu_offset_x), str(imu_offset_y), str(imu_offset_z),
                str(imu_roll), str(imu_pitch), str(imu_yaw),
                'base_link', 'imu_link'
            ],
            output='screen'
        )
    )

    # 2단계: 센서 노드 실행
    nodes.append(
        Node(
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
                'scan_mode': 'Sensitivity',
            }],
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_rate': 50.0,
                'use_calibration': True,
                'use_complementary_filter': use_complementary_filter,
                'complementary_alpha': 0.98
            }],
            remappings=[
                ('imu_raw', '/imu_raw'),
                ('imu_filtered', '/imu')
            ],
            output='screen'
        )
    )

    # 5단계: Cartographer 노드 (3D 설정 사용)
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', cartographer_config_basename],
            remappings=[
                ('scan', '/scan'), # LaserScan 토픽 사용
                ('imu', imu_topic_for_cartographer) # 선택된 IMU 토픽 사용
            ]
        )
    )

    # 6단계: Cartographer Occupancy Grid 노드
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'resolution': 0.05}],
            remappings=[('map', '/map')]
        )
    )

    # 7단계: RViz
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    )

    return LaunchDescription(nodes)