import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 경로
    pkg_share = FindPackageShare('lidar_imu_cartographer')
    cartographer_pkg_share = FindPackageShare('cartographer_ros')

    # 설정 파일 경로
    cartographer_config_dir = PathJoinSubstitution([pkg_share, 'config'])
    cartographer_config_basename = 'cartographer_2d_scan_imu.lua' # 2D 설정 파일

    # RViz 설정 파일 경로
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'cartographer_2d.rviz']) # 2D RViz 설정

    # --- Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_complementary_filter_arg = DeclareLaunchArgument(
        'use_complementary_filter', default_value='true', # 상보 필터 사용 여부
        description='Use complementary filter in mpu6050_node'
    )
    # --------------------------

    # 사용할 IMU 토픽 결정
    imu_topic = PythonExpression([
        "'/imu' if '", LaunchConfiguration('use_complementary_filter'), "' == 'true' else '/imu_raw'"
    ])

    return LaunchDescription([
        use_sim_time_arg,
        use_complementary_filter_arg,

        LogInfo(msg=["Using Lua configuration: ", cartographer_config_basename]),
        LogInfo(msg=["Using IMU topic: ", imu_topic]),

        # --- TF 설정 (Static Transforms) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0.0', '0.0', '-0.019', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # --- 센서 노드 ---
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
        ),
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_rate': 50.0,
                'use_complementary_filter': LaunchConfiguration('use_complementary_filter'),
                'use_calibration': True,
            }],
            remappings=[
                ('imu_raw', '/imu_raw'),
                ('imu_filtered', '/imu')
            ],
            output='screen'
        ),

        # --- Cartographer 노드 (2D) ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename
            ],
            remappings=[
                ('scan', '/scan'), # LaserScan 사용
                ('imu', imu_topic) # 선택된 IMU 토픽
                # 'points2' 리매핑 제거
                # 'odom' 리매핑 제거
            ]
        ),

        # --- Occupancy Grid 노드 ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': 0.05
            }],
            remappings=[('map', '/map')]
        ),

        # --- RViz ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])