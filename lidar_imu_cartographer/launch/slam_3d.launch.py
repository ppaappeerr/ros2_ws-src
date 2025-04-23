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
    scan_to_pc_pkg_share = FindPackageShare('scan_to_pointcloud') # scan_to_pointcloud 패키지 경로 추가

    # 설정 파일 경로
    cartographer_config_dir = PathJoinSubstitution([pkg_share, 'config'])
    cartographer_config_basename = 'cartographer_3d_scan_imu.lua' # 사용할 Lua 파일

    # RViz 설정 파일 경로
    rviz_config_file = PathJoinSubstitution([pkg_share, 'rviz', 'cartographer_3d.rviz'])

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
            # 로봇 모델에 맞게 base_link와 laser 센서 간의 실제 오프셋 (x, y, z, yaw, pitch, roll) 입력
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            # 로봇 모델에 맞게 base_link와 imu_link 센서 간의 실제 오프셋 (x, y, z, yaw, pitch, roll) 입력
            # 예: arguments=['0.0', '0.0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
            arguments=['0.0', '0.0', '-0.019', '0.0', '0.0', '0.0', 'base_link', 'imu_link'], # 이전 값 유지
            output='screen'
        ),

        # --- 센서 노드 ---
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0', # 실제 연결된 포트 확인
                'serial_baudrate': 115200,     # SLLIDAR A1 기본값
                'frame_id': 'laser',           # static_tf_base_to_laser와 일치
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',    # 필요시 조정
            }],
            output='screen'
        ),
        Node(
            package='mpu6050_py',
            executable='mpu6050_node',
            name='mpu6050_node',
            parameters=[{
                'frame_id': 'imu_link',        # static_tf_base_to_imu와 일치
                'publish_rate': 50.0,          # Cartographer 권장 빈도
                'use_complementary_filter': LaunchConfiguration('use_complementary_filter'), # 상보 필터 사용
                'use_calibration': True,       # 보정 파일 사용 권장
                # 'calibration_path': '~/.mpu6050_calib.json' # 기본 경로 사용 시 생략 가능
            }],
            remappings=[ # 토픽 이름 표준화
                ('imu_raw', '/imu_raw'),
                ('imu_filtered', '/imu') # 상보 필터 사용 시 이 토픽이 /imu로 리매핑됨
            ],
            output='screen'
        ),

        # --- 데이터 변환 노드 (Scan -> PointCloud2) ---
        Node(
            package='scan_to_pointcloud', # 패키지 이름 확인 필요
            executable='scan_to_pointcloud', # 실행 파일 이름 확인 필요
            name='scan_to_pointcloud_node',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser', # 입력 스캔의 프레임 ID
                'output_frame': 'laser' # 출력 포인트클라우드의 프레임 ID (Cartographer가 TF 처리)
            }],
            output='screen'
        ),

        # --- Cartographer 노드 ---
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
                ('points2', '/pc_3d'), # scan_to_pointcloud 노드의 출력 토픽
                ('imu', imu_topic)     # 선택된 IMU 토픽
                # 'scan' 리매핑은 제거 (points2 사용)
                # 'odom' 리매핑은 제거 (use_odometry = false, provide_odom_frame = true)
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
                'resolution': 0.05 # 그리드 해상도
            }],
            remappings=[('map', '/map')] # 기본 /map 토픽 사용
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