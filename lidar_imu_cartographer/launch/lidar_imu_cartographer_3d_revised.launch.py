import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    cartographer_share = get_package_share_directory('cartographer_ros')
    lidar_imu_fusion_share = get_package_share_directory('lidar_imu_fusion') # 추가

    # 설정 파일 경로
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer_3d_scan_imu.lua' # 사용할 설정 파일

    # RViz 설정 파일
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')
    if not os.path.exists(rviz_config_path):
        # 파일이 없으면 기본 rviz 설정 사용 또는 에러 로깅
        # LogInfo는 generate_launch_description 함수 내에서 직접 사용하기보다
        # LaunchDescription의 일부로 반환하는 것이 일반적입니다.
        # 여기서는 rviz_config_path를 빈 문자열로 설정하여 RViz 실행을 건너뜁니다.
        rviz_config_path = ''

    # 파라미터 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_complementary_filter = LaunchConfiguration('use_complementary_filter', default='false') # 상보 필터 사용 여부

    # 사용할 IMU 토픽 결정
    imu_topic_for_fusion = PythonExpression([
        "'/imu' if '", use_complementary_filter, "' == 'true' else '/imu_raw'"
    ])
    # Cartographer가 사용할 오도메트리 토픽
    odom_topic_for_cartographer = '/odom' # imu_odom_publisher가 발행하는 토픽

    nodes = [
        LogInfo(msg=f"Cartographer 3D SLAM 시작 (LiDAR Scan + IMU) using {cartographer_config_basename}"),
        LogInfo(msg=PythonExpression([
            "'Using Complementary Filter: true' if '", use_complementary_filter, "' == 'true' else 'Using Complementary Filter: false'"
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
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'], # base_link -> laser
            output='screen'
        )
    )

    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0.0', '0', '0', '0', '1', 'base_link', 'imu_link'], # base_link -> imu_link (Z 오프셋 0으로 수정)
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
                'scan_mode': 'Sensitivity', # 추가: 스캔 모드 설정
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
                'use_complementary_filter': use_complementary_filter, # 파라미터 전달
                'complementary_alpha': 0.98,
                'use_calibration': True,
                'calibration_path': '', # 필요시 경로 지정
            }],
            output='screen'
        )
    )

    # 3단계: IMU 기반 오도메트리 발행 (TF + Odometry 메시지)
    nodes.append(
        Node(
            package='lidar_imu_fusion',
            executable='imu_odom_publisher',
            name='imu_odom_publisher',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'imu_topic': imu_topic_for_fusion, # 사용할 IMU 토픽 전달
                'odom_topic': odom_topic_for_cartographer # 발행할 Odometry 토픽 이름 전달
            }]
        )
    )

    # 4단계: 스캔 -> 포인트클라우드 변환 (지연 시작 필요 없음)
    nodes.append(
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud', # 수정: 실행 파일 이름 확인
            name='scan_to_pointcloud',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser', # 입력 스캔의 프레임 ID
                'output_frame': 'laser', # 출력 포인트클라우드의 프레임 ID
                'use_tf': False # TF 변환은 Cartographer가 처리
            }],
            output='screen'
        )
    )

    # 5단계: Cartographer 노드
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename
            ],
            remappings=[
                ('points2', 'pc_3d'),
                ('imu', imu_topic_for_fusion), # Cartographer가 사용할 IMU 토픽
                ('odom', odom_topic_for_cartographer) # Cartographer가 사용할 Odometry 토픽 리매핑
            ]
        )
    )

    # 6단계: 점유 그리드 노드 (지연 시작 필요 없음)
    nodes.append(
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05
            }],
            output='screen'
            # map 토픽은 기본값(/map) 사용
        )
    )

    # 7단계: RViz (선택 사항)
    if rviz_config_path:
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