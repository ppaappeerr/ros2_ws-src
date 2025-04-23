import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    cartographer_share = get_package_share_directory('cartographer_ros')

    # 설정 파일 경로
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    cartographer_config_basename = 'cartographer_2d_scan_imu.lua'

    # RViz 설정 파일 (2D용으로 변경하거나 새로 생성 필요)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_2d.rviz')
    # if not os.path.exists(rviz_config_path):
    #     rviz_config_path = os.path.join(cartographer_share, 'configuration_files', 'demo_2d.rviz')
    #     # LogInfo 사용 시 PythonExpression 필요
    #     # LogInfo(msg=PythonExpression(["'Warning: 2D RViz config not found. Using default: ' + '", rviz_config_path, "'"])).log()

    # 파라미터 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_complementary_filter = LaunchConfiguration('use_complementary_filter', default='true')

    # LogInfo 수정
    nodes = [
        LogInfo(msg=f"Cartographer 2D SLAM 시작 (LiDAR Scan + IMU) using {cartographer_config_basename}"),
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

    # --- 중요: 실제 IMU 장착 각도에 맞게 수정 필요 ---
    imu_roll = 0.0
    imu_pitch = 0.0
    imu_yaw = 0.0
    nodes.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=[
                '0', '0', '0.0', # x, y, z 오프셋 (base_link 기준 imu_link 위치)
                str(imu_roll), str(imu_pitch), str(imu_yaw), # roll, pitch, yaw (라디안)
                'base_link', 'imu_link'
            ],
            output='screen'
        )
    )
    # ------------------------------------------------

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
                'use_complementary_filter': use_complementary_filter, # 파라미터 전달
                'complementary_alpha': 0.98,
                'use_calibration': True,
                'calibration_path': '', # 필요시 경로 지정
            }],
            remappings=[ # 상보 필터 사용 시 토픽 이름 변경
                ('imu_raw', '/imu_raw'),
                ('imu_filtered', '/imu') # Cartographer가 사용할 토픽
            ],
            output='screen'
        )
    )

    # 3단계: imu_odom_publisher 제거

    # 4단계: scan_to_pointcloud 제거

    # 5단계: Cartographer 노드 (2D 설정 사용)
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
                ('scan', '/scan'), # LaserScan 직접 사용
                ('imu', PythonExpression(["'/imu' if '", use_complementary_filter, "' == 'true' else '/imu_raw'"])) # IMU 토픽 동적 리매핑
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