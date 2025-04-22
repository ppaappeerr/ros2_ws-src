import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')

    lua_config_basename = 'cartographer_3d_scan_imu.lua' # Lua 파일 이름 확인

    return LaunchDescription([
        LogInfo(msg=f"Cartographer 3D SLAM 시작 (LiDAR Scan + IMU Odom) using {lua_config_basename}"),

        # 1. Static TF Publishers (base_link 기준 센서 위치)
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'] # imu_link와 base_link 동일하게 설정
        ),

        # 2. Sensor Nodes
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 115200, 'frame_id': 'laser', 'angle_compensate': True}],
            output='screen'
        ),
        Node(
            package='mpu6050_py', executable='mpu6050_node', name='mpu6050_node',
            parameters=[{'frame_id': 'imu_link'}],
            remappings=[('imu', '/imu/data_raw')], # Raw 데이터 발행
            output='screen'
        ),

        # 2.5 IMU Filter Node (Madgwick)
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter_node',
            parameters=[{'use_mag': False, 'publish_tf': False}], # TF 발행 안 함
            remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')], # 필터링된 데이터 /imu/data 발행
            output='screen'
        ),

        # 2.6 IMU Odom Publisher Node (수정된 노드 추가)
        Node(
            package='lidar_imu_fusion', # 패키지 이름 확인
            executable='imu_odom_publisher', # setup.py에 정의된 실행 파일 이름
            name='imu_odom_publisher_node',
            parameters=[
                {'odom_frame': 'odom'}, # 발행할 TF: odom -> base_link
                {'base_frame': 'base_link'}
            ],
            remappings=[('imu', '/imu/data')], # 필터링된 IMU 데이터 구독
            output='screen'
        ),

        # 3. Cartographer Nodes
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', lua_config_basename],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu/data') # Cartographer는 여전히 필터링된 IMU 직접 사용
                # 오도메트리 TF는 Cartographer가 내부적으로 사용하므로 odom 리매핑 불필요
            ]
        ),
        Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}],
            remappings=[('map', '/map')]
        ),

        # 4. RViz
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])