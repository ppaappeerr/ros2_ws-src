# filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/launch/cartographer_3d_revised.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')
    lua_config_basename = 'cartographer_3d_scan_imu.lua'

    # --- Launch Arguments 추가 ---
    use_complementary_filter_arg = DeclareLaunchArgument(
        'use_complementary_filter',
        default_value='false',  # 기본값은 상보 필터 사용 안 함
        description='MPU6050 노드에서 상보 필터 사용 여부'
    )
    imu_topic_source_arg = DeclareLaunchArgument(
        'imu_topic_source',
        default_value='/imu/data_raw',  # 기본값은 Raw IMU 사용
        description='Cartographer가 사용할 IMU 토픽 (/imu/data_raw 또는 /imu/data_complementary)'
    )
    # --------------------------

    nodes_to_launch = [
        LogInfo(msg=f"Cartographer 3D SLAM 시작 (LiDAR Scan + IMU) using {lua_config_basename}"),
        LogInfo(msg=["Using Complementary Filter: ", LaunchConfiguration('use_complementary_filter')]),
        LogInfo(msg=["IMU Topic for Cartographer: ", LaunchConfiguration('imu_topic_source')]),

        # 1. Static TF Publishers
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # 2. Sensor Nodes
        Node(
            package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 115200, 'frame_id': 'laser', 'angle_compensate': True}],
            output='screen'
        ),
        Node(
            package='mpu6050_py', executable='mpu6050_node', name='mpu6050_node',
            parameters=[
                {'frame_id': 'imu_link'},
                {'use_complementary_filter': LaunchConfiguration('use_complementary_filter')}  # 파라미터 전달
            ],
            remappings=[
                ('imu_raw', '/imu/data_raw'),  # Raw 데이터 토픽
                ('imu_filtered', '/imu/data_complementary')  # 필터링된 데이터 토픽
            ],
            output='screen'
        ),

        # 3. Scan to PointCloud Node
        Node(
            package='scan_to_pointcloud', executable='scan_to_pointcloud', name='scan_to_pointcloud_node',
            parameters=[
                {'input_topic': 'scan'},
                {'output_topic': 'pc_3d'},  # Cartographer 입력으로 사용될 포인트 클라우드
                {'frame_id': 'laser'},
                {'output_frame': 'laser'}  # 원본 프레임 유지
            ],
            output='screen'
        ),

        # 4. Cartographer Node
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', lua_config_basename],
            remappings=[
                ('points2', '/pc_3d'),         # /pc_3d 직접 사용
                ('imu', LaunchConfiguration('imu_topic_source'))  # 선택된 IMU 토픽 사용
            ]
        ),

        # 5. Occupancy Grid Node
        Node(
            package='cartographer_ros', executable='cartographer_occupancy_grid_node', name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False, 'resolution': 0.05}],
            remappings=[('map', '/map')]  # Cartographer가 발행하는 /map 사용
        ),

        # 6. RViz
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ]

    return LaunchDescription([
        use_complementary_filter_arg,  # Launch Argument 등록
        imu_topic_source_arg,          # Launch Argument 등록
        *nodes_to_launch
    ])