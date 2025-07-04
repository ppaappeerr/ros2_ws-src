from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Zebedee-LIO 시스템의 모든 노드를 실행하는 통합 런치파일.
    - RPLIDAR (2D LiDAR)
    - MPU9250 (IMU)
    - PointCloud Assembler (2D Scan -> 3D PointCloud)
    - LIO Frontend (Odometry)
    """
    return LaunchDescription([
        # 1. RPLIDAR 드라이버 노드
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0', # 실제 포트에 맞게 수정
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
        ),

        # 2. MPU9250 IMU 드라이버 노드
        # (기존에 사용하시던 mpu9250_filtered_node.py를 실행하는 방식)
        Node(
            package='mpu9250',
            executable='mpu9250_filtered_node.py',
            name='mpu9250_filtered_node',
            output='screen'
        ),

        # 3. PointCloud Assembler 노드
        # (기존에 사용하시던 lidar_imu_filtered_to_pointcloud.py를 실행하는 방식)
        Node(
            package='sweep_builder',
            executable='lidar_imu_filtered_to_pointcloud.py',
            name='pointcloud_assembler_node',
            output='screen'
        ),
        
        # 4. Zebedee-LIO Frontend 노드
        Node(
            package='zebedee_lio',
            executable='lio_frontend_node',
            name='lio_frontend_node',
            output='screen',
            # 파라미터는 여기서 직접 설정하거나, YAML 파일을 로드할 수 있습니다.
            parameters=[
                {'submap.sliding_window_size': 15.0},
                {'submap.voxel_leaf_size': 0.25}
            ]
        )
    ])