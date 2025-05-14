from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Static TF (base_link→laser, base_link→imu_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link'],
            output='screen'
        ),
        # 2. LiDAR/IMU 센서 노드 (센서가 반드시 먼저 실행)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': '115200',
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity'
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
                'use_complementary_filter': True
            }],
            output='screen'
        ),
        # 3. LaserScan → PointCloud2 변환
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{'input_topic': '/scan', 'output_topic': '/pc_3d', 'frame_id': 'laser'}],
            output='screen'
        ),
        # 4. 누적 포인트클라우드
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',
            name='accumulated_pointcloud',
            parameters=[{'max_points': 10000, 'grid_size': 0.05, 'use_tf': True}],
            output='screen'
        ),
        # 5. ICP Odometry (odom, odom→base_link TF)
        Node(
            package='lidar_icp_odometry',
            executable='icp_odom_node',
            name='icp_odom_node',
            output='screen'
        ),
        # 6. EKF (odom, imu_filtered → odom_filtered, odom→base_link TF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=['/home/p/ros2_ws/src/lidar_icp_odometry/config/ekf_odom.yaml'],
            output='screen'
        ),
        # 7. IMU TF Publisher (map→base_link, rotation only, 디버깅용)
        # 필요시만 활성화
        # Node(
        #     package='lidar_imu_tf',
        #     executable='imu_tf_publisher',
        #     name='imu_tf_publisher',
        #     parameters=[{'parent_frame': 'map', 'child_frame': 'base_link', 'publish_rate': 20.0}],
        #     output='screen'
        # ),
        # 8. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])