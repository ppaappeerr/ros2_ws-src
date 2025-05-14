from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ---------- 1. 고정 TF ----------
    static_world_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_world_map',
        arguments=['0','0','0','0','0','0','1','world','map'])

    static_base_laser = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_laser',
        arguments=['0','0','0','0','0','0','1','base_link','laser'])

    static_base_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_imu',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link'])

    # ---------- 2. 센서 노드 ----------
    lidar_node = Node(                             # rplidar A2M8
        package='sllidar_ros2', executable='sllidar_node',
        parameters=[{'frame_id':'laser','channel_type':'serial',
                     'serial_port':'/dev/ttyUSB0','serial_baudrate':115200}])

    imu_node = Node(                               # 현재 MPU6050 노드
        package='mpu6050_py', executable='mpu6050_node',
        parameters=[{'frame_id':'imu_link',
                     'publish_rate':50.0,
                     'use_complementary_filter':True}])

    # ---------- 3. EKF ------------
    ekf_node = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('my_launch'),'config','ekf_minimal.yaml')])

    # ---------- 4. slam_toolbox (mapping mode) ----------
    slam_params = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config', 'mapper_params_online_async.yaml')   # 기본 mapping 파라미터

    slam_node = Node(
        package='slam_toolbox', executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params,
                    {'mode':'mapping',                # launch-file override
                     'use_sim_time':False}],
        remappings=[('/scan','scan')])                # 기본 scan 토픽

    # ---------- 5. RViz (선택) ----------
    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d',os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'rviz', 'mapper.rviz')],
                output='screen')

    return LaunchDescription([
        static_world_map, static_base_laser, static_base_imu,
        lidar_node, imu_node,
        ekf_node,
        slam_node,
        rviz,
    ])
