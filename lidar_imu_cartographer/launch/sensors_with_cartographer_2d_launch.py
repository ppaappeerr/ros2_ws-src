import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('lidar_imu_cartographer')
    sllidar_share = get_package_share_directory('sllidar_ros2')
    
    # 파라미터 선언
    use_external_rviz = LaunchConfiguration('use_external_rviz')
    
    return LaunchDescription([
        # RViz 중복 실행 방지 파라미터
        DeclareLaunchArgument(
            'use_external_rviz',
            default_value='true',
            description='RViz를 combined_sensors_launch.py에서만 사용'
        ),
        
        # 센서 노드들 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sllidar_share, '/launch/combined_sensors_launch.py']),
        ),
        
        # 2D 카르토그래퍼 실행
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_share, 'config'),
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            remappings=[
                ('scan', 'scan'),
                ('imu', 'imu'),
                ('map', 'carto_map_2d/map')  # map 토픽 이름 변경
            ]
        ),
        
        # 점유 그리드 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
            }],
            remappings=[
                ('map', 'carto_map_2d/map')  # map 토픽 이름 변경
            ]
        ),
        # carto_map_2d와 map 프레임 연결 (필요시 추가)
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_carto2d_to_map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'carto_map_2d', 'map'],
        output='screen'
    ),
    ])