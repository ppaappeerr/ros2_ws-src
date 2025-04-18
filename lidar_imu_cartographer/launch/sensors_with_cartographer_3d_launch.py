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
    lidar_imu_fusion_share = get_package_share_directory('lidar_imu_fusion')
    
    # 파라미터 선언
    use_external_rviz = LaunchConfiguration('use_external_rviz')
    
    return LaunchDescription([
        # RViz 중복 실행 방지 파라미터
        DeclareLaunchArgument(
            'use_external_rviz',
            default_value='true',
            description='RViz를 combined_sensors_launch.py에서만 사용'
        ),
        
        # 1단계: 센서 노드들 실행 (combined_sensors_launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sllidar_share, '/launch/combined_sensors_launch.py']),
        ),
        
        # 2단계: imu_tf_publisher 실행 (중요: 원점쌍과 센서쌍 연결)
        Node(
            package='lidar_imu_fusion',
            executable='imu_tf_publisher',
            name='imu_tf_publisher',
            output='screen',
            parameters=[{
                'parent_frame': 'map',
                'child_frame': 'base_link',
                'publish_rate': 20.0,
                'stabilization_time': 1.0,
                'invert_roll': False,
                'invert_pitch': False,
                'invert_yaw': False,
                'maintain_orientation': False
            }]
        ),
        
        # 3단계: 2D 스캔을 3D 포인트클라우드로 변환
        Node(
            package='scan_to_pointcloud',
            executable='scan_to_pointcloud',  # 실제 실행 파일 이름
            name='scan_to_pointcloud',
            output='screen',
            parameters=[{
                'input_topic': 'scan',
                'output_topic': 'pc_3d',
                'frame_id': 'laser',
                'use_height_coloring': True,
            }]
        ),
        
        # 4단계: 포인트클라우드 누적
        Node(
            package='scan_to_pointcloud',
            executable='accumulated_pointcloud',  # 실제 실행 파일 이름
            name='accumulated_pointcloud',
            output='screen',
            parameters=[{
                'use_tf': True,
                'max_points': 100000,
                'grid_size': 0.01,
                'publish_rate': 5.0,
                'point_skip': 2,
                'use_imu_tf_only': True,
                'disable_position_tracking': True,
            }]
        ),
        
        # 3D 카르토그래퍼 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_share, 'config'),
                '-configuration_basename', 'cartographer_3d.lua'
            ],
            remappings=[
                ('points2', 'accumulated_points'),  # 누적 포인트클라우드 사용
                ('imu', 'imu'),
                ('map', 'carto_map_3d/map')  # map 토픽 이름 변경
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
                ('map', 'carto_map_3d/map')  # map 토픽 이름 변경
            ]
        ),
        
        # carto_map_3d -> map 변환 연결
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_carto3d_to_map',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'carto_map_3d', 'map'],
            output='screen'
        ),
    ])