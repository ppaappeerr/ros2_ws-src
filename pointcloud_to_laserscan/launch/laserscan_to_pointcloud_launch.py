from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # LaserScan → PointCloud 변환 노드
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[
                ('scan_in', '/scan'),
                ('cloud', '/cloud')
            ],
            parameters=[{
                'target_frame': 'base_link', # laser -> base_link               # rplidar에서 사용하는 frame_id
                'transform_tolerance': 0.01
            }],
            output='screen'
        ),

        # 필요한 경우: laser → base_link TF (고정 변환)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser_to_base',
            arguments=[
                '0', '0', '0',   # translation
                '0', '0', '0',   # rpy
                'base_link', 'laser'
            ]
        ),
    ])

