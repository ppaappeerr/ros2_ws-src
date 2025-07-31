from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_package',  # 🔥 패키지명 수정
            executable='front_cloud_filter_node',
            name='front_cloud_filter',
            output='screen',
            parameters=[{
                'field_of_view_deg': 120.0,
                'max_distance': 3.0,
                'input_topic': '/sweep_cloud',   # 또는 '/accumulated_cloud'
                'output_topic': '/front_cloud',
                'voxel_leaf_size': 0.05
            }]
        )
    ])
