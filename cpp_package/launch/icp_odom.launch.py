from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_package',
            executable='icp_3d_odom_node',
            name='icp_3d_odom_cpp',
            parameters=[{
                'input_topic': '/points_3d',
                'output_topic': '/lio_odom',
                'max_correspondence_distance': 0.15,  # 약간 증가
                'max_iterations': 40,                 # 줄임
                'voxel_size': 0.05,                  # 약간 증가
                'publish_tf': True,
                # 성능 최적화 파라미터
                'max_processing_time': 25.0,        # 25ms 제한
                'min_points_threshold': 100,
                'max_points_threshold': 1500,        # 포인트 수 제한
                'adaptive_voxel_size': True
            }],
            output='screen'
        )
    ])