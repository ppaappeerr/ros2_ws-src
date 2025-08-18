import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the share directory for cpp_package
    cpp_package_share_dir = get_package_share_directory('cpp_package')

    # --- Declare Launch Arguments ---
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='3d_corridor',
        description="Type of pipeline to run: '2d_projection', '2d_direct', or '3d_corridor'"
    )

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='If true, records all topics to a rosbag file.'
    )
    
    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        default_value=PythonExpression(["'rosbag2_', LaunchConfiguration('pipeline')"]),
        description='Name of the rosbag file.'
    )

    # --- Define Nodes ---

    # Node for the 3D point cloud sweeper (used by 2d_projection and 3d_corridor)
    point_cloud_sweeper_node = Node(
        package='cpp_package',
        executable='point_cloud_sweeper_cpp_node',
        name='point_cloud_sweeper_cpp_node',
        output='screen',
        condition=IfCondition(PythonExpression(["LaunchConfiguration('pipeline') != '2d_direct'"]))
    )

    # Node for the 2D scan accumulator (used by 2d_direct)
    scan_accumulator_node = Node(
        package='optical_cane_rpi',
        executable='scan_accumulator_node',
        name='scan_accumulator_node',
        output='screen',
        parameters=[{'front_view_only': True}],
        condition=IfCondition(PythonExpression(["LaunchConfiguration('pipeline') == '2d_direct'"]))
    )

    # Node for the 2D projection path planner
    path_planner_2d_node = Node(
        package='cpp_package',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[{'use_voxel_filter': False}],
        remappings=[
            # Remap input if using the 2d_direct pipeline
            ('/sweep_cloud_cpp', PythonExpression([
                "' /scan_accumulation_cloud' if LaunchConfiguration('pipeline') == '2d_direct' else '/sweep_cloud_cpp'"
            ]))
        ],
        condition=IfCondition(PythonExpression(["LaunchConfiguration('pipeline') != '3d_corridor'"]))
    )

    # Node for the 3D corridor path planner
    path_planner_3d_node = Node(
        package='cpp_package',
        executable='path_planner_3d_node',
        name='path_planner_3d_node',
        output='screen',
        parameters=[{'use_voxel_filter': False}],
        condition=IfCondition(PythonExpression(["LaunchConfiguration('pipeline') == '3d_corridor'"]))
    )

    # --- ROS Bag Recording ---
    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('bag_filename')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record'))
    )

    return LaunchDescription([
        pipeline_arg,
        record_arg,
        bag_filename_arg,
        
        # Sensor nodes would typically be launched here as well,
        # but we assume they are running separately for this test.
        
        point_cloud_sweeper_node,
        scan_accumulator_node,
        path_planner_2d_node,
        path_planner_3d_node,
        
        record_process
    ])
