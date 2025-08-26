from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Declare the launch argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='A',
        description='The experimental mode to run. Options: A, B, C, D, E.'
    )

    # Common nodes for 3D pipelines (A, C, D, E)
    # Note: Mode B does not use these.
    sensor_fusion_node = Node(
        package='optical_cane_rpi',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
    )
    point_cloud_sweeper_node = Node(
        package='cpp_package',
        executable='point_cloud_sweeper_cpp_node',
        name='point_cloud_sweeper_cpp_node',
    )
    voxel_grid_filter_node = Node(
        package='cpp_package',
        executable='voxel_grid_filter_node',
        name='voxel_grid_filter_node',
        parameters=[{'leaf_size': 0.04}]
    )

    # Planner nodes for each mode
    planner_a = Node(
        package='cpp_package',
        executable='path_planner_node',
        name='path_planner_a',
        condition=IfCondition(TextSubstitution(text="'$(var mode)' == 'A'"))
    )

    planner_b_group = GroupAction(
        condition=IfCondition(TextSubstitution(text="'$(var mode)' == 'B'")),
        actions=[
            Node(
                package='optical_cane_rpi',
                executable='scan_accumulator_node',
                name='scan_accumulator_node'
            ),
            Node(
                package='cpp_package',
                executable='path_planner_node',
                name='path_planner_b',
                remappings=[('/sweep_cloud_cpp', '/scan_accumulation_cloud')]
            )
        ]
    )

    planner_c = Node(
        package='cpp_package',
        executable='path_planner_3d_node',
        name='path_planner_c',
        condition=IfCondition(TextSubstitution(text="'$(var mode)' == 'C'"))
    )

    planner_d = Node(
        package='cpp_package',
        executable='ftg_3d_node',
        name='path_planner_d',
        condition=IfCondition(TextSubstitution(text="'$(var mode)' == 'D'"))
    )

    planner_e = Node(
        package='cpp_package',
        executable='heightmap_planner_node',
        name='path_planner_e',
        condition=IfCondition(TextSubstitution(text="'$(var mode)' == 'E'"))
    )

    return LaunchDescription([
        mode_arg,
        
        # Common nodes are launched for all modes except B
        GroupAction(
            condition=UnlessCondition(TextSubstitution(text="'$(var mode)' == 'B'")),
            actions=[
                sensor_fusion_node,
                point_cloud_sweeper_node,
                voxel_grid_filter_node
            ]
        ),

        # Planner nodes
        planner_a,
        planner_b_group,
        planner_c,
        planner_d,
        planner_e,
    ])