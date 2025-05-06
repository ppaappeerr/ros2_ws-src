from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_share = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(
        get_package_share_directory('sweep_builder'),
        'config',
        'obstacle_costmap_params.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time':  'false',
                'params_file':   params_file,
                'autostart':     'true',
                'map':           ''
            }.items()
        )
    ])
# 필요하면 controller_server 등 다른 BT 노드도 YAML에서 함께 끌 수 있습니다.
# config 폴더도 없다면 sweep_builder/config/obstacle_costmap_params.yaml 경로에 파일 만들어주기!