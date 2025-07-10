import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # EKF 파라미터 파일의 절대 경로를 가져옵니다.
    # 사용자의 홈 디렉토리를 기준으로 경로를 설정합니다.
    # 'p'를 실제 사용자 이름으로 변경하시거나, 아래 주석처리된 코드를 사용하세요.
    # ekf_config_path = os.path.join(
    #     get_package_share_directory('optical_cane_rpi'),
    #     'config',
    #     'ekf.yaml'
    # )
    
    # 사용자님이 성공했다고 알려주신 경로를 직접 사용합니다.
    # 'p'가 사용자 이름이 맞다면 그대로 사용하시면 됩니다.
    ekf_config_path = '/home/p/ros2_ws/src/optical_cane_rpi/config/ekf.yaml'

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            # 네임스페이스 문제를 피하기 위해 remappings을 여기에 추가할 수 있습니다.
            # remappings=[('/odometry/filtered', '/odom')] 
        )
    ])