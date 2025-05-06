from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
import math
import numpy as np  # numpy import 추가

def generate_launch_description():
    slam_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/slam_online.yaml')
    ekf_yaml = os.path.expanduser('~/ros2_ws/src/sweep_builder/config/ekf_local.yaml')

    return LaunchDescription([

        # 1. Leveled Scan Node (주석 처리하여 비활성화)
        Node(
            package='sweep_builder',
            executable='leveled_scan_node',
            name='leveled_scan_node',
            output='screen',
            parameters=[{
                'in_scan': '/scan',
                'out_scan': '/scan_leveled'
            }]
        ),

        # 🧭 2. EKF 필터
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf_yaml]
        ),

        # 🗺️ 3. slam_toolbox (딜레이 실행)
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[slam_yaml]
                )
            ]
        ),

        # 🔄 4. lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 10.0,
                'node_names': ['slam_toolbox']
            }]
        )
    ])

    def scan_cb(self, m):
        out = LaserScan()
        out.header = m.header
        out.header.frame_id = 'laser'  # 중요
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        factor = math.cos(self.roll) * math.cos(self.pitch)
        # out.ranges = [r * factor for r in m.ranges] # 기존 코드 주석 처리

        # NaN/inf 검사 및 필터링 추가
        ranges_np = np.array(m.ranges) * factor
        valid_indices = np.isfinite(ranges_np) # NaN과 inf가 아닌 유효한 인덱스 찾기
        if not np.all(valid_indices):
            self.get_logger().warn(f"Invalid range values detected! Original count: {len(m.ranges)}, Valid count: {np.sum(valid_indices)}")
            # 유효하지 않은 값은 예를 들어 max_range 값으로 대체하거나, 다른 처리 방식 선택 가능
            # 여기서는 간단히 max_range로 대체
            ranges_np[~valid_indices] = out.range_max
        out.ranges = ranges_np.tolist() # 리스트로 변환하여 할당

        out.intensities = m.intensities
        self.pub.publish(out)
