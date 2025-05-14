# scan_to_pointcloud_node.py (핵심 수정 부분)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from std_msgs.msg import Header
import numpy as np
import math
# import struct # sensor_msgs_py.point_cloud2 사용 시 직접 필요 없음
# import tf2_ros # 이 노드에서는 TF listener/broadcaster 직접 사용 안 함 (IMU 데이터를 직접 사용)
from geometry_msgs.msg import Quaternion # IMU 데이터 타입
import sensor_msgs_py.point_cloud2 as pc2 # PointCloud2 생성/읽기 표준 라이브러리

# tf_transformations 라이브러리는 이미 설치 확인됨
from tf_transformations import euler_from_quaternion, quaternion_matrix, quaternion_multiply, quaternion_from_euler

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud_node') # 노드 이름 명시

        # 파라미터 선언
        self.declare_parameter('input_scan_topic', 'scan')
        self.declare_parameter('input_imu_topic', 'imu_filtered') # IMU 토픽 파라미터 추가
        self.declare_parameter('output_pc_topic', 'pc_3d')
        self.declare_parameter('output_frame_id', 'base_link')  # 출력 PointCloud2의 frame_id
        self.declare_parameter('use_imu_for_3d_transform', True) # IMU를 3D 변환에 사용할지 여부

        # 파라미터 가져오기
        self.input_scan_topic = self.get_parameter('input_scan_topic').value
        self.input_imu_topic = self.get_parameter('input_imu_topic').value
        self.output_pc_topic = self.get_parameter('output_pc_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.use_imu_for_3d_transform = self.get_parameter('use_imu_for_3d_transform').value

        # LaserScan 구독자
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self.scan_callback,
            10 # QoS
        )

        # IMU 구독자 (use_imu_for_3d_transform가 True일 때)
        self.latest_imu_msg = None
        if self.use_imu_for_3d_transform:
            self.imu_sub = self.create_subscription(
                Imu,
                self.input_imu_topic,
                self.imu_callback,
                10 # QoS
            )
            self.get_logger().info(f"IMU를 사용한 3D 변환 활성화. IMU 토픽 구독: '{self.input_imu_topic}'")
        else:
            self.get_logger().info("IMU를 사용한 3D 변환 비활성화. 포인트는 output_frame_id의 z=0 평면에 생성됩니다.")


        # PointCloud2 발행자
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_pc_topic, 10)

        # base_link와 laser 간의 정적 변환 (사용자가 launch 파일에서 설정한 값을 여기서도 알아야 함)
        # 또는, 이 노드에서는 laser 프레임 기준으로 포인트를 생성하고,
        # TF 시스템이 base_link -> laser (static, 기울기 포함 가능) 와 odom -> base_link (dynamic)를 통해
        # 최종적으로 odom 이나 map 에서 3D로 보이게 하는 방법도 있습니다.
        # 사용자님의 설명을 보면, 이 노드 자체에서 IMU로 3D화를 하고 base_link 기준으로 발행하길 원하시는 것 같습니다.
        # 이 경우, base_link와 laser 간의 (물리적 장착) 상대 위치/자세는 알아야 합니다.
        # 여기서는 LiDAR 포인트가 이미 laser 프레임에 있고, IMU는 base_link의 기울기를 나타낸다고 가정합니다.
        # LiDAR 포인트 (laser 프레임) -> base_link 프레임으로 변환 -> base_link의 기울기(IMU) 적용
        # 만약 LiDAR와 IMU가 base_link에 대해 다른 정적 자세로 달려있다면, 그 TF도 고려해야 합니다.
        # 우선은 LiDAR의 측정값(angle, range)을 laser 프레임의 (x,y,0)으로 보고,
        # 이 laser 프레임이 IMU(base_link)의 기울기를 그대로 따른다고 가정하고 변환합니다.
        # 이는 laser 프레임과 base_link 프레임이 동일한 방향을 가진다고 단순화한 것입니다.
        # 더 정확하려면, combined_sensors_launch.py의 base_link->laser TF를 알아야 합니다.
        # 여기서는 IMU의 orientation을 직접 사용하여 laser 프레임의 점들을 base_link 기준으로 변환합니다.

        self.get_logger().info(f"Scan to PointCloud 노드 시작. 입력 Scan: '{self.input_scan_topic}', "
                             f"입력 IMU: '{self.input_imu_topic if self.use_imu_for_3d_transform else '사용 안 함'}', "
                             f"출력 PC: '{self.output_pc_topic}', 출력 Frame: '{self.output_frame_id}'")

    def imu_callback(self, msg: Imu):
        self.latest_imu_msg = msg

    def scan_callback(self, msg: LaserScan):
        points_data = []  # 변환된 3D 포인트 [x, y, z, intensity] 저장

        current_orientation_quaternion = np.array([0.0, 0.0, 0.0, 1.0]) # 기본값 (회전 없음, x,y,z,w 순서)

        if self.use_imu_for_3d_transform and self.latest_imu_msg is not None: # IMU 데이터가 수신된 경우
            q = self.latest_imu_msg.orientation # IMU 데이터를 쿼터니언 형태로 가져옴
            current_orientation_quaternion = np.array([q.x, q.y, q.z, q.w])
        elif self.use_imu_for_3d_transform and self.latest_imu_msg is None:
            self.get_logger().warn("IMU 데이터 사용 설정되었으나, 수신된 IMU 메시지가 없습니다. Z=0으로 처리합니다.", throttle_duration_sec=5.0)
            # current_orientation_quaternion은 그대로 기본값 사용

        # IMU 쿼터니언으로부터 회전 행렬 생성
        # tf_transformations.quaternion_matrix는 4x4 변환 행렬을 반환 (이동 부분은 0)
        # [x,y,z,w] 순서의 쿼터니언 사용
        try:
            # 중요: current_orientation_quaternion은 base_link의 odom/map에 대한 자세.
            #       우리가 필요한 것은 laser 프레임의 점들을 이 base_link 자세에 맞춰 변환하는 것.
            #       LaserScan의 점 (r, angle)은 laser 프레임의 XY 평면에 있음 (z_laser=0).
            #       만약 output_frame_id가 base_link라면,
            #       1. (r,angle) -> (x_l, y_l, 0) in laser frame
            #       2. (x_l, y_l, 0) -> (x_b, y_b, z_b) in base_link frame (using static base_link -> laser TF)
            #       3. 이 (x_b, y_b, z_b)가 최종 결과. IMU는 base_link 자체의 자세를 나타내므로,
            #          base_link 프레임으로 변환된 포인트는 이미 IMU의 영향을 받은 것임.
            #
            #       만약 "IMU의 roll/pitch를 사용하여 스캔 데이터 자체를 3D 좌표로 변환하고,
            #       이 변환된 3D 포인트들이 `/pc_3d`로 발행" 이라는 의미가
            #       "LiDAR가 base_link에 대해 고정 각도로 기울어져 있고, base_link 자체가 IMU에 따라 움직일 때,
            #        이 모든 것을 고려하여 최종 월드(odom/map) 좌표계의 3D 포인트를 여기서 계산" 이라면 복잡해짐.
            #
            #       가장 간단하고 일반적인 접근:
            #       - scan_to_pointcloud는 (r,angle) -> (x_l,y_l,0) in laser_frame.
            #       - base_link -> laser 정적 TF (물리적 장착 각도 포함)는 launch 파일에서.
            #       - odom -> base_link 동적 TF (IMU+ICP 융합 결과)는 EKF/ICP에서.
            #       이렇게 하면 이 노드는 매우 단순해지고, TF 시스템이 3D 변환을 담당.
            #
            #       하지만 사용자님의 의도("다섯 번째 사진", "roll/pitch를 직접 사용")를 최대한 반영하면,
            #       이 노드에서 IMU의 roll/pitch를 사용하여 laser 스캔을 "기울이는" 효과를 주어야 함.
            #       즉, base_link는 수평이고 laser가 IMU에 따라 기울어지는 것처럼.
            #       또는, base_link가 IMU에 따라 기울어지고, laser는 base_link에 고정. (이것이 더 자연스러움)
            #       이 경우, 출력 프레임은 base_link가 되고, IMU는 base_link의 자세를 나타냄.

            # 여기서는 "base_link가 IMU에 따라 기울어지고, laser는 base_link에 고정되어 있으며,
            # LiDAR 스캔은 laser 프레임의 z=0 평면에서 발생한다"고 가정.
            # 출력은 base_link 프레임의 3D 포인트.
            # 필요한 변환: LaserScan 포인트 (laser 프레임) -> base_link 프레임으로 변환 (정적 TF 사용)
            # 이 과정은 사실 이 노드가 아니라 TF 시스템이 담당해야 이상적.
            #
            # 사용자 의도에 가장 가까운 구현 (IMU로 base_link 기울기 반영, 출력은 base_link 기준):
            # 1. LaserScan의 각 점을 laser 프레임의 (x,y,0)으로 변환.
            # 2. 이 점들을 base_link -> laser 정적 TF의 역변환으로 base_link 프레임으로 가져옴.
            #    (이때 base_link->laser TF는 센서의 물리적 장착 위치/자세를 나타내야 함, 예: 15도 기울기)
            # 3. base_link 프레임으로 변환된 이 점들이 최종 결과. (이때, base_link 프레임 자체가
            #    IMU에 의해 외부 좌표계에 대해 회전된 상태를 가지는 것은 EKF/Odometry의 역할)
            #
            # 현재 코드에서 R을 만드는 방식(05.13.txt)은 IMU orientation을 직접 회전행렬로 사용.
            # 이 R이 base_link의 자세(world/odom 대비)를 나타낸다면,
            # laser 프레임의 점들을 이 R로 변환하면 world/odom 기준의 점이 됨.
            # 하지만 출력 프레임이 base_link라면, R을 사용해서는 안됨.
            #
            # 3번째 사진(물리적으로 기울임)과 4번째 사진(AxisColor-z)을 보면,
            # 최종적으로 Z값이 살아있는 3D 포인트 클라우드를 원하시는 것이 명확.
            # 이는 scan_to_pointcloud 노드에서 IMU의 roll/pitch를 직접 사용하여
            # "LiDAR 스캔 평면 자체를 기울이는" 효과를 주어야 함.
            # 이 경우 출력 프레임은 base_link가 적절.

            # IMU는 base_link의 자세를 나타낸다고 가정.
            # LiDAR 스캔은 laser 프레임의 z=0 평면에서 발생.
            # base_link -> laser는 정적 TF (combined_sensors_launch.py 에서 정의).
            # 이 정적 TF는 LiDAR가 base_link에 어떻게 장착되었는지 (예: 15도 아래로 기울임)를 나타내야 함.
            # (만약 이 정적 TF에 이미 기울기가 반영되어 있다면, 여기서는 추가 회전 필요 없음.
            #  IMU는 EKF가 odom->base_link TF를 만들 때 사용됨)

            # 사용자님의 설명("어깨의 빗면에 부착하고 회전 정보를 imu로 제공")을 다시 생각해보면,
            # IMU는 base_link(어깨)의 기울기를 측정하고, LiDAR는 그 base_link에 고정.
            # scan_to_pointcloud는 이 IMU 정보를 "어떻게" 사용해야 할까요?
            # 방법 1: IMU 정보를 사용하여 base_link -> laser TF를 *동적으로* 변경한다. (TF 발행 노드 필요)
            # 방법 2: 이 노드에서 IMU 정보를 사용하여 LaserScan 점들을 base_link 프레임으로 직접 변환한다.
            # 방법 3: 이 노드는 LaserScan을 laser프레임 z=0으로 발행하고, EKF가 IMU를 사용하여 odom->base_link TF를
            #         정확히 만들고, base_link->laser 정적 TF는 물리적 기울기를 포함한다.
            # PDF의 "사람-본드 라이다" 컨셉: "각 스캔을 실시간으로 3D 좌표계로 투영"
            # 이는 방법 2 또는 방법 3과 유사.
            # 이전 대화에서 "scan_to_pointcloud_node.py로 z축을 달아준다"는 말씀과 연결하면 방법 2가 유력.

            # 방법 2 구현 시도 (출력 프레임: base_link):
            #  a. LaserScan 점 (r, angle) -> (x_l, y_l, 0) in laser frame
            #  b. base_link와 laser 사이의 (물리적 장착) 정적 변환 T_base_laser를 알아야 함.
            #     (combined_sensors_launch.py의 base_link -> laser TF 값)
            #  c. (x_l, y_l, 0)을 T_base_laser로 변환하여 base_link 프레임의 점 (x_b0, y_b0, z_b0)을 얻음.
            #     이때 T_base_laser는 LiDAR의 기울기 등을 포함해야 함.
            #  d. 이 (x_b0, y_b0, z_b0)가 IMU 보정이 *없는* base_link 기준 3D 포인트.
            #  e. 여기에 IMU(current_orientation_quaternion)로 인한 base_link의 현재 기울기를 적용하면
            #     최종 world/odom 기준 좌표가 되지만, 출력 프레임이 base_link라면 이 단계는 불필요.
            #     대신, base_link 프레임 자체가 IMU orientation을 가져야 함.
            #
            # 혼란을 줄이기 위해, *이 노드는 LaserScan을 입력받아, IMU의 현재 roll/pitch를 사용하여
            # 각 2D LiDAR 포인트를 3D로 변환하고, 이 3D 포인트들을 `base_link` 프레임에 발행한다*고 가정.
            # 이 경우, IMU는 `base_link`의 기울기를 나타내고, LiDAR 스캔은 이 기울어진 `base_link`를 기준으로
            # 3D 공간에 뿌려져야 함.

            # 회전 행렬 (IMU의 현재 자세를 나타냄 - base_link의 자세)
            # quaternion_matrix는 [x,y,z,w] 순서의 쿼터니언을 입력으로 받음
            rot_matrix = quaternion_matrix(current_orientation_quaternion)[:3,:3] # 3x3 회전 부분만

            num_ranges = len(msg.ranges)

            for i in range(num_ranges):
                r = msg.ranges[i]
                if not (msg.range_min < r < msg.range_max):
                    continue

                scan_angle = msg.angle_min + i * msg.angle_increment

                # 2D LiDAR 좌표 (LiDAR 프레임의 XY 평면, 즉 z=0)
                x_laser_plane = r * math.cos(scan_angle)
                y_laser_plane = r * math.sin(scan_angle)
                z_laser_plane = 0.0

                # 이 점 (x_laser_plane, y_laser_plane, z_laser_plane)은 LiDAR 센서가 수평일 때의 좌표.
                # 이 점을 IMU(base_link)의 현재 기울기(rot_matrix)로 회전시킨다.
                # 이는 LiDAR 스캔 평면 자체가 base_link와 함께 기울어지는 효과를 줌.
                # 가정: laser 프레임과 base_link 프레임의 원점은 동일하고, 방향만 IMU에 따라 변함.
                #       (또는, base_link->laser 정적 TF가 단위행렬이라고 가정)
                # 더 정확하려면 base_link->laser의 정적 변환을 먼저 적용해야 하지만,
                # 사용자님의 "어깨의 빗면에 부착하고 회전 정보를 imu로 제공" 설명에 따르면
                # IMU의 회전이 곧 base_link의 회전이고, 이 회전된 base_link에 LiDAR가 고정된 방식.

                point_in_laser_frame = np.array([x_laser_plane, y_laser_plane, z_laser_plane])
                # 이 점을 base_link의 현재 자세(IMU orientation)로 변환
                # rot_matrix는 base_link의 자세를 나타내는 회전 행렬 (world/odom 대비)
                # points_data에는 base_link 프레임 기준의 좌표를 저장해야 함.
                # 즉, IMU의 roll/pitch는 base_link의 기울기. LaserScan 점들은 이 기울어진 base_link를 기준으로 3D화.
                rotated_point = rot_matrix.dot(point_in_laser_frame)
                x, y, z = rotated_point[0], rotated_point[1], rotated_point[2]

                intensity = 0.0 # 기본 강도값
                if hasattr(msg, 'intensities') and msg.intensities and i < len(msg.intensities):
                    intensity = float(msg.intensities[i])

                points_data.append([float(x), float(y), float(z), float(intensity)])

        except Exception as e:
            self.get_logger().error(f"PointCloud2 생성 중 오류 (IMU 변환 관련): {e}", throttle_duration_sec=5.0)
            return


        if not points_data:
            return

        # PointCloud2 필드 정의
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # 헤더 설정
        header = Header()
        header.stamp = msg.header.stamp  # 입력 LaserScan의 타임스탬프 사용
        header.frame_id = self.output_frame_id # 'base_link'

        # PointCloud2 메시지 생성 및 발행
        try:
            cloud_msg = pc2.create_cloud(header, fields, points_data)
            self.cloud_pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"PointCloud2 생성 또는 발행 실패: {e}", throttle_duration_sec=5.0)

# (main 함수 등 나머지 부분은 이전과 동일)
def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()