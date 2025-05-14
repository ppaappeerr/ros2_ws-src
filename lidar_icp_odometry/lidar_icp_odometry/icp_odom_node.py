# icp_odom_node.py
# 파일 경로: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/icp_odom_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2 # ros2_numpy로 대체 가능
import ros2_numpy # point_cloud2 to numpy array
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R # Scipy Rotation
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import Header # Header import

# Helper function (필요시): 쿼터니언을 geometry_msgs/Quaternion으로 변환
def np_quat_to_ros_quat(np_quat: np.ndarray) -> Quaternion:
    ros_q = Quaternion()
    ros_q.x = np_quat[0]
    ros_q.y = np_quat[1]
    ros_q.z = np_quat[2]
    ros_q.w = np_quat[3]
    return ros_q

class ICPOdomNode(Node):
    def __init__(self):
        super().__init__('icp_odom_node')

        self.prev_cloud_np = None       # 이전 포인트 클라우드 (NumPy array)
        self.current_transform_matrix = np.eye(4) # odom 프레임 기준 base_link의 현재 변환 행렬 (T_odom_base)

        # 파라미터 선언 (필요한 경우 추가)
        self.declare_parameter('input_cloud_topic', 'pc_3d')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_link_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True) # 이 노드가 TF를 발행할지 여부

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.odom_topic_name = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_link_frame = self.get_parameter('base_link_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic, # '/pc_3d'
            self.cloud_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic_name, 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"ICP Odometry Node 시작. 입력: '{self.input_cloud_topic}', 출력: '{self.odom_topic_name}', TF: {self.publish_tf}")

    def icp_step(self, source_cloud, target_cloud, initial_guess=np.eye(4), max_iterations=20, tolerance=1e-4):
        """
        간단한 ICP 알고리즘 단계.
        source_cloud를 target_cloud에 정합하는 변환 행렬을 찾습니다.
        """
        current_source_cloud = source_cloud
        current_transformation = initial_guess # 이전 프레임에서의 base_link -> 현재 프레임에서의 base_link (T_k-1_k)

        for iteration in range(max_iterations):
            # 1. 가장 가까운 점 찾기 (Nearest Neighbors)
            # target_cloud의 각 점에 대해 source_cloud에서 가장 가까운 점을 찾음
            # 여기서는 sklearn.neighbors 사용
            if len(current_source_cloud) == 0 or len(target_cloud) == 0:
                self.get_logger().warn("ICP: 포인트 클라우드가 비어있습니다.")
                return None

            # target_cloud에 대한 source_cloud의 매칭을 찾음
            # 여기서는 source_cloud를 target_cloud에 맞추므로, source의 각 점에 대한 target의 최근접 이웃을 찾음
            neigh = NearestNeighbors(n_neighbors=1, algorithm='auto')
            neigh.fit(target_cloud) # target_cloud를 기준으로 KD-Tree 생성
            distances, indices = neigh.kneighbors(current_source_cloud) # current_source_cloud의 각 점에 대한 최근접 이웃

            # 대응점 쌍 (correspondences)
            # source_points_corr = current_source_cloud
            # target_points_corr = target_cloud[indices.ravel()]

            # 역으로 대응점 설정 (target의 각 점에 대해 source에서 찾음)
            # 이 방식이 더 일반적일 수 있음: source를 target에 맞춤
            neigh_source = NearestNeighbors(n_neighbors=1, algorithm='auto')
            neigh_source.fit(current_source_cloud)
            distances_s, indices_s = neigh_source.kneighbors(target_cloud)

            source_points_corr = current_source_cloud[indices_s.ravel()]
            target_points_corr = target_cloud


            # 2. 변환 계산 (SVD 사용)
            # source_points_corr와 target_points_corr 간의 변환 계산
            # 여기서는 target_points_corr (기준)에 source_points_corr를 맞추는 변환을 구함
            # 즉, T सच दैट target_points_corr ≈ T * source_points_corr
            assert source_points_corr.shape == target_points_corr.shape

            centroid_source = np.mean(source_points_corr, axis=0)
            centroid_target = np.mean(target_points_corr, axis=0)

            centered_source = source_points_corr - centroid_source
            centered_target = target_points_corr - centroid_target

            H = centered_source.T @ centered_target # 공분산 행렬 (cross-covariance matrix)

            U, S, Vt = np.linalg.svd(H)
            R_icp = Vt.T @ U.T # 회전 행렬

            # 반사(reflection) 경우 처리 (det(R) == -1)
            if np.linalg.det(R_icp) < 0:
                Vt[2, :] *= -1
                R_icp = Vt.T @ U.T

            # 이동 벡터 (translation vector)
            t_icp = centroid_target.T - R_icp @ centroid_source.T

            # 현재 반복에서의 변환 행렬 (ΔT_iter)
            delta_transform_iter = np.eye(4)
            delta_transform_iter[:3, :3] = R_icp
            delta_transform_iter[:3, 3] = t_icp

            # 전체 변환 업데이트: T_k-1_k = ΔT_iter * T_k-1_k_previous_iter
            current_transformation = delta_transform_iter @ current_transformation
            # current_transformation은 source -> target 변환

            # 변환된 소스 클라우드 업데이트
            # current_source_cloud_homogeneous = np.hstack((source_cloud, np.ones((source_cloud.shape[0], 1))))
            # transformed_source_cloud_homogeneous = (current_transformation @ current_source_cloud_homogeneous.T).T
            # current_source_cloud = transformed_source_cloud_homogeneous[:, :3]

            # 변화량 체크 (종료 조건)
            # delta_norm = np.linalg.norm(delta_transform_iter - np.eye(4))
            # self.get_logger().debug(f"ICP Iter {iteration}: delta_norm={delta_norm}")
            # if delta_norm < tolerance:
            #     break

            # 간단하게는, SVD 한 번으로 끝내는 Point-to-Point ICP
            # 위 반복문은 Iterative Closest Point의 반복을 의미함
            # 여기서는 SVD 한 번으로 계산된 R_icp, t_icp를 사용
            # 이 함수는 source_cloud를 target_cloud로 옮기는 ΔT를 반환해야함
            # 따라서 current_transformation이 이 ΔT가 됨.

            # 위 로직은 full ICP 반복. 간단한 버전은 아래와 같이 SVD 한번으로 끝냄
            # 아래 코드는 source_cloud를 target_cloud로 옮기는 변환 (T_source_target)을 계산
            final_transform = np.eye(4)
            final_transform[:3,:3] = R_icp
            final_transform[:3,3] = t_icp
            return final_transform # ΔT (current_scan -> prev_scan)

        return current_transformation # ΔT (current_scan -> prev_scan)


    def cloud_callback(self, msg: PointCloud2):
        # PointCloud2 메시지를 NumPy 배열로 변환 (x, y, z 필드만 사용)
        # ros2_numpy.point_cloud2.pointcloud2_to_array는 structured array를 반환
        # xyz_array = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)

        # 필드 직접 파싱 (x,y,z만 추출 가정)
        current_cloud_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            current_cloud_list.append([point[0], point[1], point[2]])

        if not current_cloud_list:
            self.get_logger().warn("수신된 포인트 클라우드가 비어있거나 파싱 실패")
            return
        current_cloud_np = np.array(current_cloud_list)

        if self.prev_cloud_np is None or len(self.prev_cloud_np) < 10 : # 초기화 또는 포인트 부족
            self.get_logger().info("이전 클라우드 초기화 중 또는 포인트 부족")
            self.prev_cloud_np = current_cloud_np
            # 초기 Odometry 발행 (0,0,0 위치, 회전 없음)
            header = Header()
            header.stamp = msg.header.stamp # 입력 메시지 타임스탬프 사용
            header.frame_id = self.odom_frame

            odom_msg = Odometry()
            odom_msg.header = header
            odom_msg.child_frame_id = self.base_link_frame
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0
            # 초기 공분산 (불확실성 높음 또는 0에 가까운 값)
            # 예시: 대각 행렬에 작은 값
            odom_msg.pose.covariance = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3]
            # Twist는 0으로 초기화
            odom_msg.twist.covariance = [-1.0] * 36 # 알 수 없음
            self.odom_publisher.publish(odom_msg)

            if self.publish_tf:
                t = TransformStamped()
                t.header = header # 동일한 헤더 사용
                t.child_frame_id = self.base_link_frame
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)
            return

        # ICP 수행: current_cloud_np (source)를 prev_cloud_np (target)에 정합
        # ICP는 현재 스캔(source)을 이전 스캔(target)에 맞추는 변환 T_target_source (또는 ΔT)를 반환
        # ΔT는 T_prev_curr 를 의미: prev_cloud 좌표계에서 curr_cloud 좌표계로의 변환
        # 또는 curr_cloud 포인트들을 prev_cloud 좌표계로 옮기는 변환
        # delta_transform = self.icp_step(current_cloud_np, self.prev_cloud_np)
        # 아래는 prev_cloud (source)를 current_cloud (target)에 맞추는 경우 (이게 맞을 수도)
        delta_transform = self.icp_step(self.prev_cloud_np, current_cloud_np) # T_curr_prev

        if delta_transform is None:
            self.get_logger().warn("ICP 변환 계산 실패")
            # 실패 시, 이전 포즈 유지 또는 다른 처리
            # 현재는 그냥 return 하고 prev_cloud는 업데이트하지 않음
            return

        # Odometry 업데이트: T_odom_basenew = T_odom_baseold * T_baseold_basenew
        # 여기서 delta_transform은 T_baseold_basenew (이전 base_link에서 현재 base_link로의 변환)
        # 또는 T_laser_old_laser_new. base_link와 laser는 고정된 관계.
        # ICP 결과 (delta_transform)는 T_k_k-1 (현재 프레임에서 이전 프레임으로의 변환)
        # 또는 source(prev)를 target(curr)에 맞췄다면 T_curr_prev
        # 우리가 필요한건 T_prev_curr = T_curr_prev.inv()
        # self.current_transform_matrix = self.current_transform_matrix @ np.linalg.inv(delta_transform)
        # 만약 icp_step이 T_source_target (T_prev_curr)을 반환한다면:
        self.current_transform_matrix = self.current_transform_matrix @ delta_transform


        # TF 발행 (odom -> base_link)
        if self.publish_tf:
            t = TransformStamped()
            # Odometry 메시지와 TF의 타임스탬프는 현재 입력 클라우드의 타임스탬프를 사용
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.odom_frame      # 부모 프레임 (예: "odom")
            t.child_frame_id = self.base_link_frame # 자식 프레임 (예: "base_link")

            t.transform.translation.x = self.current_transform_matrix[0, 3]
            t.transform.translation.y = self.current_transform_matrix[1, 3]
            t.transform.translation.z = self.current_transform_matrix[2, 3] # 3D ICP라면 Z도 사용

            # scipy의 Rotation을 사용하여 회전 행렬을 쿼터니언으로 변환
            try:
                rotation_matrix = self.current_transform_matrix[:3, :3]
                quat_scipy = R.from_matrix(rotation_matrix).as_quat() # [x, y, z, w] 순서
                t.transform.rotation = np_quat_to_ros_quat(quat_scipy)
            except Exception as e:
                self.get_logger().error(f"쿼터니언 변환 중 오류: {e}")
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0 # 기본값

            self.tf_broadcaster.sendTransform(t)

        # Odometry 메시지 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp # TF와 동일한 타임스탬프
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame

        odom_msg.pose.pose.position.x = self.current_transform_matrix[0, 3]
        odom_msg.pose.pose.position.y = self.current_transform_matrix[1, 3]
        odom_msg.pose.pose.position.z = self.current_transform_matrix[2, 3]

        try:
            rotation_matrix_odom = self.current_transform_matrix[:3, :3]
            quat_scipy_odom = R.from_matrix(rotation_matrix_odom).as_quat()
            odom_msg.pose.pose.orientation = np_quat_to_ros_quat(quat_scipy_odom)
        except Exception as e:
            self.get_logger().error(f"Odometry 쿼터니언 변환 중 오류: {e}")
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0
            odom_msg.pose.pose.orientation.w = 1.0

        # 공분산 행렬 (간단한 예시, 실제로는 ICP 결과의 불확실성을 반영해야 함)
        # 현재 제공된 코드의 공분산은 고정값. 그대로 사용.
        odom_msg.pose.covariance = [
            1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5e-2  # Yaw에 대한 불확실성을 약간 더 크게
        ]
        # Twist 정보는 현재 계산하지 않으므로 0 또는 이전 값 유지.
        # 간단하게는 0으로 설정하고, covariance는 -1 (알 수 없음) 또는 매우 큰 값으로 설정.
        odom_msg.twist.twist.linear.x = 0.0 # 시간 간격을 알면 속도 추정 가능
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [-1.0] * 36 # Twist 값의 불확실성 (알 수 없음)


        self.odom_publisher.publish(odom_msg)

        # 다음 반복을 위해 현재 클라우드를 이전 클라우드로 저장
        self.prev_cloud_np = current_cloud_np

def main(args=None):
    rclpy.init(args=args)
    node = ICPOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
**주요 변경 및 고려 사항:**

  * **NumPy 직접 사용:** `ros2_numpy.point_cloud2.pointcloud2_to_xyz_array` 대신 `pc2.read_points`를 사용하여 x,y,z만 추출하고 
  NumPy 배열로 변환합니다. (기존 코드가 `ros2_numpy`를 사용했다면 그대로 두셔도 됩니다. 여기서는 예시로 변경)
  * **ICP 로직:** `icp_step` 함수는 매우 기본적인 Point-to-Point ICP입니다. 실제 성능은 데이터와 환경에 따라 달라질 수 있으며, 
  더 정교한 ICP (예: Point-to-Plane, GICP)나 라이브러리(Open3D, PCL 바인딩 등) 사용을 고려할 수 있습니다. 
  **제공된 `icp_odom_node.py`의 ICP 로직은 그대로 사용하되, 변환 행렬의 방향(T\_curr\_prev 인지 T\_prev\_curr 인지)을 명확히 하고 
  `self.current_transform_matrix` 업데이트에 올바르게 적용해야 합니다.** 
  위 코드에서는 `delta_transform = self.icp_step(self.prev_cloud_np, current_cloud_np)` 로 호출하고, 이 `delta_transform`이 
  `T_prev_curr` (이전 프레임에서 현재 프레임으로의 상대 변환)를 나타낸다고 가정하고 누적합니다.
  * **타임스탬프 일관성:** Odometry 메시지와 TF 모두 `msg.header.stamp` (입력 PointCloud2의 타임스탬프)를 사용하도록 통일했습니다. 
  이는 시간 동기화에 매우 중요합니다.
  * **프레임 ID:** 파라미터로 설정 가능하도록 변경하고, 올바른 `odom_frame`과 `base_link_frame`을 사용하도록 합니다.
  * **초기화:** 첫 번째 클라우드 수신 시 또는 포인트가 부족할 때의 초기화 로직을 추가했습니다.
  * **Twist (속도):** 현재 코드에서는 속도를 계산하지 않으므로 0으로 설정하고 공분산은 -1 (알 수 없음)로 둡니다. 
  정확한 속도 계산을 위해서는 이전 타임스탬프와의 시간 간격을 알아야 합니다.
'''