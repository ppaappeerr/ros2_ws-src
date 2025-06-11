import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
from std_msgs.msg import Header

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

        self.prev_cloud_np = None
        self.current_transform_matrix = np.eye(4)

        self.declare_parameter('input_cloud_topic', 'pc_3d')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_link_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.odom_topic_name = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_link_frame = self.get_parameter('base_link_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_cloud_topic,
            self.cloud_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic_name, 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # latest_transform_stamp 초기화
        self.latest_transform_stamp = self.get_clock().now().to_msg()

        # TF/odom을 빠르게 발행하는 타이머 (25Hz)
        self.tf_publish_timer = self.create_timer(1.0 / 25.0, self.publish_tf_and_odom)

        self.get_logger().info(f"ICP Odometry Node 시작. 입력: '{self.input_cloud_topic}', 출력: '{self.odom_topic_name}', TF: {self.publish_tf}")

    def icp_step(self, source_cloud, target_cloud, initial_guess=np.eye(4), max_iterations=20, tolerance=1e-4):
        if len(source_cloud) == 0 or len(target_cloud) == 0:
            self.get_logger().warn("ICP: 포인트 클라우드가 비어있습니다.")
            return None

        neigh_source = NearestNeighbors(n_neighbors=1, algorithm='auto')
        neigh_source.fit(source_cloud)
        distances_s, indices_s = neigh_source.kneighbors(target_cloud)

        source_points_corr = source_cloud[indices_s.ravel()]
        target_points_corr = target_cloud

        centroid_source = np.mean(source_points_corr, axis=0)
        centroid_target = np.mean(target_points_corr, axis=0)

        centered_source = source_points_corr - centroid_source
        centered_target = target_points_corr - centroid_target

        H = centered_source.T @ centered_target

        U, S, Vt = np.linalg.svd(H)
        R_icp = Vt.T @ U.T

        if np.linalg.det(R_icp) < 0:
            Vt[2, :] *= -1
            R_icp = Vt.T @ U.T

        t_icp = centroid_target.T - R_icp @ centroid_source.T

        final_transform = np.eye(4)
        final_transform[:3, :3] = R_icp
        final_transform[:3, 3] = t_icp
        return final_transform

    def cloud_callback(self, msg: PointCloud2):
        # 1. PointCloud2 → numpy 변환
        current_cloud_list = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            current_cloud_list.append([point[0], point[1], point[2]])

        if not current_cloud_list:
            self.get_logger().warn("수신된 포인트 클라우드가 비어있거나 파싱 실패")
            return
        current_cloud_np = np.array(current_cloud_list)

        if self.prev_cloud_np is None or len(self.prev_cloud_np) < 10:
            self.get_logger().info("이전 클라우드 초기화 중 또는 포인트 부족")
            self.prev_cloud_np = current_cloud_np
            self.publish_identity(msg.header.stamp)
            return

        # ICP 수행 (이전 → 현재)
        delta_transform = self.icp_step(self.prev_cloud_np, current_cloud_np)
        if delta_transform is None:
            self.get_logger().warn("ICP 변환 계산 실패")
            return

        self.current_transform_matrix = self.current_transform_matrix @ delta_transform
        self.latest_transform_stamp = msg.header.stamp
        self.prev_cloud_np = current_cloud_np

    def publish_tf_and_odom(self):
        t = TransformStamped()
        current_stamp = self.get_clock().now().to_msg()
        t.header.stamp = current_stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = self.current_transform_matrix[0, 3]
        t.transform.translation.y = self.current_transform_matrix[1, 3]
        t.transform.translation.z = self.current_transform_matrix[2, 3]
        try:
            quat = R.from_matrix(self.current_transform_matrix[:3, :3]).as_quat()
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
        except Exception as e:
            self.get_logger().error(f"ICP TF 쿼터니언 변환 오류: {e}")
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            quat = [0.0, 0.0, 0.0, 1.0]
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.pose.pose.position.x = self.current_transform_matrix[0, 3]
        odom_msg.pose.pose.position.y = self.current_transform_matrix[1, 3]
        odom_msg.pose.pose.position.z = self.current_transform_matrix[2, 3]
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.pose.covariance = [
            1e-2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5e-2
        ]
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        odom_msg.twist.covariance = [-1.0] * 36

        self.odom_publisher.publish(odom_msg)

    def publish_identity(self, stamp):
        # TF, Odometry 초기화
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        if self.publish_tf:
            self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.covariance = [1e-3 if i % 7 == 0 else 0.0 for i in range(36)]
        odom_msg.twist.covariance = [-1.0] * 36
        self.odom_publisher.publish(odom_msg)

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
