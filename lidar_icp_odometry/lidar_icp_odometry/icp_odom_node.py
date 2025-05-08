import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import ros2_numpy
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors

class ICPOdomNode(Node):
    def __init__(self):
        super().__init__('icp_odom_node')

        self.prev_cloud = None
        self.current_pose = np.eye(4)

        self.subscription = self.create_subscription(
            PointCloud2,
            'accumulated_points',
            self.cloud_callback,
            10
        )

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("ICP Odometry Node Started")

    def cloud_callback(self, msg):
        cloud = ros2_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)

        if self.prev_cloud is None:
            self.prev_cloud = cloud
            return

        T = self.icp(self.prev_cloud, cloud)
        self.current_pose = self.current_pose @ T

        self.publish_odometry(msg.header.stamp)

        self.prev_cloud = cloud

    def icp(self, A, B, max_iterations=20, tolerance=1e-6):
        src = np.copy(A)
        dst = np.copy(B)

        T_total = np.eye(4)

        for _ in range(max_iterations):
            nbrs = NearestNeighbors(n_neighbors=1).fit(dst)
            distances, indices = nbrs.kneighbors(src)

            matched_dst = dst[indices[:,0]]

            src_mean = np.mean(src, axis=0)
            dst_mean = np.mean(matched_dst, axis=0)

            src_centered = src - src_mean
            dst_centered = matched_dst - dst_mean

            W = np.dot(dst_centered.T, src_centered)
            U, _, VT = np.linalg.svd(W)

            R_icp = np.dot(U, VT)
            t_icp = dst_mean.T - R_icp @ src_mean.T

            T_iter = np.eye(4)
            T_iter[:3, :3] = R_icp
            T_iter[:3, 3] = t_icp

            src = (R_icp @ src.T).T + t_icp

            if np.linalg.norm(T_iter - np.eye(4)) < tolerance:
                break

            T_total = T_iter @ T_total

        return T_total

    def publish_odometry(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.current_pose[0, 3]
        odom_msg.pose.pose.position.y = self.current_pose[1, 3]
        odom_msg.pose.pose.position.z = self.current_pose[2, 3]

        quat = R.from_matrix(self.current_pose[:3,:3]).as_quat()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ICPOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
