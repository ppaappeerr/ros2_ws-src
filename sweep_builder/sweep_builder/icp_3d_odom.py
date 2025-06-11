import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d
from geometry_msgs.msg import Pose, Twist

class ICP3DOdom(Node):
    def __init__(self):
        super().__init__('icp_3d_odom')
        self.pc_sub = self.create_subscription(PointCloud2, '/points_3d', self.pc_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/lio_odom', 10)
        self.prev_cloud = None
        self.current_pose = np.eye(4)

    def pc_callback(self, pc_msg):
        points = np.array(list(point_cloud2.read_points(pc_msg, field_names=("x","y","z"), skip_nans=True)))
        current_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

        if self.prev_cloud is not None:
            reg = o3d.pipelines.registration.registration_icp(
                current_cloud, self.prev_cloud, 0.5,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            self.current_pose = self.current_pose @ np.linalg.inv(reg.transformation)
            self.publish_odom(pc_msg.header)

        self.prev_cloud = current_cloud

    def publish_odom(self, header):
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.current_pose[0,3]
        odom.pose.pose.position.y = self.current_pose[1,3]
        odom.pose.pose.position.z = self.current_pose[2,3]

        odom.twist.twist = Twist()  # Velocity info (optional)
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = ICP3DOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
