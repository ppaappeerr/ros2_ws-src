import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
from tf_transformations import euler_from_quaternion
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import math

class AngleSweeperNode(Node):
    def __init__(self):
        super().__init__('angle_sweeper_node')

        # Parameters
        self.declare_parameter('angle_threshold_deg', 30.0)
        self.angle_threshold_rad = math.radians(self.get_parameter('angle_threshold_deg').get_parameter_value().double_value)

        # QoS Profiles - MUST match the publishers (default is RELIABLE)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        # Subscribers
        self.pc_sub = self.create_subscription(PointCloud2, '/dense_points', self.pc_callback, qos_reliable)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, qos_reliable)
        
        # Publisher
        self.publisher = self.create_publisher(PointCloud2, '/angle_sweep', 10)

        # Internal state
        self.points_buffer = []
        self.last_yaw = None
        self.start_yaw = None
        
        self.get_logger().info(f"Angle Sweeper started. Threshold: {self.get_parameter('angle_threshold_deg').get_parameter_value().double_value} degrees.")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        # Get yaw from quaternion
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.last_yaw = yaw

        if self.start_yaw is None:
            self.start_yaw = yaw

    def pc_callback(self, msg: PointCloud2):
        if self.last_yaw is None or self.start_yaw is None:
            self.get_logger().warn("Waiting for initial IMU data...", once=True)
            return

        # Add points to buffer
        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points:
            self.points_buffer.extend(points)

        # Check if angle threshold is reached
        delta_yaw = abs(self.last_yaw - self.start_yaw)
        # Handle angle wrap around (e.g., from +pi to -pi)
        if delta_yaw > math.pi:
            delta_yaw = 2 * math.pi - delta_yaw

        if delta_yaw >= self.angle_threshold_rad:
            if not self.points_buffer:
                return

            self.get_logger().info(f"Angle threshold reached ({math.degrees(delta_yaw):.2f} deg). Publishing sweep with {len(self.points_buffer)} points.")

            # Create and publish PointCloud2 message
            header = msg.header
            header.stamp = self.get_clock().now().to_msg()
            pc2_msg = pc2.create_cloud_xyz32(header, self.points_buffer)
            self.publisher.publish(pc2_msg)

            # Reset for the next sweep
            self.points_buffer.clear()
            self.start_yaw = self.last_yaw

def main(args=None):
    rclpy.init(args=args)
    node = AngleSweeperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
