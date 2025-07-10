import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class AccumulatorNode(Node):
    def __init__(self):
        super().__init__('accumulator_node')

        # Parameters
        self.declare_parameter('target_hz', 2.0)
        self.declare_parameter('input_hz', 12.0) # Approximate input frequency
        
        self.target_hz = self.get_parameter('target_hz').get_parameter_value().double_value
        self.input_hz = self.get_parameter('input_hz').get_parameter_value().double_value
        self.recalculate_params()
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Internal buffer and counter
        self.points_buffer = []
        self.msg_counter = 0

        # QoS Profile
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriber to the raw 3D point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/dense_points', # Subscribing to the direct output of the fusion node
            self.listener_callback,
            qos_profile)
        
        # Publisher for the accumulated "sweep"
        self.publisher = self.create_publisher(PointCloud2, '/accumulated_sweep', 10)
        
        self.get_logger().info(f"Accumulator started. Target: {self.target_hz}Hz. "
                               f"Will accumulate {self.accumulation_count} messages per sweep.")

    def recalculate_params(self):
        # Calculate how many messages to accumulate to reach the target Hz
        if self.target_hz > 0:
            self.accumulation_count = max(1, int(self.input_hz / self.target_hz))
        else:
            self.accumulation_count = 1000 # Effectively disabled
        self.get_logger().info(f"New accumulation count: {self.accumulation_count}")


    def parameters_callback(self, params):
        for param in params:
            if param.name == 'target_hz':
                self.target_hz = param.value
            elif param.name == 'input_hz':
                self.input_hz = param.value
        self.recalculate_params()
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        # Add points from the message to the buffer
        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points:
            self.points_buffer.extend(points)
        
        self.msg_counter += 1

        # If we have accumulated enough messages, publish and reset
        if self.msg_counter >= self.accumulation_count:
            if not self.points_buffer:
                return

            header = msg.header
            header.stamp = self.get_clock().now().to_msg()
            
            # Create PointCloud2 message from the buffer
            pc2_msg = pc2.create_cloud_xyz32(header, self.points_buffer)
            
            self.publisher.publish(pc2_msg)
            
            # Reset for the next sweep
            self.points_buffer.clear()
            self.msg_counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = AccumulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()