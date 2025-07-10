import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import collections
from rcl_interfaces.msg import SetParametersResult

class SweepNode(Node):
    def __init__(self):
        super().__init__('sweep_node')

        # Declare and get parameters
        self.declare_parameter('window_size_seconds', 1.5)
        self.window_size = self.get_parameter('window_size_seconds').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Ring buffer using deque: stores (timestamp, list_of_points)
        self.ring_buffer = collections.deque()

        # QoS Profile to match sensor data (BEST_EFFORT is common for sensors)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber to the dense point cloud from the fusion node
        self.subscription = self.create_subscription(
            PointCloud2,
            '/dense_points',
            self.listener_callback,
            qos_profile) # Apply QoS to subscriber
        
        # Publisher for the accumulated point cloud
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/accumulated_points_sliding', 
            qos_profile) # Apply QoS to publisher
        
        self.get_logger().info(f"Sliding Window Accumulator started. Window size: {self.window_size} seconds.")

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'window_size_seconds':
                self.window_size = param.value
                self.get_logger().info(f"Updated window size to {self.window_size} seconds.")
                # Clear buffer on parameter change to avoid inconsistent data
                self.ring_buffer.clear()
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        """
        Callback for incoming PointCloud2 messages.
        Adds new points to the ring buffer, removes old ones, and publishes the result.
        """
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Decode points from message
        points_in_msg = []
        point_step = msg.point_step
        for i in range(0, len(msg.data), point_step):
            x, y, z = struct.unpack_from('fff', msg.data, i)
            points_in_msg.append([x, y, z])
        
        if not points_in_msg:
            return

        # Add new data to the ring buffer
        self.ring_buffer.append((current_time_sec, points_in_msg))

        # Remove old data from the left of the buffer
        while self.ring_buffer and (current_time_sec - self.ring_buffer[0][0]) > self.window_size:
            self.ring_buffer.popleft()

        # --- Publish the accumulated cloud ---
        if not self.ring_buffer:
            return

        # Combine all points from the buffer
        all_points = [point for ts, points_list in self.ring_buffer for point in points_list]
        
        if not all_points:
            return

        # Create header
        header = Header(
            stamp=msg.header.stamp, # Use the latest timestamp
            frame_id='base_link'
        )

        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary blob
        point_cloud_data = np.array(all_points, dtype=np.float32).tobytes()
        
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(all_points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(all_points),
            data=point_cloud_data
        )
        
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
