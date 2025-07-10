import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
from rcl_interfaces.msg import SetParametersResult

class PointCloudAccumulatorNode(Node):
    def __init__(self):
        super().__init__('point_cloud_accumulator_node')

        # Declare and get parameters
        self.declare_parameter('accumulation_time_seconds', 1.5)
        self.accumulation_time = self.get_parameter('accumulation_time_seconds').get_parameter_value().double_value
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Point buffer
        self.points_buffer = []

        # QoS Profile to match sensor data
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
            qos_profile)
        
        # Publisher for the accumulated point cloud
        self.publisher = self.create_publisher(PointCloud2, '/accumulated_points', 10)
        
        # Timer to publish the accumulated cloud periodically
        self.timer = self.create_timer(self.accumulation_time, self.timer_callback)
        
        self.get_logger().info(f"PointCloud Accumulator started. Accumulating points every {self.accumulation_time} seconds.")

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'accumulation_time_seconds':
                self.accumulation_time = param.value
                # Re-create the timer with the new period
                self.timer.cancel()
                self.timer = self.create_timer(self.accumulation_time, self.timer_callback)
                self.get_logger().info(f"Updated accumulation time to {self.accumulation_time} seconds.")
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        """
        Callback for incoming PointCloud2 messages.
        Decodes the point cloud and adds the points to the buffer.
        """
        # Read points from the incoming PointCloud2 message
        point_step = msg.point_step
        for i in range(0, len(msg.data), point_step):
            # Assuming fields are x, y, z (all float32)
            x, y, z = struct.unpack_from('fff', msg.data, i)
            self.points_buffer.append([x, y, z])

    def timer_callback(self):
        """
        Periodically called by the timer.
        Publishes the accumulated points and clears the buffer.
        """
        if not self.points_buffer:
            # self.get_logger().info('Buffer is empty, nothing to publish.')
            return

        self.get_logger().info(f"Publishing {len(self.points_buffer)} accumulated points.")

        # Create header
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='base_link' # Use the same frame_id as the source
        )

        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary blob
        point_cloud_data = np.array(self.points_buffer, dtype=np.float32).tobytes()
        
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(self.points_buffer),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12, # 3 fields * 4 bytes/field
            row_step=12 * len(self.points_buffer),
            data=point_cloud_data
        )
        
        self.publisher.publish(pc2_msg)
        
        # Clear the buffer for the next accumulation cycle
        self.points_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
