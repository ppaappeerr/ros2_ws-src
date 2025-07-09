import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from rcl_interfaces.msg import SetParametersResult

# Use the standard library for PointCloud2 manipulation
import sensor_msgs_py.point_cloud2 as pc2

# Import the PCL library
import pcl

class PointCloudAccumulatorPCLVoxelNode(Node):
    def __init__(self):
        super().__init__('point_cloud_accumulator_pcl_voxel_node')

        # Declare and get parameters
        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('accumulation_period_seconds', 1.5)
        
        self.voxel_leaf_size = self.get_parameter('voxel_leaf_size').get_parameter_value().double_value
        self.accumulation_period = self.get_parameter('accumulation_period_seconds').get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.points_buffer = []

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.subscription = self.create_subscription(PointCloud2, '/dense_points', self.listener_callback, qos_profile)
        self.publisher = self.create_publisher(PointCloud2, '/accumulated_points_pcl_voxel', 10)
        self.timer = self.create_timer(self.accumulation_period, self.timer_callback)
        
        self.get_logger().info(f"PCL Voxel Accumulator started. Leaf size: {self.voxel_leaf_size}, Period: {self.accumulation_period}s.")

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'voxel_leaf_size':
                self.voxel_leaf_size = param.value
                self.get_logger().info(f"Updated voxel leaf size to {self.voxel_leaf_size}.")
            elif param.name == 'accumulation_period_seconds':
                self.accumulation_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.accumulation_period, self.timer_callback)
                self.get_logger().info(f"Updated accumulation period to {self.accumulation_period}s.")
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        """
        Decode ROS PointCloud2 message and add points to the buffer.
        """
        # Use sensor_msgs_py.point_cloud2 to read points as a generator
        # and convert to a list of lists
        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points:
            self.points_buffer.extend(points)

    def timer_callback(self):
        """
        Periodically called by the timer.
        Processes the buffered points with a VoxelGrid filter and publishes the result.
        """
        if not self.points_buffer:
            return

        # Create a PCL PointCloud object from the buffered points
        cloud_pcl = pcl.PointCloud()
        cloud_pcl.from_list(self.points_buffer)
        
        # Create a VoxelGrid filter
        vox = cloud_pcl.make_voxel_grid_filter()
        vox.set_leaf_size(self.voxel_leaf_size, self.voxel_leaf_size, self.voxel_leaf_size)
        
        # Apply the filter
        cloud_filtered = vox.filter()
        
        self.get_logger().info(f"Accumulated {len(self.points_buffer)} points, "
                               f"downsampled to {cloud_filtered.size} points.")

        # Convert the filtered PCL cloud back to a ROS PointCloud2 message
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        
        # Define the fields for the new PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Convert PCL data (which is a list of points) to a NumPy array to create the message
        points_np = np.asarray(cloud_filtered, dtype=np.float32)
        pc2_msg = pc2.create_cloud(header, fields, points_np)
        
        self.publisher.publish(pc2_msg)
        
        # Clear the buffer for the next accumulation cycle
        self.points_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    try:
        import pcl
        import sensor_msgs_py.point_cloud2
    except ImportError:
        print("="*50)
        print("ImportError: `python-pcl` or `sensor_msgs_py` not found.")
        print("Please ensure PCL is installed correctly for your ROS distribution.")
        print("e.g., sudo apt-get install ros-humble-python-pcl")
        print("="*50)
        return

    node = PointCloudAccumulatorPCLVoxelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()