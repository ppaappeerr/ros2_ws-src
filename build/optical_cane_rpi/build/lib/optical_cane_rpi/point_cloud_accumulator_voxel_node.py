import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import open3d as o3d
from rcl_interfaces.msg import SetParametersResult

class PointCloudAccumulatorVoxelNode(Node):
    def __init__(self):
        super().__init__('point_cloud_accumulator_voxel_node')

        # Declare and get parameters
        self.declare_parameter('voxel_size', 0.05)  # 5cm voxel size
        self.declare_parameter('publish_period_seconds', 1.5)
        
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.publish_period = self.get_parameter('publish_period_seconds').get_parameter_value().double_value
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        # The VoxelGrid that will store the accumulated points
        self.voxel_grid = o3d.geometry.VoxelGrid()

        # QoS Profile to match sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/dense_points',
            self.listener_callback,
            qos_profile)
        
        # Publisher
        self.publisher = self.create_publisher(PointCloud2, '/accumulated_points_voxel', 10)
        
        # Timer to publish the accumulated cloud periodically
        self.timer = self.create_timer(self.publish_period, self.timer_callback)
        
        self.get_logger().info(f"Voxel Accumulator started. Voxel size: {self.voxel_size}, Publish period: {self.publish_period}s.")

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'voxel_size':
                self.voxel_size = param.value
                self.get_logger().info(f"Updated voxel size to {self.voxel_size}.")
            elif param.name == 'publish_period_seconds':
                self.publish_period = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.publish_period, self.timer_callback)
                self.get_logger().info(f"Updated publish period to {self.publish_period}s.")
        # Clear grid on parameter change
        self.voxel_grid.clear()
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        """
        Callback for incoming PointCloud2 messages.
        Adds new points to the VoxelGrid.
        """
        # Decode points from message
        points_in_msg = []
        point_step = msg.point_step
        for i in range(0, len(msg.data), point_step):
            x, y, z = struct.unpack_from('fff', msg.data, i)
            points_in_msg.append([x, y, z])
        
        if not points_in_msg:
            return

        # Create an Open3D PointCloud object and add it to the voxel grid
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_in_msg)
        
        # This is not a direct addition, but a process of downsampling.
        # To "accumulate", we need to add points and then downsample.
        # A more correct way is to add all points and then voxelize.
        # Let's convert the current voxel grid to points, add new points, and re-voxelize.
        
        # 1. Get existing points from the grid
        existing_points = self.voxel_grid.get_voxels()
        existing_points_np = np.array([self.voxel_grid.get_voxel_center_coordinate(v.grid_index) for v in existing_points])

        # 2. Combine with new points
        if existing_points_np.size > 0:
            combined_points = np.vstack((existing_points_np, np.array(points_in_msg)))
        else:
            combined_points = np.array(points_in_msg)

        # 3. Create a new point cloud and generate a new voxel grid
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(combined_points)
        self.voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(new_pcd, voxel_size=self.voxel_size)


    def timer_callback(self):
        """
        Periodically called by the timer.
        Publishes the points stored in the VoxelGrid.
        """
        voxels = self.voxel_grid.get_voxels()
        if not voxels:
            return

        # Get the center of each voxel to represent the point cloud
        points_to_publish = np.array([self.voxel_grid.get_voxel_center_coordinate(v.grid_index) for v in voxels])
        
        self.get_logger().info(f"Publishing {len(points_to_publish)} voxel-downsampled points.")

        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='base_link'
        )
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud_data = points_to_publish.astype(np.float32).tobytes()
        
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points_to_publish),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points_to_publish),
            data=point_cloud_data
        )
        
        self.publisher.publish(pc2_msg)
        # Unlike the other methods, we don't clear the grid here.
        # It represents the map we've built so far.
        # To make it a "sliding window" voxel grid, we would need to store timestamps in the voxels,
        # which is much more complex. This version is a "global" accumulator.

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulatorVoxelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
