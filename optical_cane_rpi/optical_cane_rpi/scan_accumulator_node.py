import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from laser_geometry import LaserProjection
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from collections import deque
import numpy as np
from std_msgs.msg import Header
import math

class ScanAccumulatorNode(Node):
    """
    Accumulates LaserScan messages over a time window, transforms them to a common frame,
    merges them, and publishes the result as a single PointCloud2 message.
    This serves as a proper '2D-direct-accumulated' pipeline for comparison.
    """
    def __init__(self):
        super().__init__('scan_accumulator_node')

        # Parameters
        self.declare_parameter('accumulation_time_seconds', 1.5)
        self.declare_parameter('output_topic', '/scan_accumulation_cloud')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('fixed_frame', 'base_link')
        self.declare_parameter('front_view_only', True) # New parameter

        self.accumulation_time = self.get_parameter('accumulation_time_seconds').get_parameter_value().double_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.front_view_only = self.get_parameter('front_view_only').get_parameter_value().bool_value

        # Data storage using a deque
        self.scan_buffer = deque()

        # Publisher for the accumulated point cloud
        self.publisher_ = self.create_publisher(PointCloud2, output_topic, 10)

        # TF listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # LaserProjection utility
        self.laser_proj = LaserProjection()

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10)
        
        # Timer to periodically process the buffer and publish
        self.publish_timer = self.create_timer(0.1, self.process_and_publish) # Publish at 10Hz

        self.get_logger().info(f"Scan Accumulator Node started. Accumulating scans over {self.accumulation_time} seconds.")
        self.get_logger().info(f"Input: '{input_topic}', Output: '{output_topic}', Fixed Frame: '{self.fixed_frame}'")
        self.get_logger().info(f"Front view only: {self.front_view_only} (Front = -X)")


    def scan_callback(self, msg: LaserScan):
        # Add the new scan to the buffer
        self.scan_buffer.append(msg)

    def process_and_publish(self):
        if not self.scan_buffer:
            return

        now = self.get_clock().now()
        
        # Remove old scans from the buffer
        cutoff_time = (now - rclpy.duration.Duration(seconds=self.accumulation_time)).to_msg()
        while self.scan_buffer and \
              (self.scan_buffer[0].header.stamp.sec < cutoff_time.sec or \
              (self.scan_buffer[0].header.stamp.sec == cutoff_time.sec and \
               self.scan_buffer[0].header.stamp.nanosec < cutoff_time.nanosec)):
            self.scan_buffer.popleft()

        if not self.scan_buffer:
            return

        all_points = []

        for scan in list(self.scan_buffer):
            try:
                processed_scan = scan
                if self.front_view_only:
                    processed_scan = self.filter_front_view(scan)

                # Project laser scan to point cloud
                cloud = self.laser_proj.projectLaser(processed_scan)
                
                points_np = self.pc2_to_numpy(cloud)
                if points_np is not None:
                    all_points.append(points_np)

            except Exception as e:
                self.get_logger().warn(f"Could not process scan: {e}")
                continue
        
        if not all_points:
            return

        # Combine all points and create a new PointCloud2 message
        combined_points = np.vstack(all_points)
        
        header = Header(stamp=now.to_msg(), frame_id=self.fixed_frame)
        cloud_msg = self.numpy_to_pc2(header, combined_points)
        
        self.publisher_.publish(cloud_msg)
        # self.get_logger().info(f"Published accumulated cloud with {len(combined_points)} points.")

    def filter_front_view(self, scan_in: LaserScan) -> LaserScan:
        scan_out = LaserScan()
        scan_out.header = scan_in.header
        scan_out.angle_min = scan_in.angle_min
        scan_out.angle_max = scan_in.angle_max
        scan_out.angle_increment = scan_in.angle_increment
        scan_out.time_increment = scan_in.time_increment
        scan_out.scan_time = scan_in.scan_time
        scan_out.range_min = scan_in.range_min
        scan_out.range_max = scan_in.range_max

        ranges = np.array(scan_in.ranges)
        angles = np.arange(scan_in.angle_min, scan_in.angle_max + scan_in.angle_increment/2.0, scan_in.angle_increment)
        # NEW: front is -X (angle ~ pi or -pi). Keep angles whose wrapped distance to pi <= 90 deg
        wrapped_diff = np.arctan2(np.sin(angles - math.pi), np.cos(angles - math.pi))
        keep_mask = np.abs(wrapped_diff) <= (math.pi / 2.0)
        # Remove points not in new front
        ranges[~keep_mask] = float('inf')
        scan_out.ranges = ranges.tolist()
        if scan_in.intensities:
            intensities = np.array(scan_in.intensities)
            intensities[~keep_mask] = 0
            scan_out.intensities = intensities.tolist()
        return scan_out

    def pc2_to_numpy(self, cloud_msg):
        dtype_list = []
        for field in cloud_msg.fields:
            dtype_list.append((field.name, np.dtype(self.get_numpy_dtype(field.datatype))))
        
        dtype = np.dtype(dtype_list)
        data = np.frombuffer(cloud_msg.data, dtype=dtype)
        
        if 'x' in data.dtype.names and 'y' in data.dtype.names and 'z' in data.dtype.names:
            return np.vstack([data['x'], data['y'], data['z']]).T
        return None

    def numpy_to_pc2(self, header, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        itemsize = np.dtype(np.float32).itemsize
        data = points.astype(np.float32).tobytes()
        
        return PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=3 * itemsize,
            row_step=len(points) * 3 * itemsize,
            data=data
        )

    def get_numpy_dtype(self, ros_dtype):
        if ros_dtype == PointField.INT8: return np.int8
        if ros_dtype == PointField.UINT8: return np.uint8
        if ros_dtype == PointField.INT16: return np.int16
        if ros_dtype == PointField.UINT16: return np.uint16
        if ros_dtype == PointField.INT32: return np.int32
        if ros_dtype == PointField.UINT32: return np.uint32
        if ros_dtype == PointField.FLOAT32: return np.float32
        if ros_dtype == PointField.FLOAT64: return np.float64
        return np.float32

def main(args=None):
    rclpy.init(args=args)
    node = ScanAccumulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
