import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class ObstacleSegmenter(Node):
    def __init__(self):
        super().__init__('obstacle_segmenter')

        self.declare_parameter('ground_height_threshold', 0.3) # Max height from lowest point to be considered ground
        self.declare_parameter('min_ground_z', -0.5) # Minimum Z-value for ground points
        self.declare_parameter('max_ground_z', 0.1) # Maximum Z-value for ground points

        self.ground_height_threshold = self.get_parameter('ground_height_threshold').get_parameter_value().double_value
        self.min_ground_z = self.get_parameter('min_ground_z').get_parameter_value().double_value
        self.max_ground_z = self.get_parameter('max_ground_z').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            PointCloud2,
            '/optical_cane/pointcloud',
            self.pointcloud_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.ground_pub = self.create_publisher(PointCloud2, '/optical_cane/ground_cloud', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/optical_cane/obstacle_cloud', 10)

        self.get_logger().info('Obstacle Segmenter Node has been started.')

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received PointCloud2 data for segmentation.')

        # Read points from PointCloud2 message
        points = self.read_points(msg)
        if not points:
            self.get_logger().warn('No points to segment.')
            return

        points_np = np.array(points)

        # Simple ground segmentation based on Z-height
        # Filter points within a certain Z range to be considered potential ground
        ground_candidate_indices = np.where(
            (points_np[:, 2] > self.min_ground_z) & 
            (points_np[:, 2] < self.max_ground_z)
        )[0]

        ground_points = []
        obstacle_points = []

        if len(ground_candidate_indices) > 0:
            ground_candidates = points_np[ground_candidate_indices]
            # Find the lowest point among candidates to establish a reference ground level
            min_z_ground_candidate = np.min(ground_candidates[:, 2])

            for i, point in enumerate(points_np):
                # If point is within ground height threshold from the lowest ground candidate
                if (point[2] >= min_z_ground_candidate) and \
                   (point[2] <= min_z_ground_candidate + self.ground_height_threshold):
                    ground_points.append(point)
                else:
                    obstacle_points.append(point)
        else:
            # If no ground candidates, all points are considered obstacles for now
            obstacle_points = points.copy()
            self.get_logger().warn('No ground candidates found. All points treated as obstacles.')

        # Publish ground and obstacle point clouds
        self.publish_pointcloud(ground_points, msg.header, self.ground_pub)
        self.publish_pointcloud(obstacle_points, msg.header, self.obstacle_pub)

    def read_points(self, cloud_msg):
        points = []
        # Assuming XYZ fields are present and float32
        x_offset = -1
        y_offset = -1
        z_offset = -1

        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset == -1 or y_offset == -1 or z_offset == -1:
            self.get_logger().error('XYZ fields not found in PointCloud2 message.')
            return []

        point_step = cloud_msg.point_step
        row_step = cloud_msg.row_step

        for i in range(cloud_msg.width * cloud_msg.height):
            start_byte = i * point_step
            x = struct.unpack_from('<f', cloud_msg.data, start_byte + x_offset)[0]
            y = struct.unpack_from('<f', cloud_msg.data, start_byte + y_offset)[0]
            z = struct.unpack_from('<f', cloud_msg.data, start_byte + z_offset)[0]
            points.append([x, y, z])
        return points

    def publish_pointcloud(self, points, header, publisher):
        if not points:
            # self.get_logger().info(f'No points to publish for {publisher.topic_name}.')
            return

        points_np = np.array(points, dtype=np.float32)
        point_cloud_data = points_np.tobytes()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points),
            data=point_cloud_data
        )
        publisher.publish(pointcloud_msg)
        self.get_logger().info(f'Published {len(points)} points to {publisher.topic_name}.')

def main(args=None):
    rclpy.init(args=args)
    obstacle_segmenter = ObstacleSegmenter()
    rclpy.spin(obstacle_segmenter)
    obstacle_segmenter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
