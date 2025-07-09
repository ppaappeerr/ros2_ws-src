import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
from std_msgs.msg import Header
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation as R

class PointCloudGenerator(Node):
    def __init__(self):
        super().__init__('pointcloud_generator')
        
        # Declare parameters for mounting height and IMU offset
        self.declare_parameter('mounting_height', 1.5) # meters, approximate shoulder height
        self.declare_parameter('imu_offset_x', 0.0) # meters, offset from lidar origin
        self.declare_parameter('imu_offset_y', 0.0)
        self.declare_parameter('imu_offset_z', 0.0)

        self.mounting_height = self.get_parameter('mounting_height').get_parameter_value().double_value
        self.imu_offset_x = self.get_parameter('imu_offset_x').get_parameter_value().double_value
        self.imu_offset_y = self.get_parameter('imu_offset_y').get_parameter_value().double_value
        self.imu_offset_z = self.get_parameter('imu_offset_z').get_parameter_value().double_value

        self.get_logger().info(f'Mounting height: {self.mounting_height} m')
        self.get_logger().info(f'IMU offset: ({self.imu_offset_x}, {self.imu_offset_y}, {self.imu_offset_z}) m')

        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')

        self.ts = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.imu_sub], 10, 0.1)
        self.ts.registerCallback(self.data_callback)

        self.pointcloud_pub = self.create_publisher(PointCloud2, '/optical_cane/pointcloud', 10)
        self.get_logger().info('PointCloud Generator Node has been started.')

    def data_callback(self, scan_msg, imu_msg):
        self.get_logger().info('Received synchronized LaserScan and Imu data.')

        points = []
        # Get IMU orientation (quaternion)
        orientation = imu_msg.orientation
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])

        # Iterate through lidar scan points
        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment

                # Convert 2D polar to 2D Cartesian (in lidar frame)
                x_lidar = range_val * np.cos(angle)
                y_lidar = range_val * np.sin(angle)
                z_lidar = 0.0 # Lidar is 2D, so z is 0 in its own frame

                # Apply IMU rotation to the lidar point
                # Assuming lidar frame is aligned with IMU frame when IMU is at identity orientation
                # and lidar is mounted at (0,0,0) relative to IMU's origin.
                # The IMU's orientation gives the rotation from the IMU's body frame to the world frame.
                # We want to transform points from the lidar frame (which is essentially the IMU body frame
                # for rotation purposes) to the world frame.
                point_in_imu_frame = np.array([x_lidar, y_lidar, z_lidar])
                rotated_point = r.apply(point_in_imu_frame)

                # Add mounting height and IMU offset
                # Assuming the IMU's origin is at the mounting point, and the lidar is also there.
                # The mounting height is along the Z-axis of the world frame.
                # The IMU offset is relative to the mounting point in the world frame.
                x_world = rotated_point[0] + self.imu_offset_x
                y_world = rotated_point[1] + self.imu_offset_y
                z_world = rotated_point[2] + self.mounting_height + self.imu_offset_z

                points.append([x_world, y_world, z_world])

        if not points:
            self.get_logger().warn('No valid points to publish.')
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'base_link' # Or a more appropriate frame like 'world' or 'odom'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convert list of lists to a numpy array and then to bytes
        points_np = np.array(points, dtype=np.float32)
        point_cloud_data = points_np.tobytes()

        pointcloud_msg = PointCloud2(
            header=header,
            height=1, # Unordered point cloud
            width=len(points),
            is_dense=True, # No invalid points
            is_bigendian=False,
            fields=fields,
            point_step=12, # 3 floats * 4 bytes/float
            row_step=12 * len(points),
            data=point_cloud_data
        )

        self.pointcloud_pub.publish(pointcloud_msg)
        self.get_logger().info(f'Published PointCloud2 with {len(points)} points.')

def main(args=None):
    rclpy.init(args=args)
    pointcloud_generator = PointCloudGenerator()
    rclpy.spin(pointcloud_generator)
    pointcloud_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
