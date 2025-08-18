#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate
import struct
from std_msgs.msg import Header

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Subscribers with message_filters for synchronization
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        
        # ApproximateTimeSynchronizer to handle messages that are not perfectly aligned
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.scan_sub, self.imu_sub], 
            queue_size=10, 
            slop=0.1  # 100ms tolerance
        )
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher for the 3D point cloud
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/dense_points', 10)
        
        self.get_logger().info('Sensor Fusion Node has been started.')

    def sync_callback(self, scan_msg, imu_msg):
        """
        Callback function for synchronized scan and imu messages.
        """
        points_3d = []
        
        # Get the orientation from the IMU message
        orientation_q = imu_msg.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        q_conjugate = quaternion_conjugate(q)

        # Process each point in the laser scan
        for i, distance in enumerate(scan_msg.ranges):
            # Ignore invalid range values
            if np.isinf(distance) or np.isnan(distance) or distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue

            # Calculate the angle for the current point
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Convert 2D polar coordinates to 2D Cartesian coordinates
            # NOTE: In ROS, the X-axis is forward, Y is left, so we adjust accordingly.
            # A standard polar to cartesian would be x = r*cos(a), y = r*sin(a).
            # For a typical LiDAR, angle 0 is forward (X-axis).
            x = distance * np.cos(angle)
            y = distance * np.sin(angle)
            z = 0.0

            # Represent the 2D point as a 3D vector (as a pure quaternion)
            p_quat = [x, y, z, 0.0]
            
            # Rotate the point using the IMU's orientation
            # p' = q * p * q_conjugate
            p_rotated_quat = quaternion_multiply(q, quaternion_multiply(p_quat, q_conjugate))
            
            # Extract the 3D coordinates from the rotated quaternion
            points_3d.append(p_rotated_quat[:3])

        # Create and publish the PointCloud2 message
        if points_3d:
            header = Header(
                stamp=scan_msg.header.stamp,
                frame_id='base_link' # Or a more appropriate frame
            )
            
            # Define the fields for the PointCloud2 message
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            # Pack the 3D points into a binary blob
            point_cloud_data = struct.pack('%df' % (len(points_3d) * 3), *np.array(points_3d).flatten())
            
            pc2_msg = PointCloud2(
                header=header,
                height=1,
                width=len(points_3d),
                is_dense=True,
                is_bigendian=False,
                fields=fields,
                point_step=12, # 3 (x,y,z) * 4 (bytes per float)
                row_step=12 * len(points_3d),
                data=point_cloud_data
            )
            
            self.point_cloud_pub.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
