#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_multiply, quaternion_conjugate, quaternion_from_euler
import struct
from std_msgs.msg import Header
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        # Subscribers with message_filters for synchronization
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        
        # ApproximateTimeSynchronizer to handle messages that are not perfectly aligned
        self.ts = message_filters.ApproximateTimeSynchronizer([
            self.scan_sub, self.imu_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher for fused 3D cloud
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/dense_points', 10)
        
        # NEW parameters for shoulder mount adaptation
        self.declare_parameter('front_is_negative_x', True)          # global front = -X
        self.declare_parameter('use_roll_pitch_only', True)          # ignore yaw drift
        self.declare_parameter('yaw_offset_deg', 0.0)                # manual fine alignment
        self.declare_parameter('max_roi_distance', 2.0)              # radial ROI
        self.declare_parameter('dead_zone_box', [-0.30, 0.15, -0.20, 0.35])  # xmin,xmax,ymin,ymax near body
        self.declare_parameter('min_range_clip', 0.05)               # clip too-near returns
        self.front_neg_x = self.get_parameter('front_is_negative_x').get_parameter_value().bool_value
        self.use_rp_only = self.get_parameter('use_roll_pitch_only').get_parameter_value().bool_value
        self.yaw_off = math.radians(self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)
        self.max_roi = self.get_parameter('max_roi_distance').get_parameter_value().double_value
        dz = self.get_parameter('dead_zone_box').get_parameter_value().double_array_value
        self.dead_zone = (dz[0], dz[1], dz[2], dz[3])
        self.min_range_clip = self.get_parameter('min_range_clip').get_parameter_value().double_value
        self.get_logger().info('Sensor Fusion Node started (shoulder-adapted). Front=-X=%s ROI=%.2fm' % (self.front_neg_x, self.max_roi))

    def sync_callback(self, scan_msg, imu_msg):
        """
        Callback function for synchronized scan and imu messages.
        """
        # Build point list
        points_3d = []
        # IMU orientation
        q = [imu_msg.orientation.x, imu_msg.orientation.y,
             imu_msg.orientation.z, imu_msg.orientation.w]
        # Optionally only roll/pitch
        if self.use_rp_only:
            r, p, _ = euler_from_quaternion(q)
            q_use = quaternion_from_euler(r, p, 0.0)
        else:
            q_use = q
        q_conj = quaternion_conjugate(q_use)
        # Yaw offset quaternion
        q_yaw = quaternion_from_euler(0.0, 0.0, self.yaw_off)
        q_yaw_conj = quaternion_conjugate(q_yaw)
        xmin, xmax, ymin, ymax = self.dead_zone
        for i, distance in enumerate(scan_msg.ranges):
            if (math.isinf(distance) or math.isnan(distance) or
                distance < max(self.min_range_clip, scan_msg.range_min) or
                distance > scan_msg.range_max):
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            p_quat = [x, y, z, 0.0]
            # Apply roll/pitch (and yaw if kept)
            p_rp = quaternion_multiply(q_use, quaternion_multiply(p_quat, q_conj))
            # Apply yaw offset
            p_adj = quaternion_multiply(q_yaw, quaternion_multiply([p_rp[0], p_rp[1], p_rp[2], 0.0], q_yaw_conj))
            px, py, pz = p_adj[0], p_adj[1], p_adj[2]
            # Flip X if front is -X
            if self.front_neg_x:
                px = -px
            # ROI radial filter
            if (px * px + py * py) > (self.max_roi * self.max_roi):
                continue
            # Dead-zone filter (near body/shoulder/face)
            if xmin <= px <= xmax and ymin <= py <= ymax:
                continue
            points_3d.append([px, py, pz])
        if not points_3d:
            return
        header = Header(stamp=scan_msg.header.stamp, frame_id='base_link')
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        flat = np.array(points_3d, dtype=np.float32).flatten()
        point_cloud_data = struct.pack('%df' % flat.size, *flat)
        pc2_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points_3d),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
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
