#!/usr/bin/env python3
# scan_to_pointcloud_node.py
# (Input: /imu_filtered for Roll/Pitch only to create 3D points in base_link frame. Yaw is NOT applied here.)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from std_msgs.msg import Header
import numpy as np
import math
import sensor_msgs_py.point_cloud2 as pc2
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ScanToPointcloudRPOnlyFiltered(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud_rp_only_filtered')

        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('input_imu_topic', '/imu_filtered') # Using /imu_filtered
        self.declare_parameter('output_pc_topic', '/pc_3d')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('euler_axes_convention', 'sxyz') 

        self.scan_topic_name = self.get_parameter('input_scan_topic').value
        self.imu_topic_name = self.get_parameter('input_imu_topic').value
        self.pc_topic_name = self.get_parameter('output_pc_topic').value
        self.output_frame = self.get_parameter('output_frame_id').value
        self.euler_axes = self.get_parameter('euler_axes_convention').value

        self.get_logger().info(
            f"ScanToPointcloudRPOnlyFiltered Node Started.\n"
            f"  Input Scan: '{self.scan_topic_name}'\n"
            f"  Input IMU: '{self.imu_topic_name}' (Using Roll/Pitch for 3D Z-value in base_link)\n"
            f"  Output PC: '{self.pc_topic_name}' (Frame: '{self.output_frame}')\n"
            f"  Note: Yaw rotation is NOT applied by this node; it should be handled by TF (e.g., odom->base_link)."
        )

        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=5 
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic_name, self.scan_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic_name, self.imu_callback, qos_profile_reliable)

        self.pc_pub = self.create_publisher(PointCloud2, self.pc_topic_name, qos_profile_reliable)

        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        self.imu_data_valid = False
        self.last_imu_update_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        current_orientation_xyzw = np.array([q.x, q.y, q.z, q.w])
        norm = np.linalg.norm(current_orientation_xyzw)

        if norm < 1e-6:
            if self.imu_data_valid:
                self.get_logger().warn(f"IMU filtered quat norm too small. Using last R/P.", throttle_duration_sec=1.0)
            return

        try:
            roll_rad, pitch_rad, yaw_rad_from_imu = euler_from_quaternion(current_orientation_xyzw / norm, axes=self.euler_axes)

            self.current_roll_rad = roll_rad
            self.current_pitch_rad = pitch_rad

            self.last_imu_update_time = self.get_clock().now()
            if not self.imu_data_valid:
                self.get_logger().info(f"First valid IMU R/P for 3D (deg): R={math.degrees(self.current_roll_rad):.1f}, P={math.degrees(self.current_pitch_rad):.1f}")
            self.imu_data_valid = True

            self.get_logger().debug(f"IMU RPY (deg): R_used={math.degrees(self.current_roll_rad):.1f}, P_used={math.degrees(self.current_pitch_rad):.1f}, Y_from_IMU(ignored)={math.degrees(yaw_rad_from_imu):.1f}", throttle_duration_sec=0.1)

        except Exception as e:
            self.get_logger().error(f"Error converting IMU quat to Euler: {e}. Using R=0, P=0.")
            self.current_roll_rad = 0.0
            self.current_pitch_rad = 0.0
            self.imu_data_valid = False

    def scan_callback(self, scan_msg: LaserScan):
        current_time = self.get_clock().now()
        time_since_last_imu = (current_time - self.last_imu_update_time).nanoseconds / 1e9

        roll_to_apply = 0.0
        pitch_to_apply = 0.0

        if self.imu_data_valid and time_since_last_imu <= 0.5: 
            roll_to_apply = self.current_roll_rad
            pitch_to_apply = self.current_pitch_rad
        else:
            if not self.imu_data_valid:
                self.get_logger().warn("No valid IMU data for Roll/Pitch. Points will be flat (Z=0).", throttle_duration_sec=1.0)
            elif time_since_last_imu > 0.5:
                 self.get_logger().warn(f"IMU data is too old ({time_since_last_imu:.2f}s). Points will be flat (Z=0).", throttle_duration_sec=1.0)

        cp = math.cos(pitch_to_apply)
        sp = math.sin(pitch_to_apply)
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])

        cr = math.cos(roll_to_apply)
        sr = math.sin(roll_to_apply)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

        rotation_matrix_rp_only = Ry @ Rx 

        points_data = []
        current_angle = scan_msg.angle_min

        for i, r_val in enumerate(scan_msg.ranges):
            if not (scan_msg.range_min < r_val < scan_msg.range_max and np.isfinite(r_val)):
                current_angle += scan_msg.angle_increment
                continue

            x_laser = r_val * math.cos(current_angle)
            y_laser = r_val * math.sin(current_angle)
            z_laser = 0.0 
            point_in_laser_frame = np.array([x_laser, y_laser, z_laser])

            point_in_base_link_frame = rotation_matrix_rp_only.dot(point_in_laser_frame)

            px = float(point_in_base_link_frame[0])
            py = float(point_in_base_link_frame[1])
            pz = float(point_in_base_link_frame[2])

            intensity_val = 0.0
            if hasattr(scan_msg, 'intensities') and scan_msg.intensities and i < len(scan_msg.intensities):
                if np.isfinite(scan_msg.intensities[i]):
                    intensity_val = float(scan_msg.intensities[i])

            points_data.append([px, py, pz, intensity_val])
            current_angle += scan_msg.angle_increment

        if not points_data:
            return

        pc_header = Header(stamp=scan_msg.header.stamp, frame_id=self.output_frame)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        try:
            cloud_msg = pc2.create_cloud(pc_header, fields, points_data)
            self.pc_pub.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to create or publish PointCloud2: {e}")

def destroy_node(self): # 추가: 명시적 종료 로그
    self.get_logger().info("Shutting down ScanToPointcloudRPOnlyFiltered node.")
    super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointcloudRPOnlyFiltered()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted.")
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()