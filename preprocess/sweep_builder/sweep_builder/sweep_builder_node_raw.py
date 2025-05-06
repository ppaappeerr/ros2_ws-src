# sweep_builder/sweep_builder_node_raw.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from tf_transformations import euler_from_quaternion
from builtin_interfaces.msg import Time

class SweepBuilderRaw(Node):
    def __init__(self):
        super().__init__('sweep_builder_raw')
        self.declare_parameter('accum_duration', 0.2)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 10.0)
        self.accum_dur   = self.get_parameter('accum_duration').value
        self.min_range   = self.get_parameter('min_range').value
        self.max_range   = self.get_parameter('max_range').value

        self.points = []
        self.start_time = None

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu_raw', self.imu_cb, 10)

        self.pc_pub = self.create_publisher(PointCloud2, 'sweep_pc_raw3d', 10)

        self.roll, self.pitch = 0.0, 0.0

    def imu_cb(self, msg):
        # orientation 없으므로 angular_velocity 적분
        self.roll += msg.angular_velocity.x * 0.02  # assuming 50Hz, dt ≈ 0.02s
        self.pitch += msg.angular_velocity.y * 0.02

    def scan_cb(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, num=len(msg.ranges), endpoint=False)
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = np.logical_and(ranges > self.min_range, ranges < self.max_range)
        angles = angles[valid]
        ranges = ranges[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        pts = np.vstack((xs, ys, zs))

        # roll/pitch 기반 회전 행렬 구성
        cr, sr = np.cos(self.roll), np.sin(self.roll)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        rot_mat = np.array([[cp, sr*sp, cr*sp],
                            [0.0,  cr,   -sr   ],
                            [-sp, sr*cp, cr*cp]])
        pts_rot = rot_mat @ pts

        now = msg.header.stamp
        if self.start_time is None:
            self.start_time = now
        self.points.extend(pts_rot.T.tolist())

        elapsed = (now.sec - self.start_time.sec) + (now.nanosec - self.start_time.nanosec) * 1e-9
        if elapsed >= self.accum_dur:
            self.publish_pc(self.points, now, 'laser')
            self.points.clear()
            self.start_time = None

    def publish_pc(self, points, stamp, frame_id):
        if not points:
            return
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = pc2.create_cloud(header, fields, points)
        self.pc_pub.publish(pc_msg)

def main():
    rclpy.init()
    node = SweepBuilderRaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
