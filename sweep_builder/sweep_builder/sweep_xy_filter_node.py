# sweep_xy_filter_node.py
import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class XYFilter(Node):
    def __init__(self):
        super().__init__('sweep_xy_filter')
        self.sub = self.create_subscription(PointCloud2, '/sweep_cloud', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2,  '/sweep_xy',     10)
        self.z_range = 0.05          # ± 5 cm
        self.leaf   = 0.05           # 5 cm voxel

    def cb(self, msg):
        pts_raw = list(pc2.read_points(msg, skip_nans=True))
        if len(pts_raw) == 0:
            return
        pts = np.array([[p[0], p[1], p[2]] for p in pts_raw], dtype=np.float32)
        if pts.size == 0:
            return
        # z‑clip
        mask = np.abs(pts[:,2]) < self.z_range
        pts   = pts[mask]
        if pts.size == 0:
            return
        # very simple voxel grid
        idx = np.floor(pts[:,:3]/self.leaf).astype(np.int32)
        _, uniq = np.unique(idx, axis=0, return_index=True)
        pts = pts[uniq]

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, pts[:,:3]))

def main():
    rclpy.init()
    rclpy.spin(XYFilter())
    rclpy.shutdown()
