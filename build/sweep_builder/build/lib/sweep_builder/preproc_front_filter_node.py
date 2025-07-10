#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node         import Node
from sensor_msgs.msg    import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

class FrontFilter(Node):
    def __init__(self):
        super().__init__('front_filter')
        self.declare_parameter('in_topic',  '/points_3d')
        self.declare_parameter('out_topic', '/front_cloud')
        self.declare_parameter('fov_deg',   180.0)   # 전방 ±90°
        self.declare_parameter('leaf',      0.05)    # [m] voxel leaf

        self.half_cos = np.cos(np.deg2rad(self.get_parameter('fov_deg').value/2))
        self.leaf     = self.get_parameter('leaf').value
        self.sub  = self.create_subscription(PointCloud2,
                                             self.get_parameter('in_topic').value,
                                             self.cb, 10)
        self.pub  = self.create_publisher(PointCloud2,
                                          self.get_parameter('out_topic').value, 10)

    # ─────────────────────────────────────────────────────────────
    def cb(self, msg: PointCloud2):
        pts = np.asarray([p[:3] for p in pc2.read_points(msg,
                        field_names=('x','y','z'), skip_nans=True)],
                        dtype=np.float32)
        if pts.size == 0:  return

        # 전방 ±FOV 필터 (laser +X 축이 전방)
        dir_norm = np.linalg.norm(pts[:, :2], axis=1, keepdims=True)
        front_idx = (dir_norm.squeeze() > 0) & \
                    (pts[:,0] / dir_norm.squeeze() > self.half_cos)
        pts = pts[front_idx]

        # 초간단 Voxel down‑sample
        if self.leaf > 0:
            grid = np.floor(pts / self.leaf)
            _, idx = np.unique(grid, axis=0, return_index=True)
            pts = pts[idx]

        fields = [PointField(name=n, offset=i*4,
                             datatype=PointField.FLOAT32, count=1)
                  for i,n in enumerate(('x','y','z'))]
        self.pub.publish(pc2.create_cloud(msg.header, fields, pts))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FrontFilter())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
