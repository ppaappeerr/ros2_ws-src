#!/usr/bin/env python3
"""
voxel_z_clip.py  ──  /points_3d → /points_icp
- Z-축 ±clip 범위 밖 점 제거
- VoxelGrid(leaf) 로 다운샘플 → CPU 부하 절감
"""

import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class VoxelClip(Node):
    def __init__(self):
        super().__init__('voxel_z_clip')
        # 파라미터
        self.declare_parameter('z_min', -0.20)
        self.declare_parameter('z_max',  0.20)
        self.declare_parameter('leaf',   0.05)

        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.leaf  = self.get_parameter('leaf').value

        self.sub  = self.create_subscription(PointCloud2, '/sweep_cloud', self.cb, 10)
        self.pub  = self.create_publisher(PointCloud2,  '/points_icp', 10)

    def cb(self, msg: PointCloud2):
        # PointCloud2 → numpy (x,y,z)
        pts_list = list(pc2.read_points(msg, field_names=('x', 'y', 'z')))
        if len(pts_list) == 0:
            return
        pts = np.array([ [x, y, z] for x, y, z in pts_list ], dtype=np.float32)

        # Z clip
        mask = np.logical_and(pts[:,2] > self.z_min, pts[:,2] < self.z_max)
        pts = pts[mask]
        if pts.size == 0:
            return

        # Voxel down-sample
        leaf = self.leaf
        vox = np.floor(pts / leaf).astype(np.int32)
        _, idx = np.unique(vox, axis=0, return_index=True)
        pts_ds = pts[idx]

        # Publish
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'laser'
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)]
        pc_out = pc2.create_cloud(header, fields, pts_ds)
        self.pub.publish(pc_out)

def main():
    rclpy.init()
    node = VoxelClip()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
