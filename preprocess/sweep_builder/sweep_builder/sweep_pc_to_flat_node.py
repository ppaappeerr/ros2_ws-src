#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node         import Node
from sensor_msgs.msg    import PointCloud2, PointField
from std_msgs.msg       import Header
import sensor_msgs_py.point_cloud2 as pc2

class SweepPcToFlat(Node):
    def __init__(self):
        super().__init__('sweep_pc_to_flat')
        # 파라미터: ROI 와 down-sampling
        self.declare_parameter('z_min',   -5.0)   # 필요 시 수정
        self.declare_parameter('z_max',    5.0)
        self.declare_parameter('voxel',    0.05)  # 5 cm 그리드
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.voxel = self.get_parameter('voxel').value

        self.create_subscription(PointCloud2, '/sweep_pc',
                                 self.cb, 10)
        self.pub = self.create_publisher(PointCloud2,
                                         '/obstacles_pc_flat', 10)

    def cb(self, msg: PointCloud2):
        pts = np.array(list(pc2.read_points(
                 msg, field_names=('x','y','z'), skip_nans=True)))
        
        # z-ROI
        mask = np.logical_and(pts[:,2] >= self.z_min,
                              pts[:,2] <= self.z_max)
        pts2d = pts[mask][:,:2]                     # (N,2)

        if pts2d.size == 0:
            return

        # voxel down-sampling (hash grid)
        voxel = self.voxel
        idx = np.floor(pts2d / voxel).astype(np.int32)
        _, unique_idx = np.unique(idx, axis=0, return_index=True)
        flat = pts2d[unique_idx]

        # z=0 으로 압축
        flat3 = np.hstack([flat, np.zeros((flat.shape[0],1))])

        header = Header()
        header.stamp    = msg.header.stamp
        header.frame_id = msg.header.frame_id
        fields = [ PointField(name='x',offset=0, datatype=7,count=1),
                   PointField(name='y',offset=4, datatype=7,count=1),
                   PointField(name='z',offset=8, datatype=7,count=1) ]

        cloud = pc2.create_cloud(header, fields, flat3.tolist())
        self.pub.publish(cloud)

def main():
    rclpy.init(); rclpy.spin(SweepPcToFlat()); rclpy.shutdown()

if __name__ == '__main__':
    main()
