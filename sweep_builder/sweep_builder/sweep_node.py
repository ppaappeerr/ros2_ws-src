import rclpy, collections, time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
import numpy as np

class SweepNode(Node):
    def __init__(self):
        super().__init__('sweep_node')

        # ───── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('in_pc_topic',  'points_3d')
        self.declare_parameter('imu_topic',    '/imu/data')
        self.declare_parameter('buffer_secs',   1.0)    # N 초간 링버퍼

        self.in_pc_topic  = self.get_parameter('in_pc_topic').value
        self.imu_topic    = self.get_parameter('imu_topic').value
        self.buffer_secs  = self.get_parameter('buffer_secs').value

        # ───── 구독/발행 ────────────────────────────────────────────
        self.sub_pc  = self.create_subscription(PointCloud2,
                                                self.in_pc_topic,
                                                self.pc_cb, 10)
        self.sub_imu = self.create_subscription(Imu,
                                                self.imu_topic,
                                                self.imu_cb, 50)

        self.pub_sweep = self.create_publisher(PointCloud2, 'sweep_cloud', 10)

        # ───── 내부 버퍼 ────────────────────────────────────────────
        self.ring = collections.deque()          # (stamp, N×3 np.ndarray)
        self.last_yaw = 0.0

    # ────────────────────────────────────────────────────────────────
    def imu_cb(self, msg: Imu):
        q = msg.orientation
        # z‑yaw only
        self.last_yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx')[0]

    # ────────────────────────────────────────────────────────────────
    def pc_cb(self, msg: PointCloud2):
        pts = np.array([[p[0], p[1], p[2]] for p in
                        pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True)],
                       dtype=np.float32)
        if pts.shape[0] == 0:
            return

        # LiDAR frame → base_link 회전(roll/pitch 이미 제거, yaw만 적용)
        Rz = R.from_euler('z', self.last_yaw).as_matrix()
        pts_bl = (Rz @ pts.T).T

        # ── 링버퍼 업데이트
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.ring.append((current_time, pts_bl))
        
        # 오래된 데이터 제거
        while self.ring and (current_time - self.ring[0][0]) > self.buffer_secs:
            self.ring.popleft()

        # ── 퍼블리시 (ring이 비어있지 않을 때만)
        if self.ring:
            ring_points = np.vstack([p for _,p in self.ring])
            self.pub_sweep.publish(self.np2msg(ring_points, msg.header, 'base_link'))

    def np2msg(self, pts: np.ndarray, hdr, frame) -> PointCloud2:
        hdr.frame_id = frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(hdr, fields, pts)

def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()