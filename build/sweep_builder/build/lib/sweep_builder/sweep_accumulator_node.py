import rclpy, collections, time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, PointField
from std_msgs.msg import Header
from std_srvs.srv import Empty
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R
import numpy as np

class SweepAccumulator(Node):
    def __init__(self):
        super().__init__('sweep_accumulator')

        # ───── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter('in_pc_topic',  'points_3d')
        self.declare_parameter('imu_topic',    '/imu/data')
        self.declare_parameter('sweep_size',    4000)      # points per sweep
        self.declare_parameter('buffer_secs',      1.0)    # N 초간 링버퍼
        self.declare_parameter('voxel_leaf',     0.1)     # [m] 누적 맵 voxel

        self.in_pc_topic  = self.get_parameter('in_pc_topic').value
        self.imu_topic    = self.get_parameter('imu_topic').value
        self.sweep_pts    = self.get_parameter('sweep_size').value
        self.buffer_secs  = self.get_parameter('buffer_secs').value
        self.voxel_leaf   = self.get_parameter('voxel_leaf').value

        # ───── 구독/발행 ────────────────────────────────────────────
        self.sub_pc  = self.create_subscription(PointCloud2,
                                                self.in_pc_topic,
                                                self.pc_cb, 10)
        self.sub_imu = self.create_subscription(Imu,
                                                self.imu_topic,
                                                self.imu_cb, 50)

        self.pub_sweep = self.create_publisher(PointCloud2, 'sweep_cloud', 10)
        self.pub_map   = self.create_publisher(PointCloud2, 'map_cloud',   3)

        self.srv_reset = self.create_service(Empty, 'reset_map', self.reset_cb)

        # ───── 내부 버퍼 ────────────────────────────────────────────
        self.ring   = collections.deque()          # (stamp, N×3 np.ndarray)
        self.map_pc = np.empty((0, 3), np.float32)

        self.last_yaw = 0.0
        self.tf_buf = Buffer()
        self.tf_lst = TransformListener(self.tf_buf, self)

        self.timer = self.create_timer(0.1, self.flush_map)

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

        # LiDAR frame → base_link 회전(roll/pitch 이미 제거, yaw만 적용)
        Rz = R.from_euler('z', self.last_yaw).as_matrix()
        pts_bl = (Rz @ pts.T).T

        # ── 링버퍼 업데이트
        self.ring.append((msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9, pts_bl))
        while self.ring and (time.time() - self.ring[0][0]) > self.buffer_secs:
            self.ring.popleft()

        # ── 누적 맵 추가 (Voxel down‑sample)
        self.map_pc = np.vstack((self.map_pc, pts_bl))
        if self.map_pc.shape[0] > 1e6:      # safety clip
            self.map_pc = self.map_pc[-500000:]

        # ── 퍼블리시
        self.pub_sweep.publish(self.np2msg(np.vstack([p for _,p in self.ring]),
                                           msg.header, 'base_link'))
        # 누적 맵은 10 Hz 미만으로 timer에서 발행

    # ────────────────────────────────────────────────────────────────
    def flush_map(self):
        if self.map_pc.shape[0] == 0:
            return
        # voxel filter
        step = int(max(1, self.voxel_leaf / 0.01))
        pc_ds = self.map_pc[::step]
        stamp = self.get_clock().now().to_msg()
        # header = Header(stamp=stamp, frame_id='map')
        # self.pub_map.publish(self.np2msg(pc_ds, header, 'map'))
        
        # frame_id를 'map'에서 'odom' 또는 'base_link'로 변경
        self.pub_map.publish(self.np2msg(pc_ds, Header(stamp=stamp, frame_id='laser'), 'laser'))

    # ────────────────────────────────────────────────────────────────
    def reset_cb(self, req, resp):
        self.map_pc = np.empty((0,3), np.float32)
        self.get_logger().info("Accumulated map cleared")
        return resp

    def np2msg(self, pts: np.ndarray, hdr, frame) -> PointCloud2:
        hdr.frame_id = frame
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(hdr, fields, pts)
        return pc2.create_cloud(hdr, fields, pts)

def main(args=None):
    rclpy.init(args=args)
    node = SweepAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
