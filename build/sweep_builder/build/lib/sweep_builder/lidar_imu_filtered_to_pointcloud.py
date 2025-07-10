import rclpy, numpy as np, tf_transformations, struct
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointField
from scipy.spatial.transform import Rotation as R

class LidarImuToPointCloudNode(Node):
    def __init__(self):
        super().__init__('lidar_imu_to_pointcloud_node')
        self.create_subscription(LaserScan,'/scan',self.scan_cb,10)
        self.create_subscription(Imu,'/imu/data',self.imu_cb,50)
        self.points_3d_publisher = self.create_publisher(PointCloud2,'/points_3d',10)
        self.q_imu = [0,0,0,1]

    def imu_cb(self,msg:Imu):
        self.q_imu = [msg.orientation.x,msg.orientation.y,
                      msg.orientation.z,msg.orientation.w]

    def scan_cb(self,scan:LaserScan):
        points = []
        time_increment = scan.time_increment
        angles = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
        
        for i, (dist, angle) in enumerate(zip(scan.ranges, angles)):
            if scan.range_min < dist < scan.range_max:
                # 기존 3D 좌표 계산
                point_2d = np.array([dist * np.cos(angle), dist * np.sin(angle), 0])
                point_3d_rotated = R.from_quat(self.q_imu).apply(point_2d)
                
                # 각 포인트의 상대적 시간(offset) 계산
                time_offset = i * time_increment
                
                # x, y, z와 함께 time_offset 추가
                points.append([point_3d_rotated[0], point_3d_rotated[1], point_3d_rotated[2], time_offset])

        if not points:
            return

        # PointCloud2 메시지의 필드 정의
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # PointCloud2 메시지 생성
        header = scan.header
        header.frame_id = 'laser'
        
        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,  # 4개의 float = 4 * 4 = 16 bytes
            row_step=16 * len(points),
            data=np.asarray(points, dtype=np.float32).tobytes()
        )
        self.points_3d_publisher.publish(point_cloud_msg)

def main():
    rclpy.init(); rclpy.spin(LidarImuToPointCloudNode()); rclpy.shutdown()
