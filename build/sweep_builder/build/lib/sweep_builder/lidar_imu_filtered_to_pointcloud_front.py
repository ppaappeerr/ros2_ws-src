import rclpy, numpy as np, tf_transformations, struct
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointField
from scipy.spatial.transform import Rotation as R

class LidarImuToPointCloudFrontNode(Node):
    def __init__(self):
        super().__init__('lidar_imu_to_pointcloud_front_node')
        
        # 파라미터 선언 (전방 FOV 각도)
        self.declare_parameter('front_fov_deg', 180.0)  # 전방 ±90도
        
        self.front_fov_deg = self.get_parameter('front_fov_deg').value
        self.half_fov_rad = np.deg2rad(self.front_fov_deg / 2.0)  # ±90도 → ±π/2
        
        self.create_subscription(LaserScan,'/scan',self.scan_cb,10)
        self.create_subscription(Imu,'/imu/data',self.imu_cb,50)
        self.points_3d_publisher = self.create_publisher(PointCloud2,'/points_3d',10)
        self.q_imu = [0,0,0,1]
        
        self.get_logger().info(f'Front LiDAR node started - FOV: ±{self.front_fov_deg/2.0}°')

    def imu_cb(self,msg:Imu):
        self.q_imu = [msg.orientation.x,msg.orientation.y,
                      msg.orientation.z,msg.orientation.w]

    def scan_cb(self,scan:LaserScan):
        points = []
        time_increment = scan.time_increment
        angles = [scan.angle_min + i * scan.angle_increment for i in range(len(scan.ranges))]
        
        for i, (dist, angle) in enumerate(zip(scan.ranges, angles)):
            if scan.range_min < dist < scan.range_max:
                
                # **전방 FOV 필터링** (LiDAR +X축이 전방)
                # angle이 ±90도 범위 안에 있는지 확인
                if abs(angle) <= self.half_fov_rad:
                    
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
        
        # 디버그 정보 (필요시 주석 해제)
        # self.get_logger().debug(f'Published {len(points)} front points (FOV: ±{self.front_fov_deg/2.0}°)')

def main():
    rclpy.init(); rclpy.spin(LidarImuToPointCloudFrontNode()); rclpy.shutdown()

if __name__ == '__main__':
    main()