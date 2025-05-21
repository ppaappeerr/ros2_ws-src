import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ScanToFlatObstaclePC(Node):
    def __init__(self):
        super().__init__('scan_to_obstacles_pc_flat')
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('quadrant', 'all')  # 여기서 파라미터 선언 필요
        ``
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/obstacles_pc_flat', 10)

    def cb(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, num=len(msg.ranges), endpoint=False)
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = np.logical_and(ranges > self.min_range, ranges < self.max_range)

        angles = angles[valid]
        ranges = ranges[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        # 사분면 필터 적용 (cloud 생성 전에 적용해야 함)
        quadrant = self.get_parameter('quadrant').get_parameter_value().string_value
        if quadrant == 'front':
            mask = xs > 0
            xs = xs[mask]; ys = ys[mask]; zs = zs[mask]
        elif quadrant == 'rear':
            mask = xs < 0
            xs = xs[mask]; ys = ys[mask]; zs = zs[mask]
        # 필요 시 좌/우도 추가 가능

        # 필터링 후에 포인트 생성 및 발행
        points = np.vstack((xs, ys, zs)).T.tolist()

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        cloud = pc2.create_cloud(header, fields, points)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ScanToFlatObstaclePC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()