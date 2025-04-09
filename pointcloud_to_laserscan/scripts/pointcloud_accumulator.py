import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')

        # QoS 설정: 여기서는 Best Effort + KEEP_LAST(10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # /cloud 구독
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud',
            self.listener_callback,
            qos_profile
        )

        # /accumulated_cloud 발행
        self.publisher = self.create_publisher(
            PointCloud2,
            '/accumulated_cloud',
            qos_profile
        )

        # 누적할 포인트들
        self.accumulated_points = []

    def listener_callback(self, msg):
        # 새 프레임에서 포인트 추출
        points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        self.accumulated_points.extend(points)

        # 헤더 재생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # 'laser' 등

        # 누적된 포인트 → PointCloud2
        cloud_out = pc2.create_cloud_xyz32(header, self.accumulated_points)
        self.publisher.publish(cloud_out)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

