import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class ObstacleFilter(Node):
    def __init__(self):
        super().__init__('obstacle_filter_node')
        self.declare_parameter('z_min', -0.2)
        self.declare_parameter('z_max', 0.2)
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.create_subscription(PointCloud2, '/sweep_pc', self.pc_callback, 10)
        self.pub = self.create_publisher(PointCloud2, '/obstacles_pc', 10)

    def pc_callback(self, msg: PointCloud2):
        # Read incoming pointcloud
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Filter by z range
        filtered = [p for p in points if self.z_min <= p[2] <= self.z_max]

        if not filtered:
            self.get_logger().warn("Filtered pointcloud is empty.")
            return

        # Create new PointCloud2
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        pc_msg = pc2.create_cloud(header, fields, filtered)
        self.pub.publish(pc_msg)

def main():
    rclpy.init()
    node = ObstacleFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
