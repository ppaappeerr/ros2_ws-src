import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformListener, Buffer
from rclpy.duration import Duration
import numpy as np
import time

# ★★★★★ [수정] ★★★★★
# 잘못된 라이브러리 대신, 올바른 TF2 변환 라이브러리를 import 합니다.
import tf2_sensor_msgs

class SweepAccumulatorNode(Node):
    def __init__(self):
        super().__init__('sweep_accumulator_node')

        self.declare_parameter('input_topic', 'points_3d')
        self.declare_parameter('output_topic', 'sweep_cloud')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('source_frame', 'laser')
        self.declare_parameter('sweep_size', 4000)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        self.sweep_pts = self.get_parameter('sweep_size').value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_sweep = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        self.point_buffer = []
        self.last_stamp = None
        self.sub_pc = None

        self.get_logger().info(f"'{self.get_name()}' started.")
        
    def wait_for_transform(self):
        self.get_logger().info(f"Waiting for transform from '{self.target_frame}' to '{self.source_frame}'...")
        while rclpy.ok():
            try:
                self.tf_buffer.can_transform(self.target_frame, self.source_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0))
                self.get_logger().info("Transform is now available!")
                return True
            except Exception as e:
                self.get_logger().warn(f"Still waiting for transform: {e}")
        return False

    def start_subscription(self):
        self.sub_pc = self.create_subscription(PointCloud2, self.input_topic, self.pc_callback, 10)
        self.get_logger().info(f"Subscribing to '{self.input_topic}'...")

    def pc_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed in callback: {e}")
            return

        # ★★★★★ [수정] ★★★★★
        # pc2.do_transform_cloud 대신, 올바른 tf2_sensor_msgs 라이브러리의 함수를 사용합니다.
        cloud_in_odom_frame = tf2_sensor_msgs.do_transform_cloud(msg, transform)
        
        points_in_odom = pc2.read_points_numpy(cloud_in_odom_frame, field_names=('x', 'y', 'z'))
        
        self.point_buffer.extend(points_in_odom)
        self.last_stamp = msg.header.stamp

        if len(self.point_buffer) >= self.sweep_pts:
            self.publish_sweep()

    def publish_sweep(self):
        points_to_publish = np.array(self.point_buffer[:self.sweep_pts], dtype=np.float32)
        self.point_buffer = self.point_buffer[self.sweep_pts:]

        header = Header(stamp=self.last_stamp, frame_id=self.target_frame)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        sweep_msg = pc2.create_cloud(header, fields, points_to_publish)
        self.pub_sweep.publish(sweep_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SweepAccumulatorNode()
    
    # 별도의 스레드에서 실행할 필요 없이, main 스레드에서 순차적으로 실행해도 됩니다.
    if node.wait_for_transform():
        node.start_subscription()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()