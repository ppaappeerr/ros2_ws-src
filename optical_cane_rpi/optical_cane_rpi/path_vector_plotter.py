import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
import math
import numpy as np

class PathVectorPlotterNode(Node):
    def __init__(self):
        super().__init__('path_vector_plotter_node')
        
        # Publishers for plotting data
        self.angle_publisher = self.create_publisher(Float64, '/plot/path_angle', 10)
        self.velocity_publisher = self.create_publisher(Float64, '/plot/path_angular_velocity', 10)

        # Subscribers to both 2D and 3D path planner outputs
        self.create_subscription(Vector3Stamped, '/safe_path_vector', self.vector_callback, 10)
        self.create_subscription(Vector3Stamped, '/safe_path_vector_3d', self.vector_callback, 10)

        self.last_angle = None
        self.last_time = None

        self.get_logger().info("Path Vector Plotter Node has been started.")
        self.get_logger().info("Listening on /safe_path_vector and /safe_path_vector_3d.")
        self.get_logger().info("Publishing to /plot/path_angle and /plot/path_angular_velocity.")

    def vector_callback(self, msg):
        current_time = self.get_clock().now()
        
        # --- Calculate Angle ---
        vector = msg.vector
        current_angle = math.atan2(vector.y, vector.x)
        
        angle_msg = Float64()
        angle_msg.data = math.degrees(current_angle)
        self.angle_publisher.publish(angle_msg)

        # --- Calculate Angular Velocity ---
        if self.last_angle is not None and self.last_time is not None:
            time_diff = (current_time - self.last_time).nanoseconds / 1e9
            
            if time_diff > 1e-6: # Avoid division by zero
                # Calculate shortest angle difference to handle wrap-around from +180 to -180 deg
                angle_diff = current_angle - self.last_angle
                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                if angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                angular_velocity = angle_diff / time_diff
                
                velocity_msg = Float64()
                velocity_msg.data = math.degrees(angular_velocity)
                self.velocity_publisher.publish(velocity_msg)

        self.last_angle = current_angle
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PathVectorPlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
