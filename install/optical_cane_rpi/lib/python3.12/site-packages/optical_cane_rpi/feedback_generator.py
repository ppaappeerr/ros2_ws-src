import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from optical_cane_rpi.msg import VibrationCommand
import numpy as np
import struct

class FeedbackGenerator(Node):
    # Define command types
    STOP = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    FORWARD = 3

    def __init__(self):
        super().__init__('feedback_generator')

        self.declare_parameter('obstacle_distance_threshold', 1.0) # meters
        self.declare_parameter('front_angle_threshold', 30.0) # degrees, half angle for front sector

        self.obstacle_distance_threshold = self.get_parameter('obstacle_distance_threshold').get_parameter_value().double_value
        self.front_angle_threshold_rad = np.deg2rad(self.get_parameter('front_angle_threshold').get_parameter_value().double_value)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/optical_cane/obstacle_cloud',
            self.obstacle_cloud_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.vibration_pub = self.create_publisher(VibrationCommand, '/optical_cane/vibration_command', 10)

        self.get_logger().info('Feedback Generator Node has been started.')

    def obstacle_cloud_callback(self, msg):
        self.get_logger().info('Received obstacle PointCloud2 data.')

        points = self.read_points(msg)
        if not points:
            self.get_logger().info('No obstacle points received.')
            self.publish_vibration_command(self.FORWARD, 0.0, 0.0) # No obstacles, move forward
            return

        points_np = np.array(points)

        # Filter points within the obstacle distance threshold
        close_obstacles = points_np[np.linalg.norm(points_np[:, :2], axis=1) < self.obstacle_distance_threshold]

        if len(close_obstacles) == 0:
            self.get_logger().info('No close obstacles detected.')
            self.publish_vibration_command(self.FORWARD, 0.0, 0.0)
            return

        # Determine obstacle location (front, left, right)
        front_obstacles = False
        left_obstacles = False
        right_obstacles = False

        for p in close_obstacles:
            # Calculate angle from the x-axis (forward direction)
            angle = np.arctan2(p[1], p[0]) # atan2(y, x)

            if -self.front_angle_threshold_rad <= angle <= self.front_angle_threshold_rad:
                front_obstacles = True
            elif angle > self.front_angle_threshold_rad: # Left side
                left_obstacles = True
            elif angle < -self.front_angle_threshold_rad: # Right side
                right_obstacles = True
        
        command_type = self.STOP
        intensity = 0.8
        duration = 0.5

        if front_obstacles:
            command_type = self.STOP
            self.get_logger().info('Obstacle in front. Sending STOP command.')
        elif left_obstacles and not right_obstacles:
            command_type = self.TURN_RIGHT
            self.get_logger().info('Obstacle on left. Sending TURN_RIGHT command.')
        elif right_obstacles and not left_obstacles:
            command_type = self.TURN_LEFT
            self.get_logger().info('Obstacle on right. Sending TURN_LEFT command.')
        elif left_obstacles and right_obstacles: # Obstacles on both sides, or complex scenario
            command_type = self.STOP # Default to stop for safety
            self.get_logger().info('Obstacles on both sides. Sending STOP command.')
        else:
            command_type = self.FORWARD
            intensity = 0.0
            duration = 0.0
            self.get_logger().info('No significant obstacles. Sending FORWARD command.')

        self.publish_vibration_command(command_type, intensity, duration)

    def read_points(self, cloud_msg):
        points = []
        x_offset = -1
        y_offset = -1
        z_offset = -1

        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset == -1 or y_offset == -1 or z_offset == -1:
            self.get_logger().error('XYZ fields not found in PointCloud2 message.')
            return []

        point_step = cloud_msg.point_step

        for i in range(cloud_msg.width * cloud_msg.height):
            start_byte = i * point_step
            x = struct.unpack_from('<f', cloud_msg.data, start_byte + x_offset)[0]
            y = struct.unpack_from('<f', cloud_msg.data, start_byte + y_offset)[0]
            z = struct.unpack_from('<f', cloud_msg.data, start_byte + z_offset)[0]
            points.append([x, y, z])
        return points

    def publish_vibration_command(self, command_type, intensity, duration):
        msg = VibrationCommand()
        msg.command_type = command_type
        msg.intensity = intensity
        msg.duration = duration
        self.vibration_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    feedback_generator = FeedbackGenerator()
    rclpy.spin(feedback_generator)
    feedback_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
