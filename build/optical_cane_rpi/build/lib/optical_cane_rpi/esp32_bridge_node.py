import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

class ESP32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')

        # Command mapping from string to integer
        self.command_map = {
            "CLEAR": 0,
            "STOP": 1,
            "TURN_LEFT": 2,
            "TURN_RIGHT": 3,
            "FRONT_STEP_UP": 4,
            "LEFT_STEP_UP": 5,
            "RIGHT_STEP_UP": 6,
            "DROPOFF_AHEAD": 7, # Assuming we might add this back later
        }

        # Subscriber to the high-level command topic
        self.subscription = self.create_subscription(
            String,
            '/haptic_command',
            self.command_callback,
            10)
        
        # Publisher for the micro-ROS agent
        self.publisher_ = self.create_publisher(UInt8, '/micro_ros_haptic_command', 10)

        self.get_logger().info('ESP32 Bridge Node has started.')
        self.get_logger().info('Subscribing to /haptic_command and publishing to /micro_ros_haptic_command.')

    def command_callback(self, msg):
        command_str = msg.data
        
        # Look up the command in the map
        command_id = self.command_map.get(command_str, -1) # Default to -1 if command not found

        if command_id != -1:
            pub_msg = UInt8()
            pub_msg.data = command_id
            self.publisher_.publish(pub_msg)
            self.get_logger().debug(f'Received command "{command_str}", sent ID {command_id}')
        else:
            self.get_logger().warn(f'Received unknown command: "{command_str}"')

def main(args=None):
    rclpy.init(args=args)
    esp32_bridge_node = ESP32BridgeNode()
    rclpy.spin(esp32_bridge_node)
    esp32_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
