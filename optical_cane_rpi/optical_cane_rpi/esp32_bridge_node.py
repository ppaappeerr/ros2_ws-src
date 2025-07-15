# esp32_bridge_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge_node')
        
        # 1. /haptic_command 토픽(String)을 구독
        self.subscription = self.create_subscription(
            String,
            '/haptic_command',
            self.command_callback,
            10)
        
        # 2. ESP32로 보낼 /micro_ros_haptic_command 토픽(Int32)을 발행
        self.publisher_ = self.create_publisher(Int32, '/micro_ros_haptic_command', 10)
        
        # 3. 문자열 명령과 정수 ID 매핑 정의
        self.command_to_id = {
            "STOP": 0,
            "FRONT_LOW": 1,
            "FRONT_MID": 2,
            "FRONT_HIGH": 3,
            "LEFT_LOW": 11,
            "LEFT_MID": 12,
            "LEFT_HIGH": 13,
            "RIGHT_LOW": 21,
            "RIGHT_MID": 22,
            "RIGHT_HIGH": 23,
            # ...필요에 따라 더 구체적인 명령 추가...
        }
        self.get_logger().info('ESP32 Bridge is running...')

    def command_callback(self, msg):
        command_str = msg.data
        
        # 4. 수신된 문자열을 정수 ID로 변환 (없으면 -1)
        command_id = self.command_to_id.get(command_str, -1)
        
        if command_id != -1:
            int_msg = Int32()
            int_msg.data = command_id
            self.publisher_.publish(int_msg)
            self.get_logger().info(f'Relaying command: "{command_str}" as ID: {command_id}')

def main(args=None):
    rclpy.init(args=args)
    esp32_bridge = ESP32Bridge()
    rclpy.spin(esp32_bridge)
    esp32_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()