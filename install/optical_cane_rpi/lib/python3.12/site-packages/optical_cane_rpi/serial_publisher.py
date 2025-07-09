import rclpy
from rclpy.node import Node
from optical_cane_rpi.msg import VibrationCommand
import serial
import json

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Successfully opened serial port {self.serial_port} at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.ser = None

        self.subscription = self.create_subscription(
            VibrationCommand,
            '/optical_cane/vibration_command',
            self.vibration_command_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Serial Publisher Node has been started.')

    def vibration_command_callback(self, msg):
        if self.ser is None:
            self.get_logger().warn('Serial port not open. Cannot send vibration command.')
            return

        # Convert VibrationCommand message to a JSON string for easy parsing on ESP32
        command_data = {
            'command_type': msg.command_type,
            'intensity': msg.intensity,
            'duration': msg.duration
        }
        json_command = json.dumps(command_data) + '\n' # Add newline as a delimiter

        try:
            self.ser.write(json_command.encode('utf-8'))
            self.get_logger().info(f'Sent command to ESP32: {json_command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {e}')

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    serial_publisher = SerialPublisher()
    rclpy.spin(serial_publisher)
    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
