import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math  
import json
from pathlib import Path

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 파라미터 선언
        self.declare_parameter('frame_id', 'imu_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68

        try:
            self.bus.write_byte_data(self.device_address, 0x6B, 0x00)
            self.get_logger().info("MPU6050 Initialized!")
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {e}")

        # 보정값 로드
        self.acc_offset = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}
        self.load_calibration()

        self.timer = self.create_timer(0.02, self.publish_imu_data)

    def load_calibration(self):
        try:
            calib_file = Path.home() / ".mpu6050_calib.json"
            calib = json.loads(calib_file.read_text())
            self.acc_offset = calib["accel_offset"]
            self.gyro_offset = calib["gyro_offset"]
            self.get_logger().info("Calibration loaded.")
        except Exception as e:
            self.get_logger().warn(f"Calibration load failed: {e}")

    def read_raw_data(self, addr): # 센서에서 2바이트 읽어서 signed 16bit 정수로 변환
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)
        value = ((high << 8) | low)
        return value - 65536 if value > 32767 else value

    def publish_imu_data(self):
        imu_msg = Imu()
        try:
            acc_x = self.read_raw_data(0x3B) - self.acc_offset['x']
            acc_y = self.read_raw_data(0x3D) - self.acc_offset['y']
            acc_z = self.read_raw_data(0x3F) - self.acc_offset['z']
            gyro_x = self.read_raw_data(0x43) - self.gyro_offset['x']
            gyro_y = self.read_raw_data(0x45) - self.gyro_offset['y']
            gyro_z = self.read_raw_data(0x47) - self.gyro_offset['z']
        except OSError as e:
            self.get_logger().error(f"MPU6050 read failed: {e}")
            return

        imu_msg.linear_acceleration.x = acc_x / 16384.0 * 9.81
        imu_msg.linear_acceleration.y = acc_y / 16384.0 * 9.81
        imu_msg.linear_acceleration.z = acc_z / 16384.0 * 9.81
        imu_msg.angular_velocity.x = math.radians(gyro_x / 131.0)
        imu_msg.angular_velocity.y = math.radians(gyro_y / 131.0)
        imu_msg.angular_velocity.z = math.radians(gyro_z / 131.0)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id  # 파라미터에서 가져온 frame_id 사용
        # base_link -> self.frame_id
        self.pub_imu.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

