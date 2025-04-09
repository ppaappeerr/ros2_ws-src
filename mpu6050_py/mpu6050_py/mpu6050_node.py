import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math  
import json
from pathlib import Path
import numpy as np
from geometry_msgs.msg import Vector3

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 파라미터 선언
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('calibration_file', '~/.mpu6050_calib.json')
        self.declare_parameter('gravity_compensation', True)
        
        # 파라미터 로드
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        calib_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.gravity_compensation = self.get_parameter('gravity_compensation').get_parameter_value().bool_value
        
        # 퍼블리셔
        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        self.pub_raw = self.create_publisher(Imu, 'imu/raw', 10)  # 보정 전 원시 데이터
        
        # I2C 통신 설정
        self.bus = smbus.SMBus(1)
        self.device_address = 0x68

        # 센서 초기화
        try:
            # 파워 관리 레지스터(0x6B)에 0을 쓰면 슬립 모드 해제
            self.bus.write_byte_data(self.device_address, 0x6B, 0x00)
            
            # 자이로스코프 설정: ±250°/s (0x00)
            self.bus.write_byte_data(self.device_address, 0x1B, 0x00)
            
            # 가속도계 설정: ±2g (0x00)
            self.bus.write_byte_data(self.device_address, 0x1C, 0x00)
            
            self.get_logger().info("MPU6050 initialized with ±250°/s gyro and ±2g accel range")
        except Exception as e:
            self.get_logger().error(f"MPU6050 initialization failed: {e}")

        # 보정값 로드
        self.acc_offset = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}
        self.load_calibration(Path(calib_file).expanduser())
        
        # 기타 변수
        self.prev_time = self.get_clock().now()
        
        # 타이머 설정
        period = 1.0 / rate_hz
        self.timer = self.create_timer(period, self.publish_imu_data)
        
        self.get_logger().info(f"MPU6050 node initialized with {rate_hz}Hz publish rate")

    def load_calibration(self, calib_path):
        try:
            calib = json.loads(calib_path.read_text())
            self.acc_offset = calib["accel_offset"]
            self.gyro_offset = calib["gyro_offset"]
            self.get_logger().info(f"Calibration loaded from {calib_path}")
            self.get_logger().info(f"Accel offset: x={self.acc_offset['x']:.1f}, y={self.acc_offset['y']:.1f}, z={self.acc_offset['z']:.1f}")
            self.get_logger().info(f"Gyro offset: x={self.gyro_offset['x']:.1f}, y={self.gyro_offset['y']:.1f}, z={self.gyro_offset['z']:.1f}")
        except Exception as e:
            self.get_logger().warn(f"Calibration load failed: {e}. Using default values.")

    def read_raw_data(self, addr):
        """센서에서 2바이트 읽어서 signed 16bit 정수로 변환"""
        try:
            high = self.bus.read_byte_data(self.device_address, addr)
            low = self.bus.read_byte_data(self.device_address, addr+1)
            value = ((high << 8) | low)
            return value - 65536 if value > 32767 else value
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor data: {e}")
            return 0

    def publish_imu_data(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now
        
        # 원시 데이터 읽기
        try:
            acc_x_raw = self.read_raw_data(0x3B)
            acc_y_raw = self.read_raw_data(0x3D)
            acc_z_raw = self.read_raw_data(0x3F)
            gyro_x_raw = self.read_raw_data(0x43)
            gyro_y_raw = self.read_raw_data(0x45)
            gyro_z_raw = self.read_raw_data(0x47)
            temp_raw = self.read_raw_data(0x41)
        except Exception as e:
            self.get_logger().error(f"MPU6050 read failed: {e}")
            return
            
        # 보정값 적용
        acc_x = acc_x_raw - self.acc_offset['x']
        acc_y = acc_y_raw - self.acc_offset['y']
        acc_z = acc_z_raw - self.acc_offset['z']
        gyro_x = gyro_x_raw - self.gyro_offset['x']
        gyro_y = gyro_y_raw - self.gyro_offset['y']
        gyro_z = gyro_z_raw - self.gyro_offset['z']
        
        # SI 단위로 변환
        acc_x_mps2 = acc_x / 16384.0 * 9.81  # ±2g 범위, 16384 LSB/g
        acc_y_mps2 = acc_y / 16384.0 * 9.81
        acc_z_mps2 = acc_z / 16384.0 * 9.81
        gyro_x_rads = math.radians(gyro_x / 131.0)  # ±250°/s 범위, 131 LSB/°/s
        gyro_y_rads = math.radians(gyro_y / 131.0)
        gyro_z_rads = math.radians(gyro_z / 131.0)
        temp_c = temp_raw / 340.0 + 36.53  # 온도 계산식
        
        # 원시 IMU 메시지 생성
        raw_msg = Imu()
        raw_msg.header.stamp = now.to_msg()
        raw_msg.header.frame_id = self.frame_id
        raw_msg.linear_acceleration.x = acc_x_mps2
        raw_msg.linear_acceleration.y = acc_y_mps2
        raw_msg.linear_acceleration.z = acc_z_mps2
        raw_msg.angular_velocity.x = gyro_x_rads
        raw_msg.angular_velocity.y = gyro_y_rads
        raw_msg.angular_velocity.z = gyro_z_rads
        
        # 공분산 설정 - 원시 데이터는 확실하지 않으므로 높은 값 사용
        raw_msg.linear_acceleration_covariance = [0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1]
        raw_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # 보정된 IMU 메시지 생성
        imu_msg = Imu()
        imu_msg.header = raw_msg.header
        imu_msg.linear_acceleration = raw_msg.linear_acceleration
        imu_msg.angular_velocity = raw_msg.angular_velocity
        
        # 공분산 설정 - 보정된 값은 더 신뢰할 수 있으므로 낮은 값 사용
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        imu_msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
        
        # 지정된 주기로 로깅 (50Hz면 너무 많으므로 1초에 한 번)
        if int(now.nanoseconds / 1e9) % 1 == 0:
            self.get_logger().debug(f"Accel: x={acc_x_mps2:.2f}, y={acc_y_mps2:.2f}, z={acc_z_mps2:.2f} m/s²")
            self.get_logger().debug(f"Gyro: x={gyro_x_rads:.4f}, y={gyro_y_rads:.4f}, z={gyro_z_rads:.4f} rad/s")
            self.get_logger().debug(f"Temp: {temp_c:.1f}°C")
        
        # 메시지 발행
        self.pub_raw.publish(raw_msg)
        self.pub_imu.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()