import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math
import json
from pathlib import Path
import numpy as np

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 파라미터
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_calibration', True)  # 보정값 사용 여부
        self.declare_parameter('calibration_path', '')  # 빈 문자열이면 기본 경로 사용
        
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.calibration_path = self.get_parameter('calibration_path').value
        
        # 퍼블리셔
        self.pub_imu = self.create_publisher(Imu, 'imu', 10)
        
        # I2C 설정
        try:
            self.bus = smbus.SMBus(1)  # Raspberry Pi의 I2C 버스
            self.device_address = 0x68  # MPU6050 주소
            
            # 디바이스 초기화
            self.init_mpu6050()
            self.get_logger().info("MPU6050 초기화 성공")
        except Exception as e:
            self.get_logger().error(f"MPU6050 초기화 실패: {e}")
            raise
        
        # 보정값 로드
        self.acc_offset = {'x': 0, 'y': 0, 'z': 0}
        self.gyro_offset = {'x': 0, 'y': 0, 'z': 0}
        if self.use_calibration:
            self.load_calibration()
        
        # 통계 및 진단 데이터
        self.publish_count = 0
        self.error_count = 0
        self.last_success_time = self.get_clock().now()
        
        # 타이머 설정
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_imu_data)
        self.status_timer = self.create_timer(5.0, self.print_status)
    
    def init_mpu6050(self):
        """MPU6050 초기화 및 설정"""
        # 전원 관리 레지스터 - 절전 모드 해제
        self.bus.write_byte_data(self.device_address, 0x6B, 0x00)
        time.sleep(0.1)  # 안정화를 위한 대기
        
        # 디지털 저주파 필터 설정 - 노이즈 감소
        self.bus.write_byte_data(self.device_address, 0x1A, 0x03)  # ~44Hz 대역폭
        
        # 자이로스코프 설정: ±250°/s (더 안정적인 값으로 변경)
        self.bus.write_byte_data(self.device_address, 0x1B, 0x00)  # 0x00: ±250°/s
        
        # 가속도계 설정: ±2g (더 안정적인 값으로 변경)
        self.bus.write_byte_data(self.device_address, 0x1C, 0x00)  # 0x00: ±2g
        
        # 샘플링 속도 설정: 1kHz / (1 + 9) = 100Hz
        self.bus.write_byte_data(self.device_address, 0x19, 0x09)
    
    def load_calibration(self):
        """보정 파일 로드"""
        try:
            if not self.calibration_path:
                calib_file = Path.home() / ".mpu6050_calib.json"
            else:
                calib_file = Path(self.calibration_path)
                
            if calib_file.exists():
                calib_data = json.loads(calib_file.read_text())
                self.acc_offset = calib_data["accel_offset"]
                self.gyro_offset = calib_data["gyro_offset"]
                self.get_logger().info(f"보정값 로드 완료: {calib_file}")
            else:
                self.get_logger().warn(f"보정 파일을 찾을 수 없음: {calib_file}")
        except Exception as e:
            self.get_logger().error(f"보정값 로드 실패: {e}")
    
    def read_raw_data(self, addr):
        """안정적인 데이터 읽기"""
        try:
            high = self.bus.read_byte_data(self.device_address, addr)
            low = self.bus.read_byte_data(self.device_address, addr+1)
            value = ((high << 8) | low)
            
            # 부호 있는 16비트 값으로 변환
            if value > 32767:
                value -= 65536
                
            return value
        except Exception as e:
            self.get_logger().error(f"데이터 읽기 오류: {e}")
            raise
    
    def print_status(self):
        """진단 정보 출력"""
        now = self.get_clock().now()
        diff = (now - self.last_success_time).nanoseconds / 1e9
        
        self.get_logger().info(
            f"IMU 상태: 성공={self.publish_count}/5초, 오류={self.error_count}/5초, "
            f"마지막 성공={diff:.1f}초 전"
        )
        
        self.publish_count = 0
        self.error_count = 0
    
    def publish_imu_data(self):
        """IMU 데이터 발행"""
        try:
            # IMU 데이터 읽기
            acc_x = self.read_raw_data(0x3B) - self.acc_offset['x']
            acc_y = self.read_raw_data(0x3D) - self.acc_offset['y']
            acc_z = self.read_raw_data(0x3F) - self.acc_offset['z']
            
            gyro_x = self.read_raw_data(0x43) - self.gyro_offset['x']
            gyro_y = self.read_raw_data(0x45) - self.gyro_offset['y']
            gyro_z = self.read_raw_data(0x47) - self.gyro_offset['z']
            
            # 물리적 단위로 변환 (±2g 범위, 16,384 LSB/g)
            acc_x_ms2 = acc_x / 16384.0 * 9.81
            acc_y_ms2 = acc_y / 16384.0 * 9.81
            acc_z_ms2 = acc_z / 16384.0 * 9.81
            
            # 물리적 단위로 변환 (±250°/s 범위, 131 LSB/°/s)
            gyro_x_rads = gyro_x / 131.0 * math.pi / 180.0
            gyro_y_rads = gyro_y / 131.0 * math.pi / 180.0
            gyro_z_rads = gyro_z / 131.0 * math.pi / 180.0
            
            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # 가속도 설정
            imu_msg.linear_acceleration.x = acc_x_ms2
            imu_msg.linear_acceleration.y = acc_y_ms2
            imu_msg.linear_acceleration.z = acc_z_ms2
            
            # 각속도 설정
            imu_msg.angular_velocity.x = gyro_x_rads
            imu_msg.angular_velocity.y = gyro_y_rads
            imu_msg.angular_velocity.z = gyro_z_rads
            
            # 방향 정보는 알 수 없음을 표시
            imu_msg.orientation.w = 1.0
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = 0.0
            imu_msg.orientation_covariance[0] = -1.0
            
            # 공분산 설정
            for i in range(9):
                imu_msg.angular_velocity_covariance[i] = 0.0
                imu_msg.linear_acceleration_covariance[i] = 0.0
            
            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01
            
            imu_msg.linear_acceleration_covariance[0] = 0.01
            imu_msg.linear_acceleration_covariance[4] = 0.01
            imu_msg.linear_acceleration_covariance[8] = 0.01
            
            # 메시지 발행
            self.pub_imu.publish(imu_msg)
            self.publish_count += 1
            self.last_success_time = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f"IMU 데이터 처리 오류: {e}")
            self.error_count += 1
            
            # 5번 연속 실패 시 하드웨어 재초기화 시도
            if self.error_count >= 5:
                self.get_logger().warn("5회 연속 실패, MPU6050 재초기화 시도")
                try:
                    self.init_mpu6050()
                    self.error_count = 0
                except:
                    pass
        # 각 노드에 아래 함수 추가

    def print_tf_tree_status(self):
        """현재 TF 트리 상태 출력"""
        try:
            # 기본 TF 체크
            frames = [
                ('world', 'map'),
                ('map', 'base_link'),
                ('base_link', 'laser'),
                ('base_link', 'imu_link')
            ]
            
            status = []
            for parent, child in frames:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                    status.append(f"{parent}→{child}: ✓")
                except Exception:
                    status.append(f"{parent}→{child}: ✗")
                    
            self.get_logger().info(f"TF 트리 상태: {', '.join(status)}")
        except Exception as e:
            self.get_logger().error(f"TF 상태 확인 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MPU6050()
        rclpy.spin(node)
    except Exception as e:
        print(f"치명적 오류: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

