#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/mpu6050_py/mpu6050_py/mpu6050_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time
import math
import json
from pathlib import Path
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 파라미터
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('use_calibration', True)  # 보정값 사용 여부
        self.declare_parameter('calibration_path', '')  # 빈 문자열이면 기본 경로 사용
        
        # 방향 계산 관련 새 파라미터
        self.declare_parameter('compute_orientation', True)  # 방향 계산 여부
        self.declare_parameter('orientation_gain', 0.035)    # 낮은 게인으로 변경: 0.05 → 0.035
        self.declare_parameter('gravity_filter', 0.015)      # 약간 강화: 0.01 → 0.015
        
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.calibration_path = self.get_parameter('calibration_path').value
        self.compute_orientation = self.get_parameter('compute_orientation').value
        self.orientation_gain = self.get_parameter('orientation_gain').value
        self.gravity_filter = self.get_parameter('gravity_filter').value
        
        # TF 리스너 (추가)
        self.tf_buffer = Buffer(Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
        
        # 방향 계산 변수 (추가)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.gravity = np.array([0.0, 0.0, 9.81])  # 기본 중력 벡터
        self.prev_time = None
        
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
        
        if self.compute_orientation:
            # 방향 정보 로그 추가
            q = self.orientation
            roll, pitch, yaw = self.euler_from_quaternion(q[1], q[2], q[3], q[0])
            self.get_logger().info(
                f"IMU 방향: roll={math.degrees(roll):.1f}°, "
                f"pitch={math.degrees(pitch):.1f}°, "
                f"yaw={math.degrees(yaw):.1f}°"
            )
        
        self.publish_count = 0
        self.error_count = 0
    
    # 방향 계산 함수 수정
    def update_orientation(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt):
        """각속도와 가속도를 이용한 방향 추정 (간소화된 Madgwick)"""
        # 현재 시간에 변화량이 없으면 무시
        if dt <= 0:
            return
            
        # 추가: 아주 작은 각속도는 0으로 처리 (드리프트 방지)
        threshold = 0.15  # 더 높은 임계값 (0.15 deg/s)
        if abs(gyro_x) < threshold: gyro_x = 0
        if abs(gyro_y) < threshold: gyro_y = 0
        if abs(gyro_z) < threshold: gyro_z = 0
        
        # 각속도 적분 (쿼터니언 미분방정식)
        gx_rad = gyro_x * math.pi / 180.0
        gy_rad = gyro_y * math.pi / 180.0
        gz_rad = gyro_z * math.pi / 180.0
        
        # 센서 위치에 따라 축 반전이 필요할 수 있음
        # gx_rad = -gx_rad  # 필요시 주석 해제
        # gy_rad = -gy_rad  # 필요시 주석 해제
        # gz_rad = -gz_rad  # 필요시 주석 해제
        
        # 각속도에 의한 방향 변화율
        qDot = np.zeros(4)
        q = self.orientation
        
        qDot[0] = 0.5 * (-q[1]*gx_rad - q[2]*gy_rad - q[3]*gz_rad)
        qDot[1] = 0.5 * (q[0]*gx_rad + q[2]*gz_rad - q[3]*gy_rad)
        qDot[2] = 0.5 * (q[0]*gy_rad - q[1]*gz_rad + q[3]*gx_rad)
        qDot[3] = 0.5 * (q[0]*gz_rad + q[1]*gy_rad - q[2]*gx_rad)
        
        # 각속도 적분
        q_gyro = q + qDot * dt
        q_gyro = self.normalize_quaternion(q_gyro)
        
        # 가속도 벡터 정규화
        acc_norm = np.sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)
        if acc_norm > 0.1:  # 충분한 크기의 가속도가 있는 경우
            acc_x /= acc_norm
            acc_y /= acc_norm
            acc_z /= acc_norm
            
            # 중력 방향 업데이트 (저주파 필터)
            self.gravity = self.gravity * (1 - self.gravity_filter) + \
                           np.array([acc_x, acc_y, acc_z]) * self.gravity_filter
            self.gravity = self.gravity / np.linalg.norm(self.gravity)
            
            # 현재 쿼터니언으로부터 중력 방향 계산
            v_gravity = self.rotate_by_quaternion(np.array([0, 0, 1]), q_gyro)
            
            # 실제 중력과 예상 중력 간의 회전 계산
            cross = np.cross(v_gravity, self.gravity)
            dot = np.dot(v_gravity, self.gravity)
            
            # 보정 쿼터니언 계산 (SLERP의 간소화 버전)
            q_corr = np.array([1.0, 0, 0, 0])
            if np.linalg.norm(cross) > 1e-6:
                q_corr[0] = dot
                q_corr[1:] = cross
                q_corr = self.normalize_quaternion(q_corr)
                
                # 방향 보정 적용 (게인에 따라 비중 조절)
                q_acc = self.quaternion_multiply(q_corr, q_gyro)
                q_acc = self.normalize_quaternion(q_acc)
                
                # 최종 쿼터니언 = 각속도 적분 + 가속도 보정
                self.orientation = q_gyro * (1 - self.orientation_gain) + q_acc * self.orientation_gain
            else:
                self.orientation = q_gyro
        else:
            # 가속도가 불안정한 경우 각속도만 사용
            self.orientation = q_gyro
            
        # 정규화
        self.orientation = self.normalize_quaternion(self.orientation)
        
        return self.orientation
    
    def normalize_quaternion(self, q):
        """쿼터니언 정규화"""
        norm = np.linalg.norm(q)
        if norm < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm
        
    def quaternion_multiply(self, q1, q2):
        """두 쿼터니언 곱하기"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def rotate_by_quaternion(self, v, q):
        """벡터를 쿼터니언으로 회전"""
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        q_v = np.array([0, v[0], v[1], v[2]])
        q_tmp = self.quaternion_multiply(q, q_v)
        q_rotated = self.quaternion_multiply(q_tmp, q_conj)
        
        return np.array([q_rotated[1], q_rotated[2], q_rotated[3]])
    
    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 오일러 각 변환"""
        # 롤 (x축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # 피치 (y축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # 요 (z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
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
            
            # 방향 계산 (추가된 부분)
            if self.compute_orientation:
                now = self.get_clock().now()
                if self.prev_time is not None:
                    dt = (now - self.prev_time).nanoseconds / 1e9
                    
                    # 방향 업데이트
                    self.update_orientation(
                        acc_x_ms2, acc_y_ms2, acc_z_ms2,
                        gyro_x, gyro_y, gyro_z,
                        dt
                    )
                    
                    # 계산된 방향 설정
                    imu_msg.orientation.w = float(self.orientation[0])
                    imu_msg.orientation.x = float(self.orientation[1])
                    imu_msg.orientation.y = float(self.orientation[2])
                    imu_msg.orientation.z = float(self.orientation[3])
                    
                    # 방향 정보가 유효함을 표시
                    imu_msg.orientation_covariance[0] = 0.01  # 양의 값 = 유효
                else:
                    # 첫 데이터는 방향 계산 없이 기본값
                    imu_msg.orientation.w = 1.0
                    imu_msg.orientation.x = 0.0
                    imu_msg.orientation.y = 0.0
                    imu_msg.orientation.z = 0.0
                    imu_msg.orientation_covariance[0] = -1.0  # 방향 정보 무효
                
                self.prev_time = now
            else:
                # 방향 정보 없음 (원래 동작)
                imu_msg.orientation.w = 1.0
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation_covariance[0] = -1.0
            
            # 공분산 설정
            for i in range(1, 9):  # 첫 번째 값은 위에서 설정
                imu_msg.orientation_covariance[i] = 0.0
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