import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import smbus2 as smbus
import time
import math
import json
import numpy as np
from pathlib import Path
from rclpy.time import Time, Duration
from rclpy.clock import ClockType
from builtin_interfaces.msg import Time as BuiltinTime
import traceback

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # 파라미터
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('use_complementary_filter', False)
        self.declare_parameter('complementary_alpha', 0.98)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.calibration_path = self.get_parameter('calibration_path').value
        self.use_complementary_filter = self.get_parameter('use_complementary_filter').value
        self.alpha = self.get_parameter('complementary_alpha').value

        # --- 타임스탬프 관리 로직 제거 ---
        # self.initial_time, self.last_published_time 관련 코드 제거
        # self.publish_interval_ns 는 상보필터 계산에 사용되므로 유지
        if self.publish_rate <= 0:
            self.get_logger().fatal("publish_rate must be positive.")
            raise ValueError("publish_rate must be positive.")
        self.publish_interval_ns = int(1e9 / self.publish_rate)
        self.get_logger().info(f"Publish Rate: {self.publish_rate} Hz")
        self.get_logger().info("Using self.get_clock().now() for timestamps.")
        # --------------------------------

        # 퍼블리셔
        self.pub_imu_raw = self.create_publisher(Imu, 'imu_raw', 10)
        if self.use_complementary_filter:
            self.pub_imu_filtered = self.create_publisher(Imu, 'imu', 10)

        # I2C 설정
        self.device_address = 0x68
        try:
            self.bus = smbus.SMBus(1)
            self.init_mpu6050()
        except Exception as e:
            self.get_logger().fatal(f"MPU6050 초기화 실패: {e}\n{traceback.format_exc()}")
            raise

        # 보정값 로드
        self.acc_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyro_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if self.use_calibration:
            self.load_calibration()

        # 상보 필터 상태 변수
        self.comp_roll = 0.0
        self.comp_pitch = 0.0
        self.comp_yaw = 0.0

        # 통계 및 진단 데이터
        self.publish_count_raw = 0
        self.publish_count_filtered = 0
        self.error_count = 0
        # last_success_time 초기화 방식 변경
        self.last_success_time = self.get_clock().now() # 현재 시간으로 초기화
        self.status_timer = self.create_timer(5.0, self.print_status)

        # 메인 발행 타이머
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)

    def init_mpu6050(self):
        """MPU6050 초기화"""
        try:
            self.bus.write_byte_data(self.device_address, 0x6B, 0x00)
            time.sleep(0.1)
            self.bus.write_byte_data(self.device_address, 0x1A, 0x00)
            time.sleep(0.1)
            self.get_logger().info("MPU6050 DLPF 설정 완료 (0x1A = 0x00)")
            self.get_logger().info("MPU6050 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"MPU6050 초기화 중 오류: {e}")
            raise

    def load_calibration(self):
        """보정값 파일 로드"""
        if self.calibration_path:
            calib_file = Path(self.calibration_path)
        else:
            calib_file = Path.home() / ".mpu6050_calib.json"

        if calib_file.exists():
            try:
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                self.acc_offset = {k: float(v) for k, v in calib_data.get('accel_offset', {}).items()}
                self.gyro_offset = {k: float(v) for k, v in calib_data.get('gyro_offset', {}).items()}
                self.get_logger().info(f"보정값 로드 완료: {calib_file}")
                self.get_logger().info(f"  Acc offset: {self.acc_offset}")
                self.get_logger().info(f"  Gyro offset: {self.gyro_offset}")
            except Exception as e:
                self.get_logger().error(f"보정값 로드 실패 ({calib_file}): {e}")
        else:
            self.get_logger().warning(f"보정 파일 없음: {calib_file}. 보정 없이 진행합니다.")
        self.get_logger().info(f"Complementary Filter 사용: {self.use_complementary_filter}")

    def read_raw_data(self, addr):
        """센서에서 raw 데이터 읽기"""
        try:
            high = self.bus.read_byte_data(self.device_address, addr)
            low = self.bus.read_byte_data(self.device_address, addr + 1)
            value = ((high << 8) | low)
            if value > 32767:
                value -= 65536
            return value
        except OSError as e:
            self.get_logger().error(f"I2C read error at address {hex(addr)}: {e}")
            self.error_count += 1
            return None

    def print_status(self):
        """주기적으로 상태 정보 출력"""
        now = self.get_clock().now()
        # last_success_time이 초기 Time 객체와 다를 경우에만 계산
        # (혹은 초기화 시 None으로 두고 None 체크)
        try:
            time_diff = now - self.last_success_time
            time_since_success = time_diff.nanoseconds / 1e9
            time_since_success_str = f"{time_since_success:.2f}s"
        except TypeError: # self.last_success_time이 아직 유효한 Time 객체가 아닐 수 있음
             time_since_success_str = "N/A"

        self.get_logger().info(
            f"Raw Pub: {self.publish_count_raw}, Filtered Pub: {self.publish_count_filtered}, "
            f"Errors: {self.error_count}, Time since last success: {time_since_success_str}"
        )

    def publish_imu_data(self):
        """IMU 데이터 읽고 발행 (현재 ROS 시간 사용)"""
        try:
            # --- 타임스탬프: 현재 ROS 시간 사용 ---
            current_time = self.get_clock().now()
            # -----------------------------------

            # 센서 데이터 읽기
            acc_x_raw = self.read_raw_data(0x3B)
            acc_y_raw = self.read_raw_data(0x3D)
            acc_z_raw = self.read_raw_data(0x3F)
            gyro_x_raw = self.read_raw_data(0x43)
            gyro_y_raw = self.read_raw_data(0x45)
            gyro_z_raw = self.read_raw_data(0x47)

            # 읽기 오류 확인
            if None in [acc_x_raw, acc_y_raw, acc_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw]:
                self.get_logger().error("Sensor data read failed. Skipping IMU message publish.")
                return # 오류 발생 시 발행 중단

            # 스케일링 및 보정 적용
            acc_scale = 9.80665 / 16384.0
            acc_x = (float(acc_x_raw) - self.acc_offset.get('x', 0.0)) * acc_scale
            acc_y = (float(acc_y_raw) - self.acc_offset.get('y', 0.0)) * acc_scale
            acc_z = (float(acc_z_raw) - self.acc_offset.get('z', 0.0)) * acc_scale

            gyro_scale = (250.0 / 32768.0) * (math.pi / 180.0)
            gyro_x = (float(gyro_x_raw) - self.gyro_offset.get('x', 0.0)) * gyro_scale
            gyro_y = (float(gyro_y_raw) - self.gyro_offset.get('y', 0.0)) * gyro_scale
            gyro_z = (float(gyro_z_raw) - self.gyro_offset.get('z', 0.0)) * gyro_scale

            # IMU 메시지 생성 (Raw)
            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = self.get_clock().now().to_msg() # 현재 시간 사용
            imu_raw_msg.header.frame_id = self.frame_id

            imu_raw_msg.orientation.x = 0.0
            imu_raw_msg.orientation.y = 0.0
            imu_raw_msg.orientation.z = 0.0
            imu_raw_msg.orientation.w = 1.0
            imu_raw_msg.orientation_covariance[0] = -1.0

            imu_raw_msg.angular_velocity.x = gyro_x
            imu_raw_msg.angular_velocity.y = gyro_y
            imu_raw_msg.angular_velocity.z = gyro_z
            imu_raw_msg.angular_velocity_covariance[0] = 0.01
            imu_raw_msg.angular_velocity_covariance[4] = 0.01
            imu_raw_msg.angular_velocity_covariance[8] = 0.01

            imu_raw_msg.linear_acceleration.x = acc_x
            imu_raw_msg.linear_acceleration.y = acc_y
            imu_raw_msg.linear_acceleration.z = acc_z
            imu_raw_msg.linear_acceleration_covariance[0] = 0.01
            imu_raw_msg.linear_acceleration_covariance[4] = 0.01
            imu_raw_msg.linear_acceleration_covariance[8] = 0.01

            # Raw 데이터 발행
            self.pub_imu_raw.publish(imu_raw_msg)
            self.publish_count_raw += 1

            # 상보 필터 사용 시
            if self.use_complementary_filter:
                dt = self.publish_interval_ns / 1e9 # dt는 여전히 publish_rate 기반으로 계산
                acc_roll = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))
                acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2))
                self.comp_roll = self.alpha * (self.comp_roll + gyro_x * dt) + (1.0 - self.alpha) * acc_roll
                self.comp_pitch = self.alpha * (self.comp_pitch + gyro_y * dt) + (1.0 - self.alpha) * acc_pitch
                self.comp_yaw += gyro_z * dt
                quat = self.euler_to_quaternion(self.comp_roll, self.comp_pitch, self.comp_yaw)

                # 필터링된 IMU 메시지 생성
                imu_filtered_msg = Imu()
                imu_filtered_msg.header.stamp = imu_raw_msg.header.stamp # 동일한 타임스탬프 사용
                imu_filtered_msg.header.frame_id = self.frame_id

                imu_filtered_msg.orientation.x = quat[0]
                imu_filtered_msg.orientation.y = quat[1]
                imu_filtered_msg.orientation.z = quat[2]
                imu_filtered_msg.orientation.w = quat[3]

                # --- Orientation Covariance 설정 (중요) ---
                imu_filtered_msg.orientation_covariance[0] = 0.01  # Roll variance
                imu_filtered_msg.orientation_covariance[4] = 0.01  # Pitch variance
                imu_filtered_msg.orientation_covariance[8] = 0.1   # Yaw variance (보통 더 불확실)
                # -----------------------------------------

                imu_filtered_msg.linear_acceleration = imu_raw_msg.linear_acceleration
                imu_filtered_msg.angular_velocity = imu_raw_msg.angular_velocity
                imu_filtered_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
                imu_filtered_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance

                # 필터링된 데이터 발행
                self.pub_imu_filtered.publish(imu_filtered_msg)
                self.publish_count_filtered += 1

            self.last_success_time = self.get_clock().now()

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"IMU 데이터 처리 중 오류: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각도를 쿼터니언으로 변환"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return [q.x, q.y, q.z, q.w]

    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 오일러 각도 추출 (디버깅용)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPU6050()
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in node execution: {e}", exc_info=True)
        else:
            print(f"Failed to initialize node: {e}")
    finally:
        if node and rclpy.ok() and hasattr(node, '_is_shutdown') and not node._is_shutdown:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    try:
        import rclpy
        import smbus2
        import math
        import numpy as np
        import json
        import time
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import Quaternion
        from rclpy.time import Time, Duration
        from rclpy.clock import ClockType
        from builtin_interfaces.msg import Time as BuiltinTime
        import traceback
    except ImportError as e:
        print(f"필수 라이브러리 임포트 실패: {e}. 설치가 필요합니다.")
        exit()
    main()

