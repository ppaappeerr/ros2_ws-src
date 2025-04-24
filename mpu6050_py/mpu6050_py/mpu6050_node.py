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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # QoS import 추가

class MPU6050(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        # 파라미터
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('use_complementary_filter', True)
        self.declare_parameter('complementary_alpha', 0.95)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.calibration_path = self.get_parameter('calibration_path').value
        self.use_complementary_filter = self.get_parameter('use_complementary_filter').value
        self.alpha = self.get_parameter('complementary_alpha').value

        if self.publish_rate <= 0:
            self.get_logger().fatal("publish_rate must be positive.")
            raise ValueError("publish_rate must be positive.")
        self.publish_interval_ns = int(1e9 / self.publish_rate)
        self.get_logger().info(f"Publish Rate: {self.publish_rate} Hz")
        self.get_logger().info("Using self.get_clock().now() for timestamps.")

        # QoS 설정 (Reliable)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # BEST_EFFORT -> RELIABLE
            history=HistoryPolicy.KEEP_LAST,
            depth=10, # Depth는 유지하거나 늘릴 수 있음
            durability=DurabilityPolicy.VOLATILE # VOLATILE 유지
        )

        # 퍼블리셔 (수정된 QoS 적용)
        self.pub_imu_raw = self.create_publisher(Imu, 'imu_raw', qos_profile) # qos_profile 적용
        if self.use_complementary_filter:
            self.pub_imu_filtered = self.create_publisher(Imu, 'imu_filtered', qos_profile) # qos_profile 적용

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
        self.last_filter_time = self.get_clock().now()

        # 통계 및 진단 데이터
        self.publish_count_raw = 0
        self.publish_count_filtered = 0
        self.last_pub_time_raw_ns = 0
        self.last_pub_time_filtered_ns = 0
        self.error_count = 0
        self.last_error_time = None

        # 메인 발행 타이머
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)
        self.status_timer = self.create_timer(5.0, self.print_status)

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
        time_since_last_error = "N/A"
        if self.last_error_time:
            time_since_last_error = f"{(now - self.last_error_time).nanoseconds / 1e9:.2f}s ago"

        try:
            time_diff = now - self.last_success_time
            time_since_success = time_diff.nanoseconds / 1e9
            time_since_success_str = f"{time_since_success:.2f}s"
        except TypeError:
             time_since_success_str = "N/A"

        self.get_logger().info(
            f"Raw Pub: {self.publish_count_raw}, Filtered Pub: {self.publish_count_filtered}, "
            f"Errors: {self.error_count} (Last: {time_since_last_error}), Time since last success: {time_since_success_str}"
        )

    def publish_imu_data(self):
        try:
            # --- Timestamp acquisition moved earlier ---
            current_time_msg = self.get_clock().now().to_msg() # Get timestamp as close as possible to data read

            # Read raw data
            acc_x = self.read_raw_data(0x3B)
            acc_y = self.read_raw_data(0x3D)
            acc_z = self.read_raw_data(0x3F)
            gyro_x = self.read_raw_data(0x43)
            gyro_y = self.read_raw_data(0x45)
            gyro_z = self.read_raw_data(0x47)

            # Apply calibration offsets if enabled
            if self.use_calibration:
                acc_x -= self.acc_offset['x']
                acc_y -= self.acc_offset['y']
                acc_z -= self.acc_offset['z']
                gyro_x -= self.gyro_offset['x']
                gyro_y -= self.gyro_offset['y']
                gyro_z -= self.gyro_offset['z']

            # Convert raw values to SI units (m/s^2 and rad/s)
            accel_x_mps2 = acc_x / 16384.0 * 9.80665
            accel_y_mps2 = acc_y / 16384.0 * 9.80665
            accel_z_mps2 = acc_z / 16384.0 * 9.80665
            gyro_x_radps = gyro_x / 131.0 * (math.pi / 180.0)
            gyro_y_radps = gyro_y / 131.0 * (math.pi / 180.0)
            gyro_z_radps = gyro_z / 131.0 * (math.pi / 180.0)

            # Create raw IMU message
            imu_raw_msg = Imu()
            imu_raw_msg.header.stamp = current_time_msg # Use the earlier timestamp
            imu_raw_msg.header.frame_id = self.frame_id
            imu_raw_msg.linear_acceleration.x = accel_x_mps2
            imu_raw_msg.linear_acceleration.y = accel_y_mps2
            imu_raw_msg.linear_acceleration.z = accel_z_mps2
            # Set covariance if known, otherwise zero
            imu_raw_msg.linear_acceleration_covariance = [0.0] * 9 # Example: Unknown covariance
            imu_raw_msg.angular_velocity.x = gyro_x_radps
            imu_raw_msg.angular_velocity.y = gyro_y_radps
            imu_raw_msg.angular_velocity.z = gyro_z_radps
            imu_raw_msg.angular_velocity_covariance = [0.0] * 9 # Example: Unknown covariance
            # Orientation is not calculated from raw data directly
            imu_raw_msg.orientation_covariance = [-1.0] * 9 # Indicate orientation is not available

            self.pub_imu_raw.publish(imu_raw_msg)
            self.publish_count_raw += 1

            # Complementary Filter (if enabled)
            if self.use_complementary_filter:
                now = self.get_clock().now()
                dt = (now - self.last_filter_time).nanoseconds / 1e9
                self.last_filter_time = now

                # Calculate orientation from accelerometer
                roll_acc = math.atan2(accel_y_mps2, math.sqrt(accel_x_mps2**2 + accel_z_mps2**2))
                pitch_acc = math.atan2(-accel_x_mps2, accel_z_mps2)

                # Integrate gyroscope data
                self.comp_roll += gyro_x_radps * dt
                self.comp_pitch += gyro_y_radps * dt
                self.comp_yaw += gyro_z_radps * dt

                # Apply complementary filter
                alpha = self.alpha
                self.comp_roll = alpha * self.comp_roll + (1.0 - alpha) * roll_acc
                self.comp_pitch = alpha * self.comp_pitch + (1.0 - alpha) * pitch_acc
                # Yaw is typically not corrected by accelerometer

                # 디버깅용 로그 추가
                self.get_logger().info(f"[FILTERED] roll={math.degrees(self.comp_roll):.2f}, pitch={math.degrees(self.comp_pitch):.2f}, yaw={math.degrees(self.comp_yaw):.2f}")

                # Create filtered IMU message
                imu_filtered_msg = Imu()
                imu_filtered_msg.header.stamp = current_time_msg
                imu_filtered_msg.header.frame_id = self.frame_id
                imu_filtered_msg.linear_acceleration = imu_raw_msg.linear_acceleration
                imu_filtered_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
                imu_filtered_msg.angular_velocity = imu_raw_msg.angular_velocity
                imu_filtered_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance

                # Convert filtered Euler angles to quaternion
                q = self.euler_to_quaternion(self.comp_roll, self.comp_pitch, self.comp_yaw)
                imu_filtered_msg.orientation.x = q[0]
                imu_filtered_msg.orientation.y = q[1]
                imu_filtered_msg.orientation.z = q[2]
                imu_filtered_msg.orientation.w = q[3]
                imu_filtered_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                                        0.0, 0.01, 0.0,
                                                        0.0, 0.0, 0.1]

                self.pub_imu_filtered.publish(imu_filtered_msg)
                self.publish_count_filtered += 1

            self.last_success_time = self.get_clock().now()

        except OSError as e:
            self.error_count += 1
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Error in publish_imu_data: {e}", throttle_duration_sec=5.0)

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

