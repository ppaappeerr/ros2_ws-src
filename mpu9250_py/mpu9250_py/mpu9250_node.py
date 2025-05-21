import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
import smbus2 as smbus # Using smbus2 as in the original node
import time
import math
import json
import numpy as np
from pathlib import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import traceback

# MPU9250 & AK8963 Registers and constants
MPU9250_ADDR = 0x68
AK8963_ADDR = 0x0C

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
INT_PIN_CFG = 0x37
USER_CTRL = 0x6A

I2C_SLV0_ADDR = 0x25
I2C_SLV0_REG = 0x26
I2C_SLV0_CTRL = 0x27
I2C_SLV0_DO = 0x63 # For writing to AK8963
EXT_SENS_DATA_00 = 0x49 # Start of data read from AK8963

AK8963_WIA = 0x00
AK8963_CNTL1 = 0x0A
AK8963_CNTL2 = 0x0B # Soft reset
AK8963_ST1 = 0x02   # Data ready status
AK8963_HXL = 0x03   # Mag data start
AK8963_ST2 = 0x09   # Overflow status
AK8963_ASAX = 0x10  # X-axis sensitivity adjustment
AK8963_ASAY = 0x11  # Y-axis sensitivity adjustment
AK8963_ASAZ = 0x12  # Z-axis sensitivity adjustment

AK8963_MODE_PWR_DOWN = 0x00
AK8963_MODE_FUSE_ROM = 0x0F
AK8963_MODE_CONT_MEAS_100HZ = 0x16 # 16-bit output, 100Hz

# Default sensitivity values (can be adjusted based on MPU9250 settings)
ACCEL_FS_SENSITIVITY_2G = 16384.0  # LSB/g for ±2g range
GYRO_FS_SENSITIVITY_250DPS = 131.0 # LSB/(deg/s) for ±250dps range
MAG_RAW_TO_MICRO_TESLA = 4912.0 / 32760.0 # uT/LSB for 16-bit output (AK8963)
G_MPS2 = 9.80665

class MPU9250Node(Node):
    def __init__(self):
        super().__init__('mpu9250_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('mag_frame_id', 'mag_link') # Frame ID for magnetometer
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('calibration_path', str(Path.home() / ".mpu9250_calib.json"))
        self.declare_parameter('use_complementary_filter', True)
        self.declare_parameter('complementary_alpha', 0.98) # Adjusted alpha

        self.frame_id = self.get_parameter('frame_id').value
        self.mag_frame_id = self.get_parameter('mag_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value
        self.calibration_path = self.get_parameter('calibration_path').value
        self.use_complementary_filter = self.get_parameter('use_complementary_filter').value
        self.alpha = self.get_parameter('complementary_alpha').value

        if self.publish_rate <= 0:
            self.get_logger().fatal("publish_rate must be positive.")
            raise ValueError("publish_rate must be positive.")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_imu_raw = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', qos_profile)
        if self.use_complementary_filter:
            self.pub_imu_filtered = self.create_publisher(Imu, 'imu/data', qos_profile)

        self.bus = smbus.SMBus(1)
        self.mag_asa = {'x': 1.0, 'y': 1.0, 'z': 1.0} # Sensitivity Adjustment
        self.init_mpu9250()
        self.init_ak8963()

        self.acc_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyro_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.mag_offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        if self.use_calibration:
            self.load_calibration()

        self.comp_roll = 0.0
        self.comp_pitch = 0.0
        self.comp_yaw = 0.0 # Yaw from gyro only for now
        self.last_filter_time = self.get_clock().now()
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)
        self.get_logger().info("MPU9250 Node started.")

    def _write_mpu_reg(self, reg, val):
        self.bus.write_byte_data(MPU9250_ADDR, reg, val)
        time.sleep(0.005) # Small delay

    def _read_mpu_reg(self, reg):
        return self.bus.read_byte_data(MPU9250_ADDR, reg)

    def _read_mpu_regs(self, reg_start, num_bytes):
        return self.bus.read_i2c_block_data(MPU9250_ADDR, reg_start, num_bytes)

    def _write_ak8963_reg(self, reg, val):
        self._write_mpu_reg(I2C_SLV0_ADDR, AK8963_ADDR) # AK8963 I2C addr for write
        self._write_mpu_reg(I2C_SLV0_REG, reg)          # Register to write
        self._write_mpu_reg(I2C_SLV0_DO, val)           # Data to write
        self._write_mpu_reg(I2C_SLV0_CTRL, 0x81)        # Enable SLV0, 1 byte
        # self.get_logger().debug(f"Wrote AK8963 reg {hex(reg)} with {hex(val)}")

    def _read_ak8963_regs(self, reg_start, num_bytes):
        self._write_mpu_reg(I2C_SLV0_ADDR, AK8963_ADDR | 0x80) # AK8963 I2C addr for read
        self._write_mpu_reg(I2C_SLV0_REG, reg_start)           # Register to read from
        self._write_mpu_reg(I2C_SLV0_CTRL, 0x80 | num_bytes)   # Enable SLV0, read num_bytes
        time.sleep(0.01) # Ensure transaction completes
        # self.get_logger().debug(f"Reading {num_bytes} from AK8963 reg {hex(reg_start)}")
        return self._read_mpu_regs(EXT_SENS_DATA_00, num_bytes)

    def init_mpu9250(self):
        self._write_mpu_reg(PWR_MGMT_1, 0x80) # Reset device
        time.sleep(0.1)
        self._write_mpu_reg(PWR_MGMT_1, 0x00) # Wake up
        time.sleep(0.1)
        # Configure Gyro and Accel (e.g., FS, DLPF) - using defaults for now
        # self._write_mpu_reg(0x1B, 0x00) # Gyro Config: +/- 250 dps
        # self._write_mpu_reg(0x1C, 0x00) # Accel Config: +/- 2g
        
        # Enable I2C Master mode
        self._write_mpu_reg(USER_CTRL, 0x20) # I2C_MST_EN = 1
        time.sleep(0.05)
        # Set I2C Master clock to 400kHz
        self._write_mpu_reg(0x24, 0x0D) # I2C_MST_CTRL: I2C_MST_CLK = 13 (400kHz)
        time.sleep(0.05)
        self.get_logger().info("MPU9250 initialized.")

    def init_ak8963(self):
        try:
            # Verify AK8963 Who Am I
            wia_val = self._read_ak8963_regs(AK8963_WIA, 1)[0]
            if wia_val != 0x48:
                self.get_logger().error(f"AK8963 WIA check failed. Expected 0x48, got {hex(wia_val)}")
                # Try to disable bypass if it was somehow enabled
                self._write_mpu_reg(INT_PIN_CFG, 0x00) # Disable BYPASS_EN
                time.sleep(0.05)
                self._write_mpu_reg(USER_CTRL, 0x20)   # Re-Enable I2C_MST_EN
                time.sleep(0.05)
                wia_val = self._read_ak8963_regs(AK8963_WIA, 1)[0]
                if wia_val != 0x48:
                     self.get_logger().error(f"AK8963 WIA still incorrect after attempting fix: {hex(wia_val)}")
                     # raise RuntimeError("AK8963 not found or not responding correctly.")
                else:
                    self.get_logger().info("AK8963 WIA correct after re-enabling I2C Master.")

            # Soft reset AK8963
            self._write_ak8963_reg(AK8963_CNTL2, 0x01)
            time.sleep(0.1)

            # Read sensitivity adjustment values from Fuse ROM
            self._write_ak8963_reg(AK8963_CNTL1, AK8963_MODE_FUSE_ROM) # Enter Fuse ROM access mode
            time.sleep(0.05)
            asa_bytes = self._read_ak8963_regs(AK8963_ASAX, 3)
            self.mag_asa['x'] = ((asa_bytes[0] - 128.0) / 256.0) + 1.0
            self.mag_asa['y'] = ((asa_bytes[1] - 128.0) / 256.0) + 1.0
            self.mag_asa['z'] = ((asa_bytes[2] - 128.0) / 256.0) + 1.0
            self._write_ak8963_reg(AK8963_CNTL1, AK8963_MODE_PWR_DOWN) # Power down before changing mode
            time.sleep(0.05)

            # Set to 16-bit output, 100Hz continuous measurement mode
            self._write_ak8963_reg(AK8963_CNTL1, AK8963_MODE_CONT_MEAS_100HZ)
            time.sleep(0.05)
            self.get_logger().info(f"AK8963 initialized. ASA: {self.mag_asa}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize AK8963: {e}\n{traceback.format_exc()}")
            raise

    def load_calibration(self):
        calib_file = Path(self.calibration_path)
        if calib_file.exists():
            try:
                with open(calib_file, 'r') as f:
                    data = json.load(f)
                self.acc_offset = data.get('accel_offset', self.acc_offset)
                self.gyro_offset = data.get('gyro_offset', self.gyro_offset)
                self.mag_offset = data.get('mag_offset', self.mag_offset)
                self.get_logger().info(f"Calibration data loaded from {calib_file}")
                self.get_logger().info(f"  Acc offset: {self.acc_offset}")
                self.get_logger().info(f"  Gyro offset: {self.gyro_offset}")
                self.get_logger().info(f"  Mag offset: {self.mag_offset}")

                # --- 사람이 읽는 단위로 변환해서 출력 ---
                accel_offset_si = {k: v / ACCEL_FS_SENSITIVITY_2G * G_MPS2 for k, v in self.acc_offset.items()}
                gyro_offset_si = {k: v / GYRO_FS_SENSITIVITY_250DPS for k, v in self.gyro_offset.items()}
                mag_offset_si = {k: v * MAG_RAW_TO_MICRO_TESLA for k, v in self.mag_offset.items()}
                self.get_logger().info("  Acc offset (m/s^2): " + str({k: f"{v:.4f}" for k, v in accel_offset_si.items()}))
                self.get_logger().info("  Gyro offset (deg/s): " + str({k: f"{v:.4f}" for k, v in gyro_offset_si.items()}))
                self.get_logger().info("  Mag offset (μT): " + str({k: f'{v:.4f}' for k, v in mag_offset_si.items()}))
            except Exception as e:
                self.get_logger().error(f"Failed to load calibration: {e}")
        else:
            self.get_logger().warning(f"Calibration file not found: {calib_file}. Using zero offsets.")

    def read_raw_mpu_word(self, addr_h):
        try:
            data = self._read_mpu_regs(addr_h, 2)
            val = (data[0] << 8) | data[1]
            if val > 32767:
                val -= 65536
            return val
        except Exception as e:
            self.get_logger().error(f"Error reading MPU word at {hex(addr_h)}: {e}", throttle_duration_sec=5)
            return 0 # Or raise

    def read_raw_mag_data(self):
        try:
            # Check ST1 DRDY (Data Ready)
            st1 = self._read_ak8963_regs(AK8963_ST1, 1)[0]
            if not (st1 & 0x01):
                # self.get_logger().debug("AK8963 data not ready (ST1).")
                return None, None, None # Data not ready

            # Read 6 data bytes (HXL to HZH) and ST2
            data = self._read_ak8963_regs(AK8963_HXL, 7)
            
            # Check ST2 HOFL (Overflow)
            if data[6] & 0x08:
                self.get_logger().warning("AK8963 magnetic sensor overflow (ST2).", throttle_duration_sec=5)
                # Data might be invalid, could return None or last known good
            
            mx_raw = (data[1] << 8) | data[0]
            my_raw = (data[3] << 8) | data[2]
            mz_raw = (data[5] << 8) | data[4]

            if mx_raw > 32767: mx_raw -= 65536
            if my_raw > 32767: my_raw -= 65536
            if mz_raw > 32767: mz_raw -= 65536
            
            return mx_raw, my_raw, mz_raw
        except Exception as e:
            self.get_logger().error(f"Error reading AK8963 data: {e}", throttle_duration_sec=5)
            return None, None, None


    def publish_data(self):
        current_time = self.get_clock().now()
        current_time_msg = current_time.to_msg()

        # Read Accel & Gyro
        acc_x_raw = self.read_raw_mpu_word(ACCEL_XOUT_H)
        acc_y_raw = self.read_raw_mpu_word(ACCEL_XOUT_H + 2)
        acc_z_raw = self.read_raw_mpu_word(ACCEL_XOUT_H + 4)
        gyro_x_raw = self.read_raw_mpu_word(GYRO_XOUT_H)
        gyro_y_raw = self.read_raw_mpu_word(GYRO_XOUT_H + 2)
        gyro_z_raw = self.read_raw_mpu_word(GYRO_XOUT_H + 4)

        # Apply calibration
        acc_x = acc_x_raw - self.acc_offset['x']
        acc_y = acc_y_raw - self.acc_offset['y']
        acc_z = acc_z_raw - self.acc_offset['z']
        gyro_x = gyro_x_raw - self.gyro_offset['x']
        gyro_y = gyro_y_raw - self.gyro_offset['y']
        gyro_z = gyro_z_raw - self.gyro_offset['z']

        # Convert to SI units
        accel_x_mps2 = acc_x / ACCEL_FS_SENSITIVITY_2G * G_MPS2
        accel_y_mps2 = acc_y / ACCEL_FS_SENSITIVITY_2G * G_MPS2
        accel_z_mps2 = acc_z / ACCEL_FS_SENSITIVITY_2G * G_MPS2
        gyro_x_radps = math.radians(gyro_x / GYRO_FS_SENSITIVITY_250DPS)
        gyro_y_radps = math.radians(gyro_y / GYRO_FS_SENSITIVITY_250DPS)
        gyro_z_radps = math.radians(gyro_z / GYRO_FS_SENSITIVITY_250DPS)

        # Read Magnetometer
        mag_x_raw, mag_y_raw, mag_z_raw = self.read_raw_mag_data()

        if mag_x_raw is not None: # If mag data is valid
            # Apply ASA and calibration offset
            mag_x_cal = (mag_x_raw * self.mag_asa['x']) - self.mag_offset['x']
            mag_y_cal = (mag_y_raw * self.mag_asa['y']) - self.mag_offset['y']
            mag_z_cal = (mag_z_raw * self.mag_asa['z']) - self.mag_offset['z']

            # Convert to Tesla for MagneticField message
            mag_x_tesla = mag_x_cal * MAG_RAW_TO_MICRO_TESLA * 1e-6
            mag_y_tesla = mag_y_cal * MAG_RAW_TO_MICRO_TESLA * 1e-6
            mag_z_tesla = mag_z_cal * MAG_RAW_TO_MICRO_TESLA * 1e-6

            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time_msg
            mag_msg.header.frame_id = self.mag_frame_id
            mag_msg.magnetic_field.x = mag_x_tesla
            mag_msg.magnetic_field.y = mag_y_tesla
            mag_msg.magnetic_field.z = mag_z_tesla
            # Example covariance (diagonal, assuming uncorrelated errors)
            mag_msg.magnetic_field_covariance = [0.000001, 0.0, 0.0,
                                                 0.0, 0.000001, 0.0,
                                                 0.0, 0.0, 0.000001]
            self.pub_mag.publish(mag_msg)

        # --- IMU Raw Message ---
        imu_raw_msg = Imu()
        imu_raw_msg.header.stamp = current_time_msg
        imu_raw_msg.header.frame_id = self.frame_id
        
        imu_raw_msg.linear_acceleration.x = accel_x_mps2
        imu_raw_msg.linear_acceleration.y = accel_y_mps2
        imu_raw_msg.linear_acceleration.z = accel_z_mps2
        imu_raw_msg.linear_acceleration_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01] # Example

        imu_raw_msg.angular_velocity.x = gyro_x_radps
        imu_raw_msg.angular_velocity.y = gyro_y_radps
        imu_raw_msg.angular_velocity.z = gyro_z_radps
        imu_raw_msg.angular_velocity_covariance = [0.0025,0,0, 0,0.0025,0, 0,0,0.0025] # Example

        imu_raw_msg.orientation_covariance = [-1.0] * 9 # Orientation not provided in raw
        self.pub_imu_raw.publish(imu_raw_msg)

        # --- Complementary Filter (if enabled) ---
        if self.use_complementary_filter:
            dt_duration = current_time - self.last_filter_time
            dt = dt_duration.nanoseconds / 1e9
            self.last_filter_time = current_time

            if dt <= 0: # Avoid division by zero or negative dt if time jumps backward
                self.get_logger().warn(f"dt is not positive ({dt}), skipping filter update.")
                return

            # Orientation from accelerometer (pitch and roll)
            # Handle potential division by zero if accel_z_mps2 is zero or very small
            # or if accel_x_mps2**2 + accel_z_mps2**2 is zero
            try:
                roll_acc = math.atan2(accel_y_mps2, math.sqrt(accel_x_mps2**2 + accel_z_mps2**2))
                pitch_acc = math.atan2(-accel_x_mps2, accel_z_mps2) # atan2 handles signs correctly
            except ZeroDivisionError:
                self.get_logger().warn("Division by zero in accelerometer angle calculation.", throttle_duration_sec=5)
                roll_acc = self.comp_roll # Use previous value
                pitch_acc = self.comp_pitch # Use previous value


            # Integrate gyroscope data
            self.comp_roll += gyro_x_radps * dt
            self.comp_pitch += gyro_y_radps * dt
            self.comp_yaw += gyro_z_radps * dt # Yaw from gyro only

            # Apply complementary filter
            self.comp_roll = self.alpha * self.comp_roll + (1.0 - self.alpha) * roll_acc
            self.comp_pitch = self.alpha * self.comp_pitch + (1.0 - self.alpha) * pitch_acc
            
            # Normalize Yaw to [-pi, pi]
            self.comp_yaw = (self.comp_yaw + math.pi) % (2 * math.pi) - math.pi

            q = self.euler_to_quaternion(self.comp_roll, self.comp_pitch, self.comp_yaw)

            imu_filtered_msg = Imu()
            imu_filtered_msg.header.stamp = current_time_msg
            imu_filtered_msg.header.frame_id = self.frame_id
            imu_filtered_msg.linear_acceleration = imu_raw_msg.linear_acceleration
            imu_filtered_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
            imu_filtered_msg.angular_velocity = imu_raw_msg.angular_velocity
            imu_filtered_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
            imu_filtered_msg.orientation = q
            imu_filtered_msg.orientation_covariance = [0.001,0,0, 0,0.001,0, 0,0,0.1] # Example
            
            self.pub_imu_filtered.publish(imu_filtered_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPU9250Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt, shutting down.")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception: {e}\n{traceback.format_exc()}")
        else:
            print(f"Failed to initialize node: {e}\n{traceback.format_exc()}")
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()