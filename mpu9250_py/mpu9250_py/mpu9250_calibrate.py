import smbus
import time
import json
from pathlib import Path

# MPU9250 & AK8963 Registers
MPU9250_ADDR = 0x68
AK8963_ADDR = 0x0C

PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37 # Bypass enable
USER_CTRL = 0x6A   # I2C Master enable

# AK8963 Registers for MPU9250's I2C Master
I2C_SLV0_ADDR = 0x25
I2C_SLV0_REG = 0x26
I2C_SLV0_CTRL = 0x27
I2C_SLV0_DO = 0x63
EXT_SENS_DATA_00 = 0x49

AK8963_WIA = 0x00    # Who I Am
AK8963_CNTL1 = 0x0A  # Control 1
AK8963_CNTL2 = 0x0B  # Control 2 (for reset)
AK8963_ST1 = 0x02    # Status 1
AK8963_HXL = 0x03    # Mag data
AK8963_ST2 = 0x09    # Status 2 (overflow)

AK8963_MODE_PWR_DOWN = 0x00
AK8963_MODE_CONT_MEAS_100HZ = 0x16 # 16-bit output, 100Hz continuous measurement

# 변환 상수
ACCEL_FS_SENSITIVITY_2G = 16384.0
GYRO_FS_SENSITIVITY_250DPS = 131.0
MAG_RAW_TO_MICRO_TESLA = 4912.0 / 32760.0
G_MPS2 = 9.80665

accel_offset = {'x': 248.92, 'y': 55.58, 'z': 15961.62}
gyro_offset = {'x': 308.285, 'y': 56.88, 'z': -172.91}
mag_offset = {'x': -106.22, 'y': 254.815, 'z': 134.35}

# 가속도 (m/s^2)
accel_offset_si = {k: v / ACCEL_FS_SENSITIVITY_2G * G_MPS2 for k, v in accel_offset.items()}
# 자이로 (deg/s)
gyro_offset_si = {k: v / GYRO_FS_SENSITIVITY_250DPS for k, v in gyro_offset.items()}
# 자기장 (μT)
mag_offset_si = {k: v * MAG_RAW_TO_MICRO_TESLA for k, v in mag_offset.items()}

print("Accel Offsets (m/s^2):", {k: f"{v:.4f} m/s^2" for k, v in accel_offset_si.items()})
print("Gyro Offsets (deg/s):", {k: f"{v:.4f} deg/s" for k, v in gyro_offset_si.items()})
print("Mag Offsets (μT):", {k: f"{v:.4f} μT" for k, v in mag_offset_si.items()})

# Helper functions to R/W AK8963 registers via MPU9250
def write_ak8963_reg(bus, reg, val):
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_ADDR, AK8963_ADDR) # Set AK8963 for write
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_REG, reg)
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_DO, val)
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_CTRL, 0x81) # Enable SLV0 and write 1 byte
    time.sleep(0.05)

def read_ak8963_regs(bus, reg_start, num_bytes):
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_ADDR, AK8963_ADDR | 0x80) # Set AK8963 for read
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_REG, reg_start)
    bus.write_byte_data(MPU9250_ADDR, I2C_SLV0_CTRL, 0x80 | num_bytes) # Enable SLV0 and read num_bytes
    time.sleep(0.05)
    data = []
    for i in range(num_bytes):
        data.append(bus.read_byte_data(MPU9250_ADDR, EXT_SENS_DATA_00 + i))
    return data

def main():
    bus = smbus.SMBus(1)

    # Wake up MPU9250
    bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)

    # Enable I2C Master mode and Bypass mode (to directly access AK8963 initially for setup if needed,
    # but for MPU9250 master control, bypass should be disabled after AK8963 setup via MPU's master I2C)
    # It's generally better to disable bypass and use MPU9250's I2C master interface.
    bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x02) # Set BYPASS_EN to allow direct access to AK8963 (if needed for complex init)
    time.sleep(0.1)
    # Check AK8963 WIA (optional, direct access)
    # ak_wia = bus.read_byte_data(AK8963_ADDR, AK8963_WIA)
    # print(f"AK8963 WIA (direct): {hex(ak_wia)}") # Should be 0x48

    # Disable Bypass mode, enable MPU9250 I2C Master
    bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x00) # Disable BYPASS_EN
    time.sleep(0.1)
    bus.write_byte_data(MPU9250_ADDR, USER_CTRL, 0x20)   # Enable I2C_MST_EN
    time.sleep(0.1)

    # Initialize AK8963
    try:
        # Reset AK8963
        write_ak8963_reg(bus, AK8963_CNTL2, 0x01) # Soft reset
        time.sleep(0.1)
        # Set AK8963 to continuous measurement mode, 100Hz, 16-bit
        write_ak8963_reg(bus, AK8963_CNTL1, AK8963_MODE_CONT_MEAS_100HZ)
        time.sleep(0.1)
        print("AK8963 initialized for 100Hz continuous measurement (16-bit).")
    except Exception as e:
        print(f"Error initializing AK8963: {e}")
        print("Ensure MPU9250 I2C master is configured correctly and AK8963 is connected.")
        return

    def read_mpu_raw_data(addr):
        high = bus.read_byte_data(MPU9250_ADDR, addr)
        low = bus.read_byte_data(MPU9250_ADDR, addr + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read_raw_mag_data():
        # Read ST1 to ensure data is ready (optional, but good practice)
        # st1 = read_ak8963_regs(bus, AK8963_ST1, 1)[0]
        # if not (st1 & 0x01): # Check DRDY bit
        #     # print("AK8963 data not ready")
        #     return None, None, None # Or handle error

        raw_data = read_ak8963_regs(bus, AK8963_HXL, 7) # Read 6 data bytes + ST2
        
        # Check ST2 for overflow
        if raw_data[6] & 0x08: # HOFL bit
            # print("AK8963 magnetic sensor overflow")
            # Potentially return error or last known good values
            pass

        mx = (raw_data[1] << 8) | raw_data[0]
        my = (raw_data[3] << 8) | raw_data[2]
        mz = (raw_data[5] << 8) | raw_data[4]

        if mx > 32767: mx -= 65536
        if my > 32767: my -= 65536
        if mz > 32767: mz -= 65536
        return mx, my, mz

    print("Calibrating MPU9250... Keep sensor steady.")
    time.sleep(2)

    acc_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    gyro_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    mag_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    samples = 200 # Increased samples for better averaging

    print(f"Collecting {samples} samples...")
    for i in range(samples):
        acc_offsets['x'] += read_mpu_raw_data(0x3B)
        acc_offsets['y'] += read_mpu_raw_data(0x3D)
        acc_offsets['z'] += read_mpu_raw_data(0x3F)
        gyro_offsets['x'] += read_mpu_raw_data(0x43)
        gyro_offsets['y'] += read_mpu_raw_data(0x45)
        gyro_offsets['z'] += read_mpu_raw_data(0x47)
        
        mx, my, mz = read_raw_mag_data()
        if mx is not None: # Check if data was valid
            mag_offsets['x'] += mx
            mag_offsets['y'] += my
            mag_offsets['z'] += mz
        else: # Decrement sample count if mag data failed, or handle differently
            print(f"Warning: Failed to read mag data on sample {i}")


        if (i+1) % (samples // 10) == 0:
            print(f"Collected {i+1}/{samples} samples...")
        time.sleep(0.02) # Adjust sleep time as needed, considering 100Hz mag rate

    for key in acc_offsets:
        acc_offsets[key] /= samples
    for key in gyro_offsets:
        gyro_offsets[key] /= samples
    for key in mag_offsets:
        mag_offsets[key] /= samples
    
    # Z-axis gravity compensation for accelerometer (1g ~ 16384 for default sensitivity)
    # This depends on the orientation during calibration. If flat, Z should be ~1g.
    # acc_offsets['z'] -= 16384 # Or +16384 depending on Z-axis direction and gravity reading

    calib_data = {
        "accel_offset": acc_offsets,
        "gyro_offset": gyro_offsets,
        "mag_offset": mag_offsets
    }
    
    file_path = Path.home().joinpath(".mpu9250_calib.json")
    file_path.write_text(json.dumps(calib_data, indent=4))
    print(f"Calibration done! Saved to {file_path}")
    print(f"Accel Offsets (LSB): {acc_offsets}")
    print(f"Gyro Offsets (LSB): {gyro_offsets}")
    print(f"Mag Offsets (LSB): {mag_offsets}")

    # ---- 사람이 읽는 단위로 변환해서 출력 ----
    accel_offset_si = {k: v / ACCEL_FS_SENSITIVITY_2G * G_MPS2 for k, v in acc_offsets.items()}
    gyro_offset_si = {k: v / GYRO_FS_SENSITIVITY_250DPS for k, v in gyro_offsets.items()}
    mag_offset_si = {k: v * MAG_RAW_TO_MICRO_TESLA for k, v in mag_offsets.items()}

    print("Accel Offsets (m/s^2):", {k: f"{v:.4f} m/s^2" for k, v in accel_offset_si.items()})
    print("Gyro Offsets (deg/s):", {k: f"{v:.4f} deg/s" for k, v in gyro_offset_si.items()})
    print("Mag Offsets (μT):", {k: f"{v:.4f} μT" for k, v in mag_offset_si.items()})

if __name__ == '__main__':
    main()