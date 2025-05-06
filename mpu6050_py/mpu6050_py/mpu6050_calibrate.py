import smbus
import time
import json
from pathlib import Path

def main():
    bus = smbus.SMBus(1)
    address = 0x68
    bus.write_byte_data(address, 0x6B, 0x00)  # Wake up sensor

    def read_raw_data(addr):
        high = bus.read_byte_data(address, addr)
        low = bus.read_byte_data(address, addr+1)
        value = ((high << 8) | low)
        if value > 32767:
            value -= 65536
        return value

    print("Calibrating... Keep MPU6050 steady for 5 seconds")
    time.sleep(2)

    acc_offsets = {'x': 0, 'y': 0, 'z': 0}
    gyro_offsets = {'x': 0, 'y': 0, 'z': 0}
    samples = 100

    for i in range(samples):
        acc_offsets['x'] += read_raw_data(0x3B)
        acc_offsets['y'] += read_raw_data(0x3D)
        acc_offsets['z'] += read_raw_data(0x3F)
        gyro_offsets['x'] += read_raw_data(0x43)
        gyro_offsets['y'] += read_raw_data(0x45)
        gyro_offsets['z'] += read_raw_data(0x47)
        time.sleep(0.01)

    for key in acc_offsets:
        acc_offsets[key] /= samples
    for key in gyro_offsets:
        gyro_offsets[key] /= samples

    # Z축 중력 보정 (9.81 m/s² ≈ raw 16384)
    # acc_offsets['z'] -= 16384 # EKF 사용시 해제 후 사용

    # Save to JSON
    calib_data = {"accel_offset": acc_offsets, "gyro_offset": gyro_offsets}
    Path.home().joinpath(".mpu6050_calib.json").write_text(json.dumps(calib_data, indent=2))
    print("Calibration done! Saved to ~/.mpu6050_calib.json")

if __name__ == '__main__':
    main()

