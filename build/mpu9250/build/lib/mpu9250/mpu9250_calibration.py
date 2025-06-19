import smbus
from imusensor.MPU9250.MPU9250 import MPU9250
import time
import os
import json

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250(bus, address)
imu.begin()

print("MPU9250 센서 초기화 완료.")

# 기록용 리스트
accel_log = []
gyro_log = []
mag_log = []

# 자이로스코프 보정 시작
print("\n자이로스코프(Gyro) 보정을 시작합니다. 센서를 절대 움직이지 마세요! (약 20초 소요)")
time.sleep(2)
for _ in range(200):
    imu.readSensor()
    gyro_log.append(list(getattr(imu, "GyroVals", [0.0, 0.0, 0.0])))
    time.sleep(0.1)
imu.caliberateGyro()
print("자이로스코프 보정 완료!\n")
time.sleep(1)

# 가속도계 보정 시작 (데이터 수집 + 보정)
print("가속도계(Accelerometer) 보정을 위한 데이터 수집을 시작합니다.")
print("아래 안내에 따라, 6방향(+X, -X, +Y, -Y, +Z, -Z)에서 각각 엔터를 눌러주세요.")

# 1단계: accel_log 수집 (플로팅용)
print("\n--- accel_log 기록용 데이터 수집 시작 (참고용) ---")
print("센서의 물리적 축을 기준으로 아래 안내에 따라 6방향으로 센서를 위치시키고 엔터를 눌러주세요.")
physical_directions_prompt_logging = [
    "1. [기록용] 센서의 물리적 +X축이 아래를 향하도록 (중력이 물리적 +X축으로 작용)",
    "2. [기록용] 센서의 물리적 -X축이 아래를 향하도록 (중력이 물리적 -X축으로 작용)",
    "3. [기록용] 센서의 물리적 +Y축이 아래를 향하도록 (중력이 물리적 +Y축으로 작용)",
    "4. [기록용] 센서의 물리적 -Y축이 아래를 향하도록 (중력이 물리적 -Y축으로 작용)",
    "5. [기록용] 센서의 물리적 +Z축이 아래를 향하도록 (중력이 물리적 +Z축으로 작용)",
    "6. [기록용] 센서의 물리적 -Z축이 아래를 향하도록 (중력이 물리적 -Z축으로 작용)"
]
for i in range(6):
    input(f"{physical_directions_prompt_logging[i]}. 준비되면 엔터를 누르세요...")
    for _ in range(40): # 각 방향에서 40회 샘플링
        imu.readSensor()
        # transformationMatrix가 단위행렬이므로 AccelVals는 물리축 값과 동일
        accel_log.append(list(getattr(imu, "AccelVals", [0.0, 0.0, 0.0])))
        time.sleep(0.05)
print("--- accel_log 기록용 데이터 수집 완료 ---\n")


print("데이터 수집 완료! 이제 실제 보정을 시작합니다.")
print("MPU9250 라이브러리의 안내에 따라 센서를 다시 한번 각 방향으로 위치시켜주세요.")
print("!!! 중요: MPU9250.py의 transformationMatrix가 단위 행렬로 수정되었다고 가정합니다. !!!")
print("라이브러리가 요청하는 'position'은 센서의 '물리적' 축을 의미합니다.")
print("아래 순서대로 라이브러리 안내에 맞춰 센서의 '물리적' 방향을 설정하세요:")

calibration_physical_directions_prompt_actual = [
    "1. 라이브러리가 'position 1' (+X 물리축) 요청 시: 센서의 물리적 +X축을 아래로 향하게 하세요.",
    "2. 라이브러리가 'position 2' (-X 물리축) 요청 시: 센서의 물리적 -X축을 아래로 향하게 하세요.",
    "3. 라이브러리가 'position 3' (+Y 물리축) 요청 시: 센서의 물리적 +Y축을 아래로 향하게 하세요.",
    "4. 라이브러리가 'position 4' (-Y 물리축) 요청 시: 센서의 물리적 -Y축을 아래로 향하게 하세요.",
    "5. 라이브러리가 'position 5' (+Z 물리축) 요청 시: 센서의 물리적 +Z축을 아래로 향하게 하세요.",
    "6. 라이브러리가 'position 6' (-Z 물리축) 요청 시: 센서의 물리적 -Z축을 아래로 향하게 하세요."
]
for prompt in calibration_physical_directions_prompt_actual:
    print(prompt)

# 2단계: 실제 보정 (라이브러리 함수)
# imu.caliberateAccelerometer() 함수 내부에서 "Put the IMU in X position..." 메시지가 나옵니다.
# 사용자는 이 메시지에 맞춰 위의 calibration_physical_directions_prompt_actual의 순서대로 센서의 '물리적' 방향을 설정해야 합니다.
imu.caliberateAccelerometer()
print("\n가속도계 보정 완료!\n")
time.sleep(1)

# 자기장 센서 보정 시작
print("자기장 센서(Magnetometer) 보정을 시작합니다.")
print("센서를 손에 들고 8자 형태로 천천히 약 30~40초 동안 움직이세요.")
input("준비되었으면 엔터를 누르세요...")
for _ in range(400):
    imu.readSensor()
    mag_log.append(list(getattr(imu, "MagVals", [0.0, 0.0, 0.0])))
    time.sleep(0.1)
imu.caliberateMagPrecise()
print("자기장 센서 보정 완료!\n")

# 보정 데이터 파일 저장
calib_dir = os.path.expanduser("~/ros2_ws/src/calib/")
os.makedirs(calib_dir, exist_ok=True)
calib_file = os.path.join(calib_dir, "mpu9250_calib.json")
imu.saveCalibDataToFile(calib_file)
print(f"모든 보정이 완료되었습니다. 보정 데이터를 {calib_file}에 저장했습니다.")

# 원시 데이터 기록 저장 (플로팅용)
with open(os.path.join(calib_dir, "accel_log.json"), "w") as f:
    json.dump(accel_log, f)
with open(os.path.join(calib_dir, "gyro_log.json"), "w") as f:
    json.dump(gyro_log, f)
with open(os.path.join(calib_dir, "mag_log.json"), "w") as f:
    json.dump(mag_log, f)
print(f"원시 센서 데이터(accel, gyro, mag)를 {calib_dir}에 저장했습니다.")

def main():
    pass

if __name__ == '__main__':
    main()
