import json
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Accelerometer plot
try:
    with open('accel_log.json') as f:
        accel = json.load(f)
    accel = list(map(list, zip(*accel)))  # x, y, z 분리

    plt.figure(figsize=(12, 4))
    plt.plot(accel[0], label='Accel X')
    plt.plot(accel[1], label='Accel Y')
    plt.plot(accel[2], label='Accel Z')
    plt.legend()
    plt.title('Accelerometer (m/s²)')
    plt.xlabel('Sample')
    plt.ylabel('Acceleration (m/s²)')
    plt.grid(True)
    plt.show()
except FileNotFoundError:
    print("accel_log.json 파일을 찾을 수 없습니다.")

# Gyroscope plot
try:
    with open('gyro_log.json') as f:
        gyro = json.load(f)
    gyro = list(map(list, zip(*gyro)))  # x, y, z 분리

    plt.figure(figsize=(12, 4))
    plt.plot(gyro[0], label='Gyro X')
    plt.plot(gyro[1], label='Gyro Y')
    plt.plot(gyro[2], label='Gyro Z')
    plt.legend()
    plt.title('Gyroscope (rad/s)')
    plt.xlabel('Sample')
    plt.ylabel('Angular velocity (rad/s)')
    plt.grid(True)
    plt.show()
except FileNotFoundError:
    print("gyro_log.json 파일을 찾을 수 없습니다.")

# Magnetometer 3D plot
try:
    with open('mag_log.json') as f:
        mag = np.array(json.load(f))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(mag[:,0], mag[:,1], mag[:,2], s=2, alpha=0.6)
    ax.set_xlabel('Mag X')
    ax.set_ylabel('Mag Y')
    ax.set_zlabel('Mag Z')
    ax.set_title('Magnetometer 3D Distribution')
    plt.show()
except FileNotFoundError:
    print("mag_log.json 파일을 찾을 수 없습니다.")