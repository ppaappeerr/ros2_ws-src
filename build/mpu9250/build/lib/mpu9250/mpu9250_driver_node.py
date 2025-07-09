# 파일 위치: ~/ros2_ws/src/mpu9250/mpu9250/mpu9250_driver_node.py
# (기존 mpu9250_filtered.py를 이 파일로 대체하거나, launch 파일에서 실행 파일 이름 변경)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import smbus
import numpy as np
import os

# 올바른 import 방식 및 Madgwick 필터 추가
from .imusensor.MPU9250.MPU9250 import MPU9250
from ahrs.filters import Madgwick

class MPU9250DriverNode(Node):
    def __init__(self):
        super().__init__('mpu9250_driver_node')
        
        # 파라미터 선언
        self.declare_parameter('calibration_path', os.path.expanduser('~/ros2_ws/src/calib/mpu9250_calib.json'))
        self.declare_parameter('frame_id', 'base_link') # IMU와 base_link를 동일 프레임으로 가정
        self.declare_parameter('publish_rate', 100.0)
        
        # 파라미터 가져오기
        calib_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # I2C 버스 및 MPU9250 초기화
        try:
            bus = smbus.SMBus(1)
            self.imu = MPU9250(bus, 0x68)
            self.imu.begin()
            
            # [수정] 보정 데이터 로드 (올바른 함수 이름 사용)
            if os.path.exists(calib_path):
                self.imu.loadCalibDataFromFile(calib_path)
                self.get_logger().info(f'성공: IMU 보정 파일을 로드했습니다: {calib_path}')
            else:
                self.get_logger().warn(f'경고: IMU 보정 파일을 찾을 수 없습니다: {calib_path}. 보정되지 않은 상태로 실행됩니다.')

        except Exception as e:
            self.get_logger().error(f'치명적 오류: MPU9250 초기화에 실패했습니다: {e}')
            return
            
        # [추가] Madgwick 필터 초기화
        self.madgwick = Madgwick(frequency=self.publish_rate)
        
        # 퍼블리셔 설정 (QoS: SensorData)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', rclpy.qos.qos_profile_sensor_data)
        
        # 타이머 설정
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)
        self.get_logger().info('MPU9250 Driver Node (with Madgwick Filter)가 시작되었습니다.')

    def publish_imu_data(self):
        try:
            self.imu.readSensor()
            
            # Madgwick 필터에 사용할 데이터 (rad/s, m/s^2)
            gyro_rad = np.radians(self.imu.GyroVals)
            accel_ms2 = np.array(self.imu.AccelVals) * 9.80665

            # 필터 업데이트 (자력계는 불안정하므로 사용 안 함)
            q = self.madgwick.updateIMU(self.madgwick.Q, gyr=gyro_rad, acc=accel_ms2)
            
            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # 방향 (Madgwick 필터 결과)
            imu_msg.orientation.w = q[0]
            imu_msg.orientation.x = q[1]
            imu_msg.orientation.y = q[2]
            imu_msg.orientation.z = q[3]
            
            # 각속도 (보정된 값, rad/s)
            imu_msg.angular_velocity.x = float(gyro_rad[0])
            imu_msg.angular_velocity.y = float(gyro_rad[1])
            imu_msg.angular_velocity.z = float(gyro_rad[2])
            
            # 가속도 (보정된 값, m/s^2)
            imu_msg.linear_acceleration.x = float(accel_ms2[0])
            imu_msg.linear_acceleration.y = float(accel_ms2[1])
            imu_msg.linear_acceleration.z = float(accel_ms2[2])
            
            # 공분산은 0으로 설정 (사용 안 함을 명시)
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0
            
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'IMU 데이터 읽기/발행 중 오류 발생: {e}', throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = MPU9250DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()