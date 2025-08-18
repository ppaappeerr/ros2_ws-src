import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
from .imusensor.MPU9250.MPU9250 import MPU9250
from .imusensor.filters.kalman import Kalman
import smbus
import numpy as np
import json
import time

class MPU9250KalmanNode(Node):
    def __init__(self):
        super().__init__('mpu9250_kalman_node')
        
        # 매개변수 선언
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        
        # 매개변수 가져오기
        calib_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # I2C 버스 초기화
        try:
            bus = smbus.SMBus(1)
            self.imu = MPU9250(bus, 0x68)
            self.imu.begin()
            
            # 캘리브레이션 데이터 로드
            if calib_path and calib_path != '':
                try:
                    with open(calib_path, 'r') as f:
                        calib_data = json.load(f)
                        self.imu.loadCalibDataFromJSON(calib_data)
                        self.get_logger().info(f'Calibration loaded from: {calib_path}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to load calibration: {e}')
            
            self.get_logger().info('MPU9250 initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU9250: {e}')
            return
        
        # Kalman 필터 초기화
        self.kalman_filter = Kalman()
        
        # 퍼블리셔 설정
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # 타이머 설정
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        # 이전 시간 저장
        self.prev_time = time.time()
        
        self.get_logger().info('MPU9250 Kalman Filter Node started')
    
    def publish_imu_data(self):
        try:
            # 센서 데이터 읽기
            self.imu.readSensor()
            
            # 현재 시간 및 dt 계산
            current_time = self.get_clock().now()
            current_time_sec = time.time()
            dt = current_time_sec - self.prev_time
            self.prev_time = current_time_sec
            
            # Kalman 필터 업데이트
            self.kalman_filter.computeAndUpdateRollPitchYaw(
                self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2],
                self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2],
                self.imu.MagVals[0], self.imu.MagVals[1], self.imu.MagVals[2],
                dt)
            
            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Kalman 필터 결과를 라디안으로 변환
            roll_rad = np.radians(self.kalman_filter.roll)
            pitch_rad = np.radians(self.kalman_filter.pitch)
            yaw_rad = np.radians(self.kalman_filter.yaw)
            
            # 쿼터니언으로 변환
            quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            # 각속도 데이터 (rad/s로 변환)
            imu_msg.angular_velocity.x = float(np.radians(self.imu.GyroVals[0]))
            imu_msg.angular_velocity.y = float(np.radians(self.imu.GyroVals[1]))
            imu_msg.angular_velocity.z = float(np.radians(self.imu.GyroVals[2]))
            
            # 가속도 데이터 (m/s^2로 변환)
            imu_msg.linear_acceleration.x = float(self.imu.AccelVals[0] * 9.81)
            imu_msg.linear_acceleration.y = float(self.imu.AccelVals[1] * 9.81)
            imu_msg.linear_acceleration.z = float(self.imu.AccelVals[2] * 9.81)
            
            # 공분산 설정 (임시값)
            imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            # 메시지 발행
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading IMU data: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MPU9250KalmanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()