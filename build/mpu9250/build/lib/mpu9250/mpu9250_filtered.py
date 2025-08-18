import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import json
import time
import smbus
import numpy as np
from tf_transformations import quaternion_from_euler

# 올바른 import 방식
from .imusensor.MPU9250.MPU9250 import MPU9250

class MPU9250FilteredNode(Node):
    def __init__(self):
        super().__init__('mpu9250_filtered_node')
        
        # 매개변수 선언
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        
        # 매개변수 가져오기
        calib_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # I2C 버스 초기화
        try:
            bus = smbus.SMBus(1)
            self.imu = MPU9250(bus, 0x68)  # 수정된 부분
            self.imu.begin()
            
            # 캘리브레이션 데이터 로드
            if calib_path and calib_path != '':
                try:
                    with open(calib_path, 'r') as f:
                        calib_data = json.load(f)
                        self.imu.loadCalibDataFromFile(calib_path)  # 수정된 부분
                        self.get_logger().info(f'Calibration loaded from: {calib_path}')
                except Exception as e:
                    self.get_logger().warn(f'Failed to load calibration: {e}')
            
            self.get_logger().info('MPU9250 initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU9250: {e}')
            return
        
        # 퍼블리셔 설정
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # 타이머 설정
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        # 이전 시간 저장
        self.prev_time = time.time()
        
        self.get_logger().info('MPU9250 Filtered Node started')
    
    def publish_imu_data(self):
        try:
            # 센서 데이터 읽기
            self.imu.readSensor()
            
            # 현재 시간
            current_time = self.get_clock().now()
            
            # IMU 메시지 생성
            imu_msg = Imu()
            imu_msg.header.stamp = current_time.to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # 가속도 데이터 (m/s^2로 변환)
            imu_msg.linear_acceleration.x = float(self.imu.AccelVals[0] * 9.81)
            imu_msg.linear_acceleration.y = float(self.imu.AccelVals[1] * 9.81)
            imu_msg.linear_acceleration.z = float(self.imu.AccelVals[2] * 9.81)
            
            # 각속도 데이터 (rad/s로 변환)
            imu_msg.angular_velocity.x = float(np.radians(self.imu.GyroVals[0]))
            imu_msg.angular_velocity.y = float(np.radians(self.imu.GyroVals[1]))
            imu_msg.angular_velocity.z = float(np.radians(self.imu.GyroVals[2]))
            
            # 간단한 방향 계산 (가속도계 기반)
            # 실제 프로젝트에서는 더 정교한 필터가 필요
            roll = np.arctan2(self.imu.AccelVals[1], self.imu.AccelVals[2])
            pitch = np.arctan2(-self.imu.AccelVals[0], 
                             np.sqrt(self.imu.AccelVals[1]**2 + self.imu.AccelVals[2]**2))
            yaw = 0.0  # 자기계 없이는 yaw 계산 불가
            
            # 쿼터니언으로 변환
            quaternion = quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
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
        node = MPU9250FilteredNode()
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
