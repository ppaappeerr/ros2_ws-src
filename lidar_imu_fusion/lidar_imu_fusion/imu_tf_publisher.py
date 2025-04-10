import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

class ImuTfPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        
        # 파라미터 선언
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('position_drift_correction', 0.99)
        self.declare_parameter('velocity_drift_correction', 0.95)
        
        # 파라미터 가져오기
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.use_orientation = self.get_parameter('use_imu_orientation').value
        self.position_drift_correction = self.get_parameter('position_drift_correction').value
        self.velocity_drift_correction = self.get_parameter('velocity_drift_correction').value
        
        # IMU 토픽 구독
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # TF 브로드캐스터 설정
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 상태 변수 초기화
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.accel_filter = np.array([0.0, 0.0, 0.0])  # 필터링된 가속도
        
        self.last_time = None
        
        self.get_logger().info("IMU TF Publisher initialized!")
        
    def imu_callback(self, msg):
        current_time = time.time()
        
        # 첫 메시지인 경우 시간 초기화
        if self.last_time is None:
            self.last_time = current_time
            return
            
        # 시간 간격 계산 (초)
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0 or dt > 0.1:  # 너무 작거나 큰 dt는 무시
            return
            
        # IMU 방향 가져오기
        if self.use_orientation:
            self.orientation = np.array([
                msg.orientation.x,
                msg.orientation.y, 
                msg.orientation.z, 
                msg.orientation.w
            ])
            
            # 방향 정규화
            norm = np.linalg.norm(self.orientation)
            if norm > 0:
                self.orientation /= norm
        
        # 회전 행렬 계산
        R = self.quaternion_to_rotation_matrix(self.orientation)
        
        # 중력 제거
        gravity_vector = np.array([0.0, 0.0, 9.81])  # 지구 중력
        rotated_gravity = np.dot(R, gravity_vector)
        
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # 중력 보정
        global_accel = np.dot(R, linear_accel) - rotated_gravity
        
        # 저주파 통과 필터
        alpha = 0.2
        self.accel_filter = alpha * global_accel + (1 - alpha) * self.accel_filter
        
        # 정지 상태 감지
        is_stationary = np.linalg.norm(self.accel_filter) < 0.08
        
        # 속도 업데이트
        if is_stationary:
            self.velocity *= 0.8  # 정지 시 강한 감쇠
        else:
            self.velocity += self.accel_filter * dt
            self.velocity[0] *= 0.95  # x축 감쇠
            self.velocity[1] *= 0.95  # y축 감쇠
            self.velocity[2] *= 0.97  # z축 감쇠
        
        # 속도 제한
        max_velocity = 0.3  # m/s
        vel_norm = np.linalg.norm(self.velocity)
        if vel_norm > max_velocity:
            self.velocity *= (max_velocity / vel_norm)
        
        # 위치 업데이트
        self.position += self.velocity * dt
        
        # z축 보정
        if abs(self.position[2]) > 0.5:
            self.position[2] *= 0.99
            
        # 정지 상태에서 위치 드리프트 보정
        if is_stationary and np.linalg.norm(self.velocity) < 0.01:
            self.position[0] *= 0.999
            self.position[1] *= 0.999
            if abs(self.position[2]) < 0.2:
                self.position[2] *= 0.9995
                
        # TF 발행
        self.publish_transform()
    
    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q
        
        # 회전 행렬 계산
        xx = x * x
        xy = x * y
        xz = x * z
        xw = x * w
        
        yy = y * y
        yz = y * z
        yw = y * w
        
        zz = z * z
        zw = z * w
        
        R = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
            [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
            [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)]
        ])
        
        return R
        
    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ImuTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
