import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class ImuTfPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        
        # 파라미터 선언
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_imu_orientation', False)
        self.declare_parameter('position_drift_correction', 0.99)  # 위치 드리프트 보정 계수
        self.declare_parameter('velocity_drift_correction', 0.95)  # 속도 드리프트 보정 계수
        
        # 파라미터 로드
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.use_imu_orientation = self.get_parameter('use_imu_orientation').value
        self.position_drift_coefficient = self.get_parameter('position_drift_correction').value
        self.velocity_drift_coefficient = self.get_parameter('velocity_drift_correction').value
        
        # IMU 구독
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
            
        # TF 브로드캐스터
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 위치, 속도, 방향 초기화
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        
        # 이전 시간 초기화
        self.prev_time = None
        
        # 누적 에러 보정을 위한 타이머
        self.create_timer(1.0, self.correction_callback)
        
        self.get_logger().info('IMU TF Publisher initialized')
    
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        
        # 첫 메시지인 경우
        if self.prev_time is None:
            self.prev_time = current_time
            
            # IMU 방향 정보가 있고 사용하기로 설정된 경우
            if self.use_imu_orientation:
                self.orientation = np.array([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ])
            return
        
        # 시간 간격 계산
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:  # 너무 크거나 작은 dt는 무시
            self.prev_time = current_time
            return
        
        # 쿼터니언 정규화
        if self.use_imu_orientation:
            q_norm = math.sqrt(
                msg.orientation.x**2 + 
                msg.orientation.y**2 + 
                msg.orientation.z**2 + 
                msg.orientation.w**2
            )
            
            if q_norm > 0.01:
                self.orientation = np.array([
                    msg.orientation.x / q_norm,
                    msg.orientation.y / q_norm,
                    msg.orientation.z / q_norm,
                    msg.orientation.w / q_norm
                ])
        
        # 회전 행렬 계산
        R = self.quaternion_to_rotation_matrix(self.orientation)
        
        # 가속도에서 중력 제거
        gravity = np.array([0.0, 0.0, 9.81])
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        # 글로벌 좌표계로 변환된 가속도 (중력 제거)
        global_accel = np.dot(R, linear_accel) - gravity
        
        # 속도 및 위치 업데이트
        self.velocity += global_accel * dt
        self.position += self.velocity * dt
        
        # 드리프트 보정 (감쇠 적용)
        self.velocity *= self.velocity_drift_coefficient
        
        # 가끔씩 로깅 (로그 과다 방지)
        if current_time.nanoseconds % 1000000000 < 50000000:  # 약 1초에 한 번
            self.get_logger().debug(f"Position: [{self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}]")
            self.get_logger().debug(f"Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f}]")
        
        # TF 발행
        self.publish_transform()
        
        # 현재 시간 저장
        self.prev_time = current_time
    
    def quaternion_to_rotation_matrix(self, q):
        """쿼터니언을 회전 행렬로 변환"""
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
            [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        return R
    
    def correction_callback(self):
        """정기적인 누적 에러 보정"""
        # 위치 드리프트 완화
        self.position *= self.position_drift_coefficient
        
        # 한계값 이상으로 멀어졌을 경우 리셋
        distance = np.linalg.norm(self.position)
        if distance > 100.0:  # 100m 이상 멀어지면 위치 리셋
            self.get_logger().warn(f"Position drift too large ({distance:.1f}m), resetting position")
            self.position = np.array([0.0, 0.0, 0.0])
            self.velocity = np.array([0.0, 0.0, 0.0])
    
    def publish_transform(self):
        """TF 변환 메시지 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        
        # 위치 설정
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        
        # 방향 설정
        t.transform.rotation.x = self.orientation[0]
        t.transform.rotation.y = self.orientation[1]
        t.transform.rotation.z = self.orientation[2]
        t.transform.rotation.w = self.orientation[3]
        
        # TF 발행
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()