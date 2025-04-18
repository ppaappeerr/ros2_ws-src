#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import tf2_ros

class ImuTfPublisher(Node):
    def __init__(self):
        super().__init__('imu_tf_publisher')
        
        # 주요 파라미터
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        self.declare_parameter('stabilization_time', 2.0)  # 초기 안정화 시간
        
        # 좌표계 방향 교정 파라미터 수정 (전체 반전으로 변경) ## 센서 위치에 따라 다르게 사용
        self.declare_parameter('invert_roll', False)   # x축 회전
        self.declare_parameter('invert_pitch', False)  # y축 회전
        self.declare_parameter('invert_yaw', False)    # z축 회전
        
        # 드리프트 보정
        self.declare_parameter('drift_correction', 0.995) # 드리프트 보정 계수
        
        # 방향 유지 비활성화 (오뚝이 효과 제거)
        self.declare_parameter('maintain_orientation', False) # True의 경우 오뚝이 효과 발생

        # 0.999: 거의 유지, 0.0: 완전 초기화
        self.declare_parameter('orientation_decay', 0.999)    # 방향 감쇠율 (1.0이면 무한히 유지)
        
        # 파라미터 가져오기
        self.parent_frame = self.get_parameter('parent_frame').value # 상위 좌표계 기본값: 'map'
        self.child_frame = self.get_parameter('child_frame').value   # 하위 좌표계 기본값: 'base_link'
        self.publish_rate = self.get_parameter('publish_rate').value # TF 발행 주기 기본값: 20Hz
        self.stabilization_time = self.get_parameter('stabilization_time').value # 안정화 시간 기본값: 2초
        
        self.invert_roll = self.get_parameter('invert_roll').value
        self.invert_pitch = self.get_parameter('invert_pitch').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        
        self.drift_correction = self.get_parameter('drift_correction').value 
        # 드리프트 보정 계수 기본값: 0.995
        # 드리프트 보정 계수는 0.0~1.0 사이의 값으로 설정
        # 0.0: 드리프트 보정 없음, 1.0: 드리프트 완전 보정
        
        self.maintain_orientation = self.get_parameter('maintain_orientation').value
        self.orientation_decay = self.get_parameter('orientation_decay').value
        
        # 방향 정보를 직접 연산 (MPU6050이 방향 제공 안함)
        self.use_direct_computation = True
        
        # IMU 구독
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # TF 브로드캐스터
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 상태 변수
        self.position = np.zeros(3)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.angular_velocity = np.zeros(3)
        
        # 이전 방향 저장 (초기값: 기본 방향)
        self.prev_valid_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        
        # IMU 데이터 내역
        self.recent_imu_data = []
        self.max_imu_history = 10  # 최대 IMU 내역 길이
        
        # IMU 데이터 수신 시간 확인
        self.last_imu_time = self.get_clock().now()
        self.has_received_imu = False
        
        # 이전 시간 저장
        self.previous_time = None
        
        # 안정화 플래그
        self.is_stabilized = False
        self.start_time = self.get_clock().now()
        
        # 상태 출력 타이머
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        # TF 브로드캐스트 타이머
        self.tf_timer = self.create_timer(1.0/self.publish_rate, self.publish_transform)
        
        # 주석 추가 및 로그 개선
        self.get_logger().info(
            f"IMU TF Publisher 초기화 완료 - 방향 반전: roll={self.invert_roll}, "
            f"pitch={self.invert_pitch}, yaw={self.invert_yaw}, 방향유지={self.maintain_orientation}"
        )
    
    def print_status(self):
        """상태 정보 출력"""
        now = self.get_clock().now()
        if not self.has_received_imu:
            self.get_logger().warn("아직 IMU 데이터를 수신하지 않았습니다.")
            return
        
        imu_age = (now - self.last_imu_time).nanoseconds / 1e9
        if imu_age > 1.0:
            self.get_logger().warn(f"IMU 데이터가 {imu_age:.1f}초 동안 수신되지 않음")
        else:
            self.get_logger().debug(f"IMU 데이터 수신 중 (마지막: {imu_age:.3f}초 전)")
            
        if not self.is_stabilized:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed >= self.stabilization_time:
                self.is_stabilized = True
                self.get_logger().info("안정화 기간 완료. 정상 작동 중.")
            else:
                self.get_logger().debug(f"안정화 중... ({elapsed:.1f}/{self.stabilization_time})")
    
    def imu_callback(self, msg):
        """IMU 메시지 처리"""
        # 첫 데이터 수신 체크
        if not self.has_received_imu:
            self.has_received_imu = True
            self.get_logger().info("첫 IMU 데이터 수신")
        
        # 시간 업데이트
        current_time = self.get_clock().now()
        self.last_imu_time = current_time
        
        # 현재 IMU 데이터 저장
        self.recent_imu_data.append({
            'orientation': np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]),
            'timestamp': self.get_clock().now()
        })
        
        # 내역이 너무 길어지면 오래된 데이터 삭제
        if len(self.recent_imu_data) > self.max_imu_history:
            self.recent_imu_data.pop(0)
        
        # 방향 강제 계산 사용 (MPU6050은 orientation 제공 안함)
        if self.use_direct_computation or msg.orientation_covariance[0] < 0:
            if self.previous_time is not None:
                dt = (current_time - self.previous_time).nanoseconds / 1e9
                
                if dt > 0.0 and dt < 0.1:  # 타임스탬프 체크
                    # 각속도 저장 (하드웨어/배치에 맞춰 반전 적용)
                    wx = msg.angular_velocity.x * (-1.0 if self.invert_roll else 1.0)
                    wy = msg.angular_velocity.y * (-1.0 if self.invert_pitch else 1.0)
                    wz = msg.angular_velocity.z * (-1.0 if self.invert_yaw else 1.0)
                    
                    # 임계값 필터링
                    threshold = 0.01
                    if abs(wx) < threshold: wx = 0
                    if abs(wy) < threshold: wy = 0
                    if abs(wz) < threshold: wz = 0
                    
                    # 각속도 적분하여 방향 업데이트
                    roll_increment = wx * dt
                    pitch_increment = wy * dt
                    yaw_increment = wz * dt
                    
                    # 디버깅 로그 추가
                    if max(abs(wx), abs(wy), abs(wz)) > 0.1:  # 충분히 큰 움직임만 로그
                        self.get_logger().debug(
                            f"IMU 각속도: wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}, "
                            f"증분: roll={roll_increment:.4f}, pitch={pitch_increment:.4f}, yaw={yaw_increment:.4f}"
                        )
                    
                    # 오일러 각 변화를 쿼터니언으로 변환하여 곱하기
                    dq = self.euler_to_quaternion(roll_increment, pitch_increment, yaw_increment)
                    self.orientation = self.quaternion_multiply(dq, self.orientation)
                    self.orientation = self.normalize_quaternion(self.orientation)
        
        self.previous_time = current_time
        
        # 방향유지 로직 단순화
        if not self.maintain_orientation:
            # 방향유지 비활성화: 아무것도 하지 않음 (IMU에서 온 방향 그대로 사용)
            pass
        else:
            # 방향유지 활성화: 원래 로직 유지
            weight = self.orientation_decay
            combined_w = weight * self.prev_valid_orientation[0] + (1 - weight) * self.orientation[0]
            combined_x = weight * self.prev_valid_orientation[1] + (1 - weight) * self.orientation[1]
            combined_y = weight * self.prev_valid_orientation[2] + (1 - weight) * self.orientation[2]
            combined_z = weight * self.prev_valid_orientation[3] + (1 - weight) * self.orientation[3]
            
            self.orientation = self.normalize_quaternion(np.array([combined_w, combined_x, combined_y, combined_z]))
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """오일러 각을 쿼터니언으로 변환 (작은 각도에 최적화)"""
        # 작은 각도 근사 사용
        qw = 1.0 - (roll*roll + pitch*pitch + yaw*yaw) / 8.0
        qx = roll / 2.0
        qy = pitch / 2.0
        qz = yaw / 2.0
        
        return np.array([qw, qx, qy, qz])
    
    def quaternion_multiply(self, q1, q2):
        """두 쿼터니언 곱하기 (q1 * q2)"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def normalize_quaternion(self, q):
        """쿼터니언 정규화"""
        norm = np.linalg.norm(q)
        if norm < 1e-10:  # 0에 너무 가까우면
            return np.array([1.0, 0.0, 0.0, 0.0])  # 기본값
        return q / norm
    
    def publish_transform(self):
        """TF 변환 발행"""
        if not self.has_received_imu:
            return  # IMU 데이터 대기
        
        # 변환 메시지 생성
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        
        # 위치 설정 - IMU에서는 위치를 직접 측정할 수 없음
        # 안정화 이후에만 값 사용
        if self.is_stabilized:
            t.transform.translation.x = float(self.position[0])
            t.transform.translation.y = float(self.position[1])
            t.transform.translation.z = float(self.position[2])
        else:
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
        
        # 방향 설정 (w, x, y, z 순서)
        t.transform.rotation.w = float(self.orientation[0])
        t.transform.rotation.x = float(self.orientation[1])
        t.transform.rotation.y = float(self.orientation[2])
        t.transform.rotation.z = float(self.orientation[3])
        
        # 변환 발행
        self.tf_broadcaster.sendTransform(t)
        
        # 디버깅: 현재 방향 오일러각 출력
        roll, pitch, yaw = self.euler_from_quaternion(
            self.orientation[1], self.orientation[2], 
            self.orientation[3], self.orientation[0]
        )
        
        # 원점 처박힘 현상 디버깅
        if abs(t.transform.translation.z) > 1.0:  # z가 비정상적으로 큰 경우
            self.get_logger().warn(f"비정상 Z 위치: {t.transform.translation.z:.2f}m, 리셋 중...")
            self.position = np.zeros(3)  # 위치 리셋
            # 다음 발행 때 적용됨

    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언을 오일러 각으로 변환"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

    def print_tf_tree_status(self):
        """현재 TF 트리 상태 출력"""
        try:
            # 기본 TF 체크
            frames = [
                ('world', 'map'),
                ('map', 'base_link'),
                ('base_link', 'laser'),
                ('base_link', 'imu_link')
            ]
            
            status = []
            for parent, child in frames:
                try:
                    self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                    status.append(f"{parent}→{child}: ✓")
                except Exception:
                    status.append(f"{parent}→{child}: ✗")
                    
            self.get_logger().info(f"TF 트리 상태: {', '.join(status)}")
        except Exception as e:
            self.get_logger().error(f"TF 상태 확인 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()