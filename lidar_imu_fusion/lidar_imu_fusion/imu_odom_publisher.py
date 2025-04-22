#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ImuOdomPublisher(Node): # 클래스 이름 변경 (역할 명확화)
    def __init__(self):
        super().__init__('imu_odom_publisher') # 노드 이름 변경

        # 파라미터 선언 (odom 프레임 기준)
        self.declare_parameter('odom_frame', 'odom') # 부모 프레임: odom
        self.declare_parameter('base_frame', 'base_link') # 자식 프레임: base_link
        self.declare_parameter('imu_topic', '/imu/data') # 구독할 필터링된 IMU 토픽

        # 파라미터 가져오기
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value

        # QoS 설정 (센서 데이터용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE # 센서 데이터는 VOLATILE
        )

        # IMU 구독 (필터링된 데이터 구독)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile # QoS 적용
        )

        # TF 브로드캐스터
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 상태 변수
        self.last_orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0) # 마지막 유효 방향 저장

        self.get_logger().info(f"IMU Odom Publisher 시작: {self.odom_frame} -> {self.base_frame} (회전만)")

    def imu_callback(self, msg):
        """IMU 메시지 처리 및 TF 발행"""
        # 유효한 orientation 데이터 확인
        if msg.orientation.w == 0.0 and msg.orientation.x == 0.0 and \
           msg.orientation.y == 0.0 and msg.orientation.z == 0.0:
            self.get_logger().warn("수신된 IMU 방향 데이터가 유효하지 않습니다. 이전 방향 사용.")
            orientation_to_publish = self.last_orientation
        else:
            # 정규화 추가 (Madgwick 필터가 이미 정규화하지만 안전장치)
            q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
            norm = np.linalg.norm(q)
            if norm > 1e-6:
                q = q / norm
                orientation_to_publish = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
                self.last_orientation = orientation_to_publish # 유효한 경우 마지막 방향 업데이트
            else:
                self.get_logger().warn("수신된 IMU 방향 데이터 정규화 실패. 이전 방향 사용.")
                orientation_to_publish = self.last_orientation


        # TF 변환 메시지 생성
        t = TransformStamped()
        t.header.stamp = msg.header.stamp # IMU 메시지의 타임스탬프 사용 (중요)
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # 위치는 (0, 0, 0)으로 고정
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # 방향 설정 (IMU 데이터 사용)
        t.transform.rotation = orientation_to_publish

        # TF 발행
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()