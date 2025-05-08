#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, Twist, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

class ImuOdomPublisher(Node):
    def __init__(self):
        super().__init__('imu_odom_publisher')

        # 파라미터 선언
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu_filtered') # launch 파일에서 리매핑됨
        self.declare_parameter('odom_topic', '/odom')

        # 파라미터 가져오기
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        odom_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Cartographer는 RELIABLE 선호
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # IMU 구독 (launch 파일에서 리매핑된 토픽 사용)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile
        )

        # TF 브로드캐스터 제거
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometry 퍼블리셔
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, odom_qos_profile)

        self.get_logger().info(f"IMU Odom Publisher 시작: Odom({self.odom_topic}) 메시지만 발행 (TF 발행 안 함)")

    def imu_callback(self, msg):
        """IMU 메시지 처리 및 Odometry 발행"""
        current_time = msg.header.stamp # IMU 메시지의 타임스탬프 사용

        # Odometry 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose: 위치는 (0,0,0), 회전은 단위 쿼터니언 (IMU만으로는 알 수 없음)
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        # Pose Covariance: 위치/회전 불확실성 매우 높음
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 1e6  # x pos
        odom_msg.pose.covariance[7] = 1e6  # y pos
        odom_msg.pose.covariance[14] = 1e6 # z pos
        odom_msg.pose.covariance[21] = 1e6 # roll rot
        odom_msg.pose.covariance[28] = 1e6 # pitch rot
        odom_msg.pose.covariance[35] = 1e6 # yaw rot

        # Twist: 선속도는 0, 각속도는 IMU 값 사용
        odom_msg.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = msg.angular_velocity # IMU의 각속도 사용
        # Twist Covariance: 선속도 불확실성 매우 높음, 각속도는 IMU 노이즈 반영
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 1e6  # x vel
        odom_msg.twist.covariance[7] = 1e6  # y vel
        odom_msg.twist.covariance[14] = 1e6 # z vel
        # 각속도 공분산은 IMU 메시지의 값 사용 (존재한다면)
        if len(msg.angular_velocity_covariance) == 9 and msg.angular_velocity_covariance[0] > 0:
             odom_msg.twist.covariance[21] = msg.angular_velocity_covariance[0] # roll vel
             odom_msg.twist.covariance[28] = msg.angular_velocity_covariance[4] # pitch vel
             odom_msg.twist.covariance[35] = msg.angular_velocity_covariance[8] # yaw vel
        else: # 없다면 임의의 작은 값 설정
             odom_msg.twist.covariance[21] = 0.01
             odom_msg.twist.covariance[28] = 0.01
             odom_msg.twist.covariance[35] = 0.01 # Yaw velocity might be less certain

        self.odom_pub.publish(odom_msg) # Odometry 메시지 발행

        # TF 발행 로직 제거됨

# --- main 함수는 이전과 동일하게 유지 ---
def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomPublisher()
    try:
        rclpy.spin(node) # 노드가 계속 실행되도록 유지
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        # 노드 종료 및 rclpy 종료
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    # math 임포트 확인
    try:
        import math
    except ImportError as e:
        print(f"Failed to import math: {e}")
        exit()
    # rclpy 임포트 확인
    try:
        import rclpy
    except ImportError as e:
        print(f"Failed to import rclpy: {e}")
        exit()
    main()