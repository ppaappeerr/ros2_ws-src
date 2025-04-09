import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from math import sin, cos, radians

class LidarImuCorrectionNode(Node):
    def __init__(self):
        super().__init__('lidar_imu_correction_node')

        # /imu 토픽에서 IMU 메시지 구독
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        # /scan 토픽에서 라이다 메시지 구독
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # 보정된 라이다 스캔을 퍼블리시할 퍼블리셔 생성
        self.pub_scan = self.create_publisher(LaserScan, '/scan_imu', 10)

        # 마지막 IMU에서 받은 yaw(회전속도 누적용)
        self.yaw_angle = 0.0

    def imu_callback(self, msg: Imu):
        # Angular velocity.z는 회전속도(rad/s)
        # 시간 간격 0.02초 (50Hz 가정) → 각도 누적
        dt = 0.02
        self.yaw_angle += msg.angular_velocity.z * dt

    def scan_callback(self, msg: LaserScan):
        # LaserScan 메시지 복사
        corrected_scan = LaserScan()
        corrected_scan.header = msg.header
        corrected_scan.angle_min = msg.angle_min
        corrected_scan.angle_max = msg.angle_max
        corrected_scan.angle_increment = msg.angle_increment
        corrected_scan.time_increment = msg.time_increment
        corrected_scan.scan_time = msg.scan_time
        corrected_scan.range_min = msg.range_min
        corrected_scan.range_max = msg.range_max
        corrected_scan.intensities = list(msg.intensities)

        # ranges를 회전 보정 적용
        corrected_scan.ranges = self.correct_laserscan_with_yaw(msg, self.yaw_angle)

        # 보정된 scan 퍼블리시
        self.pub_scan.publish(corrected_scan)

    def correct_laserscan_with_yaw(self, scan_data, yaw_angle_rad):
        # ranges 각도를 yaw_angle 만큼 보정한 값을 반환
        corrected_ranges = []
        angle = scan_data.angle_min

        for r in scan_data.ranges:
            # 범위 밖 값은 그대로
            if r < scan_data.range_min or r > scan_data.range_max:
                corrected_ranges.append(r)
                angle += scan_data.angle_increment
                continue

            # 극좌표 → 직교좌표
            x = r * cos(angle)
            y = r * sin(angle)

            # Yaw 회전 행렬 적용
            new_x = x * cos(yaw_angle_rad) - y * sin(yaw_angle_rad)
            new_y = x * sin(yaw_angle_rad) + y * cos(yaw_angle_rad)

            # 직교좌표 → 극좌표 거리로 복귀
            new_r = (new_x**2 + new_y**2)**0.5
            corrected_ranges.append(new_r)
            angle += scan_data.angle_increment

        return corrected_ranges

def main():
    rclpy.init()
    node = LidarImuCorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
