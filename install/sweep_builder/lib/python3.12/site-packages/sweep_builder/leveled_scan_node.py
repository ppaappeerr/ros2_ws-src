# leveled_scan_node.py  (패키지: sweep_builder)
import rclpy, numpy as np, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
class LevelScan(Node):
    def __init__(self):
        super().__init__('level_scan')
        self.roll = self.pitch = 0.0
        self.declare_parameter('in_scan',  '/scan')
        self.declare_parameter('out_scan', '/scan_leveled')
        self.sub_scan = self.create_subscription(
            LaserScan, self.get_parameter('in_scan').value, self.scan_cb, 10)
        self.sub_imu = self.create_subscription(
            Imu, '/imu_filtered', self.imu_cb, 50)
        self.pub = self.create_publisher(LaserScan,
                                         self.get_parameter('out_scan').value, 10)

    def imu_cb(self, m):
        from tf_transformations import euler_from_quaternion
        self.roll, self.pitch, _ = euler_from_quaternion([m.orientation.x,
                                                          m.orientation.y,
                                                          m.orientation.z,
                                                          m.orientation.w])

    def scan_cb(self, m):
        out = LaserScan()
        out.header = m.header
        out.header.frame_id = m.header.frame_id  # 원본 frame_id 유지
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        factor = math.cos(self.roll) * math.cos(self.pitch)
        
        # NaN/inf 값 처리 추가
        ranges_np = np.array(m.ranges, dtype=np.float32)
        # 유효한 범위 값만 필터링 (min_range, max_range 사이)
        valid_range_indices = np.logical_and(ranges_np > out.range_min, ranges_np < out.range_max)
        
        # factor 적용
        ranges_np_leveled = ranges_np * factor
        
        # NaN, inf 값 및 유효 범위를 벗어난 값 처리
        # isfinite는 NaN과 inf를 False로 처리합니다.
        # 보정된 값이 min/max_range를 벗어나는 경우도 처리합니다.
        final_valid_indices = np.logical_and(np.isfinite(ranges_np_leveled),
                                             np.logical_and(ranges_np_leveled > out.range_min,
                                                            ranges_np_leveled < out.range_max))
        
        # 유효하지 않은 값을 가진 인덱스를 찾습니다.
        # (원본부터 유효하지 않았거나, 보정 후 유효하지 않게 된 경우 모두 포함)
        invalid_indices = np.logical_not(final_valid_indices)

        if np.any(invalid_indices):
            # self.get_logger().warn(f"Invalid range values after leveling! Count: {np.sum(invalid_indices)}")
            # 유효하지 않은 값을 range_max 또는 특정 값으로 대체 (또는 inf로 두어 SLAM이 처리하게 할 수도 있음)
            ranges_np_leveled[invalid_indices] = out.range_max  # 또는 float('inf')
        
        out.ranges = ranges_np_leveled.tolist()
        out.intensities = m.intensities
        self.pub.publish(out)

def main():
    rclpy.init(); rclpy.spin(LevelScan()); rclpy.shutdown()
if __name__ == '__main__':
    main()
