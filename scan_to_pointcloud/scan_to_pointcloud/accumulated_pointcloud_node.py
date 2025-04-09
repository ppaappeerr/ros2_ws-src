#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/accumulated_pointcloud_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import numpy as np
import math
from geometry_msgs.msg import TransformStamped, Point
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
import time

# PointField 클래스 정의
class PointField:
    INT8    = 1
    UINT8   = 2
    INT16   = 3
    UINT16  = 4
    INT32   = 5
    UINT32  = 6
    FLOAT32 = 7
    FLOAT64 = 8

# 수동 point_cloud2 함수 구현
def create_cloud(header, fields, points):
    """
    간단한 포인트 클라우드 메시지 생성 함수
    """
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    
    # 필드 설정
    cloud_msg.fields = []
    for f in fields:
        field = sensor_msgs.PointField()
        field.name = f.name
        field.offset = f.offset
        field.datatype = f.datatype
        field.count = f.count
        cloud_msg.fields.append(field)
    
    # 포인트 데이터 설정
    if len(points) > 0:
        cloud_msg.height = 1
        cloud_msg.width = len(points)
    else:
        cloud_msg.height = 0
        cloud_msg.width = 0
    
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 4 bytes per float * 4 values (x,y,z,rgb)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    
    # 바이트 배열로 변환
    cloud_msg.data = bytearray()
    for p in points:
        cloud_msg.data.extend(struct.pack('ffff', p[0], p[1], p[2], p[3]))
    
    cloud_msg.is_dense = True
    
    return cloud_msg

class AccumulatedPointcloud(Node):
    def __init__(self):
        super().__init__('accumulated_pointcloud')
        
        # 파라미터 정의
        self.declare_parameter('max_points', 100000)  # 최대 누적 포인트 수
        self.declare_parameter('grid_size', 0.01)     # 그리드 크기 (m)
        self.declare_parameter('reset_time', 60.0)    # 맵 리셋 시간 (초)
        self.declare_parameter('velocity_decay', 0.98)  # 속도 감쇠 계수 (드리프트 보정용)
        self.declare_parameter('imu_to_laser_z', 0.05)  # IMU에서 라이다까지 Z축 거리 (m)
        self.declare_parameter('motion_threshold', 0.03)  # IMU 움직임 감지 임계값
        self.declare_parameter('use_tf', True)  # TF 트리 사용 여부
        
        # 파라미터 가져오기
        self.max_points = self.get_parameter('max_points').value
        self.grid_size = self.get_parameter('grid_size').value
        self.reset_time = self.get_parameter('reset_time').value
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.imu_to_laser_z = self.get_parameter('imu_to_laser_z').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        self.use_tf = self.get_parameter('use_tf').value
        
        # IMU 및 레이저 스캔 구독
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # 출력 토픽 발행자
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'live_points', 10)
        self.accumulated_pub = self.create_publisher(
            PointCloud2, 'accumulated_points', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'point_markers', 10)
        
        # TF 관련 객체들
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 상태 변수
        self.last_imu = None
        self.last_position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.last_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        
        # 속도 추적 및 움직임 추정
        self.velocity = np.array([0.0, 0.0, 0.0])  # 선형 속도
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # 각속도
        self.prev_orientation = None
        self.movement_since_last_update = 0.0
        self.prev_time = None
        
        # 누적 데이터 저장
        self.accumulated_points = []  # [x, y, z, intensity]
        self.grid_map = {}  # 그리드 -> 포인트 매핑
        
        # 시간 추적
        self.start_time = time.time()
        
        # 정기적인 발행을 위한 타이머
        self.publish_timer = self.create_timer(0.5, self.publish_accumulated)
        
        # 정기적인 움직임 확인 및 위치 업데이트 타이머
        self.motion_timer = self.create_timer(0.05, self.update_motion)
        
        # TF 발행 타이머
        self.tf_timer = self.create_timer(0.05, self.publish_tf_tree)
        
        self.get_logger().info("3D Accumulated PointCloud Node Initialized with improved IMU integration!")
    
    def imu_callback(self, msg):
        """IMU 데이터 처리 개선"""
        # 이전 IMU 데이터 저장
        prev_imu = self.last_imu
        self.last_imu = msg
        
        # 쿼터니언 정규화
        magnitude = math.sqrt(msg.orientation.x**2 + msg.orientation.y**2 + 
                            msg.orientation.z**2 + msg.orientation.w**2)
        
        if magnitude < 0.0001:  # 너무 작은 값이면 기본값 사용
            self.get_logger().warn("IMU quaternion magnitude too small, using default")
            current_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            # 정규화된 쿼터니언
            current_orientation = np.array([
                msg.orientation.x / magnitude,
                msg.orientation.y / magnitude,
                msg.orientation.z / magnitude,
                msg.orientation.w / magnitude
            ])
        
        # 현재 방향 업데이트
        self.last_orientation = current_orientation
        
        # 각속도 저장
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # 선형 가속도 저장 (중력 제거)
        linear_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z - 9.81  # 중력 제거 (대략적인 접근)
        ])
        
        # 현재 시간
        now = self.get_clock().now()
        
        # 이동 추정 및 위치 업데이트
        if self.prev_time is not None:
            # 시간 간격 계산
            dt = (now - self.prev_time).nanoseconds / 1e9  # 초 단위로 변환
            
            if dt > 0 and dt < 0.5:  # 비정상적으로 큰 시간 간격 제외
                # IMU 방향에 의한 회전 행렬 계산
                R = self.quaternion_to_rotation_matrix(self.last_orientation)
                
                # 전역 좌표계에서의 가속도 (회전 행렬을 통해 변환)
                global_accel = np.dot(R, linear_accel)
                
                # 속도 업데이트 (가속도 적분)
                self.velocity += global_accel * dt
                
                # 위치 업데이트 (속도 적분)
                self.last_position += self.velocity * dt
                
                # 움직임 계산 (각도 변화)
                if self.prev_orientation is not None:
                    roll1, pitch1, yaw1 = self.euler_from_quaternion(*self.prev_orientation)
                    roll2, pitch2, yaw2 = self.euler_from_quaternion(*current_orientation)
                    
                    # 각도 차이
                    d_roll = self.normalize_angle(roll2 - roll1)
                    d_pitch = self.normalize_angle(pitch2 - pitch1)
                    d_yaw = self.normalize_angle(yaw2 - yaw1)
                    
                    # 움직임 크기 추정
                    movement = math.sqrt(d_roll**2 + d_pitch**2 + d_yaw**2)
                    self.movement_since_last_update += movement
                
                # 드리프트 보정 (감쇠 적용)
                self.velocity *= self.velocity_decay
                
                # 로깅
                if dt > 0.1:  # 너무 자주 로그가 출력되지 않도록
                    roll, pitch, yaw = self.euler_from_quaternion(*self.last_orientation)
                    self.get_logger().debug(f"IMU: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}, " +
                                          f"pos=({self.last_position[0]:.2f}, {self.last_position[1]:.2f}, {self.last_position[2]:.2f})")
        
        # 이전 방향 및 시간 저장
        self.prev_orientation = current_orientation
        self.prev_time = now
    
    def quaternion_to_rotation_matrix(self, q):
        """쿼터니언을 회전 행렬로 변환"""
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
    
    def normalize_angle(self, angle):
        """각도를 -π와 π 사이로 정규화"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def update_motion(self):
        """움직임을 확인하고 필요시 위치 및 방향 업데이트"""
        if self.last_imu is None:
            return
        
        # 움직임이 임계값을 초과할 때만 처리
        if self.movement_since_last_update > self.motion_threshold:
            self.get_logger().debug(f"Significant motion detected: {self.movement_since_last_update:.4f}")
            self.movement_since_last_update = 0.0
    
    def transform_to_3d(self, x, y, quat):
        """2D 좌표를 IMU 방향에 기반해 3D로 변환"""
        # 쿼터니언을 회전 행렬로 변환
        R = self.quaternion_to_rotation_matrix(quat)
        
        # 입력 좌표는 라이다 평면 상의 점 (x, y, 0)
        point_local = np.array([x, y, 0.0])
        
        # 회전 행렬을 사용하여 3D 공간에서의 위치 계산
        point_global = np.dot(R, point_local)
        
        return point_global[0], point_global[1], point_global[2]
    
    def publish_tf_tree(self):
        """TF 트리 발행 - 센서 좌표계 간의 관계 설정"""
        if not self.use_tf or self.last_imu is None:
            return
        
        # 현재 시간
        now = self.get_clock().now().to_msg()
        
        # map -> base_link 변환 (IMU 위치)
        t_map_base = TransformStamped()
        t_map_base.header.stamp = now
        t_map_base.header.frame_id = 'map'
        t_map_base.child_frame_id = 'base_link'
        
        # 위치 설정
        t_map_base.transform.translation.x = self.last_position[0]
        t_map_base.transform.translation.y = self.last_position[1]
        t_map_base.transform.translation.z = self.last_position[2]
        
        # 방향 설정
        t_map_base.transform.rotation.x = self.last_orientation[0]
        t_map_base.transform.rotation.y = self.last_orientation[1]
        t_map_base.transform.rotation.z = self.last_orientation[2]
        t_map_base.transform.rotation.w = self.last_orientation[3]
        
        # base_link -> laser 변환
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = now
        t_base_laser.header.frame_id = 'base_link'
        t_base_laser.child_frame_id = 'laser'
        
        # 라이다는 IMU 위에 위치 (z축 방향으로)
        t_base_laser.transform.translation.x = 0.0
        t_base_laser.transform.translation.y = 0.0
        t_base_laser.transform.translation.z = self.imu_to_laser_z
        
        # 라이다와 IMU 간에 방향 차이가 없다고 가정 (identity rotation)
        t_base_laser.transform.rotation.x = 0.0
        t_base_laser.transform.rotation.y = 0.0
        t_base_laser.transform.rotation.z = 0.0
        t_base_laser.transform.rotation.w = 1.0
        
        # TF 변환 발행
        self.tf_broadcaster.sendTransform(t_map_base)
        self.tf_broadcaster.sendTransform(t_base_laser)
    
    def scan_callback(self, msg):
        """라이다 스캔 데이터를 처리하여 3D 포인트 클라우드 생성"""
        if self.last_imu is None:
            self.get_logger().warn("No IMU data yet")
            return
        
        # 리셋 시간 확인
        current_time = time.time()
        if current_time - self.start_time > self.reset_time:
            self.get_logger().info(f"Resetting map after {self.reset_time} seconds")
            self.accumulated_points = []
            self.grid_map = {}
            self.start_time = current_time
        
        # 현재 IMU 방향 및 위치
        quat = self.last_orientation
        position = self.last_position
        
        # 현재 스캔에서의 포인트
        current_points = []
        
        # 스캔 데이터를 3D 포인트로 변환
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 레이저 각도
            angle = msg.angle_min + i * msg.angle_increment
            
            # 로컬 좌표계에서의 점 위치 (레이저 프레임)
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)
            
            # IMU 기반 3D 변환 (개선된 방법)
            x_3d, y_3d, z_3d = self.transform_to_3d(x_local, y_local, quat)
            
            # 강도 값 (있는 경우)
            intensity = 100.0
            if hasattr(msg, 'intensities') and i < len(msg.intensities):
                intensity = float(msg.intensities[i])
            
            # 현재 프레임의 포인트 추가
            current_points.append([x_3d, y_3d, z_3d, intensity])
            
            # 글로벌 좌표계로 변환 (누적용)
            x_global = x_3d + position[0]
            y_global = y_3d + position[1]
            z_global = z_3d + position[2]
            
            # 그리드 인덱스 계산
            grid_x = int(x_global / self.grid_size)
            grid_y = int(y_global / self.grid_size)
            grid_z = int(z_global / self.grid_size)
            grid_key = (grid_x, grid_y, grid_z)
            
            # 이 그리드에 아직 포인트가 없으면 추가
            if grid_key not in self.grid_map:
                self.accumulated_points.append([x_global, y_global, z_global, intensity])
                self.grid_map[grid_key] = len(self.accumulated_points) - 1
                
                # 최대 포인트 수 관리
                if len(self.accumulated_points) > self.max_points:
                    # 가장 오래된 포인트 제거
                    removed_point = self.accumulated_points.pop(0)
                    removed_grid = (
                        int(removed_point[0] / self.grid_size),
                        int(removed_point[1] / self.grid_size),
                        int(removed_point[2] / self.grid_size)
                    )
                    self.grid_map.pop(removed_grid, None)
                    
                    # 인덱스 업데이트 (하나씩 앞으로 당김)
                    for k, v in self.grid_map.items():
                        if v > 0:  # 첫 번째 이후 항목들은 인덱스가 1 감소
                            self.grid_map[k] = v - 1
        
        # 현재 스캔의 포인트 클라우드 발행
        if current_points:
            # 헤더 설정
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'laser'  # 라이다 프레임
            
            self.publish_pointcloud(current_points, 'live_points', header)
    
    def publish_accumulated(self):
        """누적된 포인트 클라우드 발행"""
        if not self.accumulated_points:
            return
            
        # 헤더 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # 누적된 포인트는 글로벌 맵 프레임에 있음
        
        # 누적된 포인트 클라우드 발행
        self.publish_pointcloud(self.accumulated_points, 'accumulated_points', header)
        
        # 마커 배열도 발행
        self.publish_markers()
        
        # 로그
        self.get_logger().info(f"Published accumulated cloud with {len(self.accumulated_points)} points.")
    
    def publish_pointcloud(self, points, topic_name, header):
        """포인트 클라우드 메시지 생성 및 발행"""
        if not points:
            return
            
        fields = [
            sensor_msgs.PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg = create_cloud(header, fields, points)
        
        # 토픽에 따라 적절한 발행자 선택
        if topic_name == 'live_points':
            self.cloud_pub.publish(cloud_msg)
        elif topic_name == 'accumulated_points':
            self.accumulated_pub.publish(cloud_msg)
    
    def publish_markers(self):
        """누적된 포인트를 마커 배열로 시각화"""
        marker_array = MarkerArray()
        
        # 마커 생성
        marker = Marker()
        marker.header.frame_id = 'map'  # 글로벌 맵 프레임
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'accumulated_points'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # 스케일 설정
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.pose.orientation.w = 1.0
        
        # 포인트와 색상 추가
        for point in self.accumulated_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker.points.append(p)
            
            # 높이에 따른 색상
            c = ColorRGBA()
            c.a = 1.0  # 완전 불투명
            
            # 높이(z)에 따른 색상 변화
            normalized_z = (point[2] + 1.0) / 2.0  # 높이를 0~1 사이로 정규화 (-1m ~ 1m 범위 가정)
            normalized_z = max(0.0, min(1.0, normalized_z))  # 0~1 범위로 클리핑
            
            # 높이에 따른 색상 그라데이션 (파랑-초록-빨강)
            if normalized_z < 0.5:
                c.b = 1.0 - 2 * normalized_z
                c.g = 2 * normalized_z
                c.r = 0.0
            else:
                c.b = 0.0
                c.g = 1.0 - 2 * (normalized_z - 0.5)
                c.r = 2 * (normalized_z - 0.5)
            
            marker.colors.append(c)
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
    
    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 오일러 각 변환"""
        # 롤 (x축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # 피치 (y축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # 요 (z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = AccumulatedPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()