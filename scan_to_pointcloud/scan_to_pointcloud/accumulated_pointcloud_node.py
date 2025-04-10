#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/accumulated_pointcloud_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import numpy as np
import math
from geometry_msgs.msg import TransformStamped, Point, Vector3, Quaternion
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
import time
from rclpy.duration import Duration

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
        
        # 기존 파라미터 유지
        self.declare_parameter('max_points', 100000)  # 최대 누적 포인트 수
        self.declare_parameter('grid_size', 0.01)     # 그리드 크기 (m)
        self.declare_parameter('reset_time', 60.0)    # 맵 리셋 시간 (초)
        self.declare_parameter('velocity_decay', 0.98)  # 속도 감쇠 계수 (드리프트 보정용)
        self.declare_parameter('imu_to_laser_z', 0.05)  # IMU에서 라이다까지 Z축 거리 (m)
        self.declare_parameter('motion_threshold', 0.03)  # IMU 움직임 감지 임계값
        self.declare_parameter('use_tf', True)  # TF 트리 사용 여부
        self.declare_parameter('tf_buffer_duration', 5.0)  # 5초 TF 버퍼
        
        # 다운샘플링 파라미터 추가
        self.declare_parameter('point_skip', 2)  # 몇 개의 포인트마다 하나만 사용할지
        self.declare_parameter('update_rate', 2.0)  # 포인트클라우드 갱신 속도 (Hz)
        self.point_skip = self.get_parameter('point_skip').value
        
        # 빠른 초기화를 위한 파라미터 추가
        self.declare_parameter('initial_accumulation_size', 100)  # 초기 표시를 위한 최소 포인트 수
        self.initial_accumulation_size = self.get_parameter('initial_accumulation_size').value
        
        # 발행 간격 조정
        self.declare_parameter('publish_rate', 5.0)  # 더 빠른 발행 (2.0 -> 5.0)
        self.publish_timer = self.create_timer(1.0/self.get_parameter('publish_rate').value, 
                                               self.publish_accumulated)
        
        # 시각화 관련 파라미터 (누락되었던 부분)
        self.declare_parameter('use_height_coloring', True)  # 높이에 따른 색상 변화 사용
        self.use_height_coloring = self.get_parameter('use_height_coloring').value
        
        # live_points 성능 향상을 위한 설정
        self.declare_parameter('live_points_skip', 2)  # 포인트 스킵 수
        self.live_points_skip = self.get_parameter('live_points_skip').value
        
        # 중요: IMU TF 통합 관련 새 파라미터
        self.declare_parameter('use_imu_tf_only', True)  # imu_tf_publisher만 사용
        self.declare_parameter('disable_position_tracking', True)  # 위치 추적 비활성화
        self.declare_parameter('reset_on_startup', True)  # 시작시 위치 리셋
        
        # 방향 반전 설정 - imu_tf_publisher와 동일하게 맞춤
        self.declare_parameter('invert_roll', False)  # roll 반전 비활성화
        self.declare_parameter('invert_pitch', False)  # pitch 반전 비활성화
        self.declare_parameter('invert_yaw', False)  # yaw 반전 비활성화
        
        # 파라미터 가져오기
        self.use_imu_tf_only = self.get_parameter('use_imu_tf_only').value
        self.disable_position_tracking = self.get_parameter('disable_position_tracking').value
        self.reset_on_startup = self.get_parameter('reset_on_startup').value
        
        self.invert_roll = self.get_parameter('invert_roll').value
        self.invert_pitch = self.get_parameter('invert_pitch').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        
        # 초기화 시 위치 리셋
        if self.reset_on_startup:
            self.reset_position()
            self.reset_accumulated_points()
        
        # IMU 변환 관련 변수 추가
        self.imu_transform_matrix = None
        self.last_valid_transform_time = self.get_clock().now()
        
        # 발행 로직 개선
        self.accumulated_points_published = False
        
        # 로그 추가: 설정 정보 출력
        self.get_logger().info(
            f"3D Accumulated PointCloud Node 초기화: "
            f"TF전용={self.use_imu_tf_only}, 위치추적비활성화={self.disable_position_tracking}, "
            f"방향반전=(roll={self.invert_roll}, pitch={self.invert_pitch}, yaw={self.invert_yaw})"
        )
        
        # 파라미터 가져오기
        self.max_points = self.get_parameter('max_points').value
        self.grid_size = self.get_parameter('grid_size').value
        self.reset_time = self.get_parameter('reset_time').value
        self.velocity_decay = self.get_parameter('velocity_decay').value
        self.imu_to_laser_z = self.get_parameter('imu_to_laser_z').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        self.use_tf = self.get_parameter('use_tf').value
        self.tf_buffer_duration = self.get_parameter('tf_buffer_duration').value
        
        # IMU 및 레이저 스캔 구독
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # 디버깅용 카운터 추가
        self.publish_successes = 0
        self.publish_failures = 0
        
        # QoS 설정 변경 (안정성 향상)
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
        
        # 더 안정적인 QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # 출력 토픽 발행자 (QoS 변경)
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'live_points', qos)
        self.accumulated_pub = self.create_publisher(
            PointCloud2, 'accumulated_points', qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'point_markers', qos)
        
        # 디버그 로그 추가
        self.debug_timer = self.create_timer(5.0, self.debug_status)
        
        # TF 관련 객체들
        self.tf_buffer = Buffer(Duration(seconds=self.tf_buffer_duration))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # TF 트리가 준비되었는지 확인하는 플래그 추가
        self.tf_ready = False
        self.tf_check_timer = self.create_timer(0.5, self.check_tf_ready)
        
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
        
        # 정기적인 움직임 확인 및 위치 업데이트 타이머
        self.motion_timer = self.create_timer(0.05, self.update_motion)
        
        ## TF 발행 타이머 (use_imu_tf_only가 True일 때는 비활성화)
        if not self.use_imu_tf_only:
            self.tf_timer = self.create_timer(0.05, self.publish_tf_tree)
        else:
            # use_imu_tf_only 모드에서는 TF를 발행하지 않고 외부 TF만 사용
            self.get_logger().info("TF 전용 모드: 이 노드는 TF를 발행하지 않고 외부 TF만 사용합니다.")
        
        # 마지막으로 성공한 변환 저장용 속성 추가
        self.last_valid_transform = None
        
        # TF 실패 카운터 및 재시도 로직 추가
        self.tf_retry_count = 0
        self.max_tf_retries = 5
        self.tf_retry_delay = 0.1  # 초
        self.tf_lookup_failures = 0
        self.last_valid_time = None
        
        # IMU 토픽 타임아웃 추가 - IMU 데이터 수신 확인용
        self.last_imu_time = None
        self.imu_timeout = 2.0  # 2초
        self.imu_check_timer = self.create_timer(0.5, self.check_imu_active)
        
        # IMU 데이터 관련 변수 추가
        self.imu_data_history = []
        self.max_imu_history = 50  # 최대 IMU 내역 길이
        self.recent_imu_sample_time = None
        
        # IMU 데이터 재구독 메커니즘 추가
        self.imu_reconnect_count = 0
        
        self.get_logger().info("3D Accumulated PointCloud Node Initialized with improved IMU integration!")
    
    def reset_accumulated_points(self):
        """누적된 포인트 클라우드 초기화"""
        self.accumulated_points = []
        self.grid_map = {}
        self.get_logger().info("누적 포인트 클라우드 초기화 완료")
    
    def check_tf_ready(self):
        """TF 트리가 준비되었는지 확인"""
        try:
            # 필요한 TF만 확인
            self.tf_buffer.lookup_transform('world', 'map', rclpy.time.Time())
            self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
            
            if not self.tf_ready:
                self.get_logger().info("TF 트리가 준비되었습니다!")
                self.tf_ready = True
                
        except Exception as e:
            if self.tf_ready:
                self.get_logger().warn("TF 트리가 깨졌습니다.")
                self.tf_ready = False
            else:
                self.get_logger().debug(f"TF 트리 준비 중... ({str(e)[:30]})")
    
    def check_imu_active(self):
        """IMU 데이터 활성화 여부 확인 (업그레이드 버전)"""
        now = self.get_clock().now()
        
        if self.last_imu_time is None:
            if not hasattr(self, 'last_imu_warn'):
                self.last_imu_warn = now
                self.get_logger().warn("아직 IMU 데이터가 수신되지 않았습니다.")
            elif (now - self.last_imu_warn).nanoseconds / 1e9 > 3.0:
                self.last_imu_warn = now
                self.get_logger().warn("IMU 데이터 계속 수신되지 않음, 센서 연결 확인 필요")
            return
        
        # IMU 데이터가 일정 시간 이상 수신되지 않으면 경고
        diff = (now - self.last_imu_time).nanoseconds / 1e9
        
        if diff > 1.0 and diff <= 3.0:
            # 짧은 딜레이는 경고만
            self.get_logger().warn(f"IMU 데이터가 {diff:.1f}초 동안 수신되지 않았습니다.")
        elif diff > 3.0:
            # 긴 딜레이는 IMU 재구독 시도
            self.get_logger().error(f"IMU 데이터가 {diff:.1f}초 동안 수신되지 않았습니다. 재구독 시도...")
            
            # IMU 재구독 시도
            self.imu_reconnect_count += 1
            self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
            
            # 심각한 문제가 의심될 경우 (여러 차례 재시도 후)
            if self.imu_reconnect_count > 3 and diff > 8.0:
                self.get_logger().error("심각한 IMU 통신 문제 감지: 이전 유효 IMU 데이터 사용")
                # 이전에 저장한 유효한 IMU 데이터에서 가장 최근 것 사용
                if self.imu_data_history:
                    self.last_imu = self.imu_data_history[-1]
                    self.last_imu_time = now  # 시간 리셋
                    self.get_logger().info("저장된 IMU 데이터로 복구 시도")
        
        # Z축이 비정상적으로 처박혔는지 확인
        if abs(self.last_position[2]) > 1.0:
            self.get_logger().warn(f"비정상적인 Z 위치 감지: {self.last_position[2]:.2f}m, 위치 리셋")
            self.reset_position()
    
    def imu_callback(self, msg):
        """IMU 데이터 처리 - imu_tf_publisher와 유사하게 처리"""
        # 이전 IMU 데이터 저장
        prev_imu = self.last_imu
        self.last_imu = msg
        
        # IMU 데이터 내역에 추가
        self.imu_data_history.append(msg)
        if len(self.imu_data_history) > self.max_imu_history:
            self.imu_data_history.pop(0)
        
        # IMU 데이터 수신 시간 업데이트
        self.last_imu_time = self.get_clock().now()
        self.imu_reconnect_count = 0  # 재연결 카운터 리셋
        
        # 위치 추적 비활성화된 경우 위치 관련 업데이트 건너뛰기
        if self.disable_position_tracking:
            return
        
        # IMU 방향 정보를 이용한 처리
        if msg.orientation_covariance[0] >= 0:
            # 유효한 방향 정보가 있을 경우
            self.last_orientation = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])
        
        # 현재 시간 및 이전 시간 처리
        now = self.get_clock().now()
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            
            if dt > 0.0 and dt < 0.1:  # 비정상적으로 큰 시간 간격 제외
                # IMU 각속도 처리 (회전 방향 조정)
                wx = msg.angular_velocity.x * (-1.0 if self.invert_roll else 1.0)
                wy = msg.angular_velocity.y * (-1.0 if self.invert_pitch else 1.0)
                wz = msg.angular_velocity.z * (-1.0 if self.invert_yaw else 1.0)
                
                # 임계값 필터링 - 매우 작은 움직임 무시
                threshold = 0.01
                if abs(wx) < threshold: wx = 0.0
                if abs(wy) < threshold: wy = 0.0
                if abs(wz) < threshold: wz = 0.0
                
                # 각속도 저장
                self.angular_velocity = np.array([wx, wy, wz])
                
                # 이동 거리 추정
                self.movement_since_last_update += np.linalg.norm(self.angular_velocity) * dt
                
                # 가속도 보정 및 처리
                # 가속도계 데이터를 지구 좌표계로 변환하려면 회전 행렬 필요
                # 여기서는 간단하게 수직 가속도 성분만 제거
                linear_accel = np.array([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z - 9.81  # 중력 효과 제거
                ])
                
                # 가속도를 이용한 속도 갱신 (간단한 적분)
                self.velocity += linear_accel * dt
                
                # 드리프트 보정 (감쇠)
                self.velocity *= self.velocity_decay
                
                # 속도로 위치 갱신 (간단한 적분)
                self.last_position += self.velocity * dt
                
                # Z축 위치 제한 (비정상적인 값 방지)
                max_z_pos = 0.5  # 최대 Z 상승
                min_z_pos = -0.5  # 최대 Z 하강
                
                if self.last_position[2] > max_z_pos:
                    self.last_position[2] = max_z_pos
                    self.velocity[2] = 0.0  # Z속도 리셋
                
                if self.last_position[2] < min_z_pos:
                    self.last_position[2] = min_z_pos
                    self.velocity[2] = 0.0  # Z속도 리셋
        
        # 현재 시간 저장
        self.prev_time = now
    
    def reset_position(self):
        """위치와 속도를 초기화"""
        self.last_position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.get_logger().info("위치 및 속도 초기화 완료")
    
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
        """TF 트리 발행 - 명확한 역할 분리"""
        # use_imu_tf_only=True일 때는 TF 발행하지 않음 (충돌 방지)
        if self.use_imu_tf_only or not self.use_tf or self.last_imu is None:
            return
        
        # 현재 시간
        now = self.get_clock().now().to_msg()
        
        # 이 노드는 map -> base_link 변환만 발행
        t_map_base = TransformStamped()
        t_map_base.header.stamp = now
        t_map_base.header.frame_id = 'map'
        t_map_base.child_frame_id = 'base_link'
        
        # 위치 설정
        t_map_base.transform.translation.x = self.last_position[0]
        t_map_base.transform.translation.y = self.last_position[1]
        t_map_base.transform.translation.z = self.last_position[2]
        
        # 방향 설정 (w, x, y, z 순서로 저장)
        t_map_base.transform.rotation.w = self.last_orientation[3]
        t_map_base.transform.rotation.x = self.last_orientation[0]
        t_map_base.transform.rotation.y = self.last_orientation[1]
        t_map_base.transform.rotation.z = self.last_orientation[2]
        
        # map -> base_link 변환만 발행
        self.tf_broadcaster.sendTransform(t_map_base)
    
    def scan_callback(self, msg):
        """스캔 데이터를 3D 포인트로 변환하고 누적"""
        if self.last_imu is None:
            self.get_logger().warn("No IMU data yet")
            return
        
        if not self.tf_ready:
            # TF 준비 안된 경우 처리
            if not hasattr(self, 'last_tf_warn_time') or \
               (self.get_clock().now() - self.last_tf_warn_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn("TF 트리가 준비되지 않았습니다. 스캔을 스킵합니다.")
                self.last_tf_warn_time = self.get_clock().now()
            return
        
        # 항상 map->laser 직접 변환 사용 (use_imu_tf_only 설정과 관계없이)
        try:
            # 'map' -> 'laser' 직접 변환 가져오기 (필요한 TF 체인 전부)
            transform = self.tf_buffer.lookup_transform(
                'map', 'laser',
                rclpy.time.Time(),  # 현재 시간 기준
                rclpy.duration.Duration(seconds=0.1)
            )
            
            have_valid_transform = True
            self.last_valid_transform = transform
            self.last_valid_transform_time = self.get_clock().now()
            
            # 포인트 변환 및 누적 처리
            self.process_scan_with_transform(msg, transform)
            
        except Exception as e:
            have_valid_transform = False
            now = self.get_clock().now()
            time_since_valid = (now - self.last_valid_transform_time).nanoseconds / 1e9
            
            # 최근 유효 변환이 있고, 오래되지 않았다면 재사용
            if self.last_valid_transform and time_since_valid < 1.0:
                self.process_scan_with_transform(msg, self.last_valid_transform)
                self.get_logger().debug("이전 유효 변환 사용")
            else:
                self.tf_lookup_failures += 1
                if self.tf_lookup_failures % 10 == 0:
                    self.get_logger().warn(f"TF 조회 {self.tf_lookup_failures}회 연속 실패: {str(e)[:50]}...")
        
        # live_points 발행 로직
        self.publish_live_points(msg)
    
    def process_scan_with_transform(self, msg, transform):
        """변환 정보를 사용하여 스캔을 누적 포인트 클라우드에 추가"""
        # 누적 포인트가 너무 많으면 일부 제거
        if len(self.accumulated_points) > self.max_points:
            # 25% 포인트 제거 (오래된 것부터)
            remove_count = len(self.accumulated_points) // 4
            self.accumulated_points = self.accumulated_points[remove_count:]
            self.get_logger().info(f"포인트 제한: {remove_count}개 제거됨")
        
        # 스캔 데이터를 변환하여 누적
        new_points = 0
        
        # 변환 행렬 준비
        trans = transform.transform.translation
        rot = transform.transform.rotation
        
        # 변환 행렬 계산 - 사용 시 최적화 가능
        transform_matrix = None
        
        for i, r in enumerate(msg.ranges):
            # 범위 체크
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 점 건너뛰기 (다운샘플링)
            if i % self.point_skip != 0:
                continue
            
            # 2D 좌표 계산 (레이저 스캔 평면)
            angle = msg.angle_min + i * msg.angle_increment
            x_local = r * math.cos(angle)
            y_local = r * math.sin(angle)
            z_local = 0.0  # 평면상의 점
            
            # 변환 행렬 계산 (필요시)
            if transform_matrix is None:
                transform_matrix = self.compute_transform_matrix(trans, rot)
            
            # 점 변환: 로컬 -> 글로벌
            x_global, y_global, z_global = self.transform_point(
                x_local, y_local, z_local, transform_matrix)
            
            # 그리드 기반 중복 제거
            grid_key = self.get_grid_key(x_global, y_global, z_global)
            
            # 이미 있는 그리드인지 확인
            if grid_key not in self.grid_map:
                # 새 포인트 추가
                intensity = 100.0  # 기본 강도
                
                # 강도 값이 있으면 사용
                if hasattr(msg, 'intensities') and i < len(msg.intensities) and np.isfinite(msg.intensities[i]):
                    intensity = float(msg.intensities[i])
                
                # 높이 기반 색상 또는 거리 기반 색상
                if self.use_height_coloring:
                    # z값에 따른 색상 코딩 (기존 코드 유지)
                    height_color = max(0, min(255, int((z_global + 1.0) * 127.5)))
                    
                    if z_global < -0.5:
                        r, g, b = 0, 0, height_color
                    elif z_global < 0.0:
                        r, g, b = 0, height_color, 255 - height_color
                    elif z_global < 0.5:
                        r, g, b = height_color, 255, 0
                    else:
                        r, g, b = 255, 255 - height_color, 0
                    
                    # RGBA를 float로 패킹
                    intensity = struct.unpack('f', struct.pack('BBBB', r, g, b, 255))[0]
                
                # 포인트 추가 [x, y, z, intensity]
                self.accumulated_points.append([x_global, y_global, z_global, intensity])
                self.grid_map[grid_key] = len(self.accumulated_points) - 1
                new_points += 1
        
        if new_points > 0 and len(self.accumulated_points) % 100 == 0:
            self.get_logger().debug(f"{new_points}개 포인트 추가됨, 총 {len(self.accumulated_points)}개")
    
    def publish_live_points(self, msg):
        """스캔을 라이브 포인트 클라우드로 변환하여 발행"""
        live_points = []
        
        # 스캔 포인트 변환
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 포인트 샘플링 적용
            if i % self.live_points_skip != 0:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            
            # 강도값
            intensity = 100.0
            if hasattr(msg, 'intensities') and i < len(msg.intensities) and np.isfinite(msg.intensities[i]):
                intensity = float(msg.intensities[i])
            
            live_points.append([x, y, z, intensity])
        
        # 라이브 포인트 발행 (레이저 프레임 기준)
        if live_points:
            live_header = Header()
            live_header.stamp = msg.header.stamp
            live_header.frame_id = 'laser'  # 레이저 프레임 고정
            
            self.publish_pointcloud(live_points, 'live_points', live_header)
    
    def compute_transform_matrix(self, trans, rot):
        """변환 정보로 4x4 변환 행렬 계산"""
        # 회전 행렬 계산
        x, y, z, w = rot.x, rot.y, rot.z, rot.w
        
        # 쿼터니언 -> 회전 행렬
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
        
        # 4x4 변환 행렬
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [trans.x, trans.y, trans.z]
        
        return T
    
    def transform_point(self, x, y, z, transform_matrix):
        """4x4 변환 행렬로 점 변환"""
        point = np.array([x, y, z, 1.0])
        transformed = np.dot(transform_matrix, point)
        return transformed[0], transformed[1], transformed[2]
    
    def get_grid_key(self, x, y, z):
        """그리드 키 생성 (공간 양자화)"""
        grid_x = int(x / self.grid_size)
        grid_y = int(y / self.grid_size)
        grid_z = int(z / self.grid_size)
        return f"{grid_x},{grid_y},{grid_z}"
    
    def publish_accumulated(self):
        """누적된 포인트 클라우드 발행 - 개선된 버전"""
        # 충분한 포인트가 없으면 건너뛰기
        if not self.accumulated_points:
            return
        
        if len(self.accumulated_points) < self.initial_accumulation_size:
            if not self.accumulated_points_published:  # 처음 한번만 로그
                self.get_logger().info(
                    f"누적 포인트 부족: {len(self.accumulated_points)}/{self.initial_accumulation_size}, "
                    f"충분히 쌓이면 발행됩니다."
                )
            return
        
        # 헤더 설정 (항상 map 프레임)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # 글로벌 프레임
        
        try:
            # 필드 정의 (명시적)
            fields = [
                sensor_msgs.PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                sensor_msgs.PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
            ]
            
            # 포인트 수 제한 (성능 개선)
            points_to_publish = self.accumulated_points
            if len(points_to_publish) > 10000:
                # 간단한 균등 샘플링
                indices = np.linspace(0, len(points_to_publish)-1, 10000, dtype=int)
                points_to_publish = [points_to_publish[i] for i in indices]
            
            # 포인트 클라우드 메시지 생성 및 발행
            cloud_msg = create_cloud(header, fields, points_to_publish)
            self.accumulated_pub.publish(cloud_msg)
            
            # 상태 추적
            self.publish_successes += 1
            self.accumulated_points_published = True
            
            if not hasattr(self, 'last_publish_log') or \
               (self.get_clock().now() - self.last_publish_log).nanoseconds / 1e9 > 5.0:
                self.get_logger().info(f"누적 포인트 발행: {len(points_to_publish)} 포인트 (총 {len(self.accumulated_points)}개 중)")
                self.last_publish_log = self.get_clock().now()
                
            # 마커도 함께 발행
            self.publish_markers()
            
        except Exception as e:
            self.publish_failures += 1
            self.get_logger().error(f"누적 포인트 발행 실패: {e}")
    
    def publish_pointcloud(self, points, topic_name, header):
        """포인트 클라우드 메시지 생성 및 발행"""
        if not points:
            self.get_logger().warn(f"{topic_name}: 발행할 포인트 없음")
            self.publish_failures += 1
            return
        
        try:
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
                self.get_logger().debug(f"live_points 발행: {len(points)} 포인트")
            elif topic_name == 'accumulated_points':
                self.accumulated_pub.publish(cloud_msg)
                self.get_logger().debug(f"accumulated_points 발행: {len(points)} 포인트")
            
            self.publish_successes += 1
            
        except Exception as e:
            self.get_logger().error(f"{topic_name} 발행 실패: {e}")
            self.publish_failures += 1
    
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
    
    def debug_status(self):
        """메시지 발행 성공/실패 상태 로깅"""
        total = self.publish_successes + self.publish_failures
        if total > 0:
            success_rate = (self.publish_successes / total) * 100
            self.get_logger().info(
                f"메시지 발행: 성공={self.publish_successes}, 실패={self.publish_failures}, "
                f"성공률={success_rate:.1f}%"
            )
        self.publish_successes = 0
        self.publish_failures = 0
    
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
    # 각 노드에 아래 함수 추가

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
    node = AccumulatedPointcloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()