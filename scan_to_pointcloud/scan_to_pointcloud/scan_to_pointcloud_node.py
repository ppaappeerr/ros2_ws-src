#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
import numpy as np
import math
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class ScanToPointcloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')
        
        # 파라미터
        self.declare_parameter('input_topic', 'scan')  # 입력 토픽
        self.declare_parameter('output_topic', 'pc_3d')  # 출력 토픽
        self.declare_parameter('frame_id', 'base_link')  # 출력 프레임
        self.declare_parameter('use_tf', True)  # TF 사용 여부
        self.declare_parameter('output_frame', 'laser')  # base_link에서 laser로 변경
        
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('output_frame').value
        self.use_tf = self.get_parameter('use_tf').value
        
        # 시각화 관련 파라미터
        self.declare_parameter('use_height_coloring', True)
        self.declare_parameter('intensity_scale', 100.0)
        
        self.use_height_coloring = self.get_parameter('use_height_coloring').value
        self.intensity_scale = self.get_parameter('intensity_scale').value
        
        # QoS 설정 (안정성 향상)
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 포인트 클라우드 퍼블리셔 
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        
        # 구독
        self.scan_sub = self.create_subscription(
            LaserScan, self.input_topic, self.scan_callback, 10)
        
        # IMU 방향 사용 시
        self.imu_sub = self.create_subscription(
            Imu, 'imu', self.imu_callback, 10)
        
        # TF 관련
        if self.use_tf:
            self.tf_buffer = Buffer(Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 상태 변수
        self.last_imu = None
        self.consecutive_failures = 0
        self.publish_count = 0
        
        # 상태 출력
        self.timer = self.create_timer(2.0, self.print_status)
        self.debug_timer = self.create_timer(3.0, self.debug_status)
        
        self.get_logger().info(
            f"스캔→포인트클라우드 변환 노드 초기화 ({self.input_topic} → {self.output_topic})"
        )
    
    def print_status(self):
        """상태 정보 출력"""
        if self.last_imu is None:
            self.get_logger().warn("IMU 데이터 없음, 방향 정보 없이 변환합니다.")
    
    def debug_status(self):
        """PointCloud 발행 현황 로깅"""
        if self.publish_count > 0:
            self.get_logger().info(f"지난 3초 동안 {self.publish_count}개 포인트클라우드 메시지 발행")
        self.publish_count = 0
    
    def imu_callback(self, msg):
        """IMU 데이터 저장"""
        self.last_imu = msg
    
    def scan_callback(self, msg):
        """스캔 데이터를 3D 포인트로 변환"""
        # 방향 정보 확인
        orientation = None
        
        # 1. 가장 정확한 방법: base_link→laser 변환 직접 사용
        if self.use_tf:
            try:
                # 현재 시간 기준 조회 (시간 불일치 문제 회피)
                laser_to_base = self.tf_buffer.lookup_transform(
                    self.frame_id, msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.1)
                )
                
                # TF에서 방향 추출
                q_tf = laser_to_base.transform.rotation
                orientation = [q_tf.x, q_tf.y, q_tf.z, q_tf.w]
                self.consecutive_failures = 0
                
                # 변환 정보 디버그 출력
                roll, pitch, yaw = self.euler_from_quaternion(q_tf.x, q_tf.y, q_tf.z, q_tf.w)
                self.get_logger().debug(f"TF 방향: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
                
            except Exception as e:
                self.consecutive_failures += 1
                if self.consecutive_failures % 10 == 0:
                    self.get_logger().warn(f"TF 조회 실패 ({self.frame_id}→{msg.header.frame_id}): {e}")
        
        # 2. 대체 방안: base_link 기준 IMU 방향 사용
        if orientation is None and self.last_imu is not None:
            # IMU에서 방향 추출
            q_imu = self.last_imu.orientation
            orientation = [q_imu.x, q_imu.y, q_imu.z, q_imu.w]
            
            # 방향 정보 디버그 출력
            if not hasattr(self, 'last_imu_debug') or \
               (self.get_clock().now() - self.last_imu_debug).nanoseconds / 1e9 > 1.0:
                self.last_imu_debug = self.get_clock().now()
                roll, pitch, yaw = self.euler_from_quaternion(q_imu.x, q_imu.y, q_imu.z, q_imu.w)
                self.get_logger().debug(f"IMU 방향 사용: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
    
        # 스캔 데이터 처리
        points = []
        
        # 방향 정보가 있는 경우 회전 행렬 계산
        R = None
        if orientation is not None:
            # 방향 정규화
            x, y, z, w = orientation
            norm = math.sqrt(x*x + y*y + z*z + w*w)
            if norm > 0.001:  # 0이 아닌지 확인
                x, y, z, w = x/norm, y/norm, z/norm, w/norm
                R = self.quaternion_to_rotation_matrix([x, y, z, w])
            
        ## 2D 스캔을 3D 점으로 변환
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max or not np.isfinite(r):
                continue
            
            # 2D 좌표 계산
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0  # 2D 스캔이므로 z=0
            
            # 회전 행렬이 있으면 3D 회전 적용
            if R is not None:
                point = np.array([x, y, z])
                point = np.dot(R, point)
                x, y, z = point
            
            # 강도 정보
            intensity = self.intensity_scale  # 기본값
            
            # 강도 값이 있으면 사용
            if hasattr(msg, 'intensities') and i < len(msg.intensities) and np.isfinite(msg.intensities[i]):
                intensity = float(msg.intensities[i])
            
            # 높이값 기반 컬러링 적용
            if self.use_height_coloring:
                # z값을 0~255 범위로 매핑
                height_color = max(0, min(255, int((z + 1.0) * 127.5)))
                # RGB 색상 계산 (높이에 따라 다른 색)
                if z < -0.5:  # 낮은 지점
                    r, g, b = 0, 0, height_color
                elif z < 0.0:  # 중간 낮은 지점
                    r, g, b = 0, height_color, 255 - height_color
                elif z < 0.5:  # 중간 높은 지점
                    r, g, b = height_color, 255, 0
                else:  # 높은 지점
                    r, g, b = 255, 255 - height_color, 0
                
                # RGBA를 float로 패킹
                intensity = struct.unpack('f', struct.pack('BBBB', r, g, b, 255))[0]
            
            # 포인트 추가 [x, y, z, intensity]
            points.append([x, y, z, intensity])
        
        # 포인트 클라우드 생성 및 발행
        if points:
            try:
                # 필드 정의
                fields = [
                    sensor_msgs.PointField(name='x', offset=0, datatype=7, count=1),
                    sensor_msgs.PointField(name='y', offset=4, datatype=7, count=1),
                    sensor_msgs.PointField(name='z', offset=8, datatype=7, count=1),
                    sensor_msgs.PointField(name='intensity', offset=12, datatype=7, count=1)
                ]
                
                # 헤더 설정 변경
                header = Header()
                header.stamp = msg.header.stamp  # 원본 스캔과 동일한 타임스탬프
                header.frame_id = self.frame_id  # laser 프레임으로 고정
                
                # 메시지 생성 및 발행
                cloud_msg = self.create_point_cloud2(header, fields, points)
                self.cloud_pub.publish(cloud_msg)
                self.publish_count += 1
                
                # 발행 정보 디버깅 (가끔씩)
                if self.publish_count % 10 == 0:
                    self.get_logger().debug(f"포인트클라우드 발행: {len(points)} 포인트, frame_id={self.frame_id}")
                    
            except Exception as e:
                self.get_logger().error(f"포인트클라우드 발행 오류: {e}")
    
    def create_point_cloud2(self, header, fields, points):
        """PointCloud2 메시지 생성"""
        msg = PointCloud2()
        msg.header = header
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 * float32
        msg.row_step = msg.point_step * msg.width
        
        # 포인트 데이터를 바이트 배열로 변환
        buf = bytearray()
        for p in points:
            buf.extend(struct.pack('ffff', *p))
        msg.data = buf
        
        msg.is_dense = True
        return msg
    
    def quaternion_to_rotation_matrix(self, q):
        """쿼터니언을 회전 행렬로 변환"""
        x, y, z, w = q
        
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
            [1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw)],
            [2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw)],
            [2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)]
        ])
        
        return R

    def euler_from_quaternion(self, x, y, z, w):
        """쿼터니언에서 오일러 각 변환 (라디안)"""
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
    node = ScanToPointcloud()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()