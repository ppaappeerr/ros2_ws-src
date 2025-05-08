#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/scan_to_pointcloud/scan_to_pointcloud/accumulated_pointcloud_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration

class AccumulatedPointcloud(Node):
    def __init__(self):
        super().__init__('accumulated_pointcloud')
        
        # 파라미터
        self.declare_parameter('max_points', 10000)
        self.declare_parameter('grid_size', 0.05)
        self.declare_parameter('use_tf', False) # map -> laser 변환 없는 경우 lookup 실패 출력 / False 상태에서 ICP 독립 회전 여부만 파악
        self.declare_parameter('publish_rate', 5.0)
        
        # 파라미터 가져오기
        self.max_points = self.get_parameter('max_points').value
        self.grid_size = self.get_parameter('grid_size').value
        self.use_tf = self.get_parameter('use_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 발행자
        self.accumulated_pub = self.create_publisher(PointCloud2, 'accumulated_points', 10)
        
        # 포인트 클라우드 구독
        self.cloud_sub = self.create_subscription(
            PointCloud2, 'pc_3d', self.cloud_callback, 10)
        
        # TF 설정
        self.tf_buffer = tf2_ros.Buffer(Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 누적 포인트 저장 자료구조
        self.accumulated_points = {}  # 그리드 셀 -> [x, y, z, intensity]
        self.accumulated_points_list = []  # 순서 유지용 리스트
        
        # 타이머
        self.publish_timer = self.create_timer(1.0/self.publish_rate, self.publish_accumulated)
        
        # 로그
        self.get_logger().info(f"누적 포인트클라우드 노드 시작: max_points={self.max_points}, grid_size={self.grid_size}")
    
    def grid_key(self, x, y, z):
        """3D 그리드 키 계산 (Voxel Grid 구현)"""
        # 중요: 3D 공간에서 그리드 키 생성
        cell_x = int(x / self.grid_size)
        cell_y = int(y / self.grid_size)
        cell_z = int(z / self.grid_size)
        
        # 3D 셀 키 반환
        return (cell_x, cell_y, cell_z)
    
    def cloud_callback(self, msg):
        """포인트 클라우드 누적"""
        # 최대 포인트 수 제한
        if len(self.accumulated_points) >= self.max_points:
            excess = len(self.accumulated_points) - int(self.max_points * 0.8)
            if excess > 0:
                # 20%의 오래된 포인트 제거
                for _ in range(excess):
                    if self.accumulated_points_list:
                        old_key = self.accumulated_points_list.pop(0)
                        if old_key in self.accumulated_points:
                            del self.accumulated_points[old_key]
        
        # 변환 행렬 얻기 - 에러 처리 개선
        transform_matrix = None
        if self.use_tf:
            # 중요: 메시지 시간에 가장 가까운 TF를 우선 시도하되, 실패시 최신 TF 사용
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', msg.header.frame_id,
                    Time(seconds=0),  # 가장 최신 TF 사용 (타임스탬프 무시)
                    Duration(seconds=0.2))
                
                # 성공했을 때만 행렬 계산
                transform_matrix = self.compute_transform_matrix(
                    transform.transform.translation,
                    transform.transform.rotation)
                
                # 변환이 성공했다는 로그 추가 (디버깅용)
                self.get_logger().debug(f"TF 변환 성공: {msg.header.frame_id} → map")
            
            except Exception as e:
                # 변환 실패시 명확한 에러 메시지
                self.get_logger().warn(f"TF 변환 실패: {e}")
                return  # 변환 없이는 처리 불가
        
        # 포인트 클라우드 데이터 처리
        offset = 0
        point_step = msg.point_step
        row_step = msg.row_step
        
        # 필드 오프셋 찾기
        x_offset, y_offset, z_offset, intensity_offset = self.find_field_offsets(msg.fields)
        
        # 각 포인트 처리
        for i in range(msg.height):
            for j in range(msg.width):
                # 현재 포인트 데이터 위치
                point_offset = offset + i * row_step + j * point_step
                
                if point_offset + point_step > len(msg.data):
                    continue
                
                # 좌표 추출
                x = struct.unpack('f', msg.data[point_offset + x_offset:point_offset + x_offset + 4])[0]
                y = struct.unpack('f', msg.data[point_offset + y_offset:point_offset + y_offset + 4])[0]
                z = struct.unpack('f', msg.data[point_offset + z_offset:point_offset + z_offset + 4])[0]
                
                # 강도 추출
                intensity = struct.unpack('f', msg.data[point_offset + intensity_offset:point_offset + intensity_offset + 4])[0]
                
                # 유효하지 않은 포인트 건너뛰기
                if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z)):
                    continue
                
                # 변환 행렬 적용 (global 좌표계로 변환)
                if transform_matrix is not None:
                    point = np.array([x, y, z, 1.0])
                    transformed = np.dot(transform_matrix, point)
                    x, y, z = transformed[:3]
                
                # 그리드 키 계산 (3D 그리드로)
                key = self.grid_key(x, y, z)
                
                # 새 포인트 저장
                self.accumulated_points[key] = [x, y, z, intensity]
                
                # 키 순서 추적 (새 키만 추가)
                if key not in self.accumulated_points_list:
                    self.accumulated_points_list.append(key)
    
    def find_field_offsets(self, fields):
        """포인트 클라우드 필드 오프셋 찾기"""
        x_offset = y_offset = z_offset = intensity_offset = 0
        
        for field in fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
            elif field.name in ('intensity', 'rgb', 'rgba'):
                intensity_offset = field.offset
        
        return x_offset, y_offset, z_offset, intensity_offset
    
    def publish_accumulated(self):
        """누적된 포인트클라우드 발행"""
        if not self.accumulated_points:
            return
        
        points = list(self.accumulated_points.values())
        
        # 헤더 설정 - 중요: 프레임 ID를 map으로 설정
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'  # Cartographer에서 필요한 프레임
        
        # 필드 정의
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # PointCloud2 메시지 생성
        cloud_msg = self.create_cloud_msg(header, fields, points)
        
        # 발행
        self.accumulated_pub.publish(cloud_msg)
        
        # 상태 로깅 (간헐적으로)
        if len(self.accumulated_points) % 1000 == 0:
            self.get_logger().info(f"누적 포인트 수: {len(self.accumulated_points)}")
    
    def create_cloud_msg(self, header, fields, points):
        """PointCloud2 메시지 생성"""
        msg = PointCloud2()
        msg.header = header
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 x float32
        msg.row_step = msg.point_step * msg.width
        
        # 포인트 데이터를 바이트 배열로 변환
        buf = bytearray()
        for p in points:
            buf.extend(struct.pack('ffff', *p))
        msg.data = bytes(buf)
        
        msg.is_dense = True
        return msg
    
    def compute_transform_matrix(self, trans, rot):
        """변환 행렬 계산"""
        # 회전 행렬 계산
        x, y, z, w = rot.x, rot.y, rot.z, rot.w
        
        xx = x * x
        xy = x * y
        xz = x * z
        xw = x * w
        
        yy = y * y
        yz = y * z
        yw = y * w
        
        zz = z * z
        zw = z * w
        
        # 4x4 회전 행렬 (3x3 회전 + 맨 아래 행)
        rot_mat = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0],
            [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0],
            [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0],
            [0, 0, 0, 1]
        ])
        
        # 4x4 이동 행렬
        trans_mat = np.array([
            [1, 0, 0, trans.x],
            [0, 1, 0, trans.y],
            [0, 0, 1, trans.z],
            [0, 0, 0, 1]
        ])
        
        # 변환 행렬 = 이동 * 회전
        return np.dot(trans_mat, rot_mat)

def main(args=None):
    rclpy.init(args=args)
    node = AccumulatedPointcloud()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 종료됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()