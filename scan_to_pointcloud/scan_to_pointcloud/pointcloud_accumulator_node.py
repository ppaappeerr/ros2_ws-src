import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np
import math
import struct
from std_msgs.msg import Header
import sensor_msgs.msg as sensor_msgs
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException, Buffer
from geometry_msgs.msg import Point

class PointField:
    INT8    = 1
    UINT8   = 2
    INT16   = 3
    UINT16  = 4
    INT32   = 5
    UINT32  = 6
    FLOAT32 = 7
    FLOAT64 = 8

def create_cloud(header, fields, points):
    """포인트 클라우드 메시지 생성 함수"""
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    
    cloud_msg.fields = []
    for f in fields:
        field = sensor_msgs.PointField()
        field.name = f.name
        field.offset = f.offset
        field.datatype = f.datatype
        field.count = f.count
        cloud_msg.fields.append(field)
    
    if len(points) > 0:
        cloud_msg.height = 1
        cloud_msg.width = len(points)
    else:
        cloud_msg.height = 0
        cloud_msg.width = 0
    
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 4 bytes per float * 4 values (x,y,z,intensity)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    
    cloud_msg.data = bytearray()
    for p in points:
        cloud_msg.data.extend(struct.pack('ffff', p[0], p[1], p[2], p[3]))
    
    cloud_msg.is_dense = True
    
    return cloud_msg

def unpack_pointcloud2(cloud_msg):
    """PointCloud2 메시지에서 포인트 배열 추출"""
    points = []
    offset = 0
    point_step = cloud_msg.point_step
    
    while offset + point_step <= len(cloud_msg.data):
        x, y, z, intensity = struct.unpack_from('ffff', cloud_msg.data, offset)
        points.append([x, y, z, intensity])
        offset += point_step
    
    return points

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')
        
        # 파라미터 선언
        self.declare_parameter('max_points', 100000)  # 최대 누적 포인트 수
        self.declare_parameter('grid_size', 0.05)     # 그리드 크기 (m)
        self.declare_parameter('reset_time', 60.0)    # 맵 리셋 시간 (초)
        self.declare_parameter('global_frame', 'map') # 글로벌 프레임
        self.declare_parameter('input_cloud_topic', 'points2') # 입력 토픽
        
        # 파라미터 로드
        self.max_points = self.get_parameter('max_points').value
        self.grid_size = self.get_parameter('grid_size').value
        self.reset_time = self.get_parameter('reset_time').value
        self.global_frame = self.get_parameter('global_frame').value
        input_topic = self.get_parameter('input_cloud_topic').value
        
        # 포인트 클라우드 구독
        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self.pointcloud_callback, 10)
        
        # 출력 토픽 발행자
        self.accumulated_pub = self.create_publisher(
            PointCloud2, 'accumulated_points', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'point_markers', 10)
        
        # TF 리스너
        self.tf_buffer = Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 누적 데이터 저장
        self.accumulated_points = []  # [x, y, z, intensity]
        self.grid_map = {}  # 그리드 -> 포인트 인덱스
        
        # 시간 추적
        self.start_time = time.time()
        
        # 정기적인 발행을 위한 타이머
        self.publish_timer = self.create_timer(1.0, self.publish_accumulated)
        
        self.get_logger().info("PointCloud Accumulator initialized!")
    
    def pointcloud_callback(self, msg):
        """포인트 클라우드 메시지 처리 및 누적"""
        # 리셋 시간 확인
        current_time = time.time()
        if current_time - self.start_time > self.reset_time:
            self.get_logger().info(f"Resetting map after {self.reset_time} seconds")
            self.accumulated_points = []
            self.grid_map = {}
            self.start_time = current_time
        
        # 소스 프레임 -> 글로벌 프레임 변환 시도
        try:
            # 포인트 클라우드의 원본 프레임
            source_frame = msg.header.frame_id
            
            # 변환 확인
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform from {source_frame} to {self.global_frame}: {ex}")
            return
        
        # 포인트 클라우드 데이터 추출
        points = unpack_pointcloud2(msg)
        
        # 변환 행렬 계산
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        
        rotation = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        
        rotation_matrix = self.quaternion_to_rotation_matrix(rotation)
        
        # 포인트를 글로벌 프레임으로 변환하여 누적
        transformed_points = []
        for p in points:
            # 로컬 포인트
            local_point = np.array([p[0], p[1], p[2]])
            
            # 글로벌 좌표로 변환
            global_point = np.dot(rotation_matrix, local_point) + translation
            
            # 그리드 인덱스 계산
            grid_x = int(global_point[0] / self.grid_size)
            grid_y = int(global_point[1] / self.grid_size)
            grid_z = int(global_point[2] / self.grid_size)
            grid_key = (grid_x, grid_y, grid_z)
            
            # 이 그리드에 아직 포인트가 없으면 추가
            if grid_key not in self.grid_map:
                self.accumulated_points.append([global_point[0], global_point[1], global_point[2], p[3]])
                self.grid_map[grid_key] = len(self.accumulated_points) - 1
                transformed_points.append([global_point[0], global_point[1], global_point[2], p[3]])
                
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
                    updated_grid_map = {}
                    for k, v in self.grid_map.items():
                        if v > 0:  # 첫 번째 이후 항목들은 인덱스가 1 감소
                            updated_grid_map[k] = v - 1
                    self.grid_map = updated_grid_map
    
    def quaternion_to_rotation_matrix(self, q):
        """쿼터니언을 회전 행렬로 변환"""
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
            [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
            [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        return R
    
    def publish_accumulated(self):
        """누적된 포인트 클라우드 발행"""
        if not self.accumulated_points:
            return
            
        # 헤더 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame
        
        # 필드 정의
        fields = [
            sensor_msgs.PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            sensor_msgs.PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        # 포인트 클라우드 메시지 생성 및 발행
        cloud_msg = create_cloud(header, fields, self.accumulated_points)
        self.accumulated_pub.publish(cloud_msg)
        
        # 마커 배열 발행
        self.publish_markers()
        
        # 로그
        self.get_logger().info(f"Published accumulated cloud with {len(self.accumulated_points)} points")
    
    def publish_markers(self):
        """누적된 포인트를 마커 배열로 시각화"""
        marker_array = MarkerArray()
        
        # 마커 생성
        marker = Marker()
        marker.header.frame_id = self.global_frame
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
            normalized_z = (point[2] + 1.0) / 2.0  # 높이를 0~1 사이로 정규화
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

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()