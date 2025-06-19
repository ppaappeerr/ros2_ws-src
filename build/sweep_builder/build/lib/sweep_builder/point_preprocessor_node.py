import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import numpy as np
import struct

class PointPreprocessor(Node):
    def __init__(self):
        super().__init__('point_preprocessor_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points_3d',
            self.pointcloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/processed_cloud', 10)

    def pointcloud_callback(self, cloud_msg):
        # PointCloud2 메시지에서 포인트 데이터 추출
        points = self.parse_pointcloud2(cloud_msg)
        
        # 시간 필드가 없다면 추가
        processed_points = []
        for i, point in enumerate(points):
            x, y, z = point[:3]
            # 기존에 시간 필드가 있으면 사용, 없으면 인덱스 기반으로 생성
            if len(point) > 3:
                time_offset = point[3]
            else:
                time_offset = i * 0.001  # 1ms 간격으로 가정
            
            processed_points.append([x, y, z, time_offset])

        # PointCloud2 메시지 생성
        header = cloud_msg.header
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        point_cloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(processed_points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=16,
            row_step=16 * len(processed_points),
            data=np.asarray(processed_points, dtype=np.float32).tobytes()
        )
        self.publisher.publish(point_cloud_msg)

    def parse_pointcloud2(self, cloud_msg):
        """PointCloud2 메시지에서 포인트 데이터를 추출"""
        points = []
        point_step = cloud_msg.point_step
        
        # 필드 정보 파싱
        x_offset = y_offset = z_offset = time_offset = None
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
            elif field.name in ['time', 't', 'timestamp']:
                time_offset = field.offset
        
        # 데이터 추출
        for i in range(cloud_msg.width):
            base_idx = i * point_step
            
            # x, y, z 좌표 추출
            x = struct.unpack('f', cloud_msg.data[base_idx + x_offset:base_idx + x_offset + 4])[0]
            y = struct.unpack('f', cloud_msg.data[base_idx + y_offset:base_idx + y_offset + 4])[0]
            z = struct.unpack('f', cloud_msg.data[base_idx + z_offset:base_idx + z_offset + 4])[0]
            
            # 시간 정보가 있으면 추출
            if time_offset is not None:
                time_val = struct.unpack('f', cloud_msg.data[base_idx + time_offset:base_idx + time_offset + 4])[0]
                points.append([x, y, z, time_val])
            else:
                points.append([x, y, z])
        
        return points

def main(args=None):
    rclpy.init(args=args)
    node = PointPreprocessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()