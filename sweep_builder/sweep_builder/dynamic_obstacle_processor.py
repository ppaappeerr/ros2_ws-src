import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField  # 🔥 PointField 추가
from std_msgs.msg import String
import numpy as np
import open3d as o3d
import struct

# ROS PointCloud2를 Open3D로 변환하는 헬퍼 함수
def ros2_pointcloud2_to_o3d(cloud_msg):
    points = []
    point_step = cloud_msg.point_step
    data = cloud_msg.data
    for i in range(0, len(data), point_step):
        x, y, z = struct.unpack_from('fff', data, i)
        points.append([x, y, z])
    
    if not points: 
        return None
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    return pcd

# Open3D 포인트 클라우드를 ROS2로 변환하는 헬퍼 함수
def o3d_to_ros2_pointcloud2(pcd, header):
    points = np.asarray(pcd.points)
    ros_msg = PointCloud2()
    ros_msg.header = header
    ros_msg.height = 1
    ros_msg.width = len(points)
    ros_msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    ros_msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
    ros_msg.is_bigendian = False
    ros_msg.point_step = 12
    ros_msg.row_step = ros_msg.point_step * len(points)
    ros_msg.data = points.astype(np.float32).tobytes()
    ros_msg.is_dense = True
    return ros_msg

class Voxel:
    """각 복셀의 상태를 저장하는 클래스"""
    def __init__(self):
        self.state = 'EMPTY'  # EMPTY, OCCUPIED, STATIC
        self.last_seen = -1.0
        self.first_seen = -1.0

class DynamicObstacleProcessor(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_processor')
        
        # 파라미터 선언
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('ground_distance_threshold', 0.08)
        self.declare_parameter('static_duration_threshold', 3.0)
        self.declare_parameter('clearance_duration_threshold', 1.0)
        # ROI 파라미터
        self.declare_parameter('roi.x_min', 0.1)
        self.declare_parameter('roi.x_max', 1.5)
        self.declare_parameter('roi.y_max', 0.5)

        # 구독/발행 설정
        self.subscription = self.create_subscription(
            PointCloud2, '/accumulated_points', self.pointcloud_callback, 10)
        self.feedback_pub = self.create_publisher(String, '/dynamic_obstacle_feedback', 10)  # 🔥 토픽명 변경
        
        # 시각화용 발행자
        self.ground_pub = self.create_publisher(PointCloud2, '/processed/ground', 10)
        self.static_pub = self.create_publisher(PointCloud2, '/processed/static_obstacles', 10)
        self.dynamic_pub = self.create_publisher(PointCloud2, '/processed/dynamic_obstacles', 10)

        # 복셀 그리드 초기화
        self.voxel_grid = {}
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        
        self.get_logger().info('Dynamic Obstacle Processor Node is running.')

    def pointcloud_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        pcd = ros2_pointcloud2_to_o3d(msg)
        if pcd is None: 
            return

        # 1. 지면 분리
        dist_thresh = self.get_parameter('ground_distance_threshold').get_parameter_value().double_value
        try:
            plane_model, inliers = pcd.segment_plane(dist_thresh, ransac_n=3, num_iterations=100)
            ground_cloud = pcd.select_by_index(inliers)
            obstacle_candidates = pcd.select_by_index(inliers, invert=True)
            
            self.ground_pub.publish(o3d_to_ros2_pointcloud2(ground_cloud, msg.header))
        except:
            # 지면 분리 실패 시 전체를 장애물로 간주
            obstacle_candidates = pcd
            self.get_logger().warn("Ground plane segmentation failed, using all points as obstacles")

        # 2. 복셀 그리드 상태 업데이트
        for point in np.asarray(obstacle_candidates.points):
            voxel_index = tuple((point / self.voxel_size).astype(int))
            if voxel_index not in self.voxel_grid:
                self.voxel_grid[voxel_index] = Voxel()
                self.voxel_grid[voxel_index].first_seen = current_time
            
            self.voxel_grid[voxel_index].state = 'OCCUPIED'
            self.voxel_grid[voxel_index].last_seen = current_time

        # 3. 상태 변화 분석
        static_duration = self.get_parameter('static_duration_threshold').get_parameter_value().double_value
        clear_duration = self.get_parameter('clearance_duration_threshold').get_parameter_value().double_value
        
        dynamic_points = []
        static_points = []

        for index, voxel in self.voxel_grid.items():
            if voxel.state == 'OCCUPIED':
                if current_time - voxel.first_seen > static_duration:
                    voxel.state = 'STATIC'
                elif current_time - voxel.last_seen > clear_duration:
                    voxel.state = 'EMPTY'
            
            voxel_center = (np.array(index) + 0.5) * self.voxel_size
            if voxel.state == 'OCCUPIED':
                dynamic_points.append(voxel_center)
            elif voxel.state == 'STATIC':
                static_points.append(voxel_center)
        
        # 시각화용 포인트 클라우드 생성 및 발행
        if dynamic_points:
            dynamic_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dynamic_points))
            self.dynamic_pub.publish(o3d_to_ros2_pointcloud2(dynamic_pcd, msg.header))
        if static_points:
            static_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(static_points))
            self.static_pub.publish(o3d_to_ros2_pointcloud2(static_pcd, msg.header))

        # 4. 피드백 생성
        self.generate_feedback(dynamic_points)

    def generate_feedback(self, obstacles):
        roi_x_min = self.get_parameter('roi.x_min').get_parameter_value().double_value
        roi_x_max = self.get_parameter('roi.x_max').get_parameter_value().double_value
        roi_y_max = self.get_parameter('roi.y_max').get_parameter_value().double_value

        feedback_command = "Clear"
        closest_dist = float('inf')

        for point in obstacles:
            if roi_x_min < point[0] < roi_x_max and abs(point[1]) < roi_y_max:
                dist = np.linalg.norm(point)
                if dist < closest_dist:
                    closest_dist = dist
                    if point[1] > 0.15:
                        feedback_command = "Turn Right"
                    elif point[1] < -0.15:
                        feedback_command = "Turn Left"
                    else:
                        feedback_command = "Stop"

        self.get_logger().info(f'Feedback: {feedback_command} (closest: {closest_dist:.2f}m)')
        feedback_msg = String()
        feedback_msg.data = feedback_command
        self.feedback_pub.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()