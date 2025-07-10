import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Header
import numpy as np
from sklearn.linear_model import RANSACRegressor
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

class Voxel:
    def __init__(self):
        self.state = 'EMPTY'
        self.last_seen = -1.0
        self.first_seen = -1.0
        self.point = np.zeros(3)

class DynamicObstacleProcessor(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_processor')
        
        # 파라미터 선언
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('ground_distance_threshold', 0.03)
        self.declare_parameter('static_duration_threshold', 3.0) 
        self.declare_parameter('clearance_duration_threshold', 1.5)
        self.declare_parameter('roi.x_min', 0.1)
        self.declare_parameter('roi.x_max', 1.5)
        self.declare_parameter('roi.y_max', 0.5)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 구독/발행 설정
        self.subscription = self.create_subscription(
            PointCloud2, '/sweep_cloud', self.pointcloud_callback, 10)
        self.feedback_pub = self.create_publisher(String, '/obstacle_feedback', 10)
        
        # --- [디버깅] 시각화용 발행자 추가 ---
        self.candidate_pub = self.create_publisher(PointCloud2, '/processed/obstacle_candidates', 10)
        self.ground_pub = self.create_publisher(PointCloud2, '/processed/ground', 10)
        self.static_pub = self.create_publisher(PointCloud2, '/processed/static_obstacles', 10)
        self.dynamic_pub = self.create_publisher(PointCloud2, '/processed/dynamic_obstacles', 10)

        self.voxel_grid = {}
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        
        self.get_logger().info('Dynamic Obstacle Processor Node (DEBUG MODE) is running.')
    
    def ros2_pointcloud2_to_numpy(self, cloud_msg):
        point_step = cloud_msg.point_step
        data = cloud_msg.data
        if not data: return np.array([])
        dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32)]
        dtype = np.dtype(dtype_list)
        structured_array = np.frombuffer(data, dtype=dtype, count=cloud_msg.width)
        return np.vstack([structured_array['x'], structured_array['y'], structured_array['z']]).T

    def numpy_to_ros2_pointcloud2(self, points, header):
        ros_msg = PointCloud2()
        ros_msg.header = header
        ros_msg.height = 1
        ros_msg.width = len(points)
        ros_msg.is_dense = True
        ros_msg.is_bigendian = False
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        ros_msg.fields = fields
        ros_msg.point_step = 12
        ros_msg.row_step = ros_msg.point_step * len(points)
        ros_msg.data = points.astype(np.float32).tobytes()
        return ros_msg

    def pointcloud_callback(self, msg):
        current_time = self.get_clock().now()
        pcd_array = self.ros2_pointcloud2_to_numpy(msg)
        
        self.get_logger().info(f"--- New Sweep Received ({msg.header.stamp}) ---")
        self.get_logger().info(f"[DEBUG] 1. Initial points: {pcd_array.shape[0]}")

        if pcd_array.shape[0] < 10: # 점이 너무 적으면 무시
            return

        # 지면 분리 (RANSAC)
        dist_thresh = self.get_parameter('ground_distance_threshold').get_parameter_value().double_value
        XY = pcd_array[:, :2]
        z = pcd_array[:, 2]
        obstacle_candidates_np = pcd_array # 기본값은 모든 점을 장애물 후보로

        try:
            ransac = RANSACRegressor(min_samples=10, residual_threshold=dist_thresh, max_trials=100)
            ransac.fit(XY, z)
            
            # --- [로직 강화] ---
            # 찾은 평면의 법선 벡터(normal vector)를 확인
            # 평면 방정식 z = ax + by + d 에서 법선 벡터는 (a, b, -1)
            # 수평면은 a와 b가 0에 가까워야 함.
            a, b = ransac.estimator_.coef_
            normal_z = -1.0
            normal_vector = np.array([a, b, normal_z])
            normal_vector /= np.linalg.norm(normal_vector) # 정규화
            
            # Z축과 이루는 각도가 30도 이내일 때만 지면으로 인정 (수직 벽 제거)
            if np.abs(normal_vector[2]) > np.cos(np.deg2rad(30)):
                inlier_mask = ransac.inlier_mask_
                outlier_mask = np.logical_not(inlier_mask)
                ground_points_np = pcd_array[inlier_mask]
                obstacle_candidates_np = pcd_array[outlier_mask]
                
                self.get_logger().info(f"[DEBUG] 2. Ground plane found. Ground points: {ground_points_np.shape[0]}")
                if ground_points_np.shape[0] > 0:
                    self.ground_pub.publish(self.numpy_to_ros2_pointcloud2(ground_points_np, msg.header))
            else:
                self.get_logger().info("[DEBUG] 2. Plane found, but it's not horizontal (likely a wall). Treating all as obstacles.")

        except Exception as e:
            self.get_logger().warn(f"RANSAC fitting failed: {e}")
        
        self.get_logger().info(f"[DEBUG] 3. Obstacle candidates: {obstacle_candidates_np.shape[0]}")
        # --- [디버깅] ---
        # 지면 제거 후 남은 점들을 무조건 발행해서 확인
        if obstacle_candidates_np.shape[0] > 0:
            self.candidate_pub.publish(self.numpy_to_ros2_pointcloud2(obstacle_candidates_np, msg.header))

        # 복셀 그리드 상태 업데이트
        current_time_sec = current_time.nanoseconds / 1e9
        
        for point in obstacle_candidates_np:
            voxel_index = tuple((point / self.voxel_size).astype(int))
            if voxel_index not in self.voxel_grid:
                self.voxel_grid[voxel_index] = Voxel()
                self.voxel_grid[voxel_index].first_seen = current_time_sec
            self.voxel_grid[voxel_index].state = 'OCCUPIED'
            self.voxel_grid[voxel_index].last_seen = current_time_sec
            self.voxel_grid[voxel_index].point = point

        # 상태 변화 분석
        static_duration = self.get_parameter('static_duration_threshold').get_parameter_value().double_value
        clear_duration = self.get_parameter('clearance_duration_threshold').get_parameter_value().double_value
        dynamic_points = []
        static_points = []
        voxels_to_delete = []

        for index, voxel in self.voxel_grid.items():
            time_since_last_seen = current_time_sec - voxel.last_seen
            if time_since_last_seen > static_duration + clear_duration:
                 voxels_to_delete.append(index)
                 continue
            
            if voxel.state == 'OCCUPIED':
                if current_time_sec - voxel.first_seen > static_duration:
                    voxel.state = 'STATIC'
                elif time_since_last_seen > clear_duration:
                    voxel.state = 'EMPTY'
            
            if voxel.state == 'OCCUPIED':
                dynamic_points.append(voxel.point)
            elif voxel.state == 'STATIC':
                static_points.append(voxel.point)
        
        for index in voxels_to_delete:
            del self.voxel_grid[index]
            
        self.get_logger().info(f"[DEBUG] 4. Dynamic points: {len(dynamic_points)}, Static points: {len(static_points)}")

        # 시각화 및 피드백 생성
        header = Header(frame_id='odom', stamp=msg.header.stamp)
        if dynamic_points:
            self.dynamic_pub.publish(self.numpy_to_ros2_pointcloud2(np.array(dynamic_points), header))
        if static_points:
            self.static_pub.publish(self.numpy_to_ros2_pointcloud2(np.array(static_points), header))
            
        self.generate_feedback(dynamic_points, msg.header.stamp)

    def generate_feedback(self, obstacles, stamp):
        # ... (이하 피드백 생성 로직은 동일)
        # ...
        pass # To keep the code block short

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()