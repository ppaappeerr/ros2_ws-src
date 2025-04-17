-- filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/config/cartographer_3d.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- 기존 map 프레임 사용
  tracking_frame = "imu_link",  -- IMU 기준 추적
  published_frame = "base_link",  -- TF 발행 기준 프레임
  odom_frame = "odom",
  provide_odom_frame = false,  -- 외부 TF 사용
  publish_frame_projected_to_2d = false,  -- 3D 매핑
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,  -- 3D 모드에서는 스캔 없음
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 포인트클라우드 사용
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = false,  -- 중요: TF 발행 비활성화 (imu_tf_publisher 사용)
}

-- 3D SLAM 모드
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

-- 3D 설정 최적화
TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 15.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  -- 누적된 포인트 바로 사용
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1  -- 포인트 필터링
TRAJECTORY_BUILDER_3D.use_imu_data = true
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 3.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 15.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.3
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.0
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

-- 루프 클로징 설정
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.rotation_weight = 1e3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.optimize_every_n_nodes = 45

return options