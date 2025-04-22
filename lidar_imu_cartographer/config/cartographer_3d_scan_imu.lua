-- filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/config/cartographer_3d_scan_imu.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link", -- IMU 센서의 프레임
  published_frame = "base_link", -- 로봇 베이스 프레임
  odom_frame = "odom", -- 오도메트리 프레임
  provide_odom_frame = false, -- 외부 오도메트리 TF 사용 안 함 (Cartographer가 map->odom 발행)
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true, -- 외부 오도메트리 메시지(/odom) 사용
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0, -- LaserScan 대신 PointCloud2 사용
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1, -- PointCloud2 사용
  lookup_transform_timeout_sec = 1.0, -- TF 조회 타임아웃 증가
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0, -- 오도메트리 샘플링 비율
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = true, -- Cartographer가 map->odom TF를 발행하도록 설정 (중요)
}

-- 3D SLAM 설정
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D = {
  min_range = 0.1,
  max_range = 8.0,
  num_accumulated_range_data = 1, -- 포인트 클라우드를 바로 사용
  voxel_filter_size = 0.05,
  use_imu_data = true, -- IMU 데이터 사용 확인

  -- IMU 관련 설정 (필요시 조정)
  imu_gravity_time_constant = 10.0,
}

-- 서브맵 설정
TRAJECTORY_BUILDER_3D.submaps = {
  high_resolution = 0.05,
  low_resolution = 0.25,
  num_range_data = 70,
}

-- 포즈 그래프 설정
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

-- 오도메트리 관련 가중치 (use_odometry = true 일 때 중요)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options