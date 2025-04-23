include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",      -- IMU 프레임 기준 추적
  published_frame = "base_link",    -- 로봇 베이스 프레임
  odom_frame = "odom",
  provide_odom_frame = true,       -- Cartographer가 map->odom TF 발행
  publish_frame_projected_to_2d = true, -- 2D 평면에 투영된 프레임 발행
  use_pose_extrapolator = true,
  use_odometry = false,            -- 외부 오도메트리 메시지 사용 안 함
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,             -- LaserScan 사용 (중요!)
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,            -- PointCloud2 사용 안 함 (중요!)
  lookup_transform_timeout_sec = 1.0, -- TF 조회 타임아웃
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = true,            -- map->odom TF 발행 활성화
}

-- 2D SLAM 설정
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = true -- IMU 데이터 사용
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 스캔 누적 없이 바로 사용

-- IMU 관련 설정
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 5.0 -- 중력 정렬 시간 상수 (실내 환경에 맞게 조정)

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.55 -- 필요시 RPLiDAR 환경에 맞게 0.3~0.5로 낮춤
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options