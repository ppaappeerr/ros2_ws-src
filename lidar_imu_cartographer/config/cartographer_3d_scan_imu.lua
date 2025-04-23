-- filepath: /home/p/ros2_ws/src/lidar_imu_cartographer/config/cartographer_3d_scan_imu.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",      -- IMU 센서 프레임 (TF와 일치 확인)
  published_frame = "base_link",   -- 로봇 베이스 프레임 (TF와 일치 확인)
  odom_frame = "odom",           -- Cartographer가 생성할 오도메트리 프레임
  provide_odom_frame = true,     -- Cartographer가 map->odom TF 발행 (중요!)
  publish_frame_projected_to_2d = false, -- 3D SLAM
  use_odometry = false,          -- 외부 오도메트리 사용 안 함 (중요!)
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,           -- LaserScan 사용 안 함
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,          -- PointCloud2 사용 (중요!)
  lookup_transform_timeout_sec = 1.0, -- TF 조회 타임아웃 (필요시 1.5 또는 2.0으로 증가)
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = true,          -- Cartographer가 map->odom TF 발행 (중요!)
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4 -- 시스템에 따라 조정 가능

TRAJECTORY_BUILDER_3D = {
  min_range = 0.1,
  max_range = 8.0, -- LiDAR 스펙에 맞게 조정
  num_accumulated_range_data = 1, -- 포인트 클라우드를 바로 사용
  voxel_filter_size = 0.05,
  use_imu_data = true, -- IMU 데이터 사용 (필수!)

  -- IMU 관련 설정 (기본값 사용, 필요시 튜닝)
  imu_gravity_time_constant = 10.0,
  -- rotational_scan_matcher_histogram_size = 30, -- 필요시 활성화
}

-- 서브맵 설정 (기본값, 필요시 튜닝)
TRAJECTORY_BUILDER_3D.submaps = {
  high_resolution = 0.05,
  low_resolution = 0.25,
  num_range_data = 70, -- 서브맵당 포인트 클라우드 수 (튜닝 대상)
}

-- 포즈 그래프 설정 (기본값, 필요시 튜닝)
POSE_GRAPH.optimize_every_n_nodes = 35 -- 최적화 빈도 (튜닝 대상)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

-- 오도메트리 가중치 (use_odometry = false 이므로 영향 적음)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options