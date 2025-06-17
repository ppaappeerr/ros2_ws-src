include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- TF 프레임
  map_frame        = "map",
  tracking_frame   = "base_link",
  published_frame  = "odom",   -- published_frame을 odom으로 변경
  odom_frame       = "odom",

  provide_odom_frame  = false,   -- EKF가 odom TF 발행
  use_odometry        = true,    -- EKF /odom 사용
  use_nav_sat         = false,
  use_landmarks       = false,
  publish_frame_projected_to_2d = true,

  -- Cartographer가 요구하는 9 개 샘플링/타임아웃 파라미터
  lookup_transform_timeout_sec      = 0.2,
  submap_publish_period_sec         = 0.3,
  pose_publish_period_sec           = 5e-3,
  trajectory_publish_period_sec     = 30e-3,

  rangefinder_sampling_ratio        = 1.0,
  odometry_sampling_ratio           = 1.0,
  imu_sampling_ratio                = 1.0,       -- IMU를 false로 두더라도 필수
  fixed_frame_pose_sampling_ratio   = 1.0,       -- 꼭 넣어야 함
  landmarks_sampling_ratio          = 1.0,       -- use_landmarks=false 여도 필요

  -- 센서 수
  num_laser_scans               = 1,
  num_multi_echo_laser_scans    = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds              = 0,
}

-- 2‑D SLAM 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data                     = false  -- IMU 정보는 이미 /scan_flat 생성에 들어감
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data       = 2

-- 그래프 최적화 주기
POSE_GRAPH.optimize_every_n_nodes = 35

return options
