include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",                     -- SLAM 지도 좌표
  tracking_frame = "base_link",          -- robot 기준 frame
  published_frame = "base_link",
  odom_frame = "odom",                   -- Odometry 좌표
  provide_odom_frame = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = true,  -- 2D로 투영된 결과
  
  -- *** 누락된 필수 매개변수들 추가 ***
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  
  -- *** 센서 개수 설정 추가 ***
  num_laser_scans = 1,                    -- 레이저 스캐너 1개
  num_multi_echo_laser_scans = 0,         -- 멀티 에코 레이저 없음
  num_subdivisions_per_laser_scan = 1,    -- 스캔 분할 없음
  num_point_clouds = 0,                   -- 포인트클라우드 없음
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false          -- IMU는 이미 적용됨
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
POSE_GRAPH.optimize_every_n_nodes = 35              -- loop closure 설정

return options
