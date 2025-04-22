include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link", -- IMU 프레임 기준 추적
  published_frame = "odom", -- Cartographer가 map->odom TF 발행하도록 변경
  odom_frame = "odom",
  provide_odom_frame = false, -- 중요: 외부 오도메트리(TF) 사용하므로 false
  publish_frame_projected_to_2d = false,
  use_odometry = true, -- 중요: 외부 오도메트리(TF) 사용
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0, -- 오도메트리 샘플링 비율
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true

-- TRAJECTORY_BUILDER_3D 설정
-- TRAJECTORY_BUILDER_3D.use_imu_data = true -- 주석 처리 유지 (기본값 true 사용)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0 -- 필요시 조정

-- POSE_GRAPH 설정 (오도메트리 가중치 추가)
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5 -- 오도메트리 번역 가중치 (조정 필요)
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5 -- 오도메트리 회전 가중치 (조정 필요)
-- POSE_GRAPH.optimize_every_n_nodes = 30 -- 필요시 조정

return options