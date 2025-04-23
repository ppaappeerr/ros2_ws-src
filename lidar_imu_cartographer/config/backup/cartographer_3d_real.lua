include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 1.0,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = false,
}

-- 3D SLAM 설정
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

-- 중요: use_imu_data 설정이 올바른 위치에 있어야 함
TRAJECTORY_BUILDER_3D.use_imu_data = true

-- 3D 설정
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 8.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- 고해상도 필터 설정 
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.0

-- 저해상도 필터 설정
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 15.0

-- Submaps 설정
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.30
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100

-- 모션 필터
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.04

-- IMU 중력 상수
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.5

return options