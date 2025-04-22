include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- IMU 프레임을 추적 프레임으로 사용
  published_frame = "base_link",  -- 베이스 링크를 발행 프레임으로 사용
  odom_frame = "odom",
  provide_odom_frame = false,  -- 외부에서 오돔을 제공하지 않음
  publish_frame_projected_to_2d = false,  -- 3D 매핑이므로 false
  use_pose_extrapolator = true,  -- 포즈 추정
  use_odometry = false,  -- 오도메트리 사용 안 함
  use_nav_sat = false,  -- GPS 사용 안 함
  use_landmarks = false,  -- 랜드마크 사용 안 함
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 포인트 클라우드 사용
  lookup_transform_timeout_sec = 0.2,  -- TF 타임아웃
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = false,  -- TF 발행 비활성화 (주요 변경점)
}

-- 3D SLAM 설정
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

-- 3D Trajectory Builder 설정
-- 중요: use_imu_data = true 반드시 포함
TRAJECTORY_BUILDER_3D.use_imu_data = true
TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 8.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- 3D 고해상도 Voxel 필터
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.0

-- 3D 저해상도 Voxel 필터
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 15.0

-- 3D Submaps 설정
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.30
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49

-- 모션 필터 (성능 향상을 위해 설정)
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.04

-- IMU 중력 상수 설정
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

-- Ceres 스캔 매칭 (중요: 로컬 맵핑의 핵심)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight = 20.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.0

-- PoseGraph 설정 (글로벌 최적화)
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3