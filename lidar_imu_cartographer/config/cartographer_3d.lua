include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- IMU 프레임
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 포인트 클라우드 사용
  lookup_transform_timeout_sec = 0.5,  -- 타임아웃 수정
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

-- 주요 수정: 포인트 클라우드 처리 안정화
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 10  -- 충분한 스캔 축적
TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 8.0  -- 적절한 범위 제한
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15

-- 중요: IMU 파라미터 추가 (use_imu_data 제거)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0  -- IMU 안정화

-- 스캔 매칭 개선
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 200.0

-- 서브맵 설정
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 60

return options