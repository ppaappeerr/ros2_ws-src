include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,  -- 외부 TF 사용
  publish_frame_projected_to_2d = false,  -- 3D 모드
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,  -- 레이저스캔 미사용
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- 포인트클라우드 사용
  lookup_transform_timeout_sec = 2.0,  -- 타임아웃 증가 (중요)
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = false,  -- TF 발행 비활성화
}

-- 디버깅을 위한 로깅 레벨 조정
POSE_GRAPH.optimize_every_n_nodes = 30  -- 더 자주 최적화
MAP_BUILDER.collate_by_trajectory = false  -- 싱글 코어 모드 (디버깅 용이)

-- 3D SLAM 모드
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 2  -- 스레드 수 감소 (디버깅 용이)

-- 3D 설정 디버깅 모드 (덜 엄격한 설정)
TRAJECTORY_BUILDER_3D.min_range = 0.05  -- 더 가까운 범위도 허용
TRAJECTORY_BUILDER_3D.max_range = 12.0  -- 더 먼 범위도 허용
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1  -- 바로 포인트 사용
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1  -- 더 큰 복셀 크기 (더 적은 포인트)
TRAJECTORY_BUILDER_3D.use_imu_data = true  -- IMU 사용 확인

-- 필터 설정 완화
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 50  -- 더 적은 포인트 허용
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 50  -- 더 적은 포인트 허용

-- 서브맵 설정 조정
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.20  -- 더 낮은 해상도
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 12.0
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.50  -- 더 낮은 해상도
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 50  -- 서브맵당 더 적은 포인트 허용

-- 스캔 매칭 파라미터 - 더 적극적인 매칭
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 20.0  -- 증가
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2.0  -- 증가

-- 모션 필터 - 더 많은 업데이트 허용
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.2  -- 더 빠른 업데이트
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.05  -- 더 미세한 움직임도 포착
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.02  -- 더 미세한 회전도 포착

-- IMU 중력 시간 상수
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

-- 포즈 그래프 최적화 - 디버깅을 위한 더 공격적인 설정
POSE_GRAPH.constraint_builder.min_score = 0.45  -- 낮은 스코어도 허용
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.5  -- 낮은 스코어도 허용
POSE_GRAPH.optimization_problem.huber_scale = 1e3  -- 증가
POSE_GRAPH.optimization_problem.rotation_weight = 5e3  -- 증가
POSE_GRAPH.optimize_every_n_nodes = 20  -- 더 자주 최적화

return options