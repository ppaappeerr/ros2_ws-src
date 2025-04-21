include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- IMU 프레임을 추적
  published_frame = "base_link", -- 위치 발행 프레임
  odom_frame = "odom",
  provide_odom_frame = false,  -- 중요: TF 충돌 방지
  publish_to_tf = false,  -- Cartographer가 TF를 발행하지 않도록 설정
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  
  -- 중요: TF 충돌 해결을 위한 설정
  
  lookup_transform_timeout_sec = 0.2,  -- 시간 단축으로 반응성 향상
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 중요: 기본 설정은 유지하고 필요한 부분만 재정의
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

-- 포인트 클라우드 처리 안정화
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 30  -- 누적 데이터 증가
TRAJECTORY_BUILDER_3D.min_range = 0.25  -- 0.2 → 0.25
TRAJECTORY_BUILDER_3D.max_range = 8.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15  -- 노이즈 줄이기

-- IMU 설정 (중복 없이)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 20.0  -- 15.0 → 20.0 (더 안정적)
TRAJECTORY_BUILDER_3D.use_imu_data = true  -- 반드시 여기서만 선언!

-- 회전 안정화
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0  -- 이동 가중치 증가 
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 100.0    -- 매칭 안정성 개선

-- 고속 스캔 매칭
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1  -- 모션 필터링 개선
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.04   -- 모션 필터링 개선

-- 좀 더 빳빳한 지도 생성
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1  -- 높은 해상도
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80    -- 서브맵당 포인트 증가

-- 포즈 그래프 최적화 설정 (전역 안정성)
POSE_GRAPH = {}
POSE_GRAPH.constraint_builder = {}
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 더 엄격한 제약 조건
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2  -- 샘플링 비율 감소 (더 적은 제약조건 = 더 안정적)
POSE_GRAPH.optimization_problem = {}
POSE_GRAPH.optimization_problem.rotation_weight = 100  -- 회전 매칭 기준 강화
POSE_GRAPH.optimize_every_n_nodes = 50  -- 더 적은 최적화

return options