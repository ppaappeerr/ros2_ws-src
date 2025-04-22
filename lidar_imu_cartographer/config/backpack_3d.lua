-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",  -- IMU 프레임 (중력 방향 기준)
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,  -- 외부 odom 사용시 false
  publish_frame_projected_to_2d = false,  -- 3D 모드에서는 false
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,  -- 3D 모드에서는 laser scan 사용 안함
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,  -- PointCloud2 사용
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = false,  -- TF 충돌 방지
}

-- 중요: 여기서는 use_imu_data를 설정하지 않음 (이미 trajectory_builder_3d.lua에 정의됨)
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_3D.min_range = 0.1
TRAJECTORY_BUILDER_3D.max_range = 8.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- 안정성 향상을 위한 추가 설정
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10  -- IMU 중력 방향 안정화
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80  -- 서브맵당 포인트 수 감소
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- 반복 횟수 감소

-- 시간 역전 문제 해결을 위한 설정
TRAJECTORY_BUILDER.collate_fixed_frame = false  -- 고정 프레임 충돌 방지
TRAJECTORY_BUILDER.collate_landmarks = false  -- 랜드마크 충돌 방지

-- 비정렬 데이터 허용 설정
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3  -- 샘플링 비율 감소
POSE_GRAPH.constraint_builder.min_score = 0.5  -- 매칭 최소 점수
POSE_GRAPH.optimization_problem.log_solver_summary = true  -- 솔버 요약 로깅

return options
