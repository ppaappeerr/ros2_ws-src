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
  publish_to_tf = false,
  use_pose_extrapolator = true,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.5
}

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.use_trajectory_builder_2d = false

-- 최소한의 필수 설정만 사용
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

return options