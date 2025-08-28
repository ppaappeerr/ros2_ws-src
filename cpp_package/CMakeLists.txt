cmake_minimum_required(VERSION 3.8)
project(cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 필수
set(CMAKE_CXX_STANDARD 17)

# ROS2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)

# PCL 패키지
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration segmentation)

# Eigen (PCL에 포함되지만 명시적으로 추가)
find_package(Eigen3 REQUIRED)

# 헤더 디렉토리
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 컴파일 정의
add_definitions(${PCL_DEFINITIONS})

# PointCloud Sweeper Cpp Node
add_executable(point_cloud_sweeper_cpp_node src/point_cloud_sweeper_cpp_node.cpp)
ament_target_dependencies(point_cloud_sweeper_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_sweeper_cpp_node ${PCL_LIBRARIES})

# Stable Ground Fitter Node (Final Version)
add_executable(stable_ground_fitter_node src/stable_ground_fitter_node.cpp)
ament_target_dependencies(stable_ground_fitter_node rclcpp sensor_msgs pcl_conversions visualization_msgs pcl_msgs tf2_ros)
target_link_libraries(stable_ground_fitter_node ${PCL_LIBRARIES})

# User-Centric ROI Detector Node
add_executable(user_centric_roi_detector src/user_centric_roi_detector.cpp)
ament_target_dependencies(user_centric_roi_detector
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
  std_msgs
)
target_link_libraries(user_centric_roi_detector ${PCL_LIBRARIES})

# Path Planner Node
add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(path_planner_node ${PCL_LIBRARIES})

# Path Planner 3D Node
add_executable(path_planner_3d_node src/path_planner_3d_node.cpp)
ament_target_dependencies(path_planner_3d_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(path_planner_3d_node ${PCL_LIBRARIES})

# Voxel Grid Filter Node
add_executable(voxel_grid_filter_node src/voxel_grid_filter_node.cpp)
ament_target_dependencies(voxel_grid_filter_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(voxel_grid_filter_node ${PCL_LIBRARIES})

# HeightMap Planner Node (2.5D with negative obstacle detection)
add_executable(heightmap_planner_node src/heightmap_planner_node.cpp)
ament_target_dependencies(heightmap_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(heightmap_planner_node ${PCL_LIBRARIES})

# Follow-the-Gap 3D Node (기존)
add_executable(ftg_3d_node src/ftg_3d_node.cpp)
ament_target_dependencies(ftg_3d_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(ftg_3d_node ${PCL_LIBRARIES})

# ============= 통일된 구조의 새 플래너들 =============

# 베이스 클래스 라이브러리
add_library(base_path_planner src/base_path_planner.cpp)
ament_target_dependencies(base_path_planner
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(base_path_planner ${PCL_LIBRARIES})

# P1: 통일된 2D 투영 플래너
add_executable(unified_path_planner_2d src/unified_path_planner_2d.cpp)
ament_target_dependencies(unified_path_planner_2d
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(unified_path_planner_2d base_path_planner ${PCL_LIBRARIES})

# P2: 통일된 순수 2D 스캔 플래너
add_executable(unified_scan_accumulator_planner src/unified_scan_accumulator_planner.cpp)
ament_target_dependencies(unified_scan_accumulator_planner
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(unified_scan_accumulator_planner base_path_planner ${PCL_LIBRARIES})

# P3: 통일된 3D Corridor 플래너
add_executable(unified_path_planner_3d_corridor src/unified_path_planner_3d_corridor.cpp)
ament_target_dependencies(unified_path_planner_3d_corridor
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(unified_path_planner_3d_corridor base_path_planner ${PCL_LIBRARIES})

# P4: 통일된 Follow-the-Gap 3D 플래너
add_executable(unified_ftg_3d_planner src/unified_ftg_3d_planner.cpp)
ament_target_dependencies(unified_ftg_3d_planner
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(unified_ftg_3d_planner base_path_planner ${PCL_LIBRARIES})

# P5: 통일된 HeightMap 플래너
add_executable(unified_heightmap_planner src/unified_heightmap_planner.cpp)
ament_target_dependencies(unified_heightmap_planner
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  angles
)
target_link_libraries(unified_heightmap_planner base_path_planner ${PCL_LIBRARIES})

# 실행 파일 설치
install(TARGETS
  point_cloud_sweeper_cpp_node
  stable_ground_fitter_node
  user_centric_roi_detector
  path_planner_node
  path_planner_3d_node
  voxel_grid_filter_node
  heightmap_planner_node
  ftg_3d_node
  # 새로운 통일된 플래너들
  base_path_planner
  unified_path_planner_2d
  unified_scan_accumulator_planner
  unified_path_planner_3d_corridor
  unified_ftg_3d_planner
  unified_heightmap_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
