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

# 헤더 디렉토리
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
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


# 실행 파일 설치
install(TARGETS
  point_cloud_sweeper_cpp_node
  stable_ground_fitter_node
  user_centric_roi_detector
  path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
