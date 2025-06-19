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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)  # Added for pointcloud_formatter_node

# PCL 패키지 (ROS2 통합 버전)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# 헤더 디렉토리
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 컴파일 정의
add_definitions(${PCL_DEFINITIONS})

# ICP 3D Odometry Node (PCL만 사용)
add_executable(icp_3d_odom_node src/icp_3d_odom_node.cpp)

ament_target_dependencies(icp_3d_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  pcl_ros
  pcl_conversions
)

target_link_libraries(icp_3d_odom_node 
  ${PCL_LIBRARIES}
)

# ===== 새로운 C++ 노드 추가 =====
add_executable(pointcloud_formatter_node src/pointcloud_formatter_node.cpp)

ament_target_dependencies(pointcloud_formatter_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

# 실행 파일 설치
install(TARGETS
  icp_3d_odom_node
  pointcloud_formatter_node  # Added new node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()