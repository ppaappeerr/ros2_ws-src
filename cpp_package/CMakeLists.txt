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
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

# 메시지 생성을 위해 추가
find_package(rosidl_default_generators REQUIRED)

# PCL 패키지
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# 메시지 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleFeedback.msg"
  "msg/NavHint.msg"
  DEPENDENCIES std_msgs
)

# 헤더 디렉토리
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 컴파일 정의
add_definitions(${PCL_DEFINITIONS})

# ICP 3D Odometry Node
add_executable(icp_3d_odom_node src/icp_3d_odom_node.cpp)
ament_target_dependencies(icp_3d_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs tf2_eigen Eigen3 pcl_ros pcl_conversions
)
target_link_libraries(icp_3d_odom_node ${PCL_LIBRARIES})

# Pointcloud Formatter Node
add_executable(pointcloud_formatter_node src/pointcloud_formatter_node.cpp)
ament_target_dependencies(pointcloud_formatter_node
  rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions Eigen3
)

# Front Cloud Filter Node
add_executable(front_cloud_filter_node src/front_cloud_filter_node.cpp)
ament_target_dependencies(front_cloud_filter_node
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(front_cloud_filter_node ${PCL_LIBRARIES})

# Obstacle Evaluator Node
add_executable(obstacle_evaluator_node src/obstacle_evaluator_node.cpp)
ament_target_dependencies(obstacle_evaluator_node
  rclcpp sensor_msgs std_msgs pcl_conversions
)
target_link_libraries(obstacle_evaluator_node ${PCL_LIBRARIES})

# Navigation Hint Node
add_executable(navigation_hint_node src/navigation_hint_node.cpp)
ament_target_dependencies(navigation_hint_node
  rclcpp std_msgs
)

# 🔥 Haptic Driver Node (C++ 버전)
add_executable(haptic_driver_node src/haptic_driver_node.cpp)
ament_target_dependencies(haptic_driver_node
  rclcpp std_msgs
)

add_executable(deskew_node src/deskew_node.cpp)
ament_target_dependencies(deskew_node
    rclcpp
    sensor_msgs
    pcl_conversions
    tf2
    tf2_geometry_msgs
)

# obstacle_detector_node 추가
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node
    rclcpp
    sensor_msgs
    pcl_conversions
)

# 메시지와 노드 연결
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(obstacle_evaluator_node "${cpp_typesupport_target}")
target_link_libraries(navigation_hint_node "${cpp_typesupport_target}")
target_link_libraries(haptic_driver_node "${cpp_typesupport_target}")

# PointCloud Sweeper Cpp Node
add_executable(point_cloud_sweeper_cpp_node src/point_cloud_sweeper_cpp_node.cpp)
ament_target_dependencies(point_cloud_sweeper_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_sweeper_cpp_node ${PCL_LIBRARIES})

# PointCloud Accumulator Cpp Node
add_executable(point_cloud_accumulator_cpp_node src/point_cloud_accumulator_cpp_node.cpp)
ament_target_dependencies(point_cloud_accumulator_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_accumulator_cpp_node ${PCL_LIBRARIES})

# PointCloud Voxel Filter Cpp Node
add_executable(point_cloud_voxel_filter_cpp_node src/point_cloud_voxel_filter_cpp_node.cpp)
ament_target_dependencies(point_cloud_voxel_filter_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_voxel_filter_cpp_node ${PCL_LIBRARIES})

# PointCloud PassThrough Filter Cpp Node
add_executable(point_cloud_passthrough_filter_cpp_node src/point_cloud_passthrough_filter_cpp_node.cpp)
ament_target_dependencies(point_cloud_passthrough_filter_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_passthrough_filter_cpp_node ${PCL_LIBRARIES})

# PointCloud Statistical Outlier Remover Cpp Node
add_executable(point_cloud_statistical_outlier_remover_cpp_node src/point_cloud_statistical_outlier_remover_cpp_node.cpp)
ament_target_dependencies(point_cloud_statistical_outlier_remover_cpp_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_cloud_statistical_outlier_remover_cpp_node ${PCL_LIBRARIES})

# PointCloud Point Count Sweeper Cpp Node
add_executable(point_count_sweeper_node src/point_count_sweeper_node.cpp)
ament_target_dependencies(point_count_sweeper_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(point_count_sweeper_node ${PCL_LIBRARIES})

# PointCloud Z-Variance Sweeper Cpp Node
add_executable(z_variance_sweeper_node src/z_variance_sweeper_node.cpp)
ament_target_dependencies(z_variance_sweeper_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(z_variance_sweeper_node ${PCL_LIBRARIES})

# Stable Ground Detector Node
add_executable(stable_ground_detector_node src/stable_ground_detector_node.cpp)
ament_target_dependencies(stable_ground_detector_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(stable_ground_detector_node ${PCL_LIBRARIES})

# 실행 파일 설치
install(TARGETS
  icp_3d_odom_node
  pointcloud_formatter_node
  front_cloud_filter_node
  obstacle_evaluator_node
  navigation_hint_node
  haptic_driver_node  # 🔥 C++ 버전으로 변경
  deskew_node
  obstacle_detector_node
  point_cloud_sweeper_cpp_node
  point_cloud_accumulator_cpp_node
  point_cloud_voxel_filter_cpp_node
  point_cloud_passthrough_filter_cpp_node
  point_cloud_statistical_outlier_remover_cpp_node
  point_count_sweeper_node
  z_variance_sweeper_node
  stable_ground_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()