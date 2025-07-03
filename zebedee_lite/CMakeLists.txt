cmake_minimum_required(VERSION 3.8)
project(zebedee_lite)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 이 줄 추가
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(imu_preprocess_node src/imu_preprocess_node.cpp)
ament_target_dependencies(imu_preprocess_node
  rclcpp sensor_msgs geometry_msgs tf2_geometry_msgs)  # 이 줄 추가

add_executable(deskew_scan_node src/deskew_scan_node.cpp)
ament_target_dependencies(deskew_scan_node
  rclcpp sensor_msgs Eigen3 tf2 tf2_ros pcl_conversions)

add_executable(icp_odometry_node src/icp_odometry_node.cpp)
ament_target_dependencies(icp_odometry_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros
  PCL Eigen3 pcl_conversions)

target_link_libraries(icp_odometry_node
  ${PCL_LIBRARIES}
)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster
  rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS
  imu_preprocess_node
  deskew_scan_node
  icp_odometry_node
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
