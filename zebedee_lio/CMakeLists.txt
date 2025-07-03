cmake_minimum_required(VERSION 3.8)
project(zebedee_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# ikd-Tree include 경로 설정 (올바른 경로)
set(ikd_tree_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ikd-Tree)
if(EXISTS ${ikd_tree_DIR})
  # ikd-Tree 소스 파일 직접 추가
  add_library(ikd_tree ${ikd_tree_DIR}/ikd-Tree/ikd_Tree.cpp)
  target_include_directories(ikd_tree PUBLIC 
    ${ikd_tree_DIR}
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(ikd_tree
    ${PCL_LIBRARIES}
    Eigen3::Eigen
  )
  # PCL 정의 추가
  target_compile_definitions(ikd_tree PRIVATE ${PCL_DEFINITIONS})
else()
  message(FATAL_ERROR "ikd-Tree not found! Please run: git submodule add https://github.com/hku-mars/ikd-Tree.git")
endif()

# Submap Manager Library
add_library(submap_manager src/submap_manager.cpp)
target_include_directories(submap_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ikd_tree_DIR}>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(submap_manager
  ikd_tree
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(submap_manager
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "pcl_conversions"
)

# =============================================================
# == LIO Frontend Node
# =============================================================
add_executable(lio_frontend_node src/lio_frontend_node.cpp)
ament_target_dependencies(lio_frontend_node
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "tf2_ros"
  "pcl_conversions"
)

target_link_libraries(lio_frontend_node
  submap_manager
)

install(TARGETS
  lio_frontend_node
  DESTINATION lib/${PROJECT_NAME}
)
# =============================================================

ament_package()