cmake_minimum_required(VERSION 3.8)
project(lidar_imu_cartographer)

# 기본 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 기본 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Cartographer 관련 의존성 찾기
find_package(cartographer REQUIRED)
find_package(cartographer_ros REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)

# 설치 디렉토리 설정
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# RViz 설정 파일 설치 (디렉토리가 존재하는 경우에만)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(DIRECTORY
    rviz
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Python 모듈 설치 부분 제거 (이 패키지에는 Python 모듈이 필요 없음)
# ament_python_install_package(${PROJECT_NAME}) <- 이 부분 제거

# 실행 파일 설치 부분 제거 또는 수정 (필요없는 경우)
# install(PROGRAMS
#   scripts/custom_cartographer_script.py 
#   DESTINATION lib/${PROJECT_NAME}
# )

# 테스트 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 내보내기
ament_package()