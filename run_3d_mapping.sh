#!/bin/bash

# 1. 모든 ROS 노드 종료
echo "기존 ROS 노드 종료..."
pkill -f ros2
sleep 1

# 2. 센서와 고정 TF만 시작
echo "센서 및 기본 TF 시작..."
ros2 launch sllidar_ros2 sensors_only.launch.py &
sleep 3  # TF 트리가 설정될 시간 확보

# 3. IMU TF 발행자 시작 (동적 TF - map->base_link)
echo "IMU TF 발행자 시작..."
ros2 run lidar_imu_fusion imu_tf_publisher &
sleep 2

# 4. 스캔→포인트클라우드 변환
echo "스캔→포인트클라우드 변환 시작..."
ros2 run scan_to_pointcloud scan_to_pointcloud --ros-args -p output_topic:=pc_3d &
sleep 1

# 5. 포인트클라우드 누적
echo "포인트클라우드 누적기 시작..."
ros2 run scan_to_pointcloud accumulated_pointcloud --ros-args -p grid_size:=0.05 -p point_skip:=2 &

# 종료 시 모든 프로세스 정리
trap 'pkill -f ros2; echo "시스템 종료"; exit' SIGINT

echo "모든 노드가 시작되었습니다. Ctrl+C로 종료하세요."
wait