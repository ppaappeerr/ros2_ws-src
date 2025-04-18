#!/bin/bash

# 노드 실행 간격 (초)
DELAY=2

echo "Starting sensors and TF tree..."
ros2 launch sllidar_ros2 combined_sensors_launch.py &
sleep $DELAY

echo "Starting IMU TF publisher..."
ros2 run lidar_imu_fusion imu_tf_publisher &
sleep $DELAY

echo "Starting scan to pointcloud converter..."
ros2 run scan_to_pointcloud scan_to_pointcloud_node &
sleep $DELAY

echo "Starting accumulated pointcloud..."
ros2 run scan_to_pointcloud accumulated_pointcloud &

echo "All nodes started. Press Ctrl+C to terminate."
wait