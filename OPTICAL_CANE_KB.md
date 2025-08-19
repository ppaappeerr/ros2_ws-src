# Optical Cane Project: Comprehensive Knowledge Base

---

## 1. Project Overview

### 1.1. Goal
To create a real-time path guidance system for the visually impaired. The system uses a 2D LiDAR and an IMU to perceive the environment, calculates a safe path, and conveys this direction to the user via a haptic interface.

### 1.2. Current Phase: Exploration & R&D
The project is in a rigorous **research and development phase**. The immediate objective is a data-driven comparative analysis of three distinct path-planning pipelines to find the most robust and safe solution. The key challenge is ensuring the detection of **negative obstacles** (e.g., potholes, stairs), which mandates a 3D-aware solution in the long term.

---

## 2. System Architecture

### 2.1. Hardware
-   **The Brain (Raspberry Pi 5):** The core ROS 2 processing unit.
-   **The Tactile Interface (Arduino Nano RP2040 Connect):** Controls 8 vibration motors, communicating with the Pi via micro-ROS.

### 2.2. Software & Coordinate Frames (TF)
-   **Framework:** ROS 2
-   **Communication:** Standard ROS 2 topics; micro-ROS for the haptic interface.
-   **Key Coordinate Frames:**
    -   `base_link`: The primary coordinate frame of the device.
    -   `imu_link`: Represents the IMU's position, rigidly attached to `base_link`.
    -   `laser`: Represents the LiDAR's position, rigidly attached to `base_link`.

---

## 3. Operational Workflow: `ros2 run` & Aliases

This section details the step-by-step process for running the system using the user's `ros2 run` commands and custom aliases.

### 3.1. Core System Startup (Prerequisites for all Pipelines)

These commands must be run in separate terminals to start the basic sensor processing.

1.  **Start Micro-ROS Agent (Optional, for Haptics)**
    -   **Alias:** `a0`
    -   **Command:** `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
    -   **Purpose:** Bridges the gap between the main ROS 2 network and the microcontroller-based haptic interface.

2.  **Launch Base Sensors (LiDAR & IMU)**
    -   **Alias:** `a1`
    -   **Command:** `ros2 launch optical_cane_rpi optical_cane.launch.py`
    -   **Purpose:** Starts the hardware drivers for the LiDAR (`sllidar_node`) and IMU (`mpu9250_driver_node`), and publishes the static TF tree (`base_link` -> `laser`, `base_link` -> `imu_link`). This provides the raw sensor data.

3.  **Run Sensor Fusion**
    -   **Alias:** `a2`
    -   **Command:** `ros2 run optical_cane_rpi sensor_fusion_node`
    -   **Purpose:** This crucial node synchronizes raw `/scan` (LiDAR) and `/imu/data` (IMU) topics. It uses the IMU's orientation to project the 2D LiDAR scan into a 3D point cloud, publishing the result as `/dense_points`. **This is the foundational input for all 3D pipelines.**

### 3.2. Pipeline A: 3D to 2D Projection

This pipeline creates a 3D point cloud, then flattens it for 2D path planning.

4.  **Run Point Cloud Sweeper**
    -   **Alias:** `a3`
    -   **Command:** `ros2 run cpp_package point_cloud_sweeper_cpp_node`
    -   **Purpose:** Subscribes to `/dense_points` and accumulates them over a 1.5-second window into a single, denser point cloud named `/sweep_cloud_cpp`.

5.  **Run 2D Path Planner**
    -   **Alias:** `a4`
    -   **Command:** `ros2 run cpp_package path_planner_node`
    -   **Purpose:** Takes the `/sweep_cloud_cpp`, projects all points onto a 2D plane (Z=0), and calculates the safest path, publishing it as `/safe_path_vector`.

### 3.3. Pipeline B: Pure 2D Accumulated

This pipeline operates purely in 2D as a baseline for comparison. **(Do not run `a2` or `a3` for this pipeline)**.

4.  **Run 2D Scan Accumulator**
    -   **Alias:** `b3`
    -   **Command:** `ros2 run optical_cane_rpi scan_accumulator_node`
    -   **Purpose:** Subscribes to the raw `/scan` topic and accumulates laser scans over a 1.5-second window, publishing the result as a 2D point cloud named `/scan_accumulation_cloud`.

5.  **Run 2D Path Planner (with Remapping)**
    -   **Alias:** `b4`
    -   **Command:** `ros2 run cpp_package path_planner_node --ros-args -r /sweep_cloud_cpp:=/scan_accumulation_cloud`
    -   **Purpose:** The same planner as Pipeline A, but the `--ros-args -r` command remaps its input topic. It now listens to `/scan_accumulation_cloud` instead of `/sweep_cloud_cpp`.

### 3.4. Pipeline C: Full 3D Corridor Scan

This pipeline performs path planning directly in 3D space. **(Run `a2` and `a3` before this)**.

4.  **Run Voxel Grid Filter**
    -   **Alias:** `c4`
    -   **Command:** `ros2 run cpp_package voxel_grid_filter_node --ros-args -p leaf_size:=0.04`
    -   **Purpose:** Subscribes to `/sweep_cloud_cpp` and downsamples it for performance using a voxel grid filter. It publishes the smaller cloud to `/downsampled_cloud`. The `leaf_size` is set to 0.04m.

5.  **Run 3D Path Planner**
    -   **Alias:** `c5`
    -   **Command:** `ros2 run cpp_package path_planner_3d_node`
    -   **Purpose:** Subscribes to `/downsampled_cloud` and calculates a safe path by casting 3D "corridors" instead of 2D rays. This method is designed to be robust to sensor tilt. Publishes to `/safe_path_vector_3d`.

---

## 4. Node-by-Node Deep Dive

### 4.1. Sensor & Fusion Layer
-   **`sllidar_node` (`sllidar_ros2`):** Publishes raw 2D laser scans (`/scan`).
-   **`mpu9250_driver_node` & `mpu9250_filtered` (`mpu9250`):** The driver publishes raw IMU/Mag data (`/imu/data_raw`, `/imu/mag_raw`). The `filtered` node applies calibration and a Madgwick filter to publish clean, fused orientation data (`/imu/data`).
-   **`sensor_fusion_node` (`optical_cane_rpi`):**
    -   **Inputs:** `/scan`, `/imu/data`
    -   **Output:** `/dense_points` (PointCloud2)
    -   **Logic:** Rotates each point from the 2D laser scan into 3D space using the fused orientation from the IMU.

### 4.2. Perception Layer
-   **`point_cloud_sweeper_cpp_node` (`cpp_package`):**
    -   **Input:** `/dense_points`
    -   **Output:** `/sweep_cloud_cpp` (PointCloud2)
    -   **Logic:** Accumulates incoming point clouds in a 1.5-second sliding window to create a denser representation of the environment.
-   **`scan_accumulator_node.py` (`optical_cane_rpi`):**
    -   **Input:** `/scan`
    -   **Output:** `/scan_accumulation_cloud` (PointCloud2)
    -   **Logic:** Accumulates raw 2D laser scans in a 1.5-second window.
-   **`voxel_grid_filter_node.cpp` (`cpp_package`):**
    -   **Input:** `/sweep_cloud_cpp`
    -   **Output:** `/downsampled_cloud` (PointCloud2)
    -   **Logic:** Reduces point cloud density for faster processing by averaging points within 3D cubes (voxels).

### 4.3. Path Planning Layer
-   **`path_planner_node` (`cpp_package`):**
    -   **Input:** `/sweep_cloud_cpp` (or remapped topic)
    -   **Outputs:** `/safe_path_vector` (Vector3Stamped), `/candidate_rays` (MarkerArray)
    -   **Logic:** Flattens the input cloud to 2D (`point.z = 0`). Casts 15 2D rays, scores them based on obstacle distance, and selects the best direction. Applies smoothing to the final output vector.
-   **`path_planner_3d_node` (`cpp_package`):**
    -   **Input:** `/downsampled_cloud`
    -   **Outputs:** `/safe_path_vector_3d` (Vector3Stamped), `/candidate_rays_3d` (MarkerArray)
    -   **Logic:** Casts 15 virtual 3D "corridors" of a defined width. Finds the nearest obstacle within each corridor, scores them, and selects the best path. This approach is sensitive to obstacles at any height, not just on the ground plane.

---

## 5. Strategic Direction & Future R&D

### 5.1. The Core Dilemma: 2D Stability vs. 3D Safety ("Information Paradox")
-   **The Paradox:** 2D projection (`point.z = 0`) acts as an aggressive filter, eliminating Z-axis noise and creating a more stable path with simple algorithms.
-   **The Critical Risk:** This method is fundamentally blind to **negative obstacles** (potholes, curbs), posing a major safety risk.
-   **Conclusion:** A 3D-aware solution is a **mandatory long-term requirement**. The primary R&D goal is to develop a 3D pipeline that is as stable as, or more stable than, the 2D prototypes.

### 5.2. Path Forward for an Advanced 3D Pipeline
1.  **Validate Ground Plane Segmentation:** Highest priority.
2.  **Implement Simplified Obstacle Clustering:** After ground removal, identify isolated, non-wall obstacles.
3.  **Research Custom 2.5D/Height-Map:** A modified height map is needed to handle the `sweep_cloud` data and reliably detect negative obstacles.

### 5.3. Expanding Path Planning Algorithm Comparison
-   **Candidate Algorithms:** Follow the Gap Method, Vector Field Histogram (VFH).

---

## 6. Data Analysis & Validation

### 6.1. Data Recording (Aliases)
-   **`d1` (Pipeline A):** `ros2 bag record -o test_1_p1 /imu/data /safe_path_vector /tf /tf_static /sweep_cloud_cpp`
-   **`d2` (Pipeline B):** `ros2 bag record -o test_1_p2 /imu/data /safe_path_vector /tf /tf_static /scan_accumulation_cloud`
-   **`d3` (Pipeline C):** `ros2 bag record -o test_1_p3 /imu/data /safe_path_vector_3d /tf /tf_static /downsampled_cloud`

### 6.2. Post-Hoc Analysis (`plot_analysis.py`)
-   **Alias:** `d0`
-   **Command:** `python3 plot_analysis.py --test_set 1`
-   **Purpose:** Processes the recorded bags to generate quantitative metrics and plots for Path Stability, Smoothness (Angular Velocity RMS, PSD), and Tilt Robustness.