# Optical Cane Project: System Knowledge Base

*Note for Project Maintainers (and Gemini): This document is the central knowledge base for the Optical Cane project. It describes the "as-is" state of the system. For the current development direction and mandatory tasks, see `DEV_MANDATES.md`.*

---

### 1. Project Summary
The project's goal is to create a real-time path guidance system for the visually impaired. It uses a 2D LiDAR and an IMU to perceive the environment, calculates a safe path, and conveys this direction to the user via a haptic interface. Through iterative development, three distinct path-planning pipelines have been created for comparative analysis.

### 2. System Architecture
The system consists of two main components communicating via micro-ROS over Wi-Fi.

-   **The Brain (Raspberry Pi 5):** The core ROS 2 processing unit. It runs the perception and path planning nodes.
-   **The Tactile Interface (Arduino Nano RP2040 Connect):** The user-facing actuator array. It subscribes to commands from the Pi and controls 8 vibration motors.

### 3. Core Perception & Path Planning Pipelines
This section describes the key nodes and how they form three different pipelines for generating a safe path.

#### 3.1. Perception Source Nodes (Choose One)

**`point_cloud_sweeper_cpp_node` (`cpp_package`)**
-   **Purpose:** To create a dense, time-accumulated 3D point cloud from real-time sensor fusion data. This is the source for the 3D pipelines.
-   **Input:** `/dense_points` (PointCloud2) from a sensor fusion source.
-   **Output:** `/sweep_cloud_cpp` (PointCloud2).
-   **Logic:** Aggregates point cloud frames from the last 1.5 seconds into a single, denser cloud.

**`scan_accumulator_node.py` (`optical_cane_rpi`)**
-   **Purpose:** To create a time-accumulated 2D point cloud. This is the source for the pure 2D pipeline.
-   **Input:** `/scan` (LaserScan).
-   **Output:** `/scan_accumulation_cloud` (PointCloud2).
-   **Logic:** Accumulates `LaserScan` messages over a 1.5-second window and merges them into a single `PointCloud2` message.
-   **Key Parameter:** `front_view_only` (bool, default: true) filters out points behind the sensor.

---

#### 3.2. Intermediate Filtering Node

**`voxel_grid_filter_node.cpp` (`cpp_package`)**
-   **Purpose:** To reduce the density of the large `/sweep_cloud_cpp` to improve performance for the 3D path planner and RViz visualization.
-   **Input:** `/sweep_cloud_cpp` (PointCloud2).
-   **Output:** `/downsampled_cloud` (PointCloud2).
-   **Logic:** Applies two filters in sequence: 1) A PassThrough filter to remove points behind the sensor, and 2) A VoxelGrid filter to downsample the cloud.
-   **Key Parameters:**
    -   `front_view_only` (bool, default: true): Controls the PassThrough filter.
    -   `leaf_size` (double, default: 0.1): Sets the voxel size for downsampling.

---

#### 3.3. Path Planning Nodes (The Three Pipelines)

**Pipeline 1: 2D Projection using `path_planner_node`**
-   **Purpose:** Calculates a safe path by projecting a 3D point cloud onto a 2D plane.
-   **Input:** `/sweep_cloud_cpp` (PointCloud2).
-   **Logic:**
    1.  Optionally applies a VoxelGrid filter for performance.
    2.  **Projects all points onto a 2D plane** by setting their Z-coordinate to 0.
    3.  Performs 2D ray-casting to find the safest direction.
-   **Key Parameter:** `use_voxel_filter` (bool, default: true) can be set to `false` to use the raw, non-downsampled data.

**Pipeline 2: Pure 2D Accumulated using `path_planner_node`**
-   **Purpose:** Calculates a safe path using purely 2D accumulated data.
-   **Input:** `/scan_accumulation_cloud` (PointCloud2) (via topic remapping).
-   **Logic:** Same as Pipeline 1, but uses the output of `scan_accumulator_node` as its input, providing a pure 2D comparison.

**Pipeline 3: 3D Corridor Scan using `path_planner_3d_node`**
-   **Purpose:** Calculates a safe path directly from 3D data, without lossy 2D projection. Designed to be robust to sensor tilt.
-   **Input:** `/downsampled_cloud` (PointCloud2).
-   **Logic:**
    1.  Casts 15 virtual rays, each representing a 3D "corridor" with a defined width.
    2.  The depth for each corridor is determined by the closest point found *within that 3D corridor*, detecting obstacles at any height.
    3.  A bug was fixed where the path defaulted to the right; it now correctly goes straight when no obstacles are present.

### 4. Haptic Interface Implementation
-   **Hardware:** Arduino Nano RP2040 Connect controlling 8 PWM vibration motors via a ULN2803 driver array.
-   **Identified Problem:** The rigid bracelet design suffers from significant vibration crosstalk, making it difficult for a user to reliably distinguish which motor is active. This human-factors issue is unresolved and work is **ON HOLD**.

### 5. The Turning Point & New Development Mandate
Following a critical review meeting, the project's philosophy shifted from "build it" to "prove it". **All future work must be guided by the principles and mandatory validation tasks outlined in `DEV_MANDATES.md`.**
