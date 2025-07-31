> **Note for Project Maintainers (and Gemini):**
> This document is the central knowledge base for the Optical Cane project. It serves as a persistent memory for the Gemini AI assistant to ensure context is maintained across sessions.
>
> **IMPORTANT:** After any significant changes to the architecture, data pipeline, or core logic, please update this file accordingly. This keeps the AI's understanding aligned with the project's current state.

---

## Project Summary: Optical Cane (for the Visually Impaired)

### 1. The Goal

The "Optical Cane" is a wearable assistive device. Its goal is to use a 2D LiDAR and an IMU to perceive the 3D environment in real-time, to distinguish between safe ground and hazards, and to convey this information to the user through clear, intuitive haptic (vibration) feedback.

### 2. System Architecture

The system has two main components:

1.  **The Brain (Raspberry Pi 5):** The core processing unit running ROS 2.
    *   **Sensing:** Collects data from a 2D LiDAR (SLLIDAR A2M8) and an IMU (MPU9250).
    *   **Perception:** Fuses sensor data into a 3D point cloud.
    *   **Decision-Making:** Analyzes the point cloud to identify the ground, obstacles, and stairs.

2.  **The Hands (ESP32 Haptic Module):** The user-facing actuator.
    *   **Receive Commands:** Listens for instructions from the Raspberry Pi.
    *   **Provide Feedback:** Translates commands into specific vibration patterns.

The two components communicate wirelessly via **Wi-Fi**, using the **micro-ROS** framework.

### 3. The Brain's Data Pipeline

1.  **Sensor Fusion:** LiDAR (`/scan`) and IMU (`/imu/data`) data are combined into a 3D point cloud (`/dense_points`).
2.  **Point Cloud Sweeping:** Multiple frames are aggregated to create a denser cloud (`/sweep_cloud_cpp`) for better analysis.
3.  **Stable Ground Fitting:** An initial ground plane is found and **locked**. Subsequent points are compared against this locked model to stably separate ground (`/stable_ground_points`) from non-ground (`/obstacle_points`), preventing flickering.
4.  **User-Centric ROI Analysis:** Obstacles are analyzed within a **3x3x3 grid (ROI Tensor)** centered on the user to determine their precise location and type.
5.  **Command Generation:** The analysis result is converted into a human-readable string command (e.g., "STOP", "FRONT_STEP_UP") on the `/haptic_command` topic.
6.  **Command ID Conversion:** A final node converts the string command into an integer ID (e.g., "STOP" -> `0`) and publishes it to `/micro_ros_haptic_command` for the ESP32.

### 4. The Hands' Role: ESP32 Haptic Module Details

The ESP32's sole job is to receive the integer ID and act.

*   **Hardware:**
    *   **MCU:** ESP32 (LOLIN D32)
    *   **Haptic Drivers:** 2x Adafruit DRV2605L
    *   **Connection:** **Dual I2C bus** configuration.
        *   Left Motor (`drv1`): `Wire` (GPIO 21/SDA, 22/SCL)
        *   Right Motor (`drv2`): `Wire1` (GPIO 25/SDA, 26/SCL)

*   **Communication:**
    *   **Protocol:** micro-ROS
    *   **Subscribed Topic:** `/micro_ros_haptic_command` (Type: `std_msgs/msg/Int32`)

*   **Haptic Command Table:**

| ID | Meaning | Target Motor(s) | Effect ID | Effect Name | Rationale |
|:---|:---|:---|:---|:---|:---|
| **0** | **STOP** | Both | 47 | Buzz 1 (100%) | Urgent, unmistakable stop signal. |
| **1** | Front Obstacle (Low) | Both | 1 | Strong click (100%) | Sharp, quick tap for a low obstacle. |
| **2** | Front Obstacle (Mid) | Both | 14 | Strong buzz (100%) | Sustained warning for a larger obstacle. |
| **3** | Front Step/Stair (Up) | Both | 86 | Ramp Up Short Smooth 1 | Ascending feeling, indicating "step up". |
| **11**| Left Obstacle (Low) | Left Only | 1 | Strong click (100%) | Directional cue for a low obstacle. |
| **12**| Left Obstacle (Mid) | Left Only | 14 | Strong buzz (100%) | Directional cue for a larger obstacle. |
| **13**| Left Step/Stair (Up) | Left Only | 86 | Ramp Up Short Smooth 1 | Directional cue to "step up". |
| **21**| Right Obstacle (Low) | Right Only | 1 | Strong click (100%) | Directional cue for a low obstacle. |
| **22**| Right Obstacle (Mid) | Right Only | 14 | Strong buzz (100%) | Directional cue for a larger obstacle. |
| **23**| Right Step/Stair (Up)| Right Only | 86 | Ramp Up Short Smooth 1 | Directional cue to "step up". |

### 5. Key Development Goals

*   **Refine Haptics:** Experiment with the 123 available effects to create the most intuitive feedback.
*   **Improve Code Quality:** Use `enum` for Command IDs to increase readability.
*   **Enhance Connection Stability:** Implement robust error handling and reconnection logic for Wi-Fi and micro-ROS.

---

## Gemini's Language Preference

**Instruction:** Always respond in Korean, regardless of the language the user asks in.