Note for Project Maintainers (and Gemini):
This document is the central knowledge base for the Optical Cane project. It serves as a persistent memory for the Gemini AI assistant to ensure context is maintained across sessions.

IMPORTANT: After any significant changes to the architecture, data pipeline, or core logic, please update this file accordingly. This keeps the AI's understanding aligned with the project's current state.

Project Summary: Optical Cane (for the Visually Impaired)
1. The Evolved Goal: From Obstacle Avoidance to Real-time Path Guidance
The project's objective has evolved. The initial goal was to detect discrete obstacles and provide simple avoidance cues. The new, more ambitious goal is to utilize the entire surrounding point cloud to function as a real-time path planning system. The system must calculate the most viable path through a complex environment on the fly and convey this directional guidance to the user through an intuitive, multi-point haptic interface.

2. System Architecture
The system consists of two main components communicating wirelessly via Wi-Fi using the micro-ROS framework.

The Brain (Raspberry Pi 5): The core ROS 2 processing unit.

Sensing: Collects data from a 2D LiDAR (SLLIDAR A2M8) and an IMU (MPU9250).

Perception & Planning: Fuses sensor data, analyzes the environment, and computes a safe path forward.

The Tactile Interface (Arduino Nano RP2040 Connect): The user-facing actuator array.

Receive Commands: Listens for path guidance commands from the Raspberry Pi via micro-ROS.

Provide Feedback: Translates commands into directional vibration patterns using an array of 8 PWM-controlled motors.

3. Current Data Pipeline (Under Review)
The following pipeline represents the project's current implementation. However, due to the shift in goals and newly identified challenges, this entire pipeline is subject to review and significant refactoring.

Sensor Fusion: LiDAR (/scan) + IMU (/imu/data) -> 3D Point Cloud (/dense_points).

Point Cloud Sweeping: Aggregates frames into a denser cloud (/sweep_cloud_cpp).

Stable Ground Fitting: Locks an initial ground plane to separate ground (/stable_ground_points) from non-ground (/obstacle_points).

ROI Analysis: Analyzes obstacles within a 3x3x3 grid centered on the user.

Command Generation: Converts analysis into a string command (/haptic_command).

Bridge to MCU: A Python node (esp32_bridge_node.py) converts the string command to an integer ID (/micro_ros_haptic_command).

4. New Core Challenges & Direction for Refactoring
This section outlines the critical problems that have been identified. The current codebase must be evolved to address these challenges. The solutions are not yet defined and require further discussion and development.

4.1. The 'Moving Observer' Problem (Top Priority)
The Issue: The system lacks odometry. When the user moves, static objects (like walls) appear as dynamic, approaching obstacles to the current perception pipeline (sweep_cloud logic). This fundamentally undermines reliable environment analysis.

The Question: Is the frame-accumulating sweep_cloud approach fundamentally flawed for a moving user? Or can it be adapted? Should we instead develop a new perception logic that works directly with the instantaneous dense_points?

Next Step: This is the most critical issue to be investigated and discussed. The viability of the entire project hinges on solving this.

4.2. From Discrete Detection to Vector-Based Path Planning
The Issue: The current 3x3x3 ROI grid is designed for simple "obstacle here" detection. It cannot generate a nuanced "safe path to move along this vector" command.

The Question: What algorithm or approach can we use to process a scattered cloud of obstacle points and derive a single, safe directional vector for user guidance?

Next Step: We need to research and discuss various path planning strategies suitable for unstructured point cloud data in real-time.

4.3. Developing an Intuitive, Multi-Point Haptic Language
The Issue: The hardware has been upgraded to an 8-motor array to provide richer directional information. The old 2-motor system and its command table are now obsolete.

The Question: How can we map a directional vector (from 4.2) into a vibration pattern across 8 motors that feels intuitive to the user? (e.g., a "flow" of vibrations indicating a direction, pulsing intensity indicating urgency).

Next Step: Once a path planning strategy is chosen, we will need to design and prototype a new haptic command mapping.

5. The Tactile Interface: Arduino Nano RP2040 Details
Hardware:

MCU: Arduino Nano RP2040 Connect (officially supports micro-ROS).

Actuators: 8x PWM-controlled vibration motors. The DRV2605L drivers are no longer used.

Communication:

Protocol: micro-ROS

Subscribed Topic: /micro_ros_haptic_command (Type: std_msgs/msg/Int32)

Haptic Command Table: (Deprecated) The previous 2-motor command table is no longer valid. A new, more complex mapping for the 8-motor array must be designed in conjunction with the path planning algorithm.

---
### **6. Path Planning Algorithm Implementation (`path_planner_node`)**

In response to Challenge 4.2, the primitive 3x3 grid analysis has been replaced with a more sophisticated vector-based path planning algorithm within `cpp_package/src/path_planner_node.cpp`.

-   **Core Logic:**
    1.  **Ray-Casting:** The node casts 15 virtual rays across a 180-degree field of view.
    2.  **Depth Scoring:** It calculates the distance to the nearest obstacle for each ray and assigns a score based on this depth, also considering the depths of adjacent rays to favor wider paths.
    3.  **Ideal Vector Selection:** The ray with the highest score is chosen as the "ideal" raw direction for that frame.
    4.  **Temporal Smoothing:** To prevent erratic and sudden directional changes, a smoothing mechanism was introduced. It limits the rate of change (angular velocity) of the output vector, ensuring the final guidance direction (`smoothed_angle`) changes fluidly, even if the ideal raw direction shifts abruptly.
-   **New Published Topics:**
    -   `/safe_path_vector` (`geometry_msgs/msg/Vector3Stamped`): Publishes the final, smoothed directional vector. This is the topic the haptic controller will subscribe to.
    -   `/candidate_rays` (`visualization_msgs/msg/MarkerArray`): A rich debugging topic for RViz. It visualizes all candidate rays, the raw ideal vector (yellow arrow), and the final smoothed vector (blue arrow), allowing for intuitive algorithm tuning.

### **7. Haptic Controller Implementation & WiFi Connectivity Block**

The focus shifted to implementing the 8-motor haptic controller on the Arduino Nano RP2040 Connect to subscribe to the new `/safe_path_vector` topic.

-   **Haptic Logic:** An 8-motor interpolation algorithm was developed. It maps the incoming angle to the motor array, activating the two nearest motors with blended intensity to create a smooth, analog-like directional sensation.
-   **Development Environment:** PlatformIO was chosen to manage the complex dependencies for the RP2040 board.
-   **Critical Challenge: WiFi Connectivity Failure:** A major, unresolved roadblock was encountered. The `micro-ros-arduino` library, in its current state, exhibits a fundamental incompatibility with the Mbed OS WiFi stack used by the Arduino Nano RP2040 Connect within the PlatformIO build environment.
    -   **Problem Summary:** Multiple attempts to compile the code with WiFi enabled resulted in a cascade of errors, stemming from the micro-ROS library attempting to use incorrect or outdated WiFi APIs.
    -   **Failed Mitigation Attempts:** The issue persisted despite numerous strategies, including modifying `platformio.ini` build flags, specifying library versions, and attempting to manually patch the library source code by including it directly in the project's `lib` folder.
-   **Conclusion:** WiFi-based communication for the haptic controller is **currently blocked**. The path forward requires a solution to this deep-seated library and toolchain incompatibility. The project's goal remains to use WiFi.