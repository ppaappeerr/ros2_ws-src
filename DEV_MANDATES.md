# Optical Cane Project: Development Mandates & Action Plan

*Note for Project Maintainers (and Gemini): This document contains the official development philosophy and mandatory tasks. All new work must adhere to these guidelines. Refer to `PROJECT_CONTEXT.md` for the as-is system description.*

---

### 1. Development Philosophy & Mandates (The New Rules)

The project's development approach must shift from rapid prototyping to **rigorous, evidence-based engineering**. All future work must adhere to the following principles:

1.  **Hypothesize First, Then Code:** Before implementing a feature, state a clear, testable hypothesis.
2.  **Justify Every Method:** Do not assume a technical choice is obvious. Every algorithm must be explained with a clear methodology.
3.  **Validate with Data, Not Feelings:** "It seems to work" is no longer an acceptable validation. All claims must be backed by **quantitative data**, comparative analysis, and clear visualizations.
4.  **Isolate and Verify:** Do not move to the next stage of the pipeline until the current stage has been thoroughly validated.

---

### 2. Primary Objective: Comparative Pipeline Analysis

The prototyping phase is complete. We have successfully implemented three distinct path-planning pipelines as documented in `PROJECT_CONTEXT.md`. The project's sole immediate priority is to conduct a rigorous, data-driven comparison to determine which pipeline is the most effective.

-   **Governing Hypothesis:** Which of the three implemented pathfinding pipelines (1. 2D Projection, 2. Pure 2D Accumulated, 3. 3D Corridor Scan) provides the most reliable, stable, and accurate navigational guidance, particularly in scenarios involving sensor tilt?

---

### 3. Mandatory Action Plan

To answer the governing hypothesis, the following validation process must be executed.

#### 3.1. Step 1: Establish Test Scenarios
Define a consistent and repeatable set of test environments and actions. This should include, at a minimum:
-   A straight corridor
-   A 90-degree corner turn
-   A static obstacle avoidance course
-   A dynamic test involving deliberate up-and-down tilting of the sensor while navigating.

#### 3.2. Step 2: Data Acquisition
For each of the three pipelines, and for every test scenario defined in Step 1, use `ros2 bag record` to log the relevant data. The recorded topics must include:
-   Sensor inputs (`/scan`, `/imu/data`, etc.)
-   Intermediate point clouds (`/sweep_cloud_cpp`, `/scan_accumulation_cloud`, `/downsampled_cloud`)
-   Final path planning outputs (`/safe_path_vector`, `/candidate_rays`, etc.)
-   Transforms (`/tf`, `/tf_static`)

#### 3.3. Step 3: Offline Analysis & Metrics
Develop a Python script to process the rosbags from Step 2. This script must:
-   **Generate Visualizations:** Create side-by-side video comparisons of the input point cloud and the resulting path vector for each pipeline in a given scenario.
-   **Calculate Quantitative Metrics:** For each pipeline, compute objective measures of performance, such as:
    -   **Path Stability:** The variance or standard deviation of the output `smoothed_angle` over time in a stable environment.
    -   **Path Accuracy:** The Mean Absolute Error between the calculated path and a pre-defined "ground truth" path for that scenario.
    -   **Responsiveness:** How quickly the path vector reacts to new obstacles.

#### 3.4. Step 4: Conclusive Report
Based on the evidence gathered in Step 3, produce a summary that objectively declares a "winning" pipeline or provides a nuanced conclusion about the pros and cons of each approach. This decision will dictate the future direction of the project's path-planning algorithm.

---

### 4. Haptic Interface Status: ON HOLD
-   **Reason:** The core path-planning pipeline's reliability must be proven with data (as per the action plan above) before we can meaningfully test its output with a human user.
-   **Status:** All work on the haptic interface remains **paused** until the comparative analysis is complete and a final algorithm is chosen.
