# Optical Cane: Research & Development Roadmap

This document outlines the key research questions, challenges, and future development directions identified after the initial prototyping and analysis phase. This serves as a guide for the next phase of the project, focusing on creating a robust and safe navigation system.

---

### 1. The Core Dilemma: 2D Stability vs. 3D Safety

Initial tests have shown that 2D-based pipelines (which project all data onto a flat plane) produce a more stable and less noisy path *with the current simple algorithms*. This phenomenon is described as the **"Information Paradox"**:

-   **The "Pro" of 2D Projection:** The act of setting `point.z = 0` serves as an aggressive but effective filter, eliminating all Z-axis noise from IMU jitter and user movement. This simplifies the problem, leading to higher stability with basic algorithms.
-   **The "Con" of 2D Projection (The Critical Risk):** This approach completely fails to detect **negative obstacles** (e.g., potholes, curbs, stairs). A pothole is simply perceived as empty, traversable space, posing a significant safety risk.

**Conclusion:** While 2D projection offers short-term stability, its inherent inability to detect negative obstacles makes a 3D-aware solution a **mandatory long-term requirement** for user safety. The project's goal is to develop a 3D pipeline that matches or exceeds the stability of the 2D prototypes.

---

### 2. Path Forward for an Advanced 3D Pipeline

To overcome the current instability of the 3D pipeline, a more sophisticated perception process must be implemented before path planning. The proposed order of operations is:

1.  **Validate Ground Plane Segmentation:** The highest priority is to test the existing ground segmentation code with the stable, shoulder-mounted sensor setup. This is the foundational step for all subsequent 3D perception tasks.
2.  **Implement Simplified Obstacle Clustering:**
    -   **Goal:** Instead of attempting to cluster the entire scene, focus on identifying and clustering isolated, non-wall obstacles (e.g., poles, furniture, pedestrians).
    -   **Method:** After ground removal, use a clustering algorithm (e.g., Euclidean Clustering) on the remaining points. Filter out large, continuous clusters (likely walls) to isolate smaller object clusters.
    -   **Application:** Extract the centroid of these isolated clusters and publish their coordinates as TF frames. These become dynamic avoidance points for the path planner.
3.  **Research Custom 2.5D/Height-Map for `sweep_cloud`:**
    -   **Challenge:** Standard occupancy grid algorithms may not be suitable for the time-accumulated, "smeary" nature of `/sweep_cloud_cpp`.
    -   **Goal:** Develop a modified 2.5D height map representation that can effectively handle this data type to detect negative obstacles while maintaining computational efficiency.

---

### 3. Expanding the Comparative Analysis: New Path Planning Algorithms

Before finalizing the path planning logic, other established algorithms must be implemented and benchmarked against the current ray-casting method. This is a critical step to ensure we are not prematurely optimizing a suboptimal approach.

-   **Candidate Algorithms to Implement:**
    -   Follow the Gap Method
    -   Vector Field Histogram (VFH) or a simplified variant

**Next Step:** These new algorithms should be implemented and included in the upcoming 10-run comparative test series to be performed with the shoulder-mounted setup.
