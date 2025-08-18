# Gemini Session Handover: Optical Cane Project Strategic Directions

**To the next Gemini Agent:** This document contains the core strategies and technical insights for the Optical Cane project, derived from in-depth discussions with the user. **Before starting any task, you MUST read this document first**, along with `PROJECT_CONTEXT.md` and `FUTURE_DIRECTIONS.md`, to fully grasp the project's current state and the user's intent.

---

### 1. Current Project Phase: "Exploration & R&D," NOT "Final Decision"

-   **Core Insight:** We have moved beyond simply selecting one of the three initial pipelines. The current objective is a **research and development phase to find the best possible solution that guarantees safety, especially in detecting negative obstacles.** Do not rush to conclusions.
-   **User's Intent:** The upcoming **10-run, shoulder-mounted test series** will be the definitive benchmark. No pipeline should be considered a "winner" until these test results are analyzed.

---

### 2. Key Technical Insights (Based on User Feedback)

This section contains critical analysis provided by the user, which must form the basis for all future technical suggestions.

#### A. The True Value of 3D: Detecting "Negative Obstacles"
-   **Previous Misconception:** The primary benefit of 3D was thought to be detecting "overhanging obstacles."
-   **User's Correction:** The device's IMU-coupled sweeping mechanism can, in principle, detect overhanging obstacles. The **critical, non-negotiable value of 3D data** is its ability to detect **"negative obstacles"** like potholes, stairs, and curbs. This is a fatal safety flaw in any 2D projection method that cannot be overcome. Therefore, a 3D-aware solution is a **mandatory long-term requirement.**

#### B. The "Information Paradox": Why 2D is Currently More Stable
-   **User's Insight:** The reason the 2D pipeline currently appears more stable is that the `point.z = 0` operation acts as a powerful filter, eliminating all Z-axis noise (from IMU, gait vibrations, etc.).
-   **Implication:** This is a **short-term gain from simplification**, not a sign of fundamental superiority. This "Information Paradox" is the key argument for *why* we must improve the 3D pipeline's stability, rather than settling for the limited 2D approach.

#### C. Improvement Path for the 3D Pipeline (User's Specific Guidelines)
-   **Ground Plane Segmentation:** This is the **highest priority**. Its performance must be validated with the new shoulder-mounted setup.
-   **Obstacle Clustering:** Do not attempt to cluster everything. The user provided a specific, intelligent approach: **focus only on clustering isolated, non-wall obstacles (people, poles, etc.)**. The centroids of these clusters should then be published as TF frames to serve as dynamic avoidance points.
-   **Map Generation:** Standard Occupancy Grids are likely unsuitable for the "smeary" nature of the `sweep_cloud` data. Research into a **custom 2.5D height map** tailored to this data type is required.

#### D. Expansion of Path Planning Algorithm Comparison
-   **User's Directive:** We must not remain fixed on the current ray-casting method. The choice of a path planning algorithm is a critical, long-term decision. Therefore, other promising algorithms **must be implemented and benchmarked** in the upcoming 10-run test series.
-   **Algorithms for Implementation:**
    1.  **Follow the Gap**
    2.  **Vector Field Algorithm** (or a simplified variant)

---

### 3. Actionable Directives for the Next Gemini Agent

-   **DO:** Anticipate that the user's next request will likely be to implement a new path planning algorithm (e.g., "Follow the Gap") or to analyze the results of the shoulder-mounted tests. Be prepared by reviewing `FUTURE_DIRECTIONS.md` and this document.
-   **DO NOT:** Propose a final conclusion or suggest code cleanup/refactoring. We are now in a deep R&D phase.
-   **DO NOT:** Assume the `live_plotter.py` stability issue is a simple memory problem. It could be a more complex interaction between ROS GUI tools and Matplotlib. Prioritize the user's direct feedback and approach this issue cautiously if it reappears.
