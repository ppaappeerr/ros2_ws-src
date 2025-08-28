# 광학 지팡이 프로젝트: 종합 지식 베이스

**사용자와의 대화는 모두 한국어로 진행합니다. 이 문서는 Gemini CLI의 이해를 돕기 위해 영어로 작성되었지만, 기본 대화는 한국어로 수행됩니다.**

---

## 1. 프로젝트 개요

### 1.1. 목표
시각 장애인을 위한 실시간 경로 안내 시스템을 개발하는 것입니다. 이 시스템은 2D LiDAR와 IMU를 사용하여 환경을 인식하고, 안전한 경로를 계산하며, 이 방향을 햅틱 인터페이스를 통해 사용자에게 전달하도록 설계되었습니다.

### 1.2. 현재 단계: 엄격한 R&D 및 비교 분석
프로젝트는 초기 프로토타이핑을 넘어 엄격한 **연구 개발 단계**로 진입했습니다. 즉각적인 목표는 세 가지 서로 다른 경로 계획 파이프라인의 데이터 기반 비교 분석을 통해 가장 견고하고 안전한 솔루션을 결정하는 것입니다. 이 분석은 "만들어보자"에서 "증명하자"로의 개발 철학 전환에 의해 의무화되었으며, 정량적 데이터를 요구합니다.

### 1.3. 핵심 딜레마: 2D 안정성 vs 3D 안전성 ("정보 역설")
프로젝트의 현재 방향을 정의하는 중심적인 과제:

-   **2D 투영의 "장점":** 모든 3D 데이터를 평면 2D 평면으로 투영하는 파이프라인(`point.z = 0`)이 현재 더 **안정적**입니다. 이 작업은 공격적이지만 효과적인 필터 역할을 하여 IMU 진동과 사용자 움직임으로부터 모든 Z축 노이즈를 제거합니다. 이는 경로 계획 문제를 단순화하여 기본 알고리즘으로 더 높은 안정성을 달성합니다.
-   **2D 투영의 "단점" (치명적 위험):** 이 접근법은 **음의 장애물**(예: 구덩이, 연석, 계단)에 대해 근본적으로 맹목적입니다. 시스템은 구덩이를 비어있고 통과 가능한 공간으로 인식하여 중대하고 용인할 수 없는 안전 위험을 초래합니다.

**결론:** 2D 투영이 단기적 안정성을 제공하지만, 음의 장애물을 감지할 수 없는 본질적인 한계로 인해 **3D 인식 솔루션이 사용자 안전을 위한 필수적인 장기 요구사항**이 됩니다. 주요 R&D 목표는 2D 프로토타입의 안정성과 일치하거나 이를 초과하는 3D 파이프라인을 개발하는 것입니다.

---

## 2. 시스템 아키텍처

### 2.1. 하드웨어
-   **두뇌 (Raspberry Pi 5):** 인식 및 경로 계획 노드를 실행하는 핵심 ROS 2 처리 유닛입니다. (센서와 분리된 별도 유닛)
-   **LiDAR 센서 (RPLiDAR S3):** 2D 레이저 스캔 데이터를 제공하는 주요 인식 센서입니다.
-   **IMU 센서 (MPU9250):** LiDAR 바로 위에 장착되어 방향 정보를 제공합니다. (LiDAR에서 약 2cm 상부에 위치)
-   **촉각 인터페이스 (Arduino Nano RP2040 Connect):** 8개의 진동 모터를 제어합니다. micro-ROS를 통해 Pi와 통신하며, 경로 명령을 구독합니다. **(참고: 햅틱 인터페이스 개발은 현재 보류 상태)**

### 2.1.1. 하드웨어 구성 변경사항
최근 하드웨어 구성에 중요한 변경사항이 있었습니다:
1.  **센서와 SBC 분리:** Raspberry Pi 5는 이제 센서 유닛과 물리적으로 분리되어 있습니다.
2.  **LiDAR 업그레이드:** RPLiDAR A2M8에서 RPLiDAR S3로 변경하여 향상된 성능과 안정성을 확보했습니다.
3.  **IMU 재배치:** IMU가 LiDAR 바로 위 2cm에 근접 장착되어 센서 간 정렬이 크게 개선되었습니다 (기존 5cm 아래에서 2cm 위로 변경).

### 2.2. 소프트웨어 및 좌표 프레임 (TF)
-   **프레임워크:** ROS 2 Humble
-   **통신:** 표준 ROS 2 토픽; 햅틱 인터페이스용 micro-ROS
-   **주요 좌표 프레임:** 시스템은 공간에서 센서 데이터를 연관시키기 위해 표준 TF 트리에 의존합니다.
    -   `base_link`: 장치의 중심을 나타내는 주요 좌표 프레임
    -   `imu_link`: IMU의 위치와 방향을 나타냄 (LiDAR 기준으로 Z축 +0.02m 위치)
    -   `laser`: RPLiDAR S3의 위치와 방향을 나타냄
    -   **변환:** 이러한 프레임 간의 변환은 `optical_cane.launch.py` 파일에 의해 정적 변환으로 게시됩니다. IMU는 LiDAR 바로 위 2cm에 위치합니다.

### 2.3. 개발 철학 변화: "만들어보자"에서 "증명하자"로
프로젝트의 개발 방식은 빠른 프로토타이핑에서 **엄격하고 증거 기반 엔지니어링**으로 전환되었습니다. 모든 향후 작업은 다음 원칙을 준수해야 합니다:

1.  **먼저 가설을 세우고, 그 다음 코딩:** 기능을 구현하기 전에 명확하고 테스트 가능한 가설을 제시하세요.
2.  **모든 방법을 정당화:** 기술적 선택이 명백하다고 가정하지 마세요. 모든 알고리즘은 명확한 방법론으로 설명되어야 합니다.
3.  **감정이 아닌 데이터로 검증:** "작동하는 것 같다"는 더 이상 허용되지 않는 검증입니다. 모든 주장은 **정량적 데이터**, 비교 분석, 명확한 시각화로 뒷받침되어야 합니다.
4.  **격리하고 검증:** 현재 단계가 철저히 검증되기 전까지는 파이프라인의 다음 단계로 넘어가지 마세요.

---

## 3. 세 가지 경로 계획 파이프라인

프로젝트는 현재 세 가지 서로 다른 파이프라인을 평가합니다. 사용자는 시작 및 테스트를 간소화하기 위해 일련의 별칭(`a0`, `a1` 등)을 만들었습니다.

### 3.1. 핵심 시스템 시작 (파이프라인 A 및 C를 위한 전제 조건)

이 명령들은 3D 가능 파이프라인을 위한 기본 센서 처리를 시작하기 위해 별도의 터미널에서 실행되어야 합니다.

1.  **Micro-ROS Agent 시작 (선택적, 햅틱용)**
    -   **별칭:** `a0`
    -   **명령:** `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
2.  **기본 센서 시작 (LiDAR 및 IMU)**
    -   **별칭:** `a1`
    -   **명령:** `ros2 launch optical_cane_rpi optical_cane.launch.py`
3.  **센서 융합 실행**
    -   **별칭:** `a2`
    -   **명령:** `ros2 run optical_cane_rpi sensor_fusion_node`

### 3.2. 파이프라인 A: 2D로 투영된 3D 데이터

-   **철학:** 3D 인식을 사용하여 환경의 풍부한 모델을 구축한 다음, 2D 경로 계획의 안정성을 활용하기 위해 2D로 평면화합니다.
-   **데이터 흐름:** `IMU/LiDAR` → `sensor_fusion_node` → `/dense_points` → `point_cloud_sweeper_cpp_node` → `/sweep_cloud_cpp` → `path_planner_node` → `/safe_path_vector`
-   **시작 순서:**
    1.  핵심 시작 실행 (`a0`, `a1`, `a2`)
    2.  **Point Cloud Sweeper 실행:**
        -   **별칭:** `a3`
        -   **명령:** `ros2 run cpp_package point_cloud_sweeper_cpp_node`
    3.  **2D Path Planner 실행:**
        -   **별칭:** `a4`
        -   **명령:** `ros2 run cpp_package path_planner_node`

### 3.3. 파이프라인 B: 순수 2D 누적

-   **철학:** 순수하게 2D 공간에서 작동하는 베이스라인 파이프라인입니다. 3D 투영을 위해 IMU를 사용하지 않습니다.
-   **데이터 흐름:** `LiDAR` → `/scan` → `scan_accumulator_node` → `/scan_accumulation_cloud` → `path_planner_node` → `/safe_path_vector`
-   **시작 순서:**
    1.  LiDAR 및 Micro-ROS Agent 실행 (`a0`, `a1`). **`a2` (센서 융합)는 실행하지 마세요**.
    2.  **2D Scan Accumulator 실행:**
        -   **별칭:** `b3`
        -   **명령:** `ros2 run optical_cane_rpi scan_accumulator_node`
    3.  **2D Path Planner 실행 (리매핑 포함):**
        -   **별칭:** `b4`
        -   **명령:** `ros2 run cpp_package path_planner_node --ros-args -r /sweep_cloud_cpp:=/scan_accumulation_cloud`

### 3.4. 파이프라인 C: 전체 3D 복도 스캔

-   **철학:** 센서 기울기에 견고하고 음의 장애물을 감지할 수 있도록 3D 공간에서 직접 경로 계획을 수행하는 가장 고급 파이프라인입니다.
-   **데이터 흐름:** `...` → `/sweep_cloud_cpp` → `voxel_grid_filter_node` → `/downsampled_cloud` → `path_planner_3d_node` → `/safe_path_vector_3d`
-   **시작 순서:**
    1.  핵심 시작 및 Sweeper 실행 (`a0`, `a1`, `a2`, `a3`)
    2.  **Voxel Grid Filter 실행:**
        -   **별칭:** `c4`
        -   **명령:** `ros2 run cpp_package voxel_grid_filter_node --ros-args -p leaf_size:=0.04`
    3.  **3D Path Planner 실행:**
        -   **별칭:** `c5`
        -   **명령:** `ros2 run cpp_package path_planner_3d_node`

---

## 4. 노드 심층 분석: 구현 세부사항

### 4.1. 센서 및 융합 레이어

#### **`sllidar_node` (`sllidar_ros2` 패키지)**
-   **목적:** RPLiDAR S3 센서에서 원시 2D 레이저 스캔 데이터를 게시합니다.
-   **출력:** `/scan` (sensor_msgs/LaserScan)
-   **설정:** 최대 거리, 스캔 주파수, 각도 보정을 포함한 다양한 매개변수
-   **센서 사양:** RPLiDAR S3는 개선된 스캔 정확도와 안정성을 제공합니다.

#### **`mpu9250_driver_node` 및 `mpu9250_filtered` (`mpu9250` 패키지)**
-   **Driver Node:** 원시 IMU/자력계 데이터를 게시합니다.
-   **Filtered Node:** 보정과 Madgwick 필터를 적용하여 깨끗하고 융합된 방향 데이터를 게시합니다.
-   **출력:** `/imu/data` (sensor_msgs/Imu)
-   **물리적 위치:** LiDAR 센서 바로 위 약 2cm에 장착되어 있어 정확한 공간 정렬을 제공합니다.

#### **`sensor_fusion_node.py` (`optical_cane_rpi` 패키지)**
-   **목적:** 모든 3D 파이프라인의 기초 노드입니다.
-   **구독:** `/scan` (sensor_msgs/LaserScan), `/imu/data` (sensor_msgs/Imu)
-   **게시:** `/dense_points` (sensor_msgs/PointCloud2)
-   **핵심 로직:**
    -   `message_filters.ApproximateTimeSynchronizer`를 사용하여 시간적 정렬을 보장 (100ms 허용 오차)
    -   `laser_geometry.LaserProjection` 라이브러리를 사용하여 각 2D `/scan`을 3D 포인트 클라우드로 투영
    -   `/imu/data` 메시지의 방향을 사용하여 포인트를 `base_link` 프레임에 대한 올바른 3D 공간으로 회전
    -   쿼터니언 계산을 사용하여 IMU 방향 기반 3D 변환 수행: `p' = q * p * q_conjugate`

### 4.2. 인식 및 필터링 레이어

#### **`point_cloud_sweeper_cpp_node.cpp` (`cpp_package`)**
-   **목적:** 밀도 높은, 시간 누적 3D 포인트 클라우드를 생성합니다.
-   **입력:** `/dense_points` (PointCloud2)
-   **출력:** `/sweep_cloud_cpp` (PointCloud2)
-   **핵심 로직:**
    -   들어오는 포인트 클라우드의 `std::deque`를 유지
    -   각 클라우드를 `base_link` 프레임으로 변환하고 1.5초 동안 누적 (`buffer_duration_`)
    -   정적 환경의 더 밀도 높은 표현을 생성
    -   TF를 사용하여 정확한 프레임 간 변환 수행

#### **`scan_accumulator_node.py` (`optical_cane_rpi` 패키지)**
-   **목적:** 파이프라인 B를 위한 시간 누적 2D 포인트 클라우드를 생성합니다.
-   **입력:** `/scan` (LaserScan)
-   **출력:** `/scan_accumulation_cloud` (PointCloud2)
-   **핵심 로직:**
    -   1.5초 창 동안 `LaserScan` 메시지를 누적하고 단일 `PointCloud2` 메시지로 병합
    -   각 스캔 포인트를 극좌표에서 직교좌표로 변환: `x = distance * cos(angle)`, `y = distance * sin(angle)`
-   **주요 매개변수:** `front_view_only` (bool, 기본값: true) - 센서 뒤의 포인트를 필터링

#### **`voxel_grid_filter_node.cpp` (`cpp_package`)**
-   **목적:** 3D 경로 플래너와 RViz 시각화를 위한 성능 향상을 위해 큰 `/sweep_cloud_cpp`의 밀도를 줄입니다.
-   **입력:** `/sweep_cloud_cpp` (PointCloud2)
-   **출력:** `/downsampled_cloud` (PointCloud2)
-   **핵심 로직:** 두 단계 필터링
    1.  **PassThrough 필터:** 센서 뒤의 포인트 제거 (X < 0)
    2.  **VoxelGrid 필터:** 클라우드 다운샘플링으로 성능 향상
-   **주요 매개변수:**
    -   `front_view_only` (bool, 기본값: true): PassThrough 필터 제어
    -   `leaf_size` (double, 기본값: 0.1, 실제 사용시 0.04): 다운샘플링을 위한 복셀 크기

### 4.3. 경로 계획 레이어

#### **`path_planner_node.cpp` (`cpp_package`)** - 파이프라인 A 및 B용
-   **목적:** 3D 포인트 클라우드를 2D 평면으로 투영하여 안전한 경로를 계산합니다.
-   **입력:** `/sweep_cloud_cpp` (PointCloud2) 또는 `/scan_accumulation_cloud` (리매핑을 통해)
-   **출력:** `/safe_path_vector` (geometry_msgs/Vector3Stamped), `/candidate_rays` (MarkerArray), `/preprocessed_cloud` (PointCloud2)
-   **핵심 로직:**
    1.  **전처리 단계:**
        -   ROI 필터링: 전방 시야 전용 (X: 0-5m, Z: -1-2m)
        -   선택적 복셀 필터링 (`use_voxel_filter` 매개변수로 제어)
        -   **중요한 2D 투영:** 모든 포인트의 Z 좌표를 0으로 설정 (`point.z = 0`)
    2.  **안전 벡터 계산:**
        -   15개의 가상 레이 캐스팅 (각도 범위: -90도 ~ +90도)
        -   각 레이에 대해 0.15m 폭 내의 장애물까지 최단 거리 계산
        -   채점 알고리즘: `scores[i] = 0.7 * depths[i] + 0.3 * (prev_depth + next_depth)`
    3.  **각속도 제한 스무딩:**
        -   최대 각속도: 90도/초
        -   `angles::shortest_angular_distance`를 사용하여 부드러운 전환 보장
-   **주요 매개변수:**
    -   `front_view_only` (bool): 전방 시야 필터링
    -   `use_voxel_filter` (bool): 성능을 위한 복셀 필터링
    -   `voxel_leaf_size` (double): 복셀 크기

#### **`path_planner_3d_node.cpp` (`cpp_package`)** - 파이프라인 C용
-   **목적:** 센서 기울기에 견고하고 음의 장애물을 감지할 수 있도록 3D 데이터에서 직접 안전한 경로를 계산합니다.
-   **입력:** `/downsampled_cloud` (PointCloud2)
-   **출력:** `/safe_path_vector_3d` (geometry_msgs/Vector3Stamped), `/candidate_rays_3d` (MarkerArray), `/preprocessed_cloud_3d` (PointCloud2)
-   **핵심 로직:**
    1.  **3D 복도 스캔 방법:**
        -   15개의 가상 레이를 캐스팅하며, 각각은 정의된 폭을 가진 3D "복도"를 나타냄
        -   각 복도의 깊이는 **해당 3D 복도 내에서** 발견된 가장 가까운 포인트에 의해 결정
        -   복도 폭 내 포인트 검사: `dist_to_ray < (corridor_width / 2.0)`
        -   모든 높이에서 장애물을 감지하여 **음의 장애물 감지 가능**
    2.  **버그 수정:** 장애물이 없을 때 올바르게 직진하도록 수정 (이전에는 우측으로 기본 설정)
    3.  **동일한 스무딩:** 2D 버전과 동일한 각속도 제한 및 채점 시스템
-   **주요 매개변수:**
    -   `corridor_width` (double, 기본값: 0.3m): 각 3D 복도의 폭
    -   `use_voxel_filter` (bool, 기본값: false): 추가 복셀 필터링

### 4.4. 시각화 및 디버깅 개선사항
-   **RViz 마커 향상:**
    -   선택되지 않은 후보 레이들: `alpha = 0.1` (거의 투명)
    -   이상적인 (원시) 방향: 노란색, `alpha = 0.1`
    -   최종 스무딩된 방향: 파란색, `alpha = 1.0` (완전 불투명)
-   **실시간 직관 향상:** 최종 선택된 경로 벡터가 명확하게 눈에 띄도록 함

---

## 5. 햅틱 인터페이스 구현

### 5.1. 하드웨어 설계
-   **마이크로컨트롤러:** Arduino Nano RP2040 Connect
-   **액추에이터:** 8개의 PWM 진동 모터
-   **드라이버:** ULN2803 드라이버 배열을 통한 모터 제어
-   **통신:** micro-ROS를 통해 Raspberry Pi와 Wi-Fi로 통신

### 5.2. 현재 상태: 보류 (ON HOLD)
-   **이유:** 의미 있는 인간-루프 테스트가 수행되기 전에 핵심 경로 계획 파이프라인의 신뢰성이 데이터로 증명되어야 합니다.
-   **확인된 문제:** 현재의 단단한 팔찌 디자인은 심각한 진동 누화 현상을 겪고 있어, 사용자가 어떤 모터가 활성화되어 있는지 안정적으로 구별하기 어렵습니다. 이 인간공학적 문제는 향후 디자인 반복에서 해결되어야 합니다.

### 5.3. 소프트웨어 구현 (`esp32_bridge_node.py`)
-   **목적:** ROS 2 경로 벡터를 햅틱 모터 명령으로 변환
-   **구독:** `/safe_path_vector` 또는 `/safe_path_vector_3d`
-   **게시:** micro-ROS를 통해 Arduino로 모터 제어 신호 전송

---

## 6. 분석, 검증 및 워크플로우 (Gemini와 함께 개발)

의무적인 데이터 기반 분석을 지원하기 위해 Python 기반 도구 모음과 표준화된 워크플로우가 개발되었습니다.

### 6.1. 데이터 수집 (별칭 사용)
표준화된 데이터 기록을 위한 별칭들:

-   **`d1` (파이프라인 A):** `ros2 bag record -o test_1_p1 /imu/data /safe_path_vector /tf /tf_static /sweep_cloud_cpp`
-   **`d2` (파이프라인 B):** `ros2 bag record -o test_1_p2 /imu/data /safe_path_vector /tf /tf_static /scan_accumulation_cloud`  
-   **`d3` (파이프라인 C):** `ros2 bag record -o test_1_p3 /imu/data /safe_path_vector_3d /tf /tf_static /downsampled_cloud`

### 6.2. 사후 분석 (`plot_analysis.py`)
-   **별칭:** `d0`
-   **명령:** `python3 plot_analysis.py --test_set 1`
-   **목적:** 실험 후 `ros2 bag` 기록에 대한 상세하고 정량적인 분석을 수행합니다.
-   **주요 기능:**
    1.  **자동화된 테스트 세트 처리:** 단일 명령으로 번호가 매겨진 테스트 세트 분석 (예: `test_1_p1`, `test_1_p2`, `test_1_p3`)
    2.  **경로 안정성 플롯:** 세 파이프라인의 시간에 따른 경로 각도 비교. 공정한 비교를 위해 데이터를 가장 짧은 실행에 맞춰 자동 트리밍
    3.  **고급 부드러움 분석:** 경로 안내 품질 평가를 위한 중요한 메트릭 생성:
        -   **각속도 (RMS):** 경로 변화가 얼마나 "급격하거나" 부드러운지 측정. 낮을수록 좋음
        -   **전력 스펙트럼 밀도 (PSD):** 경로 신호의 고주파 "지터"나 진동 양 측정. 낮을수록 좋음
    4.  **기울기 견고성 플롯:** 센서 기울기 중 안정성을 시각화하기 위해 경로 각도와 IMU 피치 비교

### 6.3. 실시간 분석 (`live_plotter.py`)
-   **목적:** `rqt_plot` 필요성을 대체하여 라이브 테스트 중 알고리즘 성능의 즉각적이고 직관적인 시각화를 제공합니다.
-   **주요 구성요소:**
    1.  **`path_vector_plotter.py` (ROS 노드):** `Vector3Stamped` 경로 메시지를 쉬운 소비를 위해 간단한 `Float64` 토픽 (`/plot/path_angle`, `/plot/path_angular_velocity`)으로 변환
    2.  **`live_plotter.py` (독립 실행형 스크립트):** `/plot/*` 토픽을 구독하고 경로 각도와 각속도를 실시간으로 표시하는 Matplotlib 기반 GUI. 안정적인 시청을 위한 고정 Y축 기능
-   **워크플로우:**
    1.  주요 ROS 파이프라인 노드 실행
    2.  `path_vector_plotter.py` 노드 실행
    3.  `python3 live_plotter.py` 실행하여 실시간 그래프 시작

### 6.4. 핵심 성능 메트릭
분석 도구들이 계산하는 정량적 측정값들:

1.  **경로 안정성:** 안정적인 환경에서 출력 `smoothed_angle`의 시간에 따른 분산 또는 표준편차
2.  **부드러움 (각속도 RMS):** 경로 변화의 "급격함" 정도를 측정
3.  **전력 스펙트럼 분석:** 경로 신호의 고주파 노이즈 양을 측정
4.  **기울기 견고성:** 센서 기울기 중 경로 안정성

---

## 7. 전략적 R&D 방향 및 미래 개발

### 7.1. 3D 파이프라인을 위한 고급 개발 경로
현재 3D 파이프라인의 불안정성을 극복하기 위해, 경로 계획 전에 더 정교한 인식 과정이 구현되어야 합니다. 제안된 작업 순서:

#### 1. 지면 평면 분할 검증 (최고 우선순위)
-   **목적:** 기존 지면 분할 코드를 안정적인 어깨 장착 센서 설정으로 테스트
-   **중요성:** 모든 후속 3D 인식 작업의 기초 단계로, 시스템이 통과 가능한 영역과 장애물을 구별할 수 있게 함

#### 2. 단순화된 장애물 클러스터링 구현
-   **목표:** 전체 장면을 클러스터링하려고 시도하는 대신, 고립된 비벽 장애물 (예: 기둥, 가구, 보행자)의 식별 및 클러스터링에 집중
-   **방법:**
    -   지면 제거 후, 남은 포인트에 클러스터링 알고리즘 (예: 유클리드 클러스터링) 사용
    -   큰 연속 클러스터 (벽일 가능성) 필터링하여 작은 객체 클러스터 분리
    -   이러한 고립된 클러스터의 중심점 추출하고 TF 프레임으로 좌표 게시
    -   경로 플래너를 위한 동적 회피 포인트로 활용

#### 3. `sweep_cloud`를 위한 맞춤형 2.5D/높이맵 연구
-   **과제:** 표준 점유 격자 알고리즘은 시간 누적된 "번짐" 특성의 `/sweep_cloud_cpp`에 적합하지 않을 수 있음
-   **목표:** 이 데이터 유형을 효과적으로 처리하여 계산 효율성을 유지하면서 음의 장애물을 감지할 수 있는 수정된 2.5D 높이맵 표현 개발

### 7.2. 비교 분석 확장: 새로운 경로 계획 알고리즘
경로 계획 로직을 마무리하기 전에, 현재 레이 캐스팅 방법에 대해 다른 확립된 알고리즘들이 구현되고 벤치마크되어야 합니다. 이는 차선의 접근법을 조기에 최적화하지 않도록 보장하는 중요한 단계입니다.

#### 구현할 후보 알고리즘들:
1.  **Follow the Gap Method (격차 추종 방법)**
    -   동적 창 접근법을 사용하여 장애물 사이의 가장 큰 간격을 찾음
    -   실시간 내비게이션에서 입증된 강건성
2.  **Vector Field Histogram (VFH) 또는 단순화된 변형**
    -   히스토그램 기반 장애물 표현
    -   지역 최소값 문제에 대한 견고성

#### 다음 단계
이러한 새로운 알고리즘들은 어깨 장착 설정으로 수행될 10회 비교 테스트 시리즈에 구현되고 포함되어야 합니다.

### 7.3. 어깨 장착 테스트 시리즈
프로젝트는 모든 파이프라인과 새로운 알고리즘들의 결정적인 벤치마크가 될 **10회 실행, 어깨 장착 테스트 시리즈**를 준비하고 있습니다. 이 테스트 결과가 분석되기 전까지는 어떤 파이프라인도 "승자"로 간주되어서는 안 됩니다.

### 7.4. 현재 개발 의무사항
1.  **가설 먼저, 그 다음 코딩:** 기능 구현 전 명확하고 테스트 가능한 가설 제시
2.  **모든 방법 정당화:** 모든 알고리즘을 명확한 방법론으로 설명
3.  **데이터로 검증, 감정으로 하지 말기:** 모든 주장은 정량적 데이터로 뒷받침
4.  **격리하고 검증:** 현재 단계가 철저히 검증되기 전까지 다음 단계로 진행하지 말기

---

## 8. 추가 노드 및 도구들

### 8.1. 연구 개발 단계 노드들
프로젝트에는 향후 3D 파이프라인 개선을 위한 추가 연구 노드들이 포함되어 있습니다:

#### **`stable_ground_fitter_node.cpp` (`cpp_package`)**
-   **목적:** 지면 평면 분할을 위한 연구 노드
-   **상태:** 개발 중, 향후 3D 파이프라인 개선의 핵심 구성요소
-   **중요성:** 음의 장애물 감지를 위한 기초 단계

#### **`user_centric_roi_detector.cpp` (`cpp_package`)**
-   **목적:** 사용자 중심의 관심 영역 감지
-   **상태:** 실험적 기능

### 8.2. 분석 및 시각화 도구

#### **`path_vector_plotter.py` (`optical_cane_rpi` 패키지)**
-   **목적:** 실시간 분석을 위한 메시지 변환
-   **기능:**
    -   `Vector3Stamped` 경로 메시지를 `Float64` 토픽으로 변환
    -   `/plot/path_angle`: 경로 각도 (도 단위)
    -   `/plot/path_angular_velocity`: 각속도 (도/초)
-   **사용법:** `live_plotter.py`와 함께 실시간 시각화를 위해 사용

#### **Live Plotter 기능들**
-   **고정 Y축:** 안정적인 시청을 위한 일관된 스케일
-   **실시간 업데이트:** Matplotlib FuncAnimation 사용
-   **ROS 2 통합:** 백그라운드에서 ROS 노드 실행
-   **데이터 히스토리:** 최근 100개 데이터 포인트 유지

---

## 9. 시스템 설정 및 캘리브레이션

### 9.1. IMU 캘리브레이션
시스템에는 IMU 센서의 정확한 작동을 위한 캘리브레이션 파일들이 포함되어 있습니다:

#### 캘리브레이션 파일들 (`calib/` 디렉토리)
-   **`mpu9250_calib.json`:** 메인 캘리브레이션 파일
-   **`accel_log.json`:** 가속도계 캘리브레이션 데이터
-   **`gyro_log.json`:** 자이로스코프 캘리브레이션 데이터  
-   **`mag_log.json`:** 자력계 캘리브레이션 데이터
-   **`plot.py`:** 캘리브레이션 데이터 분석 및 시각화 스크립트

### 9.2. TF (Transform) 설정
시스템의 좌표 프레임 관계가 `optical_cane.launch.py`에서 정적 변환으로 정의됩니다:

-   **`base_link`:** 장치의 주요 좌표 프레임 (중심점)
-   **`imu_link`:** IMU 센서의 위치 및 방향 (LiDAR에서 Z축 +0.02m, 즉 2cm 위)
-   **`laser`:** RPLiDAR S3 센서의 위치 및 방향

**중요한 변경사항:** IMU가 이제 LiDAR 바로 위에 근접 장착되어 (2cm 차이) 센서 간 정렬이 크게 개선되었습니다.

### 9.3. Launch 파일들 (`optical_cane_rpi/launch/`)
-   **`optical_cane.launch.py`:** 메인 시스템 launch 파일 (RPLiDAR S3 + IMU + TF)
-   **`ekf_only.launch.py`:** EKF 전용 테스트 launch 파일
-   **`odom_test.launch.py`:** 오도메트리 테스트용 launch 파일

---

## 10. 성능 분석 및 결과

### 10.1. 현재까지의 테스트 결과
프로젝트는 여러 세트의 테스트 데이터를 수집했습니다:

#### 안정성 분석 결과
-   **파이프라인 1 (2D 투영):** 현재 가장 안정적인 성능
-   **파이프라인 2 (순수 2D):** 베이스라인 성능
-   **파이프라인 3 (3D 복도):** 개선이 필요하지만 음의 장애물 감지 가능

#### 기울기 견고성 테스트
-   **정보 역설:** 2D 투영이 Z축 노이즈를 필터링하여 더 안정적
-   **3D의 필요성:** 안전성을 위한 음의 장애물 감지는 필수

### 10.2. 생성된 분석 플롯들
시스템은 다음과 같은 분석 이미지들을 자동 생성합니다:
-   `path_accuracy_analysis_set_1.png`
-   `path_stability_analysis_set_1.png`
-   `smoothness_angular_velocity_set_1.png`
-   `smoothness_power_spectrum_set_1.png`
-   `tilt_robustness_Pipeline3_3D_Corridor_set_1.png`

---

## 11. 워크스페이스 구조 및 빌드 시스템

### 11.1. ROS 2 패키지 구조
-   **`cpp_package/`:** C++ 구현 노드들 (경로 플래너, 필터, 스위퍼)
-   **`optical_cane_rpi/`:** Python 구현 노드들 (센서 융합, 스캔 누적기)
-   **`mpu9250/`:** IMU 드라이버 패키지
-   **`sllidar_ros2/`:** RPLiDAR S3 드라이버 패키지
-   **`drive_base_msgs/`, `micro_ros_msgs/`:** 메시지 정의 패키지들

### 11.2. 빌드 및 설치
-   **`build/`:** 컴파일된 객체 파일들
-   **`install/`:** 설치된 패키지들
-   **`log/`:** 빌드 로그 및 실행 로그들

### 11.3. 외부 하드웨어 프로젝트
-   **`haptic_controller_platformio/`:** Arduino 햅틱 컨트롤러 코드 (PlatformIO 프로젝트)
-   **`arigato/`:** 관련 프로젝트 (세부사항 미확인)

---

## 12. 알려진 문제 및 제한사항

### 12.1. 현재 알려진 문제들
1.  **Live Plotter 안정성:** 때때로 메모리 관련 문제가 발생할 수 있음 (ROS GUI 도구와 Matplotlib 간의 복잡한 상호작용 가능성)
2.  **햅틱 피드백 누화:** 현재 팔찌 디자인의 진동 간섭 문제
3.  **3D 파이프라인 노이즈:** Z축 센서 노이즈로 인한 불안정성

### 12.2. 설계 제한사항
1.  **2D 투영의 음의 장애물 맹점:** 안전상 치명적인 제한
2.  **현재 레이 캐스팅 방법:** 다른 경로 계획 알고리즘과의 비교 필요
3.  **센서 범위:** 현재 최대 5m 범위 제한

---

## 13. 다음 Gemini 에이전트를 위한 지시사항

## 13. 다음 Gemini 에이전트를 위한 지시사항

### 13.1. 즉시 우선순위 (2025년 8월 22일)
1.  **HeightMap 2.5D 알고리즘 개선** - 🚨 최우선
    - 원통 지속성 구현 (sweep_cloud_cpp 방식 참고)
    - 높이 반영 오류 수정 (실제 포인트 높이 사용)
    - 마커 크기 및 격자 해상도 최적화
2.  **새로운 경로 계획 알고리즘 구현** (Follow the Gap, VFH)
3.  **워크스페이스 정리** (불필요 파일 정리 및 폴더 구조 개선)

### 13.2. 장기 목표
1.  **FTG-3D 시각화 개선**
2.  **10회 어깨 장착 테스트 시리즈 준비**
3.  **지면 평면 분할 검증**
4.  **3D 파이프라인 안정성 향상**

### 13.3. 현재 구현된 새로운 알고리즘들 (2025년 8월 20일 추가)

#### HeightMap 2.5D Planner (`heightmap_planner_node.cpp`)
-   **목적:** 높이 맵 기반 음의 장애물 (맨홀, 계단) 탐지
-   **입력:** `/downsampled_cloud` (PointCloud2)
-   **출력:** `/safe_path_vector_heightmap`, `/height_map_markers`, `/risk_map_markers`
-   **현재 상태:** 기본 구현 완료, 시각화 개선 필요
-   **알려진 문제:**
    - 원통 마커가 점과 함께 지속되지 않음 (DELETEALL 문제)
    - 높이 반영이 부정확 (2배 계산 오류)
    - 격자 해상도 너무 세밀 (성능 부하)

#### Follow-the-Gap 3D (`ftg_3d_node.cpp`)
-   **목적:** 3D 섹터 기반 연속 갭 탐지
-   **입력:** `/downsampled_cloud` (PointCloud2)
-   **출력:** `/safe_path_vector_ftg3d`, `/gap_markers`, `/sector_markers`
-   **현재 상태:** 기본 구현 완료, 적응형 임계값 구현됨
-   **개선사항:** 좁은 환경(1m 폭)에서도 갭 탐지 가능

#### 새로운 Launch 시스템
-   **Launch 파일:** `new_algorithms.launch.py`
-   **실행 별칭:** `new_heightmap`, `new_ftg3d`, `new_both`
-   **자동화 도구:** `automated_experiment.py`, `enhanced_analysis.py`

### 13.4. 개발 원칙 준수
-   **데이터 기반 결정:** 모든 주장을 정량적 데이터로 뒷받침
-   **단계적 검증:** 각 단계를 철저히 검증 후 다음 단계 진행
-   **안전 우선:** 음의 장애물 감지는 타협할 수 없는 요구사항
-   **사용자 피드백 우선:** 기술적 완벽성보다 실용성과 직관성 중시

### 13.5. 주의사항
-   **성급한 결론 금지:** 충분한 테스트 결과 분석 전까지 어떤 알고리즘도 "승자"로 단정하지 말 것
-   **코드 정리 적절히:** 너무 이른 최적화는 피하되, 작업 효율성을 위한 정리는 필요
-   **실험 우선:** 이론적 완벽성보다 실제 실험을 통한 검증 중시
-   **시각화 중요성:** 직관적이고 명확한 시각화가 알고리즘 이해와 디버깅에 핵심

### 13.6. 기술적 채무 (Technical Debt)
1.  **HeightMap 원통 지속성:** sweep_cloud 방식으로 시간 누적 필요
2.  **마커 시스템 개선:** DELETEALL 대신 개별 마커 관리
3.  **성능 최적화:** 격자 해상도와 시각화 품질의 균형
4.  **워크스페이스 정리:** 파일 구조 개선 및 불필요 파일 정리

---

## 14. 2025-08-22 업데이트 요약 (Front = -X 전환 및 토픽 구조 확정)

### 14.1. 전역 좌표 기준 변경
- 모든 `front_view_only` 로직에서 전방을 기존 `+X` → `-X` 로 재정의.
- 적용 노드: `path_planner_node`, `path_planner_3d_node`, `ftg_3d_node`, `heightmap_planner_node`, `voxel_grid_filter_node`, `scan_accumulator_node`.
- PassThrough / 필터 조건: `x` 범위 (기존 0..R) → `(-R .. 0)`.
- 각도 처리: 모든 레이/섹터/갭/heightmap ray angle 계산 후 `+π` 시프트.
- 초기 `smoothed_angle_` 값 `0` → `M_PI` 로 조정 (초기 방향 -X 정면).

### 14.2. 주 토픽/시각화 정리 (최종 확정)
| 기능 | 최종 토픽 | 비고 |
|------|-----------|------|
| 통합 경로 출력 | `/safe_path_vector` | 모든 플래너 동일 (과거 *_3d / *_heightmap 제거) |
| 일반 디버그 마커 | `/path_planner_debug` | 후보 레이, 안전 화살표 등 공통(혼잡 방지 최소화) |
| FTG 갭 시각화 | `/ftg_gaps` | 갭 아크 + 선택 방향 전용 (섹터는 옵션) |
| HeightMap 위험 필러 | `/height_pillars` | 드롭/장애물/불균일 Pillar (동적 높이/반경/색) |
| 지면 평면 (연구) | `/ground_plane_marker` | 변화 없음 |

### 14.3. Follow-The-Gap 3D 개선
- 전방 -X 반영. - 라이다를 교체하며 기존 +X에서 -X를 Front로 선정하는 것이 좀 더 편안해졌기에 변경.
- 섹터 거리 프로파일 180° ( -90°~+90° 상대 ) → world frame -X 중심.
- `show_sectors` 파라미터: true 시 얇은 회색 화살표를 `/path_planner_debug` 에 출력.
- 갭 전용 토픽 `/ftg_gaps` 분리 → 시각적 분리(잡음 감소).
- 중앙성 + 폭 + 깊이 복합 점수 (좁은 미로 환경에서도 중앙 통과 유도).

### 14.4. HeightMap 2.5D Planner 개선
- Front -X 적용 및 ground calibration 영역 재설정: `x∈[-2.0,-0.5]` (샘플 부족 방지 위해 최소 샘플 50).
- Pillar 표현:
  * 유형 분류: DROP / OBSTACLE / UNEVEN.
  * 높이: 유형/심각도 기반 비선형 스케일 (drop/protrusion/roughness 제한 cap).
  * 반경: 해상도 비례 + severity 가중 (기본 0.25*grid_resolution 최소 0.01).
  * 색상: DROP (노랑→빨강), OBSTACLE (주황→진홍), UNEVEN (연녹→주황).
- 경로 선택 로직 추가 개선:
  * `traversable_counts[i]==0` 레이 패널티 (공허/가장자리 쏠림 완화).
  * 중앙 bias (2차 곡선) 적용으로 ±90° 경계로 화살표 튀는 현상 감소.
- Marker keepalive: Pillar 없을 때 RViz 토픽 유지 위한 투명 dummy 추가.

### 14.5. 2D / 3D Corridor Planner 조정
- 전방 반전 + 중앙 bias 적용.
- 레이 각도 `+π` 시프트로 -X 기준.
- 기존 전방 필터 조건 `point.x < 0` 검사 방향 반전.

### 14.6. Voxel / Scan Accumulator 조정
- PassThrough x 필터 범위 변경: `0..N` → `-N..0`.
- Scan Accumulator front mask: (이전 ±90° around 0 rad) → (±90° around π rad) 로 재작성 (wrap 각도 diff 사용).

### 14.7. 토픽 과거 기술 항목 정리 필요
문서 상 남아있던 구형 토픽 (예: `/safe_path_vector_3d`, `/gap_markers`, `/height_map_markers`, `/risk_map_markers`) 는 **모두 폐기**. 분석 및 스크립트 갱신 시 최신 토픽만 사용.

### 14.8. 남은 단기 기술 부채
1. HeightMap pillar 시간 지속성(프레임별 flicker 필터) → deque 기반 누적 후 가중 감쇠 필요.
2. Gap/Height Pillar 스케일 파라미터화 (ros2 param set으로 동적 조정) 준비.
3. /corridor_preprocessed_cloud 유지 여부 결정 (장기: 파라미터로 on/off).
4. Negative obstacle 정량 이벤트 추적(드롭 감지 횟수 / 단위 거리) 로깅.

---

## 15. 2025-08-23 어깨 장착 비교 실험 계획 (5 방식 × 5 반복)

### 15.1. 비교 대상 5 방식
| 코드명 | 설명 | 플래너 노드 | 주 입력 | 비고 |
|--------|------|-------------|---------|------|
| A | 3D→2D 투영 누적 (기존 안정형) | `path_planner_node` | `/sweep_cloud_cpp` | IMU 사용, Z 평탄화 |
| B | 순수 2D 스캔 누적 | `path_planner_node` (remap) | `/scan_accumulation_cloud` | IMU 미사용 |
| C | 3D Corridor | `path_planner_3d_node` | `/downsampled_cloud` | 음의 장애물 잠재 감지 |
| D | Follow-The-Gap 3D | `ftg_3d_node` | `/downsampled_cloud` | 갭 중심 추종 |
| E | HeightMap 2.5D | `heightmap_planner_node` | `/downsampled_cloud` | Drop / Obstacle / Uneven 고려 |

모두 최종 경로 토픽 동일: `/safe_path_vector` (단일화). 반복 실행 별도 bag 이름으로 구분.

### 15.2. 실행/기록 표준화
반복 번호 r ∈ {1..5}. 세션 prefix: `smt` (shoulder mounted test). 디렉토리/rosbag 이름 규칙:
`bag_<DATE>_<MODE><r>` 예) `bag_0823_A1`.

### 15.3. 기록해야 할 공통 최소 토픽 세트
필수(모든 모드 공통):
- `/safe_path_vector`
- `/tf` `/tf_static`
- `/imu/data` (B 모드에서도 기준 프레임 로그 용도)
- 원시 또는 전처리 포인트: 모드별 1개만
  * A: `/sweep_cloud_cpp`
  * B: `/scan_accumulation_cloud`
  * C/D/E: `/downsampled_cloud`
- 시각화 근거:
  * D: `/ftg_gaps`
  * E: `/height_pillars`
  * 공통 디버그: `/path_planner_debug`

선택(추가 분석 시): `/corridor_preprocessed_cloud` (C), `/ground_plane_marker`.

### 15.4. rosbag 예시 명령 세트
(모드 A 1회차 예)
`ros2 bag record -o bag_0823_A1 /safe_path_vector /imu/data /tf /tf_static /sweep_cloud_cpp /path_planner_debug`

모드별 템플릿:
- A: `/sweep_cloud_cpp`
- B: `/scan_accumulation_cloud`
- C: `/downsampled_cloud /path_planner_debug /corridor_preprocessed_cloud`
- D: `/downsampled_cloud /ftg_gaps /path_planner_debug`
- E: `/downsampled_cloud /height_pillars /path_planner_debug`

### 15.5. 실시간 모니터링 권장
- RViz: `/safe_path_vector`, `/ftg_gaps`, `/height_pillars` 표시.
- Live Plot: 기존 `path_vector_plotter.py` + `live_plotter.py` (각 모드 동일).

### 15.6. 사후 분석 메트릭 (추가/정의 명확화)
| 카테고리 | 메트릭 | 설명 | 계산 방식 제안 |
|----------|--------|------|----------------|
| 안정성 | Path Angle Std | `safe_angle(t)` 표준편차 | 라디안→도 변환 후 std |
| 부드러움 | Angular Velocity RMS | dθ/dt RMS | 1차 미분 후 RMS |
| 저주파 편향 | Heading Drift | 시작 대비 누적 평균 편차 | mean(θ) - θ_start |
| 고주파 노이즈 | PSD High-Freq Energy | >1Hz 대역 적분 | Welch / SciPy |
| 회피 여유 | Clearance Proxy | FTG: 선택 갭 깊이, Corridor: best ray depth | 마커/내부 depth 로그 추가 고려 |
| 음의 장애물 지표 | Drop Pillar Count / min | HeightMap DROP 타입 빈도 | 시간 정규화 |
| 지형 위험 합 | Pillar Severity Sum | Σ(severity) / window | 1초 슬라이딩 |
| 탐색 안정성 | Direction Reversal Count | sign 변화(>45°) 횟수 | 임계 기반 카운트 |
| 중앙성 | Deviation from Forward | | mean(|θ - π|) (전방 -X) |
| 반응 지연 | Latency (optional) | 입력 cloud stamp→vector stamp | 타임스탬프 차 |

### 15.7. 추가 HeightMap 전용 추출
- Pillar 타입별 비율 (DROP/OBSTACLE/UNEVEN).
- Max severity vs 선택 경로 방향 각도 차 (위험 회피 정도).

### 15.8. 추가 FTG 전용 추출
- 선택된 갭 폭(deg) 히스토그램.
- 연속 프레임 gap center angle 변화율.

### 15.9. 분석 파이프라인 제안 절차
1. Bag 목록 스캔 → (모드, 반복) 파싱.
2. 공통 parser: `/safe_path_vector` → angle, angular velocity 시계열.
3. 모드별 추가 parser:
   - D: `/ftg_gaps` LINE_STRIP id→gap score (색/길이) 추출 (필요시 Marker 해석 룰 정의).
   - E: `/height_pillars` CYLINDER set → severity (색/scale) 역변환 저장 (현재 색/scale 공식 그대로 사용).
4. 메트릭 계산 후 DataFrame 합치기 (columns: mode, run, metric, value).
5. 시각화:
   - Bar(평균) + error bar(표준편차) per metric.
   - Time-series overlay (대표 run 1개씩) 안정성 비교.
   - Boxplot (Angular Velocity RMS) 5×5.
6. 결과 요약 자동 Markdown 생성 → KB append.

### 15.10. 파일/폴더 네이밍 권장
```
results/
  2025-08-23/
    raw_bags/ (심볼릭링크 또는 원본 위치 참조)
    metrics_summary.csv
    metrics_by_run.csv
    plots/
      stability_bar.png
      angular_velocity_box.png
      ftg_gap_width_hist.png
      heightmap_drop_counts.png
      psd_comparison.png
    report.md
```

### 15.11. 자동화 TODO (내일 담당 AI 위한 선행 과제)
- [ ] 기존 `plot_analysis.py` 확장: 다중 모드 자동 ingest.
- [ ] HeightMap pillar 역해석 유틸 (scale / color → severity/type) 함수화.
- [ ] FTG gap arc 파싱(라인스트립 첫 center→호 포인트→깊이/폭 재계산) 함수.
- [ ] 공통 CLI: `python analyze_batch.py --date 2025-08-23 --modes A,B,C,D,E`.
- [ ] 결과 Markdown 템플릿 자동 채움.

### 15.12. 위험 / 주의 포인트
- Shoulder 장착 시 센서 pitch 변화 → -X front 정합 확인 (TF/Axes 표시 필수).
- HeightMap ground 재캘리브레이션 지연 가능 (샘플 부족 시) → 초기 2초 정지 권장.
- 동일 환경 반복 경로 다양성 확보 위해 수행 순서 permutation (A→E 순 고정 편향 방지).

---

**오늘 변경 핵심:** Front = -X 통일, 토픽 단일화(`/safe_path_vector`), 시각화 정비(`/ftg_gaps`, `/height_pillars`), HeightMap/FTG 알고리즘 안정화 편향/패널티 도입.

**마지막 업데이트:** 2025년 8월 22일 새벽
**NEW_ALGORITHM_EXPERIMENT_GUIDE.md:** 해당 파일도 참조