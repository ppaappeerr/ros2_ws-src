# 시각 장애인을 위한 광학 지팡이 프로젝트 - Claude Code 가이드

## 프로젝트 개요

이 프로젝트는 시각 장애인을 위한 실시간 경로 안내 시스템입니다. 2D LiDAR(RPLiDAR S3)와 IMU(MPU9250)를 사용하여 환경을 인식하고, 안전한 경로를 계산하여 햅틱 인터페이스로 사용자에게 전달합니다.

**현재 단계**: 5개의 서로 다른 경로 계획 알고리즘의 비교 분석을 통해 최적 솔루션 도출

## 핵심 문제: 2D vs 3D 딜레마

- **2D의 장점**: 안정적이고 노이즈가 적음
- **2D의 치명적 단점**: 음의 장애물(구덩이, 계단 등) 감지 불가능 → 안전상 위험
- **3D의 필요성**: 안전을 위해서는 필수적이지만 현재 불안정성 문제

## 시스템 아키텍처

### 하드웨어
- **Raspberry Pi 5**: 메인 처리 유닛
- **RPLiDAR S3**: 2D 레이저 스캔 센서 (기존 A2M8에서 업그레이드)
- **MPU9250**: IMU 센서 (LiDAR 위 2cm에 근접 배치)
- **Arduino RP2040**: 햅틱 인터페이스 제어 (현재 보류)

### 좌표계 및 방향 (중요!)
**2025-08-22 변경사항**: 모든 시스템에서 전방(Front) = **-X 방향**으로 통일

### ROS2 토픽 구조 (최종 확정)
- **통합 경로 출력**: `/safe_path_vector` (모든 플래너 공통)
- **일반 디버그**: `/path_planner_debug` (후보 레이, 안전 화살표)
- **FTG 갭 시각화**: `/ftg_gaps` (갭 아크 + 선택 방향)  
- **HeightMap 위험**: `/height_pillars` (드롭/장애물 기둥들)

## 5개 경로 계획 파이프라인

### A. 3D→2D 투영 (기존 안정형)
```bash
# 시작 순서
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888  # a0
ros2 launch optical_cane_rpi optical_cane.launch.py        # a1
ros2 run optical_cane_rpi sensor_fusion_node               # a2
ros2 run cpp_package point_cloud_sweeper_cpp_node          # a3
ros2 run cpp_package path_planner_node                     # a4
```
- **특징**: 3D 데이터 수집 → 2D로 평면화 (Z=0)
- **장점**: 현재 가장 안정적
- **단점**: 음의 장애물 감지 불가

### B. 순수 2D 스캔 누적
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888  # a0
ros2 launch optical_cane_rpi optical_cane.launch.py        # a1
ros2 run optical_cane_rpi scan_accumulator_node            # b3
ros2 run cpp_package path_planner_node --ros-args -r /sweep_cloud_cpp:=/scan_accumulation_cloud  # b4
```
- **특징**: 순수 2D 기반, IMU 미사용
- **장점**: 단순함, 베이스라인
- **단점**: 제한적 환경 인식

### C. 3D Corridor 스캔
```bash
# a0, a1, a2, a3 실행 후
ros2 run cpp_package voxel_grid_filter_node --ros-args -p leaf_size:=0.04  # c4
ros2 run cpp_package path_planner_3d_node                                   # c5
```
- **특징**: 3D 복도 스캔 방식
- **장점**: 음의 장애물 감지 가능
- **단점**: 현재 불안정성 있음

### D. Follow-the-Gap 3D
```bash
# a0, a1, a2, a3, c4 실행 후
ros2 run cpp_package ftg_3d_node  # 새로운 알고리즘
```
- **특징**: 3D 섹터 기반 연속 갭 탐지
- **장점**: 좁은 환경에서도 갭 탐지
- **고유 토픽**: `/ftg_gaps` (갭 시각화)

### E. HeightMap 2.5D
```bash
# a0, a1, a2, a3, c4 실행 후
ros2 run cpp_package heightmap_planner_node  # 새로운 알고리즘
```
- **특징**: 높이 맵 기반 음의 장애물 탐지
- **장점**: DROP/OBSTACLE/UNEVEN 분류 가능
- **고유 토픽**: `/height_pillars` (위험도 시각화)

## 주요 소스 코드 위치

### 경로 계획 노드들 (cpp_package/src/)
- `path_planner_node.cpp`: 2D 기반 플래너 (A, B)
- `path_planner_3d_node.cpp`: 3D Corridor 플래너 (C) 
- `ftg_3d_node.cpp`: Follow-the-Gap 3D (D)
- `heightmap_planner_node.cpp`: HeightMap 2.5D (E)

### 센서 및 전처리 (optical_cane_rpi/)
- `sensor_fusion_node.py`: LiDAR+IMU 융합
- `scan_accumulator_node.py`: 2D 스캔 누적

### 데이터 처리 (cpp_package/src/)
- `point_cloud_sweeper_cpp_node.cpp`: 3D 포인트 클라우드 시간 누적
- `voxel_grid_filter_node.cpp`: 3D 다운샘플링

### 런치 파일들
- `optical_cane_rpi/launch/optical_cane.launch.py`: 기본 센서 시작
- `cpp_package/launch/new_algorithms.launch.py`: 새 알고리즘 테스트

## 분석 및 실험 도구

### 데이터 기록 (별칭 사용)
```bash
# 파이프라인별 기록
d1  # 파이프라인 A 기록
d2  # 파이프라인 B 기록  
d3  # 파이프라인 C 기록
```

### 실시간 분석
```bash
python3 live_plotter.py  # 실시간 경로 각도/각속도 플롯
```

### 사후 분석
```bash
d0  # python3 plot_analysis.py --test_set 1
```

## 현재 개발 상태 (2025-08-28) - 통일된 플래너 완성!

### 🎉 최신 완료사항 (세션 5시간 작업 결과)
1. **통일된 베이스 클래스 구현**: 모든 플래너가 공통 처리 사용
   - Dead-zone 필터링 (어깨 장착용)
   - Evidence-based scoring (편향 보정)
   - 적응형 스무딩 (긴급도별 각속도)
   - 통일된 파라미터 구조

2. **5개 통일된 플래너 완성**:
   - `unified_path_planner_2d` (UP1): 3D→2D 투영
   - `unified_scan_accumulator_planner` (UP2): 순수 2D 
   - `unified_path_planner_3d_corridor` (UP3): 3D Corridor
   - `unified_ftg_3d_planner` (UP4): Follow-the-Gap 3D
   - `unified_heightmap_planner` (UP5): HeightMap 2.5D

3. **핵심 문제 해결**:
   - sensor_fusion_node X축 뒤집기 원복 (점 방향 정상화)
   - UP4 직진 문제 수정 (전방 뷰 범위 조정)
   - UP4, UP5 시각화 완전 복구 (ftg_gaps, height_pillars)
   - PCL 빈 클라우드 에러 처리

4. **5×5 실험 자동화 구축**:
   - 체계적 네이밍 시스템 (up1_trial_1 ~ up5_trial_5)
   - 자동 분석 파이프라인
   - 정량적 성능 지표 (안정성, 반응성, 안전성, 편향성, 연속성, 부드러움)

### 🚀 실험 준비 완료
```bash
# 빠른 실험 설정
source ~/ros2_ws/src/quick_experiment_aliases.sh

# 플래너 실행
up1, up2, up3, up4, up5

# 데이터 기록 
bup1_1, bup1_2, ..., bup5_5

# 분석
analyze_experiment
```

### ⚡ 다음 세션 작업
1. **5×5 실제 실험 수행** (각 플래너별 5회씩)
2. **정량적 성능 비교 분석**
3. **최적 플래너 선정 및 파라미터 튜닝**

### 해결된 기술 부채
- ✅ HeightMap pillar 시각화 완전 복구
- ✅ 플래너 간 일관성 확보 (공통 베이스 클래스)
- ✅ 실험 자동화 및 분석 도구 완성

## 빌드 및 실행 명령어

### 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select optical_cane_rpi cpp_package
source install/setup.bash
```

### 기본 센서 시작
```bash
ros2 launch optical_cane_rpi optical_cane.launch.py
```

### 새 알고리즘 테스트
```bash
ros2 launch cpp_package new_algorithms.launch.py pipeline:=heightmap
ros2 launch cpp_package new_algorithms.launch.py pipeline:=ftg3d
ros2 launch cpp_package new_algorithms.launch.py pipeline:=both
```

## 향후 개선 사항 (TF 오프셋 고려)

### 🎯 사용자 중심 좌표계 필요성
**현재 상황**:
- 라이더 센서 TF (`lidar_frame`) 기준으로 방향을 지시
- 오른쪽 어깨에 라이더 부착으로 인한 중심점 편차

**문제점**:
- 사용자의 실제 중심(목/가슴 중심)은 라이더 위치에서 약 25cm 왼쪽
- 현재 dead-zone 마스킹 50cm를 고려하면, 사용자 중심 기준 방향 계산이 필요

**개선 방안**:
```
현재: [LIDAR] → 방향 계산 → 사용자에게 지시
개선: [LIDAR] → [NECK/CENTER] TF 변환 → 방향 계산 → 지시
```

**구현 고려사항**:
- TF tree에 `neck_frame` 또는 `user_center` 추가
- 라이더 위치에서 (-25cm, 0, 0) 오프셋 적용
- 모든 경로 계획 알고리즘에서 이 중심점 기준으로 방향 계산
- 장기적으로는 IMU 기반 body orientation도 고려 가능

**우선순위**: 중간 (핵심 알고리즘 안정성 확보 후 적용)

## 개발 원칙

1. **데이터 기반 결정**: 모든 주장을 정량적 데이터로 뒷받침
2. **단계적 검증**: 각 단계를 철저히 검증 후 다음 단계 진행
3. **안전 우선**: 음의 장애물 감지는 타협할 수 없는 요구사항
4. **가설→실험→검증**: 코딩 전 가설 수립 필수
5. **플래너 통일성**: 모든 경로 계획 알고리즘에서 "길찾기 방식"을 제외한 전처리, 필터링, 스무딩, 출력은 동일해야 함

## Claude Code 사용 설정

**중요**: Claude는 모든 답변을 **한국어로만** 제공해야 함

## 중요 참고사항

- **햅틱 인터페이스**: 현재 보류 (핵심 경로 계획 안정성 확보 후)
- **센서 설정**: LiDAR S3로 업그레이드, IMU 근접 배치로 정렬 개선
- **전방 방향**: **-X가 전방**임을 항상 유의 (2025-08-22 변경)
- **토픽 통일**: 모든 플래너가 `/safe_path_vector`로 경로 출력

이 문서는 Claude Code 사용을 위한 핵심 가이드입니다. 더 자세한 내용은 `OPTICAL_CANE_KB.md`를 참조하세요.