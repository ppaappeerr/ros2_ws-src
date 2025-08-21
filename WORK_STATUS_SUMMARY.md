# 🔧 작업 상황 정리 - 8월 20일 저녁

## 🚨 현재 주요 문제점

### HeightMap 2.5D 알고리즘 문제
1. **원통 지속성 부족**
   - 현재: 점이 있을 때만 원통 생성 → 점 없으면 원통 사라짐
   - 원하는 것: sweep_cloud_cpp처럼 시간 누적으로 원통이 지속되어야 함
   - 문제 원인: `DELETEALL` 마커로 매번 초기화해서 잔상 효과 없음

2. **높이 반영 오류**
   - 현재: 이상한 2배 계산 (`actual_height_diff`, Z위치 조정)
   - 원하는 것: 실제 포인트 높이 그대로 반영
   - 예시: 점이 TF 0.5m 위에 있으면 원통도 정확히 0.5m 높이에

3. **리소스 사용량**
   - 격자 해상도 너무 세밀 (0.02m) → 연산 부하
   - 원통/구체 크기 너무 작음 → 시각적으로 잘 안보임

### FTG-3D 알고리즘 문제
1. **갭 표시 개선 필요**
   - 방향은 맞게 나오지만 직관성 부족
   - 화살표와 원통 마커 조정 필요

## 🔧 수정해야 할 코드 요소들

### heightmap_planner_node.cpp - 상세 분석

#### 문제 1: DELETEALL로 잔상 효과 제거 (라인 291-294)
```cpp
// 현재 문제 코드
visualization_msgs::msg::Marker clear_marker;
clear_marker.header = header;
clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
height_markers.markers.push_back(clear_marker);
```

#### 문제 2: 복잡한 높이 계산 (라인 342-354)
```cpp
// 현재 문제 코드
risk_marker.pose.position.z = cell.mean_z + 0.05;  // 이상한 오프셋
float actual_height_diff = std::abs(cell.mean_z - ground_height_);
risk_marker.scale.z = std::max(0.02f, actual_height_diff);  // 높이 2배 계산

// 원하는 동작: 점의 실제 높이 그대로 반영
risk_marker.pose.position.z = cell.mean_z;  // 포인트 실제 높이
risk_marker.scale.z = 0.05;  // 고정 두께 또는 위험도 반영
```

#### 문제 3: 너무 세밀한 격자 (라인 42)
```cpp
// 현재: 너무 세밀해서 성능 부하
this->declare_parameter<double>("grid_resolution", 0.02);  // 2cm

// 제안: 적절한 크기로 조정
this->declare_parameter<double>("grid_resolution", 0.05);  // 5cm
```

#### 문제 4: 너무 작은 마커 크기 (라인 308-310, 345-346)
```cpp
// 현재: 너무 작아서 안보임
height_marker.scale.x = 0.01;  // 1cm 구체
risk_marker.scale.x = 0.015;   // 1.5cm 원통

// 제안: 가시성 향상
height_marker.scale.x = 0.03;  // 3cm 구체  
risk_marker.scale.x = 0.04;    // 4cm 원통
```

### 해결 방향 - sweep_cloud_cpp 방식 참고

#### point_cloud_sweeper_cpp_node.cpp에서 배울 점
```cpp
// sweep_cloud_cpp의 시간 누적 방식 (참고용)
std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buffer_;
double buffer_duration_ = 1.5;  // 1.5초간 누적

// HeightMap에 적용할 구조
struct HeightMapFrame {
    rclcpp::Time timestamp;
    std::unordered_map<std::string, HeightCell> cells;
};
std::deque<HeightMapFrame> height_history_;
```

### ftg_3d_node.cpp - 수정 가능 요소들

#### 갭 마커 크기 조정 (라인 약 380-390)
```cpp
// 현재 작은 크기
marker.scale.x = 0.3;  // gap 마커
marker.scale.y = 0.3;

// 제안: 더 명확한 표시
marker.scale.x = 0.5;  
marker.scale.y = 0.5;
```

#### 화살표 두께 조정 (라인 약 350-360)
```cpp
// 현재 얇은 화살표
marker.scale.x = 0.01;  // shaft
marker.scale.y = 0.02;  // head

// 제안: 더 굵은 화살표
marker.scale.x = 0.02;  // shaft
marker.scale.y = 0.04;  // head
```

## 🗂️ 정리 필요한 파일들 (상세)

### ros2_ws/src/ 불필요 파일들

#### 1. 분석 결과 이미지들 (src 루트에 산재)
```
path_accuracy_analysis_set_1.png          → results/plots/로 이동
path_stability_analysis_set_1.png         → results/plots/로 이동  
smoothness_angular_velocity_set_1.png     → results/plots/로 이동
smoothness_power_spectrum_set_1.png       → results/plots/로 이동
tilt_robustness_Pipeline3_3D_Corridor_set_1.png → results/plots/로 이동
```

#### 2. 테스트 데이터 폴더들 (정리 필요)
```
test_1_p1/                    → experimental_data/test_sessions/로 이동
test_1_p2/                    → experimental_data/test_sessions/로 이동  
test_1_p3/                    → experimental_data/test_sessions/로 이동
pipeline1_stability_data/     → experimental_data/pipeline_data/로 이동
pipeline2_stability_data/     → experimental_data/pipeline_data/로 이동
pipeline3_stability_data/     → experimental_data/pipeline_data/로 이동
```

#### 3. 임시/중복 파일들 (삭제 가능)
```
a.out                         → 삭제 (컴파일된 실행파일)
.vscode/                      → 개인 설정, 필요시 .gitignore에 추가
build/ (일부)                 → 빌드 결과물, 필요시 재생성 가능
install/ (일부)               → 설치 결과물, 필요시 재생성 가능
log/                          → 오래된 로그들, 정기적 정리 필요
```

#### 4. 문서들 (정리 가능)
```
NEW_ALGORITHM_EXPERIMENT_GUIDE.md → docs/experiment_guides/로 이동
EOL/ 폴더                     → 내용 확인 후 docs/ 또는 삭제
```

### 유지해야 할 중요 파일들

#### 핵심 ROS 패키지들
```
cpp_package/                  → 핵심 C++ 노드들
optical_cane_rpi/             → 핵심 Python 노드들  
mpu9250/                      → IMU 드라이버
sllidar_ros2/                 → LiDAR 드라이버
haptic_controller_platformio/ → 햅틱 컨트롤러 (보류 중이지만 유지)
uros/                         → micro-ROS 관련
```

#### 중요 설정/도구 파일들
```
OPTICAL_CANE_KB.md            → 핵심 지식베이스
WORK_STATUS_SUMMARY.md        → 현재 상황 정리
plot_analysis.py              → 주요 분석 도구
live_plotter.py               → 실시간 시각화 도구
enhanced_analysis.py          → 확장 분석 도구
calib/                        → IMU 캘리브레이션 데이터
simple_aliases.txt            → 실행 도구
new_algorithm_aliases.sh      → 새 알고리즘 도구
ros2.repos                    → 저장소 설정
```


## 📝 내일 작업 순서 (우선순위)

### 1순위: HeightMap 원통 지속성 구현
```cpp
// sweep_cloud_cpp 방식 참고하여 시간 누적 구현
std::deque<HeightMapFrame> height_history_;
struct HeightMapFrame {
    rclcpp::Time timestamp;
    std::unordered_map<std::string, HeightCell> cells;
};
```

### 2순위: 높이 반영 수정
```cpp
// 실제 높이 그대로 사용
risk_marker.pose.position.z = cell.mean_z;  // 포인트 실제 높이
risk_marker.scale.z = 0.1;  // 고정 높이 또는 위험도 반영
```

### 3순위: 크기 및 성능 조정
```cpp
// 격자 크기 조정 (성능 vs 정밀도)
this->declare_parameter<double>("grid_resolution", 0.05);  // 5cm

// 마커 크기 증가 (가시성)
height_marker.scale.x = 0.03;  // 3cm 구체
risk_marker.scale.x = 0.04;    // 4cm 원통
```

### 4순위: FTG-3D 시각화 개선
- 갭 마커 크기/색상 조정
- 화살표 시각화 개선

## 🎯 Gemini와 논의할 주제들

1. **새로운 알고리즘 성능 평가**
   - HeightMap 2.5D vs FTG-3D 비교
   - 기존 3개 파이프라인과의 성능 비교

2. **향후 개발 방향**
   - Follow the Gap, VFH 알고리즘 구현 계획
   - 지면 평면 분할 검증 방법

3. **10회 어깨 장착 테스트 시리즈 준비**
   - 테스트 환경 및 시나리오 설계
   - 데이터 수집 및 분석 방법론

## ⚠️ 주의사항

1. **빌드 승인 필요**: 코드 수정 후 반드시 빌드 승인 받고 테스트
2. **백업**: 현재 작동하는 코드 백업 후 수정
3. **단계적 검증**: 한 번에 모든 걸 바꾸지 말고 하나씩 검증
4. **데이터 기반**: 모든 변경사항을 정량적으로 측정

## 📂 제안 폴더 구조 정리

```
ros2_ws/src/
├── packages/           # 핵심 ROS 패키지들
│   ├── cpp_package/
│   ├── optical_cane_rpi/
│   └── mpu9250/
├── docs/              # 문서들
│   ├── OPTICAL_CANE_KB.md
│   └── development_notes/
├── experimental_data/ # 실험 데이터
│   ├── test_sessions/
│   └── pipeline_data/
├── analysis_tools/    # 분석 도구들
│   ├── plot_analysis.py
│   └── live_plotter.py
├── scripts/          # 실행 스크립트
│   ├── aliases/
│   └── automation/
└── results/          # 분석 결과
    ├── plots/
    └── reports/
```

---
**작성:** 2025년 8월 20일 저녁  
**다음 작업자:** Gemini CLI 또는 GitHub Copilot  
**상태:** HeightMap 원통 지속성 구현 대기 중
