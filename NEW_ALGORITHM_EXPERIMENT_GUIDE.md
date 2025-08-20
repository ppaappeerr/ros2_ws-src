# 🚀 새로운 알고리즘 실험 가이드

## 개요

이 문서는 새롭게 구현된 3D 인식 경로계획 알고리즘들의 실험 방법을 설명합니다.

### 새로운 알고리즘
1. **HeightMap 2.5D Planner**: 높이 맵 기반 음의 장애물 탐지
2. **Follow-the-Gap 3D (FTG-3D)**: 3D 섹터 기반 갭 탐지

## 🛠️ 초기 설정

### 1단계: 환경 준비
```bash
cd ~/ros2_ws
source install/setup.bash
source src/new_algorithm_aliases.sh
```

### 2단계: 빌드 확인
```bash
# 필요시 재빌드
build_new
```

## 🎯 빠른 시작

### 기본 테스트 (추천)
```bash
# 1. HeightMap 2.5D 알고리즘 시작 (시각화 포함)
new_heightmap_viz

# 2. 다른 터미널에서 데이터 기록
d4

# 3. 실험 후 분석
analyze_new
```

### 전체 실험 (자동화)
```bash
# 짧은 테스트 (약 2분)
quick_test

# 전체 비교 실험 (약 30분)
full_experiment
```

## 📋 단계별 실험 가이드

### 실험 A: HeightMap 2.5D 알고리즘

**목적**: 음의 장애물(맨홀, 낙하 지점) 탐지 성능 평가

#### A1. 수동 실험
```bash
# 터미널 1: 알고리즘 실행 (RViz 포함)
new_heightmap_viz

# 터미널 2: 데이터 기록 시작
record_negative_heightmap

# 실험 진행:
# 1. 평지 이동 (10초)
# 2. 맨홀이나 계단 근처 이동 (20초) 
# 3. 일반 장애물 근처 이동 (10초)

# 터미널 2: 기록 종료 (Ctrl+C)
```

#### A2. 자동화 실험
```bash
# 음의 장애물 특화 테스트
negative_obstacle_test
```

### 실험 B: FTG-3D 알고리즘

**목적**: 3D 갭 탐지 및 연속적 경로계획 성능 평가

#### B1. 수동 실험  
```bash
# 터미널 1: 알고리즘 실행
new_ftg3d_viz

# 터미널 2: 데이터 기록
record_straight_ftg3d

# 실험 진행:
# 1. 직진 복도 (15초)
# 2. 좁은 통로 (15초)
# 3. 복잡한 장애물 환경 (15초)
```

### 실험 C: 비교 실험

**목적**: 기존 알고리즘 대비 성능 비교

```bash
# 전체 알고리즘 비교 (자동화)
full_experiment

# 결과 분석
analyze_experiments
```

## 📊 데이터 분석

### 기본 분석
```bash
# 새로운 통합 분석 도구
analyze_new

# 기존 분석 도구 (호환)
d0
```

### 고급 분석
실험 데이터는 다음 위치에 저장됩니다:
- `/home/p/ros2_ws/experimental_data/` - 실험 데이터
- `/home/p/ros2_ws/analysis_results/` - 분석 결과

분석 결과로 생성되는 파일들:
- `safety_metrics_summary.json` - 안전성 지표
- `performance_comparison.png` - 성능 비교 차트
- `experimental_dashboard.html` - 대화형 대시보드

## 🔬 실험 시나리오

### 시나리오 1: 직진 복도 (Straight)
- **환경**: 넓은 직진 복도
- **목적**: 기본 성능 측정
- **측정 지표**: 경로 안정성, 진동 최소화

### 시나리오 2: 모서리/코너 (Corner)  
- **환경**: 90도 회전 지점
- **목적**: 회전 성능 평가
- **측정 지표**: 회전 부드러움, 각속도 안정성

### 시나리오 3: 좁은 통로 (Narrow)
- **환경**: 폭 1.5m 이하 통로
- **목적**: 정밀 제어 성능
- **측정 지표**: 중앙 유지, 안전 거리

### 시나리오 4: 장애물 환경 (Obstacle)
- **환경**: 다중 장애물 배치
- **목적**: 복잡한 환경 대응
- **측정 지표**: 충돌 회피, 경로 효율성

### 시나리오 5: 음의 장애물 (Negative) ⭐
- **환경**: 맨홀, 계단, 낙하 지점
- **목적**: 음의 장애물 탐지 성능
- **측정 지표**: 탐지 정확도, 안전 거리 확보

## 📈 성능 지표

### 안전성 지표
- **Min Distance**: 최소 장애물 거리
- **Collision Events**: 충돌 위험 횟수  
- **Near Miss Events**: 위험 근접 횟수
- **Negative Obstacle Detection**: 음의 장애물 탐지율

### 경로 품질 지표
- **Path Smoothness**: 경로 부드러움
- **Direction Changes**: 방향 변화 횟수
- **Angular Velocity Stability**: 각속도 안정성
- **Power Spectrum**: 주파수 분석

### 효율성 지표
- **Computational Load**: 연산 부하
- **Memory Usage**: 메모리 사용량
- **Response Time**: 응답 시간

## 🚨 문제 해결

### 일반적인 문제

#### 노드가 시작되지 않음
```bash
# ROS 프로세스 정리
kill_all_ros

# 로그 정리  
clean_logs

# 재빌드
build_new
source_new
```

#### 센서 데이터가 없음
```bash
# 센서 상태 확인
ros2 topic list | grep -E "(imu|cloud|lidar)"

# 센서 노드 재시작
a1  # optical_cane 런치
```

#### 시각화가 표시되지 않음
```bash
# RViz 설정 확인
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/cpp_package/launch/optical_cane_visualization.rviz
```

### 성능 튜닝

#### HeightMap 파라미터 조정
- `grid_resolution`: 그리드 해상도 (기본: 0.1m)
- `height_threshold`: 높이 임계값 (기본: 0.15m)  
- `risk_factor`: 위험도 가중치 (기본: 2.0)

#### FTG-3D 파라미터 조정
- `num_sectors`: 섹터 개수 (기본: 36)
- `max_gap_width`: 최대 갭 너비 (기본: 1.5m)
- `centrality_bonus`: 중앙 선호도 (기본: 0.3)

## 📝 실험 체크리스트

### 실험 전
- [ ] 센서 보정 완료
- [ ] 배터리 충전 상태 확인
- [ ] 실험 환경 안전 점검
- [ ] 데이터 저장 공간 확인

### 실험 중
- [ ] 실시간 모니터링 (RViz)
- [ ] 데이터 기록 상태 확인
- [ ] 예상치 못한 동작 시 즉시 중단

### 실험 후
- [ ] 데이터 백업
- [ ] 로그 파일 검토
- [ ] 분석 결과 문서화
- [ ] 다음 실험 계획 수립

## 🎯 다음 단계

1. **기본 실험**: `quick_test`로 기본 성능 확인
2. **심화 실험**: 시나리오별 상세 분석
3. **매개변수 최적화**: 성능 튜닝 실험
4. **실제 환경 테스트**: 다양한 실제 환경에서 검증

---

**💡 팁**: 실험 중 문제가 발생하면 `help_new` 명령어로 사용 가능한 도구들을 확인하세요.

**🔗 관련 파일**:
- 소스 코드: `~/ros2_ws/src/cpp_package/src/`
- 실험 스크립트: `~/ros2_ws/src/cpp_package/scripts/`
- 분석 도구: `~/ros2_ws/src/enhanced_analysis.py`
