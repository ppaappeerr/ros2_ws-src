# 기술적 이슈 및 해결 방안

## 1. 왼쪽 마스킹으로 인한 편향 문제

### 현상
- 얼굴 마스킹으로 왼쪽 정보 부족 → 왼쪽 레이가 비정상적으로 긴 depth
- 'ㄷ'자 구간에서 우회전해야 하는데 왼쪽 대각선으로 유도

### 해결 방안
1. **Evidence 기반 스코어 조정**: 실제 포인트 수가 적은 레이는 신뢰도 감소
2. **Mirror 패널티**: 좌우 대칭 위치 레이 간 depth 차이가 클 때 편향된 쪽에 패널티
3. **마운트 위치 최적화**: 얼굴에서 최대한 앞으로 위치 조정

## 2. 알고리즘별 특정 문제

### p3 (3D Corridor) - 미해결
- **문제**: 지속적인 +90도 스냅
- **원인**: Edge penalty 미적용
- **해결**: path_planner_node와 동일한 개선사항 적용 필요

### p4 (Follow-The-Gap 3D)
- **문제**: 갭 갱신 지연, 가끔 ±90도 튐
- **원인**: Sweep cloud 잔상 영향
- **해결**: 동적 누적 시간 적용 + gap 선택 안정화

### p5 (HeightMap 2.5D) - 심각
- **문제**: 센서 근처 점으로 정면 막힌 것으로 판단, 과도하게 느린 회전
- **원인**: Dead-zone이 HeightMap에는 적용되지 않음 + decay 설정 부적절
- **해결**: Dead-zone 적용 + decay tau 재조정

## 3. 회전 속도 불일치 문제

### 현상
- p1: 너무 빨라서 급격한 변화
- p5: 너무 느려서 실시간 대응 불가
- p2: 적절함

### 통일된 해결 방안
- 상황별 적응형 각속도: `ω = ω_base × (1 + α × urgency_factor)`
- urgency_factor = f(장애물 거리, 편향 정도, 환경 복잡도)

## 4. 실험 설계 요소

### 4.1 마운트 각도 변수
- 현재: 얼굴 오른쪽 (baseline)
- 테스트: 5, 10, 15, 20, 25도 pitch 각도
- 각 각도별 Dead-zone 파라미터 조정 필요

### 4.2 성능 평가 지표
1. **안정성**: Path angle standard deviation
2. **반응성**: Angular velocity RMS  
3. **안전성**: 장애물 회피 성공률
4. **편향성**: Left/right bias ratio
5. **연속성**: Direction reversal count

### 4.3 rosbag 데이터 수집
```bash
# 모든 실험에서 기록할 공통 토픽
/safe_path_vector     # 최종 경로 출력
/imu/data            # IMU 데이터  
/tf, /tf_static      # 좌표 변환
/scan               # 원시 LiDAR 데이터

# 알고리즘별 추가 토픽
p1,p3: /sweep_cloud_cpp
p2: /scan_accumulation_cloud  
p4: /ftg_gaps
p5: /height_pillars
```

## 5. 다음 단계 우선순위

1. **즉시 수정**: p3, p5 코드 개선 (p1과 동일한 로직 적용)
2. **실험 준비**: 5×5 매트릭스 실험 자동화 스크립트
3. **데이터 분석**: rosbag 기반 성능 지표 자동 계산
4. **최적화**: 부착 각도별 Dead-zone 파라미터 자동 조정
5. **통합**: 최적 알고리즘 + 각도 조합으로 햅틱 피드백 연결