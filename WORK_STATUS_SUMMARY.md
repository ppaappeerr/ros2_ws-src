# WORK_STATUS_SUMMARY (Auto-generated)

## 1. 현재 포지션 (어깨 장착 전환 직후 안정화 단계)
- 전방 기준: 전역 Front = -X (sensor_fusion 포함 전 노드 통일 필요 → 이번 커밋으로 sensor_fusion 반영 완료)
- 5 알고리즘 모드 (A~E) 중 비교 실험(5x5) 착수 전 사전 교정 단계
- 핵심 문제: 어깨 장착 후 데이터 분포 변화(하향 피치, 좌측/신체 차폐, 근거리 잡음)로 FTG/HeightMap 불안정, Corridor 반응 저하

## 2. 금회 수행 작업
- sensor_fusion_node 확장:
  - 파라미터 추가: front_is_negative_x, use_roll_pitch_only, yaw_offset_deg, max_roi_distance, dead_zone_box, min_range_clip
  - Roll/Pitch만 적용 + Yaw 오프셋 + 전방 -X 반전 + ROI(반경) + Dead-zone 박스 필터 + 근거리 클립
  - 목적: 모든 후속 플래너에 깨끗하고 정규화된 전방 데이터 공급

## 3. 즉시 후속 적용 대상 (다음 수정 우선순위)
1) FTG 빈 섹터 차단: min_points_per_sector 파라미터 추가 후 관측 부족 섹터 depth=0 처리
2) HeightMap 반응성: history_duration 2.0→0.8~1.0, sweep window 1.5→0.8 조정 및 drop/obstacle threshold 소폭 상향(잡음 완화)
3) 공통 파라미터 YAML 생성 (shoulder_defaults.yaml) 및 launch에서 include
4) 분석 파이프라인: enhanced_analysis.py → jerk/clearance/left_bias 통합(analysis 노드 or 오프라인) 정규화

## 4. 시험 계획 (사전 단축 테스트 ▶ 정식 5x5)
- 단계 1: A,C,E 소규모 러닝 (각 1회) → ROI/Dead-zone, front 반영 여부 시각 검증
- 단계 2: FTG(D) 적용 후 빈섹터 필터 튜닝 (num_sectors * corridor_width * 환경 포인트 밀도 기준)
- 단계 3: HeightMap(E) 지면 캘리브레이션 대기시간 측정 (정지 2초 vs 1초) 비교
- 단계 4: 안정화 후 5x5 전체 실험 (순서 라틴 스퀘어 혹은 무작위화)

## 5. 선정 지표 (요약)
- 안정성: angle_std_deg, p95_jerk_deg_s ↓
- 부드러움: median_jerk_deg_s ↓
- 안전성: median_clearance_m ↑, drop_detect_events (E 전용) ↑(민감도) vs false_positive_rate ↓
- 일관성: IQR(angle), dropout_ratio ↓

## 6. 결정 경계 (Fail Fast 규칙 예시)
- Drop 검출 전혀 없음 + angle_std_deg > 25° (3회 이상) → 해당 설정 폐기/재튜닝
- FTG: dropout_ratio > 0.15 또는 p95_jerk_deg_s > 140°/s → smoothing/sector 재조정
- HeightMap: false drop rate > 10% (수동 태깅 대비) → threshold 상향 또는 누적창 확장

## 7. 향후 코드 변경 백로그
- ftg_3d_node: counts 벡터 및 min_points_per_sector 파라미터
- heightmap_planner_node: dynamic param (rclcpp::ParameterEvent) 수용하여 threshold 실시간 조정
- 공통 launch: comparative_test.launch.py 에 shoulder_defaults.yaml 로딩
- 분석 스크립트: universal_analyzer / path_vector_analyzer 통합 → batch aggregator

## 8. 하드웨어 연계
- 볼조인트 각도 범위 탐색(5° increments) → yaw_offset_deg 보정 최소화 목표
- Dead-zone 박스 실측 후 파라미터 업데이트 (현재 추정값: [-0.30,0.15,-0.20,0.35])

## 9. 리스크 & 대응
| 리스크 | 영향 | 대응 |
|--------|------|------|
| IMU yaw drift | 전방 정렬 오차 | use_roll_pitch_only + 수동 yaw_offset_deg |
| Left occlusion 변동 | FTG/HeightMap 편향 | dead_zone + min_points_per_sector + 중앙 bias 유지 |
| HeightMap 캘리브레이션 지연 | 초기 경로 null | 시작 정지 2s 안내 및 샘플 count 로그 |
| 과도한 필터로 데이터 희석 | GAP 탐지 실패 | ROI 반경 단계적 축소 (2.5→2.0→1.8) 실험 |

## 10. 다음 액션 (Executable Checklist)
- [ ] ftg_3d_node 섹터 포인트 카운트 패치
- [ ] sweeper window 파라미터 수정 및 launch 반영
- [ ] heightmap history_duration 파라미터 단축 + thresholds 튜닝
- [ ] shoulder_defaults.yaml 작성
- [ ] batch 분석 스크립트 골격 (CSV merge + summary json → md 리포트)
- [ ] 3 모드(ACE) 연습 로그 1회씩 수집 & 시각 검증
- [ ] FTG / HeightMap 튜닝 후 full 5x5 일정 수립

## 11. 결론
센서 융합 단계에서 어깨 장착 특성(전방 -X, roll/pitch 기준 정렬, 근거리 차폐, ROI 단축)을 반영하는 기반 정리가 완료됨. 이제 플래너별 관측 품질 보정(FTG 빈섹터, HeightMap 응답성)과 분석 일관화 작업을 단기 마무리 후 5x5 정량 비교로 단일 알고리즘 선정 단계 진입 예정.

(업데이트: 자동 생성 시각 기준)
