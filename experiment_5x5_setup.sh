#!/bin/bash
# 5×5 실험 매트릭스 자동화 스크립트

# 현재 디렉토리에서 실행되도록 수정
cd ~/ros2_ws/src

# 실험 디렉토리 생성
EXPERIMENT_BASE="experiments_5x5_$(date +%Y%m%d_%H%M)"
mkdir -p "$EXPERIMENT_BASE"

echo "🧪 5×5 실험 매트릭스 설정"
echo "실험 디렉토리: $EXPERIMENT_BASE"
echo ""

# 공통 토픽들
COMMON_TOPICS="/safe_path_vector /imu/data /tf /tf_static /scan"

# 플래너별 추가 토픽 정의
declare -A PLANNER_TOPICS
PLANNER_TOPICS["up1"]="${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug"
PLANNER_TOPICS["up2"]="${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"
PLANNER_TOPICS["up3"]="${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"
PLANNER_TOPICS["up4"]="${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"
PLANNER_TOPICS["up5"]="${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"

# 각 플래너별로 5회 반복 실험용 alias 생성
for planner in up1 up2 up3 up4 up5; do
    for trial in {1..5}; do
        alias_name="b${planner}_${trial}"
        bag_name="${planner}_trial_${trial}"
        bag_path="${EXPERIMENT_BASE}/${bag_name}"
        
        # alias 정의
        alias_cmd="alias ${alias_name}='ros2 bag record -o ${bag_path} ${PLANNER_TOPICS[$planner]}'"
        echo "$alias_cmd"
        eval "$alias_cmd"
        
        # 시간 제한 버전 (90초)
        alias_timed="${alias_name}t"
        alias_timed_cmd="alias ${alias_timed}='ros2 bag record -o ${bag_path} --max-bag-duration 90 ${PLANNER_TOPICS[$planner]}'"
        echo "$alias_timed_cmd"
        eval "$alias_timed_cmd"
    done
    echo ""
done

# 분석 스크립트 alias
analysis_cmd="alias analyze_5x5='python3 unified_planner_analysis.py --bag_dir ${EXPERIMENT_BASE} --output_dir ${EXPERIMENT_BASE}/analysis'"
echo "$analysis_cmd"
eval "$analysis_cmd"

echo ""
echo "✅ 5×5 실험 alias 설정 완료!"
echo ""
echo "📋 사용법:"
echo "  UP1 실험:"
echo "    bup1_1  - UP1 첫 번째 시도"
echo "    bup1_2  - UP1 두 번째 시도"
echo "    ..."
echo "    bup1_5  - UP1 다섯 번째 시도"
echo ""
echo "  시간 제한 버전 (90초):"
echo "    bup1_1t - UP1 첫 번째 시도 (90초 자동 종료)"
echo ""
echo "  전체 분석:"
echo "    analyze_5x5"
echo ""
echo "📁 실험 파일 위치: $EXPERIMENT_BASE"
echo ""

# 실험 진행 상황 추적 파일 생성
cat > "${EXPERIMENT_BASE}/experiment_checklist.md" << EOF
# 5×5 실험 체크리스트

## 실험 진행 상황

### UP1 (2D 투영)
- [ ] Trial 1: \`bup1_1\` 
- [ ] Trial 2: \`bup1_2\`
- [ ] Trial 3: \`bup1_3\`
- [ ] Trial 4: \`bup1_4\`
- [ ] Trial 5: \`bup1_5\`

### UP2 (순수 2D)
- [ ] Trial 1: \`bup2_1\`
- [ ] Trial 2: \`bup2_2\`
- [ ] Trial 3: \`bup2_3\`
- [ ] Trial 4: \`bup2_4\`
- [ ] Trial 5: \`bup2_5\`

### UP3 (3D Corridor)
- [ ] Trial 1: \`bup3_1\`
- [ ] Trial 2: \`bup3_2\`
- [ ] Trial 3: \`bup3_3\`
- [ ] Trial 4: \`bup3_4\`
- [ ] Trial 5: \`bup3_5\`

### UP4 (Follow-the-Gap)
- [ ] Trial 1: \`bup4_1\`
- [ ] Trial 2: \`bup4_2\`
- [ ] Trial 3: \`bup4_3\`
- [ ] Trial 4: \`bup4_4\`
- [ ] Trial 5: \`bup4_5\`

### UP5 (HeightMap)
- [ ] Trial 1: \`bup5_1\`
- [ ] Trial 2: \`bup5_2\`
- [ ] Trial 3: \`bup5_3\`
- [ ] Trial 4: \`bup5_4\`
- [ ] Trial 5: \`bup5_5\`

## 실험 후 분석
\`\`\`bash
analyze_5x5
\`\`\`

## 파일 구조
\`\`\`
${EXPERIMENT_BASE}/
├── up1_trial_1/
├── up1_trial_2/
├── ...
├── up5_trial_5/
└── analysis/
    ├── detailed_results.csv
    ├── summary_stats.csv
    ├── performance_comparison.png
    └── planner_rankings.csv
\`\`\`
EOF

echo "📝 실험 체크리스트: ${EXPERIMENT_BASE}/experiment_checklist.md"