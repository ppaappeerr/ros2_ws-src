#!/bin/bash
# 간단한 실험 alias 설정

echo "🚀 통일된 플래너 실험 alias 설정 중..."

# 현재 날짜/시간으로 실험 디렉토리
EXPERIMENT_DIR="experiment_$(date +%m%d_%H%M)"
mkdir -p "$EXPERIMENT_DIR"
echo "📁 실험 디렉토리: $EXPERIMENT_DIR"

# 공통 토픽 (공백 제거)
COMMON_TOPICS="/safe_path_vector /imu/data /tf /tf_static /scan"

# UP1-UP5 실험용 rosbag 기록 (간단 버전)
alias bup1_1="ros2 bag record -o ${EXPERIMENT_DIR}/up1_trial_1 /safe_path_vector /imu/data /tf /tf_static /scan /sweep_cloud_cpp /path_planner_debug"
alias bup1_2="ros2 bag record -o ${EXPERIMENT_DIR}/up1_trial_2 ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug"
alias bup1_3="ros2 bag record -o ${EXPERIMENT_DIR}/up1_trial_3 ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug"
alias bup1_4="ros2 bag record -o ${EXPERIMENT_DIR}/up1_trial_4 ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug"
alias bup1_5="ros2 bag record -o ${EXPERIMENT_DIR}/up1_trial_5 ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug"

alias bup2_1="ros2 bag record -o ${EXPERIMENT_DIR}/up2_trial_1 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"
alias bup2_2="ros2 bag record -o ${EXPERIMENT_DIR}/up2_trial_2 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"
alias bup2_3="ros2 bag record -o ${EXPERIMENT_DIR}/up2_trial_3 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"
alias bup2_4="ros2 bag record -o ${EXPERIMENT_DIR}/up2_trial_4 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"
alias bup2_5="ros2 bag record -o ${EXPERIMENT_DIR}/up2_trial_5 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug"

alias bup3_1="ros2 bag record -o ${EXPERIMENT_DIR}/up3_trial_1 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"
alias bup3_2="ros2 bag record -o ${EXPERIMENT_DIR}/up3_trial_2 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"
alias bup3_3="ros2 bag record -o ${EXPERIMENT_DIR}/up3_trial_3 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"
alias bup3_4="ros2 bag record -o ${EXPERIMENT_DIR}/up3_trial_4 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"
alias bup3_5="ros2 bag record -o ${EXPERIMENT_DIR}/up3_trial_5 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug"

alias bup4_1="ros2 bag record -o ${EXPERIMENT_DIR}/up4_trial_1 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"
alias bup4_2="ros2 bag record -o ${EXPERIMENT_DIR}/up4_trial_2 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"
alias bup4_3="ros2 bag record -o ${EXPERIMENT_DIR}/up4_trial_3 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"
alias bup4_4="ros2 bag record -o ${EXPERIMENT_DIR}/up4_trial_4 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"
alias bup4_5="ros2 bag record -o ${EXPERIMENT_DIR}/up4_trial_5 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps"

alias bup5_1="ros2 bag record -o ${EXPERIMENT_DIR}/up5_trial_1 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"
alias bup5_2="ros2 bag record -o ${EXPERIMENT_DIR}/up5_trial_2 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"
alias bup5_3="ros2 bag record -o ${EXPERIMENT_DIR}/up5_trial_3 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"
alias bup5_4="ros2 bag record -o ${EXPERIMENT_DIR}/up5_trial_4 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"
alias bup5_5="ros2 bag record -o ${EXPERIMENT_DIR}/up5_trial_5 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars"

# 플래너 실행 alias
alias up1='ros2 run cpp_package unified_path_planner_2d'
alias up2='ros2 run cpp_package unified_scan_accumulator_planner'
alias up3='ros2 run cpp_package unified_path_planner_3d_corridor'
alias up4='ros2 run cpp_package unified_ftg_3d_planner'
alias up5='ros2 run cpp_package unified_heightmap_planner'

# 분석 명령
alias analyze_experiment="python3 ~/ros2_ws/src/unified_planner_analysis.py --bag_dir ${EXPERIMENT_DIR} --output_dir ${EXPERIMENT_DIR}/analysis"

echo ""
echo "✅ Alias 설정 완료!"
echo ""
echo "📋 사용 방법:"
echo "  플래너 실행: up1, up2, up3, up4, up5"
echo "  데이터 기록: bup1_1, bup1_2, ..., bup5_5"
echo "  분석 실행: analyze_experiment"
echo ""
echo "📁 실험 파일은 여기에 저장됩니다: $EXPERIMENT_DIR"