#!/bin/bash
# 통일된 플래너 실험용 rosbag 기록 alias

# 공통 토픽들 (모든 실험에서 기록)
COMMON_TOPICS="/safe_path_vector /imu/data /tf /tf_static /scan"

# 실험별 rosbag 기록 alias
echo "통일된 플래너 rosbag 기록 alias 설정 중..."

# UP1 (2D 투영) - sweep_cloud_cpp 사용
alias b1='ros2 bag record -o up1_experiment ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug'

# UP2 (순수 2D) - scan_accumulation_cloud 사용  
alias b2='ros2 bag record -o up2_experiment ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug'

# UP3 (3D Corridor) - downsampled_cloud 사용
alias b3='ros2 bag record -o up3_experiment ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug'

# UP4 (Follow-the-Gap) - downsampled_cloud + ftg_gaps
alias b4='ros2 bag record -o up4_experiment ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps'

# UP5 (HeightMap) - downsampled_cloud + height_pillars
alias b5='ros2 bag record -o up5_experiment ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars'

# 비교용 기존 플래너들
alias bp1='ros2 bag record -o p1_legacy ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug'
alias bp2='ros2 bag record -o p2_legacy ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug'
alias bp3='ros2 bag record -o p3_legacy ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug'
alias bp4='ros2 bag record -o p4_legacy ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps'
alias bp5='ros2 bag record -o p5_legacy ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars'

# 시간 기반 자동 종료 (90초 실험)
alias b1t='ros2 bag record -o up1_timed --max-bag-duration 90 ${COMMON_TOPICS} /sweep_cloud_cpp /path_planner_debug'
alias b2t='ros2 bag record -o up2_timed --max-bag-duration 90 ${COMMON_TOPICS} /scan_accumulation_cloud /path_planner_debug'
alias b3t='ros2 bag record -o up3_timed --max-bag-duration 90 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug'
alias b4t='ros2 bag record -o up4_timed --max-bag-duration 90 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /ftg_gaps'
alias b5t='ros2 bag record -o up5_timed --max-bag-duration 90 ${COMMON_TOPICS} /downsampled_cloud /path_planner_debug /height_pillars'

echo "✅ Rosbag alias 설정 완료!"
echo ""
echo "사용법:"
echo "  b1    - UP1 무제한 기록"  
echo "  b1t   - UP1 30초 기록"
echo "  bp1   - 기존 P1 비교용"
echo ""
echo "분석하려면:"
echo "  python3 unified_planner_analysis.py --bag_dir ./rosbags --output_dir ./analysis"