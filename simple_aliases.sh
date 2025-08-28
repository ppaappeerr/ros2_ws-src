#!/bin/bash
# 간단한 rosbag 기록 alias (한 줄로 깔끔하게)

echo "🚀 간단한 실험 alias 설정..."

# 실험 디렉토리
EXP_DIR="exp_$(date +%m%d_%H%M)"
mkdir -p "$EXP_DIR"
echo "📁 디렉토리: $EXP_DIR"

# 플래너 실행
alias up1='ros2 run cpp_package unified_path_planner_2d'
alias up2='ros2 run cpp_package unified_scan_accumulator_planner'
alias up3='ros2 run cpp_package unified_path_planner_3d_corridor'
alias up4='ros2 run cpp_package unified_ftg_3d_planner'  
alias up5='ros2 run cpp_package unified_heightmap_planner'

# 간단한 rosbag 기록 (핵심 토픽만)
alias bag1="ros2 bag record -o ${EXP_DIR}/up1 /safe_path_vector /sweep_cloud_cpp"
alias bag2="ros2 bag record -o ${EXP_DIR}/up2 /safe_path_vector /scan_accumulation_cloud"
alias bag3="ros2 bag record -o ${EXP_DIR}/up3 /safe_path_vector /downsampled_cloud"
alias bag4="ros2 bag record -o ${EXP_DIR}/up4 /safe_path_vector /downsampled_cloud /ftg_gaps"
alias bag5="ros2 bag record -o ${EXP_DIR}/up5 /safe_path_vector /downsampled_cloud /height_pillars"

# 디버그 포함 버전 (더 많은 토픽)
alias bagd1="ros2 bag record -o ${EXP_DIR}/up1_debug /safe_path_vector /sweep_cloud_cpp /path_planner_debug /imu/data"
alias bagd2="ros2 bag record -o ${EXP_DIR}/up2_debug /safe_path_vector /scan_accumulation_cloud /path_planner_debug /imu/data" 
alias bagd3="ros2 bag record -o ${EXP_DIR}/up3_debug /safe_path_vector /downsampled_cloud /path_planner_debug /imu/data"
alias bagd4="ros2 bag record -o ${EXP_DIR}/up4_debug /safe_path_vector /downsampled_cloud /ftg_gaps /path_planner_debug /imu/data"
alias bagd5="ros2 bag record -o ${EXP_DIR}/up5_debug /safe_path_vector /downsampled_cloud /height_pillars /path_planner_debug /imu/data"

echo ""
echo "✅ 설정 완료!"
echo ""
echo "사용법:"
echo "  플래너: up1, up2, up3, up4, up5"
echo "  기록:   bag1, bag2, bag3, bag4, bag5"
echo "  디버그: bagd1, bagd2, bagd3, bagd4, bagd5"
echo ""
echo "예시:"
echo "  up1      # UP1 플래너 실행"
echo "  bag1     # UP1 데이터 기록"
echo "  bagd1    # UP1 디버그 포함 기록"