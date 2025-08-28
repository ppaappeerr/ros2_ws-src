#!/bin/bash
# 통일된 플래너 실험용 alias (UP1, UP2, UP3, UP5 비교용)

# 플래너 실행 alias
alias up1='ros2 run cpp_package unified_path_planner_2d'
alias up2='ros2 run cpp_package unified_scan_accumulator_planner'
alias up3='ros2 run cpp_package unified_path_planner_3d_corridor'
alias up5='ros2 run cpp_package unified_heightmap_planner'

# UP1 기록 (3D→2D 사영, scan_accumulation_cloud로 통일)
alias bag1_1='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up1_trial_1 /safe_path_vector /path_planner_debug'
alias bag1_2='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up1_trial_2 /safe_path_vector /path_planner_debug'
alias bag1_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up1_trial_3 /safe_path_vector /path_planner_debug'
alias bag1_4='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up1_trial_4 /safe_path_vector /path_planner_debug'
alias bag1_5='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up1_trial_5 /safe_path_vector /path_planner_debug'

# UP2 기록 (순수 2D 스캔 누적)
alias bag2_1='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up2_trial_1 /safe_path_vector /path_planner_debug'
alias bag2_2='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up2_trial_2 /safe_path_vector /path_planner_debug'
alias bag2_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up2_trial_3 /safe_path_vector /path_planner_debug'
alias bag2_4='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up2_trial_4 /safe_path_vector /path_planner_debug'
alias bag2_5='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up2_trial_5 /safe_path_vector /path_planner_debug'

# UP3 기록 (3D Corridor)
alias bag3_1='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_1 /safe_path_vector /path_planner_debug'
alias bag3_2='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_2 /safe_path_vector /path_planner_debug'
alias bag3_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_3 /safe_path_vector /path_planner_debug'
alias bag3_4='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_4 /safe_path_vector /path_planner_debug'
alias bag3_5='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_5 /safe_path_vector /path_planner_debug'

# UP5 기록 (HeightMap 플래너)
alias bag5_1='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_1 /safe_path_vector /path_planner_debug'
alias bag5_2='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_2 /safe_path_vector /path_planner_debug'
alias bag5_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_3 /safe_path_vector /path_planner_debug'
alias bag5_4='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_4 /safe_path_vector /path_planner_debug'
alias bag5_5='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_5 /safe_path_vector /path_planner_debug'

# 분석 명령
alias anal='python3 /home/p/ros2_ws/src/simple_bag_analysis.py --bag_dir /home/p/ros2_ws/src/rosbags_5x5 --output_dir /home/p/ros2_ws/src/rosbags_5x5/analysis'

# P5만 재실험용 (망친 데이터 덮어쓰기)
alias rebag5_1='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_1 /safe_path_vector /path_planner_debug'
alias rebag5_2='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_2 /safe_path_vector /path_planner_debug'
alias rebag5_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_3 /safe_path_vector /path_planner_debug'
alias rebag5_4='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_4 /safe_path_vector /path_planner_debug'
alias rebag5_5='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up5_trial_5 /safe_path_vector /path_planner_debug'

# 잘못 저장된 파일명 수정용
alias fixbag3_3='ros2 bag record -s sqlite3 -o /home/p/ros2_ws/src/rosbags_5x5/up3_trial_3 /safe_path_vector /path_planner_debug'

echo "✅ 통일된 플래너 alias 로드 완료 (UP1, UP2, UP3, UP5)"
echo "   UP1: 3D→2D Projection + 2D ray-casting"
echo "   UP2: Pure 2D scan accumulation + ray-casting" 
echo "   UP3: 3D corridor method"
echo "   UP5: HeightMap planner (height info utilization)"
echo "🔄 P5 재실험: rebag5_1 ~ rebag5_5 (덮어쓰기)"
echo "🔧 UP3 파일명 수정: fixbag3_3 (up4_trial_3 → up3_trial_3)"
