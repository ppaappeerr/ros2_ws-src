#!/bin/bash
# 새로운 알고리즘 실험을 위한 확장된 Alias 시스템
# 사용법: source ~/ros2_ws/src/new_algorithm_aliases.sh

echo "🚀 새로운 알고리즘 Alias 로딩 중..."

# ==============================================================================
# 기존 시스템 (A1-A3, B3-B4, C4-C5) 유지
# ==============================================================================

# 기본 센서 시작
alias a0='ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888'
alias a1='ros2 launch optical_cane_rpi optical_cane.launch.py'
alias a2='ros2 run optical_cane_rpi sensor_fusion_node'
alias a3='ros2 run cpp_package point_cloud_sweeper_cpp_node'

# 기존 파이프라인
alias a4='ros2 run cpp_package path_planner_node'  # 2D 투영
alias b3='ros2 run optical_cane_rpi scan_accumulator_node'  # 순수 2D
alias b4='ros2 run cpp_package path_planner_node --ros-args -r /sweep_cloud_cpp:=/scan_accumulation_cloud'
alias c4='ros2 run cpp_package voxel_grid_filter_node --ros-args -p leaf_size:=0.04'  # 3D 전처리
alias c5='ros2 run cpp_package path_planner_3d_node'  # 3D 복도

# ==============================================================================
# 새로운 알고리즘 (D, E 시리즈)
# ==============================================================================

# D 시리즈: HeightMap 2.5D Planner
alias d_setup='a1 && sleep 2 && a2 && sleep 2 && a3 && sleep 2 && c4'  # 전처리 파이프라인
alias d5='ros2 run cpp_package heightmap_planner_node'  # HeightMap 플래너 실행
alias d_full='echo "🏔️  HeightMap 2.5D 파이프라인 시작..." && (a1 & sleep 2 && a2 & sleep 2 && a3 & sleep 2 && c4 & sleep 3 && d5)'

# E 시리즈: Follow-the-Gap 3D
alias e5='ros2 run cpp_package ftg_3d_node'  # FTG-3D 플래너 실행
alias e_full='echo "🎯 FTG-3D 파이프라인 시작..." && (a1 & sleep 2 && a2 & sleep 2 && a3 & sleep 2 && c4 & sleep 3 && e5)'

# ==============================================================================
# 통합 실행 (Launch 파일 기반)
# ==============================================================================

# 한 방에 실행
alias new_heightmap='ros2 launch cpp_package new_algorithms.launch.py pipeline:=heightmap'
alias new_ftg3d='ros2 launch cpp_package new_algorithms.launch.py pipeline:=ftg3d' 
alias new_both='ros2 launch cpp_package new_algorithms.launch.py pipeline:=both'

# RViz 포함
alias new_heightmap_viz='ros2 launch cpp_package new_algorithms.launch.py pipeline:=heightmap use_rviz:=true'
alias new_ftg3d_viz='ros2 launch cpp_package new_algorithms.launch.py pipeline:=ftg3d use_rviz:=true'

# ==============================================================================
# 확장된 데이터 수집 Alias
# ==============================================================================

# 기존 d1-d3 유지
alias d1='ros2 bag record -o test_1_p1 /imu/data /safe_path_vector /tf /tf_static /sweep_cloud_cpp'
alias d2='ros2 bag record -o test_1_p2 /imu/data /safe_path_vector /tf /tf_static /scan_accumulation_cloud'
alias d3='ros2 bag record -o test_1_p3 /imu/data /safe_path_vector_3d /tf /tf_static /downsampled_cloud'

# 새로운 알고리즘용 데이터 수집
alias d4='ros2 bag record -o test_1_p4_heightmap /imu/data /safe_path_vector_heightmap /tf /tf_static /downsampled_cloud /height_map_markers /risk_map_markers'
alias d5='ros2 bag record -o test_1_p5_ftg3d /imu/data /safe_path_vector_ftg3d /tf /tf_static /downsampled_cloud /gap_markers /sector_markers'

# 시나리오별 데이터 수집 (자동 명명)
alias record_straight_heightmap='ros2 bag record -o straight_heightmap_$(date +%Y%m%d_%H%M%S) /imu/data /safe_path_vector_heightmap /tf /tf_static /downsampled_cloud /height_map_markers /risk_map_markers'
alias record_negative_heightmap='ros2 bag record -o negative_heightmap_$(date +%Y%m%d_%H%M%S) /imu/data /safe_path_vector_heightmap /tf /tf_static /downsampled_cloud /height_map_markers /risk_map_markers'
alias record_straight_ftg3d='ros2 bag record -o straight_ftg3d_$(date +%Y%m%d_%H%M%S) /imu/data /safe_path_vector_ftg3d /tf /tf_static /downsampled_cloud /gap_markers /sector_markers'
alias record_negative_ftg3d='ros2 bag record -o negative_ftg3d_$(date +%Y%m%d_%H%M%S) /imu/data /safe_path_vector_ftg3d /tf /tf_static /downsampled_cloud /gap_markers /sector_markers'

# ==============================================================================
# 분석 도구 Alias
# ==============================================================================

# 기존 분석 (d0 유지)
alias d0='python3 ~/ros2_ws/src/plot_analysis.py --test_set 1'

# 새로운 확장 분석
alias analyze_new='python3 ~/ros2_ws/src/enhanced_analysis.py'
alias analyze_experiments='python3 ~/ros2_ws/src/enhanced_analysis.py --experiment_dir ~/ros2_ws/experimental_data --output_dir ~/ros2_ws/analysis_results'

# ==============================================================================
# 자동화된 실험 Alias
# ==============================================================================

# 빠른 테스트 (각 알고리즘 1회씩)
alias quick_test='python3 ~/ros2_ws/src/cpp_package/scripts/automated_experiment.py --pipelines heightmap ftg3d --scenarios straight negative --runs 1 --duration 30'

# 전체 실험 (모든 알고리즘 3회씩)
alias full_experiment='python3 ~/ros2_ws/src/cpp_package/scripts/automated_experiment.py --pipelines heightmap ftg3d original2d original3d --scenarios straight corner narrow obstacle negative --runs 3 --duration 60'

# 음의 장애물 특화 실험
alias negative_obstacle_test='python3 ~/ros2_ws/src/cpp_package/scripts/automated_experiment.py --pipelines heightmap ftg3d --scenarios negative --runs 5 --duration 45'

# ==============================================================================
# 유틸리티 Alias
# ==============================================================================

# 모든 ROS 노드 종료
alias kill_all_ros='pkill -f ros2 && pkill -f optical_cane && pkill -f cpp_package'

# 워크스페이스 빌드
alias build_new='cd ~/ros2_ws && colcon build --packages-select cpp_package --symlink-install'

# 소스 반영
alias source_new='source ~/ros2_ws/install/setup.bash'

# 로그 정리
alias clean_logs='rm -rf ~/ros2_ws/log/* && rm -rf ~/.ros/log/*'

# ==============================================================================
# 도움말
# ==============================================================================

alias help_new='cat << EOF
🚀 새로운 알고리즘 실험 도구

📋 기본 사용법:
  new_heightmap     - HeightMap 2.5D 플래너 실행
  new_ftg3d         - Follow-the-Gap 3D 플래너 실행
  new_both          - 두 알고리즘 동시 실행

📊 데이터 수집:
  d4                - HeightMap 데이터 기록
  d5                - FTG-3D 데이터 기록
  record_*          - 시나리오별 자동 기록

🔬 실험 자동화:
  quick_test        - 빠른 테스트 (30초 x 2 scenario)
  full_experiment   - 전체 비교 실험 (60초 x 5 scenario x 3 runs)
  negative_obstacle_test - 음의 장애물 특화 테스트

📈 분석:
  analyze_new       - 새로운 분석 도구 실행
  d0                - 기존 분석 (호환)

🛠️  유틸리티:
  build_new         - 새 패키지 빌드
  kill_all_ros     - 모든 ROS 노드 종료
  clean_logs        - 로그 정리

💡 예시:
  1. 시스템 시작: new_heightmap_viz
  2. 데이터 기록: d4 (별도 터미널)
  3. 분석: analyze_new
EOF'

echo "✅ 새로운 알고리즘 Alias 로딩 완료!"
echo "💡 사용법 확인: help_new"
