#!/bin/bash
# 두 알고리즘 성능 비교 테스트 스크립트

echo "=== HeightMap 2.5D vs FTG-3D 성능 비교 테스트 ==="

# 1. 환경 설정
source /home/p/ros2_ws/install/setup.bash

# 2. 백그라운드에서 시각화 시스템 실행
echo "Starting visualization system..."
ros2 launch cpp_package visualization_test.launch.py &
LAUNCH_PID=$!

# 3. 비교 분석기 실행
echo "Starting algorithm comparator..."
sleep 5  # 시스템 초기화 대기
python3 /home/p/ros2_ws/src/algorithm_comparator.py &
COMPARATOR_PID=$!

echo "=== 테스트 진행 중 ==="
echo "- HeightMap 2.5D: /height_markers, /risk_map_markers 토픽 확인"
echo "- FTG-3D: /gap_markers, /sector_markers 토픽 확인" 
echo "- 실시간 비교: algorithm_comparator 로그 확인"
echo ""
echo "Ctrl+C로 테스트 종료 시 결과가 algorithm_comparison.json에 저장됩니다"

# 4. 사용자 입력 대기
wait $COMPARATOR_PID

# 5. 정리
echo "Cleaning up..."
kill $LAUNCH_PID 2>/dev/null
echo "Test completed!"
