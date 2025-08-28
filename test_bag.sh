#!/bin/bash
# rosbag 테스트용 스크립트

echo "🧪 rosbag 테스트 중..."

# 간단한 테스트 (핵심 토픽 1개)
echo "1. 간단한 테스트:"
echo "ros2 bag record -o test_simple /safe_path_vector"

# 좀 더 복잡한 테스트 (토픽 3개)
echo ""
echo "2. 중간 테스트:"  
echo "ros2 bag record -o test_medium /safe_path_vector /sweep_cloud_cpp /path_planner_debug"

# 직접 실행해보기
echo ""
echo "3. 직접 테스트 실행 (10초):"
timeout 10s ros2 bag record -o test_direct /safe_path_vector 2>/dev/null || echo "테스트 완료"

echo ""
echo "✅ 위 명령어들이 에러 없이 실행되면 alias 문제입니다."
echo "에러가 계속 나면 rosbag 자체 문제일 수 있습니다."