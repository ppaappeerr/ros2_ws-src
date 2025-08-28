#!/usr/bin/env python3
"""
새로운 알고리즘 성능 비교를 위한 자동화된 데이터 수집 스크립트

사용법:
    python3 automated_experiment.py --pipelines heightmap ftg3d --scenarios straight corner --runs 5

지원하는 파이프라인:
    - heightmap: HeightMap 2.5D Planner (음의 장애물 감지)
    - ftg3d: Follow-the-Gap 3D
    - original2d: 기존 2D 투영 (path_planner_node)
    - original3d: 기존 3D 복도 (path_planner_3d_node)

지원하는 시나리오:
    - straight: 직선 복도
    - corner: 90도 코너  
    - narrow: 좁은 통로
    - obstacle: 장애물 회피
    - negative: 음의 장애물 (맨홀, 구덩이)
"""

import os
import sys
import argparse
import time
import subprocess
import json
from datetime import datetime
from pathlib import Path

class AutomatedExperiment:
    def __init__(self):
        self.workspace_root = Path.home() / "ros2_ws"
        self.data_dir = self.workspace_root / "experimental_data"
        self.data_dir.mkdir(exist_ok=True)
        
        # 파이프라인 설정
        self.pipelines = {
            'heightmap': {
                'launch_cmd': 'ros2 launch cpp_package new_algorithms.launch.py pipeline:=heightmap use_rviz:=false',
                'vector_topic': '/safe_path_vector_heightmap',
                'additional_topics': ['/height_map_markers', '/risk_map_markers']
            },
            'ftg3d': {
                'launch_cmd': 'ros2 launch cpp_package new_algorithms.launch.py pipeline:=ftg3d use_rviz:=false',
                'vector_topic': '/safe_path_vector_ftg3d', 
                'additional_topics': ['/gap_markers', '/sector_markers']
            },
            'original2d': {
                'launch_cmd': 'ros2 run cpp_package path_planner_node',
                'vector_topic': '/safe_path_vector',
                'additional_topics': ['/candidate_rays', '/preprocessed_cloud']
            },
            'original3d': {
                'launch_cmd': 'ros2 run cpp_package path_planner_3d_node',
                'vector_topic': '/safe_path_vector_3d',
                'additional_topics': ['/candidate_rays_3d', '/preprocessed_cloud_3d']
            }
        }
        
        # 공통 토픽 (모든 실험에서 기록)
        self.common_topics = [
            '/imu/data',
            '/tf',
            '/tf_static', 
            '/downsampled_cloud',
            '/sweep_cloud_cpp'
        ]
        
    def setup_experiment(self, pipeline_name, scenario, run_number):
        """실험 설정 및 메타데이터 생성"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_id = f"{scenario}_{pipeline_name}_run{run_number:02d}_{timestamp}"
        
        experiment_dir = self.data_dir / experiment_id
        experiment_dir.mkdir(exist_ok=True)
        
        # 메타데이터 저장
        metadata = {
            'experiment_id': experiment_id,
            'pipeline': pipeline_name,
            'scenario': scenario,
            'run_number': run_number,
            'timestamp': timestamp,
            'workspace_path': str(self.workspace_root),
            'git_commit': self.get_git_commit(),
            'pipeline_config': self.pipelines[pipeline_name],
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown')
        }
        
        metadata_file = experiment_dir / 'metadata.json'
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
            
        return experiment_dir, experiment_id
    
    def get_git_commit(self):
        """현재 git commit 해시 가져오기"""
        try:
            result = subprocess.run(
                ['git', 'rev-parse', 'HEAD'],
                cwd=self.workspace_root / 'src',
                capture_output=True,
                text=True
            )
            return result.stdout.strip() if result.returncode == 0 else 'unknown'
        except:
            return 'unknown'
    
    def start_pipeline(self, pipeline_name):
        """파이프라인 시작"""
        pipeline_config = self.pipelines[pipeline_name]
        
        print(f"  Starting pipeline: {pipeline_name}")
        print(f"  Command: {pipeline_config['launch_cmd']}")
        
        # 파이프라인 프로세스 시작
        process = subprocess.Popen(
            pipeline_config['launch_cmd'].split(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # 초기화 대기
        time.sleep(5)
        
        return process
    
    def record_data(self, pipeline_name, experiment_dir, duration=60):
        """로그백 데이터 기록"""
        pipeline_config = self.pipelines[pipeline_name]
        
        # 기록할 토픽 리스트 구성
        topics_to_record = (
            self.common_topics + 
            [pipeline_config['vector_topic']] + 
            pipeline_config['additional_topics']
        )
        
        bag_file = experiment_dir / 'data.bag'
        
        # rosbag2 기록 명령
        record_cmd = [
            'ros2', 'bag', 'record',
            '-o', str(bag_file)
        ] + topics_to_record
        
        print(f"  Recording data for {duration} seconds...")
        print(f"  Topics: {', '.join(topics_to_record)}")
        
        # 기록 시작
        record_process = subprocess.Popen(record_cmd)
        
        # 지정된 시간만큼 기록
        time.sleep(duration)
        
        # 기록 종료
        record_process.terminate()
        record_process.wait()
        
        return bag_file
    
    def run_single_experiment(self, pipeline_name, scenario, run_number, duration=60):
        """단일 실험 실행"""
        print(f"\n{'='*60}")
        print(f"실험 시작: {pipeline_name} | {scenario} | Run {run_number}")
        print(f"{'='*60}")
        
        # 실험 설정
        experiment_dir, experiment_id = self.setup_experiment(pipeline_name, scenario, run_number)
        
        try:
            # 파이프라인 시작
            pipeline_process = self.start_pipeline(pipeline_name)
            
            # 시나리오별 안내 메시지 출력
            self.print_scenario_instructions(scenario)
            
            # 데이터 기록
            bag_file = self.record_data(pipeline_name, experiment_dir, duration)
            
            # 파이프라인 종료
            pipeline_process.terminate()
            pipeline_process.wait()
            
            print(f"  ✅ 실험 완료: {experiment_id}")
            print(f"  📁 데이터 저장: {bag_file}")
            
            return experiment_dir
            
        except Exception as e:
            print(f"  ❌ 실험 실패: {e}")
            # 프로세스 정리
            try:
                pipeline_process.terminate()
            except:
                pass
            return None
    
    def print_scenario_instructions(self, scenario):
        """시나리오별 실험 지시사항 출력"""
        instructions = {
            'straight': "📍 직선 복도를 따라 천천히 이동하세요 (센서를 정면으로 유지)",
            'corner': "📍 90도 코너를 돌아가세요 (코너 진입 → 회전 → 직진)",
            'narrow': "📍 좁은 통로(문틀)를 통과하세요",
            'obstacle': "📍 전방 장애물을 우회하세요 (기둥, 의자 등)",
            'negative': "📍 음의 장애물 앞에서 테스트하세요 (매트, 박스로 만든 가짜 구덩이)"
        }
        
        instruction = instructions.get(scenario, "📍 지정된 시나리오에 따라 실험하세요")
        print(f"  {instruction}")
        print(f"  ⏱️  준비가 되면 3초 후 자동으로 기록이 시작됩니다...")
        
        for i in range(3, 0, -1):
            print(f"     {i}...")
            time.sleep(1)
        print("     🔴 기록 시작!")
    
    def run_comparative_experiment(self, pipelines, scenarios, runs_per_combo, duration=60):
        """비교 실험 실행"""
        total_experiments = len(pipelines) * len(scenarios) * runs_per_combo
        completed = 0
        
        print(f"🚀 비교 실험 시작")
        print(f"📊 파이프라인: {', '.join(pipelines)}")
        print(f"🎯 시나리오: {', '.join(scenarios)}")
        print(f"🔄 반복 횟수: {runs_per_combo}")
        print(f"📈 총 실험 수: {total_experiments}")
        
        results = []
        
        for scenario in scenarios:
            for pipeline in pipelines:
                for run in range(1, runs_per_combo + 1):
                    completed += 1
                    print(f"\n[{completed}/{total_experiments}]", end=" ")
                    
                    result = self.run_single_experiment(pipeline, scenario, run, duration)
                    results.append({
                        'pipeline': pipeline,
                        'scenario': scenario, 
                        'run': run,
                        'success': result is not None,
                        'data_dir': str(result) if result else None
                    })
                    
                    # 실험 간 휴식
                    if completed < total_experiments:
                        print("  ⏸️  다음 실험까지 5초 대기...")
                        time.sleep(5)
        
        # 결과 요약 저장
        summary_file = self.data_dir / f"experiment_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(summary_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n🎉 모든 실험 완료!")
        print(f"📋 결과 요약: {summary_file}")
        
        return results

def main():
    parser = argparse.ArgumentParser(description='새로운 알고리즘 자동 실험 도구')
    parser.add_argument('--pipelines', nargs='+', 
                       choices=['heightmap', 'ftg3d', 'original2d', 'original3d'],
                       default=['heightmap', 'ftg3d'],
                       help='테스트할 파이프라인 선택')
    parser.add_argument('--scenarios', nargs='+',
                       choices=['straight', 'corner', 'narrow', 'obstacle', 'negative'],
                       default=['straight', 'negative'],
                       help='테스트할 시나리오 선택')
    parser.add_argument('--runs', type=int, default=3,
                       help='시나리오당 반복 횟수')
    parser.add_argument('--duration', type=int, default=60,
                       help='실험당 기록 시간(초)')
    
    args = parser.parse_args()
    
    # 실험 객체 생성 및 실행
    experiment = AutomatedExperiment()
    experiment.run_comparative_experiment(
        pipelines=args.pipelines,
        scenarios=args.scenarios, 
        runs_per_combo=args.runs,
        duration=args.duration
    )

if __name__ == '__main__':
    main()
