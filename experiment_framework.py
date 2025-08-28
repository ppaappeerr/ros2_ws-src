#!/usr/bin/env python3
"""
5×5 실험 프레임워크: 5가지 알고리즘 × 5가지 마운트 각도 자동화 테스트
"""

import os
import time
import subprocess
import json
from datetime import datetime
from pathlib import Path

class ExperimentFramework:
    def __init__(self):
        self.algorithms = {
            'p1': {
                'name': '3D→2D 투영',
                'setup_cmds': ['a1', 'a3', 'a4'],  # LiDAR+IMU → sensor_fusion → sweeper
                'run_cmd': 'p1',
                'topics': ['/safe_path_vector', '/sweep_cloud_cpp', '/path_planner_debug']
            },
            'p2': {
                'name': '순수 2D 스캔',
                'setup_cmds': ['a1', 'a2'],  # LiDAR+IMU → scan_accumulator
                'run_cmd': 'p2', 
                'topics': ['/safe_path_vector', '/scan_accumulation_cloud', '/path_planner_debug']
            },
            'p3': {
                'name': '3D Corridor',
                'setup_cmds': ['a1', 'a3', 'a4', 'a5'],  # + voxel_filter
                'run_cmd': 'p3',
                'topics': ['/safe_path_vector', '/downsampled_cloud', '/path_planner_debug']
            },
            'p4': {
                'name': 'Follow-The-Gap 3D', 
                'setup_cmds': ['a1', 'a3', 'a4', 'a5'],
                'run_cmd': 'p4',
                'topics': ['/safe_path_vector', '/downsampled_cloud', '/ftg_gaps']
            },
            'p5': {
                'name': 'HeightMap 2.5D',
                'setup_cmds': ['a1', 'a3', 'a4', 'a5'], 
                'run_cmd': 'p5',
                'topics': ['/safe_path_vector', '/downsampled_cloud', '/height_pillars']
            }
        }
        
        self.mount_angles = {
            'angle_0': {'pitch': 0, 'description': 'Baseline (얼굴 오른쪽)'},
            'angle_5': {'pitch': 5, 'description': '5도 아래'},
            'angle_10': {'pitch': 10, 'description': '10도 아래'},
            'angle_15': {'pitch': 15, 'description': '15도 아래'},  
            'angle_20': {'pitch': 20, 'description': '20도 아래'}
        }
        
        # 실험 결과 저장 디렉토리
        self.base_dir = Path(f"experiments_{datetime.now().strftime('%Y%m%d_%H%M')}")
        self.base_dir.mkdir(exist_ok=True)
        
        # rosbag 저장 디렉토리
        self.bag_dir = self.base_dir / "rosbags"
        self.bag_dir.mkdir(exist_ok=True)
        
        # 분석 결과 디렉토리  
        self.analysis_dir = self.base_dir / "analysis"
        self.analysis_dir.mkdir(exist_ok=True)

    def run_single_experiment(self, algo_key, angle_key, trial_num=1):
        """단일 실험 실행"""
        print(f"\n🧪 실험 시작: {algo_key} × {angle_key} (시도 {trial_num})")
        
        # rosbag 파일명
        bag_name = f"{algo_key}_{angle_key}_trial{trial_num}"
        bag_path = self.bag_dir / bag_name
        
        algo = self.algorithms[algo_key]
        angle = self.mount_angles[angle_key]
        
        # 1. 기본 센서 노드들 실행 (백그라운드)
        print("📡 센서 노드 시작...")
        setup_processes = []
        for cmd in algo['setup_cmds']:
            proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL)
            setup_processes.append(proc)
            time.sleep(1)  # 노드 간 시작 간격
        
        time.sleep(3)  # 센서 안정화 대기
        
        # 2. 알고리즘 노드 실행
        print(f"🤖 {algo['name']} 알고리즘 시작...")
        algo_proc = subprocess.Popen(algo['run_cmd'], shell=True, stdout=subprocess.DEVNULL)
        time.sleep(2)  # 알고리즘 초기화 대기
        
        # 3. rosbag 기록 시작  
        common_topics = ['/safe_path_vector', '/imu/data', '/tf', '/tf_static']
        all_topics = common_topics + algo['topics']
        bag_cmd = f"ros2 bag record -o {bag_path} {' '.join(all_topics)}"
        
        print(f"📼 데이터 기록 시작 ({bag_name})")
        print("🚶 실험 시나리오를 수행하세요 (완료 후 Enter)...")
        
        bag_proc = subprocess.Popen(bag_cmd, shell=True)
        
        # 사용자 입력 대기 (실제 실험 수행)
        input("실험이 완료되면 Enter를 누르세요...")
        
        # 4. 정리
        print("🛑 실험 종료, 프로세스 정리 중...")
        bag_proc.terminate()
        algo_proc.terminate()
        
        for proc in setup_processes:
            proc.terminate()
            
        time.sleep(2)
        print(f"✅ 실험 완료: {bag_name}")
        
        return bag_path

    def run_full_matrix_experiment(self):
        """5×5 전체 매트릭스 실험"""
        experiment_log = {
            'start_time': datetime.now().isoformat(),
            'experiments': [],
            'total_count': len(self.algorithms) * len(self.mount_angles)
        }
        
        print(f"🎯 5×5 실험 매트릭스 시작 (총 {experiment_log['total_count']}개)")
        
        completed = 0
        for algo_key in self.algorithms:
            for angle_key in self.mount_angles:
                completed += 1
                print(f"\n진행률: {completed}/{experiment_log['total_count']}")
                
                # 각도 조정 안내
                angle_desc = self.mount_angles[angle_key]['description']
                print(f"📐 마운트 각도를 {angle_desc}로 조정하고 Enter를 누르세요...")
                input()
                
                try:
                    bag_path = self.run_single_experiment(algo_key, angle_key)
                    
                    # 실험 로그 기록
                    experiment_log['experiments'].append({
                        'algorithm': algo_key,
                        'angle': angle_key, 
                        'bag_path': str(bag_path),
                        'timestamp': datetime.now().isoformat(),
                        'status': 'completed'
                    })
                    
                except Exception as e:
                    print(f"❌ 실험 실패: {e}")
                    experiment_log['experiments'].append({
                        'algorithm': algo_key,
                        'angle': angle_key,
                        'error': str(e),
                        'timestamp': datetime.now().isoformat(),
                        'status': 'failed'
                    })
                
                # 다음 실험 전 대기
                if completed < experiment_log['total_count']:
                    print("⏳ 다음 실험 준비 (10초 후)...")
                    time.sleep(10)
        
        # 실험 로그 저장
        log_file = self.base_dir / "experiment_log.json"
        with open(log_file, 'w') as f:
            json.dump(experiment_log, f, indent=2, ensure_ascii=False)
            
        print(f"\n🎉 모든 실험 완료! 로그 저장: {log_file}")
        return experiment_log

    def analyze_results(self):
        """결과 분석 (별도 실행)"""
        print("📊 결과 분석을 시작합니다...")
        
        # enhanced_analysis.py 실행
        analysis_cmd = f"python3 enhanced_analysis.py --input_dir {self.bag_dir} --output_dir {self.analysis_dir}"
        subprocess.run(analysis_cmd, shell=True)
        
        print(f"📈 분석 결과 저장: {self.analysis_dir}")

def main():
    framework = ExperimentFramework()
    
    print("🔬 Optical Cane 5×5 실험 프레임워크")
    print("=" * 50)
    
    choice = input("""
선택하세요:
1. 단일 실험 (테스트용)
2. 전체 5×5 매트릭스 실험  
3. 기존 결과 분석만
입력: """)
    
    if choice == '1':
        # 테스트용 단일 실험
        print("사용 가능한 알고리즘:", list(framework.algorithms.keys()))
        algo = input("알고리즘 선택: ")
        print("사용 가능한 각도:", list(framework.mount_angles.keys()))
        angle = input("각도 선택: ")
        
        if algo in framework.algorithms and angle in framework.mount_angles:
            framework.run_single_experiment(algo, angle)
        else:
            print("잘못된 선택입니다.")
            
    elif choice == '2':
        # 전체 실험 수행
        framework.run_full_matrix_experiment()
        
        # 자동으로 분석도 실행
        analyze = input("\n분석도 바로 실행하시겠습니까? (y/n): ")
        if analyze.lower() == 'y':
            framework.analyze_results()
            
    elif choice == '3':
        # 분석만 실행
        framework.analyze_results()
    else:
        print("잘못된 선택입니다.")

if __name__ == "__main__":
    main()