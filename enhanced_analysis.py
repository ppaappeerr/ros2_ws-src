#!/usr/bin/env python3
"""
확장된 분석 스크립트: 새로운 알고리즘들의 성능 분석

주요 기능:
1. 기존 3개 파이프라인 + 새로운 2개 알고리즘 분석
2. 음의 장애물 감지 성능 평가
3. 통합 대시보드 생성
4. 자동화된 비교 리포트

사용법:
    python3 enhanced_analysis.py --experiment_dir /path/to/experimental_data --output_dir /path/to/results
"""

import os
import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import seaborn as sns

# 기존 분석 스크립트 import (plot_analysis.py 기능 활용)
sys.path.append(str(Path(__file__).parent))
from plot_analysis import read_rosbag

class EnhancedAnalyzer:
    def __init__(self, experiment_dir: Path, output_dir: Path):
        self.experiment_dir = Path(experiment_dir)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # 알고리즘별 토픽 매핑
        self.algorithm_topics = {
            'heightmap': '/safe_path_vector_heightmap',
            'ftg3d': '/safe_path_vector_ftg3d',
            'original2d': '/safe_path_vector',
            'original3d': '/safe_path_vector_3d'
        }
        
        # 시나리오별 Ground Truth 정의
        self.scenario_gt = {
            'straight': 0.0,  # 정면 (0도)
            'corner': None,   # 동적으로 계산 (코너 진입 타이밍)
            'narrow': 0.0,    # 정면 유지
            'obstacle': None, # 회피 후 복귀
            'negative': None  # 위험 감지 후 회피/정지
        }
        
    def discover_experiments(self) -> List[Dict]:
        """실험 디렉토리에서 실험 데이터 자동 발견"""
        experiments = []
        
        for exp_dir in self.experiment_dir.iterdir():
            if not exp_dir.is_dir():
                continue
                
            metadata_file = exp_dir / 'metadata.json'
            bag_file = exp_dir / 'data.bag'
            
            if metadata_file.exists() and bag_file.exists():
                with open(metadata_file, 'r') as f:
                    metadata = json.load(f)
                
                experiments.append({
                    'metadata': metadata,
                    'data_path': bag_file,
                    'experiment_dir': exp_dir
                })
        
        print(f"📊 발견된 실험: {len(experiments)}개")
        return experiments
    
    def extract_path_data(self, bag_path: Path, algorithm: str) -> Optional[Dict]:
        """로그백에서 경로 데이터 추출"""
        topic = self.algorithm_topics.get(algorithm)
        if not topic:
            print(f"⚠️  알 수 없는 알고리즘: {algorithm}")
            return None
        
        try:
            messages = read_rosbag(str(bag_path), topic)
            if not messages:
                print(f"⚠️  토픽 {topic}에서 데이터를 찾을 수 없음: {bag_path}")
                return None
            
            times = []
            angles = []
            
            start_time = messages[0][0]
            
            for timestamp, msg in messages:
                # 시간을 초 단위로 변환
                time_sec = (timestamp - start_time) * 1e-9
                
                # 벡터에서 각도 계산
                angle_rad = np.arctan2(msg.vector.y, msg.vector.x)
                angle_deg = np.degrees(angle_rad)
                
                times.append(time_sec)
                angles.append(angle_deg)
            
            return {
                'times': np.array(times),
                'angles': np.array(angles),
                'message_count': len(messages)
            }
            
        except Exception as e:
            print(f"❌ 데이터 추출 실패 {bag_path}: {e}")
            return None
    
    def calculate_safety_metrics(self, path_data: Dict, scenario: str) -> Dict:
        """안전성 지표 계산"""
        angles = path_data['angles']
        times = path_data['times']
        
        # 기본 안정성 지표
        angle_std = np.std(angles)
        angle_range = np.max(angles) - np.min(angles)
        
        # 각속도 계산
        if len(times) > 1:
            dt = np.diff(times)
            angle_diff = np.diff(angles)
            angular_velocity = angle_diff / dt
            angular_velocity_rms = np.sqrt(np.mean(angular_velocity**2))
        else:
            angular_velocity_rms = 0.0
        
        # 시나리오별 특수 지표
        scenario_metrics = {}
        
        if scenario == 'negative':
            # 음의 장애물 시나리오: 위험 감지 및 회피 반응
            scenario_metrics['danger_detection'] = self.analyze_danger_response(angles, times)
        elif scenario == 'straight':
            # 직선 시나리오: 정면 유지 능력
            front_deviation = np.abs(angles)
            scenario_metrics['forward_accuracy'] = {
                'mean_deviation': np.mean(front_deviation),
                'max_deviation': np.max(front_deviation),
                'within_10deg_percent': np.sum(front_deviation <= 10.0) / len(front_deviation) * 100
            }
        
        return {
            'stability': {
                'angle_std': angle_std,
                'angle_range': angle_range,
                'angular_velocity_rms': angular_velocity_rms
            },
            'scenario_specific': scenario_metrics
        }
    
    def analyze_danger_response(self, angles: np.ndarray, times: np.ndarray) -> Dict:
        """음의 장애물 감지 및 회피 반응 분석"""
        # 급격한 각도 변화를 위험 감지 신호로 간주
        if len(angles) < 2:
            return {'detection_events': 0, 'avg_response_time': 0.0}
        
        angle_changes = np.abs(np.diff(angles))
        large_changes = angle_changes > 20.0  # 20도 이상 급변
        
        detection_events = np.sum(large_changes)
        
        # 첫 번째 큰 변화까지의 시간 (위험 감지 지연)
        first_detection_idx = np.where(large_changes)[0]
        if len(first_detection_idx) > 0:
            response_time = times[first_detection_idx[0] + 1]
        else:
            response_time = float('inf')  # 감지 실패
        
        return {
            'detection_events': detection_events,
            'response_time': response_time,
            'max_angle_change': np.max(angle_changes) if len(angle_changes) > 0 else 0.0
        }
    
    def create_comparative_dashboard(self, analysis_results: List[Dict]):
        """통합 비교 대시보드 생성"""
        # 데이터 구조화
        df_list = []
        
        for result in analysis_results:
            metadata = result['metadata']
            metrics = result['metrics']
            
            if not metrics:  # 분석 실패한 케이스 스킵
                continue
                
            row = {
                'algorithm': metadata['pipeline'],
                'scenario': metadata['scenario'],
                'run': metadata['run_number'],
                'angle_std': metrics['stability']['angle_std'],
                'angular_velocity_rms': metrics['stability']['angular_velocity_rms'],
                'angle_range': metrics['stability']['angle_range']
            }
            
            # 시나리오별 추가 지표
            if 'scenario_specific' in metrics:
                scenario_metrics = metrics['scenario_specific']
                if 'danger_detection' in scenario_metrics:
                    row['detection_events'] = scenario_metrics['danger_detection']['detection_events']
                    row['response_time'] = scenario_metrics['danger_detection']['response_time']
                elif 'forward_accuracy' in scenario_metrics:
                    row['mean_deviation'] = scenario_metrics['forward_accuracy']['mean_deviation']
                    row['within_10deg_percent'] = scenario_metrics['forward_accuracy']['within_10deg_percent']
            
            df_list.append(row)
        
        if not df_list:
            print("❌ 분석할 데이터가 없습니다.")
            return
        
        df = pd.DataFrame(df_list)
        
        # 대시보드 플롯 생성
        self.plot_stability_comparison(df)
        self.plot_safety_heatmap(df)
        self.plot_scenario_performance(df)
        
        # 종합 성과 테이블
        self.generate_summary_table(df)
    
    def plot_stability_comparison(self, df: pd.DataFrame):
        """안정성 비교 플롯"""
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        # 각도 표준편차
        sns.boxplot(data=df, x='algorithm', y='angle_std', ax=axes[0])
        axes[0].set_title('경로 안정성 (각도 표준편차)')
        axes[0].set_ylabel('각도 표준편차 (도)')
        axes[0].tick_params(axis='x', rotation=45)
        
        # 각속도 RMS
        sns.boxplot(data=df, x='algorithm', y='angular_velocity_rms', ax=axes[1])
        axes[1].set_title('경로 부드러움 (각속도 RMS)')
        axes[1].set_ylabel('각속도 RMS (도/초)')
        axes[1].tick_params(axis='x', rotation=45)
        
        # 각도 범위
        sns.boxplot(data=df, x='algorithm', y='angle_range', ax=axes[2])
        axes[2].set_title('경로 범위 (최대-최소)')
        axes[2].set_ylabel('각도 범위 (도)')
        axes[2].tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'stability_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_safety_heatmap(self, df: pd.DataFrame):
        """안전성 히트맵"""
        # 알고리즘 x 시나리오 성과 매트릭스
        pivot_data = df.groupby(['algorithm', 'scenario']).agg({
            'angle_std': 'mean',
            'detection_events': 'mean',
            'response_time': 'mean'
        }).reset_index()
        
        fig, axes = plt.subplots(1, 3, figsize=(20, 6))
        
        # 안정성 히트맵
        stability_pivot = pivot_data.pivot(index='algorithm', columns='scenario', values='angle_std')
        sns.heatmap(stability_pivot, annot=True, fmt='.2f', ax=axes[0], cmap='RdYlGn_r')
        axes[0].set_title('안정성 점수 (낮을수록 좋음)')
        
        # 위험 감지 이벤트 (음의 장애물 시나리오만)
        if 'detection_events' in df.columns:
            detection_pivot = pivot_data.pivot(index='algorithm', columns='scenario', values='detection_events')
            sns.heatmap(detection_pivot, annot=True, fmt='.1f', ax=axes[1], cmap='RdYlGn')
            axes[1].set_title('위험 감지 이벤트 수 (높을수록 좋음)')
        
        # 응답 시간
        if 'response_time' in df.columns:
            response_pivot = pivot_data.pivot(index='algorithm', columns='scenario', values='response_time')
            sns.heatmap(response_pivot, annot=True, fmt='.2f', ax=axes[2], cmap='RdYlGn_r')
            axes[2].set_title('위험 응답 시간 (낮을수록 좋음)')
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'safety_heatmap.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def plot_scenario_performance(self, df: pd.DataFrame):
        """시나리오별 성능 비교"""
        scenarios = df['scenario'].unique()
        n_scenarios = len(scenarios)
        
        fig, axes = plt.subplots(2, n_scenarios, figsize=(5*n_scenarios, 10))
        if n_scenarios == 1:
            axes = axes.reshape(2, 1)
        
        for i, scenario in enumerate(scenarios):
            scenario_data = df[df['scenario'] == scenario]
            
            # 상단: 안정성
            sns.barplot(data=scenario_data, x='algorithm', y='angle_std', ax=axes[0, i])
            axes[0, i].set_title(f'{scenario}: 안정성')
            axes[0, i].set_ylabel('각도 표준편차 (도)')
            axes[0, i].tick_params(axis='x', rotation=45)
            
            # 하단: 시나리오별 특수 지표
            if scenario == 'negative' and 'detection_events' in scenario_data.columns:
                sns.barplot(data=scenario_data, x='algorithm', y='detection_events', ax=axes[1, i])
                axes[1, i].set_title(f'{scenario}: 위험 감지')
                axes[1, i].set_ylabel('감지 이벤트 수')
            elif scenario == 'straight' and 'mean_deviation' in scenario_data.columns:
                sns.barplot(data=scenario_data, x='algorithm', y='mean_deviation', ax=axes[1, i])
                axes[1, i].set_title(f'{scenario}: 정확도')
                axes[1, i].set_ylabel('평균 편차 (도)')
            else:
                sns.barplot(data=scenario_data, x='algorithm', y='angular_velocity_rms', ax=axes[1, i])
                axes[1, i].set_title(f'{scenario}: 부드러움')
                axes[1, i].set_ylabel('각속도 RMS (도/초)')
            
            axes[1, i].tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'scenario_performance.png', dpi=300, bbox_inches='tight')
        plt.show()
    
    def generate_summary_table(self, df: pd.DataFrame):
        """종합 성과 요약 테이블"""
        summary = df.groupby('algorithm').agg({
            'angle_std': ['mean', 'std'],
            'angular_velocity_rms': ['mean', 'std'],
            'angle_range': ['mean', 'std']
        }).round(3)
        
        # 컬럼명 정리
        summary.columns = ['_'.join(col).strip() for col in summary.columns.values]
        
        # 순위 계산 (낮을수록 좋은 지표들)
        summary['stability_rank'] = summary['angle_std_mean'].rank()
        summary['smoothness_rank'] = summary['angular_velocity_rms_mean'].rank()
        summary['overall_rank'] = (summary['stability_rank'] + summary['smoothness_rank']).rank()
        
        # 파일로 저장
        summary_file = self.output_dir / 'performance_summary.csv'
        summary.to_csv(summary_file)
        
        print("\n📊 알고리즘 성능 요약")
        print("=" * 80)
        print(summary)
        print(f"\n💾 상세 결과 저장: {summary_file}")
        
        return summary
    
    def run_analysis(self):
        """전체 분석 실행"""
        print("🔍 실험 데이터 분석 시작...")
        
        # 실험 발견
        experiments = self.discover_experiments()
        if not experiments:
            print("❌ 분석할 실험 데이터가 없습니다.")
            return
        
        # 각 실험 분석
        analysis_results = []
        
        for exp in experiments:
            metadata = exp['metadata']
            print(f"📈 분석 중: {metadata['pipeline']} | {metadata['scenario']} | Run {metadata['run_number']}")
            
            # 경로 데이터 추출
            path_data = self.extract_path_data(exp['data_path'], metadata['pipeline'])
            
            if path_data:
                # 성능 지표 계산
                metrics = self.calculate_safety_metrics(path_data, metadata['scenario'])
                
                analysis_results.append({
                    'metadata': metadata,
                    'path_data': path_data,
                    'metrics': metrics
                })
            else:
                print(f"⚠️  데이터 추출 실패: {exp['experiment_dir'].name}")
        
        # 대시보드 생성
        if analysis_results:
            print(f"\n📊 대시보드 생성 중... ({len(analysis_results)}개 실험)")
            self.create_comparative_dashboard(analysis_results)
        else:
            print("❌ 분석 가능한 데이터가 없습니다.")

def main():
    parser = argparse.ArgumentParser(description='확장된 알고리즘 성능 분석')
    parser.add_argument('--experiment_dir', type=str, 
                       default='~/ros2_ws/experimental_data',
                       help='실험 데이터 디렉토리')
    parser.add_argument('--output_dir', type=str,
                       default='~/ros2_ws/analysis_results', 
                       help='분석 결과 출력 디렉토리')
    
    args = parser.parse_args()
    
    # 경로 확장
    experiment_dir = Path(args.experiment_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    
    if not experiment_dir.exists():
        print(f"❌ 실험 디렉토리가 존재하지 않습니다: {experiment_dir}")
        return
    
    # 분석 실행
    analyzer = EnhancedAnalyzer(experiment_dir, output_dir)
    analyzer.run_analysis()

if __name__ == '__main__':
    main()
