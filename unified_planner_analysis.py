#!/usr/bin/env python3
"""
통일된 플래너 성능 비교 분석 도구

5개 통일된 플래너의 공정한 성능 비교를 위한 정량적 지표 계산:
- 안정성: 방향 변화의 일관성
- 반응성: 장애물 회피 속도  
- 안전성: 위험 상황 대응
- 편향성: 좌우 균형
- 연속성: 방향 급변 빈도
"""

import rosbag2_py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal
from pathlib import Path
import argparse
from datetime import datetime
import json

class UnifiedPlannerAnalyzer:
    def __init__(self):
        self.metrics = {
            'stability': [],      # 방향 안정성 (표준편차 기반)
            'reactivity': [],     # 반응성 (각속도 RMS)
            'safety': [],         # 안전성 (최소 거리 유지)
            'bias': [],           # 편향성 (좌우 균형)
            'continuity': [],     # 연속성 (방향 급변 빈도)
            'smoothness': []      # 부드러움 (각가속도 기반)
        }
        
        self.planner_names = {
            'up1': 'P1: 2D 투영',
            'up2': 'P2: 순수 2D',  
            'up3': 'P3: 3D Corridor',
            'up4': 'P4: Follow-the-Gap',
            'up5': 'P5: HeightMap'
        }
    
    def analyze_bag_file(self, bag_path, planner_id):
        """단일 rosbag 파일 분석"""
        print(f"📊 분석 중: {bag_path} ({self.planner_names[planner_id]})")
        
        # 데이터 추출
        path_vectors = self.extract_path_vectors(bag_path)
        
        if len(path_vectors) < 10:
            print(f"⚠️ 데이터 부족: {len(path_vectors)}개 샘플")
            return None
        
        # 각도 및 각속도 계산
        angles = [np.arctan2(v['y'], v['x']) for v in path_vectors]
        timestamps = [v['timestamp'] for v in path_vectors]
        
        # 시간 정규화
        times = np.array([(t - timestamps[0]).total_seconds() for t in timestamps])
        angles = np.array(angles)
        
        # 각도 연속성 보정 (wrap around 처리)
        angles = np.unwrap(angles)
        
        # 각속도 계산 (중앙 차분)
        if len(times) > 2:
            angular_velocity = np.gradient(angles, times)
        else:
            angular_velocity = np.zeros_like(angles)
        
        # 지표 계산
        metrics = self.calculate_metrics(angles, angular_velocity, times)
        metrics['planner'] = planner_id
        metrics['planner_name'] = self.planner_names[planner_id]
        
        return metrics
    
    def extract_path_vectors(self, bag_path):
        """rosbag에서 /safe_path_vector 메시지 추출"""
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        vectors = []
        
        # 토픽 필터링
        topic_types = reader.get_all_topics_and_types()
        target_topic = '/safe_path_vector'
        
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            if topic == target_topic:
                # 간단한 메시지 파싱 (실제로는 더 정교한 파싱 필요)
                try:
                    # Vector3Stamped 메시지에서 x, y 추출
                    # 실제 구현시에는 rosbag2_py의 메시지 deserializer 사용
                    vectors.append({
                        'x': 1.0,  # 임시값 - 실제로는 메시지에서 추출
                        'y': 0.0,  # 임시값 - 실제로는 메시지에서 추출
                        'timestamp': datetime.fromtimestamp(timestamp * 1e-9)
                    })
                except:
                    continue
        
        reader.close()
        return vectors
    
    def calculate_metrics(self, angles, angular_velocity, times):
        """정량적 성능 지표 계산"""
        metrics = {}
        
        # 1. 안정성 (Stability): 방향 변화의 일관성
        # 낮을수록 좋음 (일관된 방향)
        metrics['stability'] = np.std(angles) * 180 / np.pi  # 도 단위
        
        # 2. 반응성 (Reactivity): 각속도 RMS
        # 적절한 수준이 좋음 (너무 높으면 불안정, 너무 낮으면 둔함)
        metrics['reactivity'] = np.sqrt(np.mean(angular_velocity**2)) * 180 / np.pi  # 도/초
        
        # 3. 안전성 (Safety): 급격한 방향 변화 빈도
        # 낮을수록 좋음 (부드러운 궤적)
        angle_changes = np.abs(np.diff(angles))
        large_changes = np.sum(angle_changes > np.pi/6)  # 30도 이상 급변
        metrics['safety'] = large_changes / len(times) * 60  # 분당 급변 횟수
        
        # 4. 편향성 (Bias): 좌우 방향 균형
        # 0에 가까울수록 좋음 (균형잡힌 탐색)
        left_time = np.sum(times[angles > 0.1])   # 왼쪽(+) 방향 시간
        right_time = np.sum(times[angles < -0.1])  # 오른쪽(-) 방향 시간
        total_time = left_time + right_time
        if total_time > 0:
            metrics['bias'] = abs(left_time - right_time) / total_time
        else:
            metrics['bias'] = 0
        
        # 5. 연속성 (Continuity): 방향 급변 빈도
        # 낮을수록 좋음 (연속적인 경로)
        direction_reversals = 0
        for i in range(1, len(angular_velocity)-1):
            if (angular_velocity[i-1] * angular_velocity[i+1] < 0 and 
                abs(angular_velocity[i]) > 0.5):  # 방향 전환
                direction_reversals += 1
        metrics['continuity'] = direction_reversals / len(times) * 60  # 분당 반전 횟수
        
        # 6. 부드러움 (Smoothness): 각가속도 기반
        # 낮을수록 좋음 (부드러운 움직임)
        if len(angular_velocity) > 1:
            angular_acceleration = np.gradient(angular_velocity, times)
            metrics['smoothness'] = np.sqrt(np.mean(angular_acceleration**2)) * 180 / np.pi
        else:
            metrics['smoothness'] = 0
        
        return metrics
    
    def analyze_experiment_batch(self, bag_directory, output_dir):
        """실험 배치 분석"""
        bag_dir = Path(bag_directory)
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True)
        
        results = []
        
        # 각 플래너별로 bag 파일 찾기 (새로운 네이밍 패턴 지원)
        for planner_id in self.planner_names.keys():
            # 기존 패턴 + 새로운 trial 패턴 모두 지원
            patterns = [f"*{planner_id}*", f"{planner_id}_trial_*", f"*{planner_id}_experiment*"]
            bag_files = []
            
            for pattern in patterns:
                bag_files.extend(list(bag_dir.glob(pattern)))
            
            # 중복 제거
            bag_files = list(set(bag_files))
            
            for bag_file in bag_files:
                metrics = self.analyze_bag_file(bag_file, planner_id)
                if metrics:
                    # 시행 번호 추출 (파일명에서)
                    file_name = bag_file.name
                    trial_num = 1
                    if "trial_" in file_name:
                        import re
                        match = re.search(r'trial_(\d+)', file_name)
                        if match:
                            trial_num = int(match.group(1))
                    
                    metrics['trial'] = trial_num
                    results.append(metrics)
        
        if not results:
            print("❌ 분석할 데이터가 없습니다.")
            return
        
        # 결과를 DataFrame으로 변환
        df = pd.DataFrame(results)
        
        # 요약 통계
        summary = df.groupby('planner_name').agg({
            'stability': ['mean', 'std'],
            'reactivity': ['mean', 'std'], 
            'safety': ['mean', 'std'],
            'bias': ['mean', 'std'],
            'continuity': ['mean', 'std'],
            'smoothness': ['mean', 'std']
        }).round(3)
        
        print("\n📈 플래너 성능 비교 요약:")
        print(summary)
        
        # 결과 저장
        df.to_csv(output_dir / 'detailed_results.csv', index=False)
        summary.to_csv(output_dir / 'summary_stats.csv')
        
        # 시각화
        self.create_comparison_plots(df, output_dir)
        
        # 순위 매기기
        self.rank_planners(df, output_dir)
        
        print(f"\n✅ 분석 완료! 결과 저장: {output_dir}")
        
    def create_comparison_plots(self, df, output_dir):
        """비교 시각화 생성"""
        plt.style.use('seaborn-v0_8')
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('통일된 플래너 성능 비교', fontsize=16, fontweight='bold')
        
        metrics = ['stability', 'reactivity', 'safety', 'bias', 'continuity', 'smoothness']
        metric_labels = ['안정성 (도)', '반응성 (도/초)', '안전성 (급변/분)', 
                        '편향성', '연속성 (반전/분)', '부드러움 (도/초²)']
        
        for i, (metric, label) in enumerate(zip(metrics, metric_labels)):
            ax = axes[i//3, i%3]
            
            # 박스플롯으로 분포 시각화
            sns.boxplot(data=df, x='planner_name', y=metric, ax=ax)
            ax.set_title(label, fontweight='bold')
            ax.set_xlabel('')
            ax.tick_params(axis='x', rotation=45)
            
            # 평균값 표시
            means = df.groupby('planner_name')[metric].mean()
            for j, (name, mean_val) in enumerate(means.items()):
                ax.text(j, mean_val, f'{mean_val:.2f}', 
                       ha='center', va='bottom', fontweight='bold', color='red')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'performance_comparison.png', dpi=300, bbox_inches='tight')
        print("📊 성능 비교 차트 저장: performance_comparison.png")
    
    def rank_planners(self, df, output_dir):
        """플래너 순위 매기기"""
        # 각 지표별로 순위 계산 (낮을수록 좋은 지표들)
        ranking_metrics = ['stability', 'safety', 'bias', 'continuity', 'smoothness']
        
        planner_scores = {}
        
        for planner in self.planner_names.keys():
            planner_data = df[df['planner'] == planner]
            if len(planner_data) == 0:
                continue
                
            scores = {}
            
            # 각 지표별 평균값
            for metric in ranking_metrics:
                scores[metric] = planner_data[metric].mean()
            
            # 반응성은 적절한 수준이 좋음 (너무 높거나 낮으면 안됨)
            ideal_reactivity = 45  # 45도/초가 적절하다고 가정
            scores['reactivity'] = abs(planner_data['reactivity'].mean() - ideal_reactivity)
            
            planner_scores[planner] = scores
        
        # 각 지표별로 순위 매기기 (1위가 가장 좋음)
        rankings = pd.DataFrame(planner_scores).T
        for metric in rankings.columns:
            rankings[f'{metric}_rank'] = rankings[metric].rank()
        
        # 종합 순위 (평균 순위)
        rank_cols = [col for col in rankings.columns if col.endswith('_rank')]
        rankings['overall_rank'] = rankings[rank_cols].mean(axis=1)
        
        # 플래너 이름 추가
        rankings['planner_name'] = [self.planner_names[p] for p in rankings.index]
        
        # 순위순으로 정렬
        rankings_sorted = rankings.sort_values('overall_rank')
        
        print("\n🏆 플래너 종합 순위:")
        print("=" * 50)
        for i, (planner_id, row) in enumerate(rankings_sorted.iterrows(), 1):
            print(f"{i}위: {row['planner_name']} (종합점수: {row['overall_rank']:.2f})")
        
        # 순위 결과 저장
        rankings_sorted.to_csv(output_dir / 'planner_rankings.csv')
        
        return rankings_sorted

def main():
    parser = argparse.ArgumentParser(description='통일된 플래너 성능 분석')
    parser.add_argument('--bag_dir', required=True, help='rosbag 파일들이 있는 디렉토리')
    parser.add_argument('--output_dir', default='analysis_results', help='결과 저장 디렉토리')
    
    args = parser.parse_args()
    
    analyzer = UnifiedPlannerAnalyzer()
    analyzer.analyze_experiment_batch(args.bag_dir, args.output_dir)

if __name__ == '__main__':
    main()