#!/usr/bin/env python3
"""
간단한 rosbag 분석 도구 - rosbag2_py 대신 파일 기반 분석
"""

import os
import sqlite3
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
from datetime import datetime

class SimpleBagAnalyzer:
    def __init__(self):
        self.planner_names = {
            'up1': 'P1: 2D 투영',
            'up2': 'P2: 순수 2D',
            'up3': 'P3: 3D Corridor', 
            'up4': 'P4: Follow-the-Gap',
            'up5': 'P5: HeightMap'
        }
    
    def analyze_bag_directory(self, bag_dir, output_dir):
        """rosbag 디렉토리 분석"""
        bag_path = Path(bag_dir)
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True, parents=True)
        
        results = []
        
        print(f"📁 분석 디렉토리: {bag_path}")
        
        # 각 실험 디렉토리 찾기
        for trial_dir in bag_path.glob("*trial*"):
            if trial_dir.is_dir():
                print(f"📊 분석 중: {trial_dir.name}")
                
                # 실험 정보 추출
                planner_id = self.extract_planner_id(trial_dir.name)
                trial_num = self.extract_trial_number(trial_dir.name)
                
                if planner_id:
                    # SQLite DB 파일 찾기
                    db_files = list(trial_dir.glob("*.db3"))
                    if db_files:
                        metrics = self.analyze_sqlite_bag(db_files[0], planner_id, trial_num)
                        if metrics:
                            results.append(metrics)
                    else:
                        print(f"⚠️ {trial_dir.name}: DB 파일을 찾을 수 없음")
        
        if results:
            # 결과 저장 및 시각화
            df = pd.DataFrame(results)
            self.save_results(df, output_path)
            self.create_simple_plots(df, output_path)
            print(f"✅ 분석 완료: {len(results)}개 실험")
            print(f"📈 결과 저장: {output_path}")
        else:
            print("❌ 분석할 데이터가 없습니다.")
    
    def extract_planner_id(self, dirname):
        """디렉토리 이름에서 플래너 ID 추출"""
        for planner_id in self.planner_names.keys():
            if planner_id in dirname:
                return planner_id
        return None
    
    def extract_trial_number(self, dirname):
        """디렉토리 이름에서 시행 번호 추출"""
        import re
        match = re.search(r'trial_(\d+)', dirname)
        return int(match.group(1)) if match else 1
    
    def analyze_sqlite_bag(self, db_path, planner_id, trial_num):
        """SQLite rosbag 분석 (간단 버전)"""
        try:
            conn = sqlite3.connect(str(db_path))
            cursor = conn.cursor()
            
            # 메시지 개수 확인
            cursor.execute("SELECT COUNT(*) FROM messages WHERE topic_id IN (SELECT id FROM topics WHERE name='/safe_path_vector')")
            message_count = cursor.fetchone()[0]
            
            if message_count < 5:
                print(f"⚠️ {planner_id} trial {trial_num}: 메시지 부족 ({message_count}개)")
                conn.close()
                return None
            
            # 실행 시간 계산
            cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
            min_time, max_time = cursor.fetchone()
            duration = (max_time - min_time) / 1e9  # 나노초 → 초
            
            # 간단한 메트릭 계산
            metrics = {
                'planner': planner_id,
                'planner_name': self.planner_names[planner_id],
                'trial': trial_num,
                'message_count': message_count,
                'duration_seconds': duration,
                'frequency_hz': message_count / duration if duration > 0 else 0,
                'analysis_timestamp': datetime.now().isoformat()
            }
            
            conn.close()
            return metrics
            
        except Exception as e:
            print(f"❌ {planner_id} trial {trial_num} 분석 오류: {e}")
            return None
    
    def save_results(self, df, output_path):
        """결과 저장"""
        # 상세 결과
        df.to_csv(output_path / 'experiment_results.csv', index=False)
        
        # 요약 통계
        summary = df.groupby('planner_name').agg({
            'message_count': ['mean', 'std', 'min', 'max'],
            'duration_seconds': ['mean', 'std'],
            'frequency_hz': ['mean', 'std']
        }).round(2)
        
        summary.to_csv(output_path / 'summary_statistics.csv')
        
        print("\n📊 실험 요약:")
        print("="*60)
        for planner in df['planner_name'].unique():
            planner_data = df[df['planner_name'] == planner]
            trials = len(planner_data)
            avg_duration = planner_data['duration_seconds'].mean()
            avg_frequency = planner_data['frequency_hz'].mean()
            
            print(f"{planner}:")
            print(f"  시행: {trials}회, 평균 시간: {avg_duration:.1f}초, 평균 주파수: {avg_frequency:.1f}Hz")
    
    def create_simple_plots(self, df, output_path):
        """Enhanced visualization for 5x5 experiment"""
        # Prepare data summary
        planner_summary = df.groupby('planner_name').agg({
            'message_count': ['count', 'mean', 'std'],
            'duration_seconds': ['mean', 'std'],
            'frequency_hz': ['mean', 'std']
        }).round(2)
        
        # Main plots
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('5-Planner Experiment Results (5 trials each)', fontsize=16, fontweight='bold')
        
        # Message count boxplot
        df.boxplot(column='message_count', by='planner_name', ax=axes[0,0])
        axes[0,0].set_title('Message Count Distribution')
        axes[0,0].set_xlabel('Planner')
        axes[0,0].tick_params(axis='x', rotation=45)
        
        # Duration boxplot
        df.boxplot(column='duration_seconds', by='planner_name', ax=axes[0,1])
        axes[0,1].set_title('Duration Distribution (seconds)')
        axes[0,1].set_xlabel('Planner')
        axes[0,1].tick_params(axis='x', rotation=45)
        
        # Frequency boxplot
        df.boxplot(column='frequency_hz', by='planner_name', ax=axes[1,0])
        axes[1,0].set_title('Message Frequency (Hz)')
        axes[1,0].set_xlabel('Planner')
        axes[1,0].tick_params(axis='x', rotation=45)
        
        # Performance comparison bar chart
        planner_means = df.groupby('planner_name')['frequency_hz'].mean().sort_values(ascending=False)
        planner_stds = df.groupby('planner_name')['frequency_hz'].std()
        
        bars = axes[1,1].bar(range(len(planner_means)), planner_means.values, 
                            yerr=planner_stds[planner_means.index].values, 
                            capsize=5, alpha=0.7)
        axes[1,1].set_title('Average Frequency Comparison')
        axes[1,1].set_xlabel('Planner (ranked)')
        axes[1,1].set_ylabel('Frequency (Hz)')
        axes[1,1].set_xticks(range(len(planner_means)))
        axes[1,1].set_xticklabels(planner_means.index, rotation=45)
        
        # Add value labels on bars
        for i, (bar, value) in enumerate(zip(bars, planner_means.values)):
            axes[1,1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                          f'{value:.1f}', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(output_path / 'experiment_summary.png', dpi=300, bbox_inches='tight')
        print(f"📊 상세 차트 저장: experiment_summary.png")
        
        # Additional summary table
        print("\n🏆 성능 순위:")
        print("="*50)
        for i, (planner, freq) in enumerate(planner_means.items(), 1):
            trials = df[df['planner_name'] == planner].shape[0]
            print(f"{i}. {planner}: {freq:.1f} Hz (±{planner_stds[planner]:.1f}, {trials}회 시행)")
        
        # Save detailed summary
        planner_summary.to_csv(output_path / 'detailed_summary.csv')
        print(f"📋 상세 통계 저장: detailed_summary.csv")

def main():
    parser = argparse.ArgumentParser(description='간단한 rosbag 실험 분석')
    parser.add_argument('--bag_dir', required=True, help='rosbag 디렉토리')
    parser.add_argument('--output_dir', default='analysis_simple', help='결과 저장 디렉토리')
    
    args = parser.parse_args()
    
    analyzer = SimpleBagAnalyzer()
    analyzer.analyze_bag_directory(args.bag_dir, args.output_dir)

if __name__ == '__main__':
    main()