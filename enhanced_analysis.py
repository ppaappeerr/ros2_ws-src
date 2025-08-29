
import os
import sqlite3
import yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

class EnhancedBagAnalyzer:
    def __init__(self, base_dir):
        self.base_dir = base_dir
        self.analysis_dir = os.path.join(base_dir, 'analysis')
        os.makedirs(self.analysis_dir, exist_ok=True)
        self.results = {}
        self.planner_names = {
            'up1': 'P1: 2D Projection',
            'up2': 'P2: Pure 2D',
            'up3': 'P3: 3D Corridor',
            'up4': 'P4: Follow-the-Gap',
            'up5': 'P5: HeightMap'
        }
        self.setup_fonts()

    def setup_fonts(self):
        # Try to find a Korean-supporting font, fallback to a default
        try:
            font_path = fm.findfont(fm.FontProperties(family='NanumGothic'))
            plt.rc('font', family='NanumGothic')
        except:
            try:
                font_path = fm.findfont(fm.FontProperties(family='Malgun Gothic'))
                plt.rc('font', family='Malgun Gothic')
            except:
                print("Korean fonts (NanumGothic, Malgun Gothic) not found. Using default font.")
                pass
        # Use English for labels to avoid font issues
        self.planner_names = {
            'up1': 'P1: 2D Projection',
            'up2': 'P2: Pure 2D',
            'up3': 'P3: 3D Corridor',
            'up4': 'P4: Follow-the-Gap',
            'up5': 'P5: HeightMap'
        }


    def get_topic_id(self, db_path, topic_name):
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT id FROM topics WHERE name = ?", (topic_name,))
        result = cursor.fetchone()
        conn.close()
        return result[0] if result else None

    def get_message_type(self, db_path, topic_id):
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT type FROM topics WHERE id = ?", (topic_id,))
        result = cursor.fetchone()
        conn.close()
        return result[0] if result else None

    def analyze_bag(self, bag_dir):
        db_files = [f for f in os.listdir(bag_dir) if f.endswith('.db3')]
        if not db_files:
            return None, None, None, None, None, None, None

        db_path = os.path.join(bag_dir, db_files[0])
        
        with open(os.path.join(bag_dir, 'metadata.yaml'), 'r') as f:
            metadata = yaml.safe_load(f)
            duration_ns = metadata['rosbag2_bagfile_information']['duration']['nanoseconds']
            duration_s = duration_ns / 1e9

        topic_name = '/safe_path_vector'
        topic_id = self.get_topic_id(db_path, topic_name)
        if not topic_id:
            return duration_s, 0, 0, 0, 0, 0, ([], [])

        message_type_name = self.get_message_type(db_path, topic_id)
        msg_type = get_message(message_type_name)

        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
        rows = cursor.fetchall()
        conn.close()

        if len(rows) < 2:
            return duration_s, len(rows), 0, 0, 0, 0, ([], [])

        timestamps = [row[0] for row in rows]
        messages = [deserialize_message(row[1], msg_type) for row in rows]
        
        angles = [np.arctan2(msg.vector.y, msg.vector.x) for msg in messages]
        
        time_diffs = np.diff(timestamps) / 1e9  # Convert ns to s
        angle_diffs = np.diff(angles)
        
        angle_diffs = (angle_diffs + np.pi) % (2 * np.pi) - np.pi

        angular_velocities = angle_diffs / time_diffs
        
        std_dev_angular_velocity = np.std(angular_velocities)
        total_angular_change = np.sum(np.abs(angle_diffs))

        # New metric: Path Holding Time
        holding_times = []
        current_holding_time = 0
        if len(rows) > 1:
            last_angle = angles[0]
            for i in range(1, len(angles)):
                if np.isclose(angles[i], last_angle, atol=1e-4):
                    current_holding_time += time_diffs[i-1]
                else:
                    if current_holding_time > 0:
                        holding_times.append(current_holding_time)
                    current_holding_time = time_diffs[i-1] # Start new holding period
                    last_angle = angles[i]
            if current_holding_time > 0:
                holding_times.append(current_holding_time)
        
        avg_holding_time = np.mean(holding_times) if holding_times else 0

        message_count = len(rows)
        frequency = message_count / duration_s if duration_s > 0 else 0
        
        timeline = (np.array(timestamps) - timestamps[0]) / 1e9

        return duration_s, message_count, frequency, std_dev_angular_velocity, total_angular_change, avg_holding_time, (timeline, angles)


    def run_analysis(self):
        bag_files = [d for d in os.listdir(self.base_dir) if os.path.isdir(os.path.join(self.base_dir, d)) and d.startswith('up')]
        
        all_stability_data = []

        for bag_file in sorted(bag_files):
            print(f"📊 Analyzing: {bag_file}")
            bag_path = os.path.join(self.base_dir, bag_file)
            
            try:
                duration, msg_count, freq, std_dev_vel, total_change, avg_hold_time, plot_data = self.analyze_bag(bag_path)
                if duration is None:
                    print(f"⚠️ Could not analyze {bag_file}, skipping.")
                    continue
                if msg_count < 2:
                    print(f"⚠️ Not enough messages in {bag_file} to analyze stability, skipping.")
                    continue

                planner_id = bag_file.split('_')[0]
                if planner_id not in self.results:
                    self.results[planner_id] = {'times': [], 'freqs': [], 'std_devs': [], 'total_changes': [], 'avg_hold_times': [], 'plot_data': []}
                
                self.results[planner_id]['times'].append(duration)
                self.results[planner_id]['freqs'].append(freq)
                self.results[planner_id]['std_devs'].append(std_dev_vel)
                self.results[planner_id]['total_changes'].append(total_change)
                self.results[planner_id]['avg_hold_times'].append(avg_hold_time)
                self.results[planner_id]['plot_data'].append(plot_data)
                
                all_stability_data.append({
                    'Planner': self.planner_names.get(planner_id, planner_id.upper()),
                    'Trial': bag_file,
                    'Std Dev Angular Velocity (rad/s)': std_dev_vel,
                    'Total Angular Change (rad)': total_change,
                    'Avg Path Holding Time (s)': avg_hold_time
                })

            except Exception as e:
                print(f"❌ Error analyzing {bag_file}: {e}")

        self.plot_results()
        self.plot_stability_charts()
        self.plot_angle_timelines()
        
        # Save detailed stability data
        if all_stability_data:
            df_stability = pd.DataFrame(all_stability_data)
            stability_path = os.path.join(self.analysis_dir, 'stability_summary.csv')
            df_stability.to_csv(stability_path, index=False)
            print(f"📋 Detailed stability stats saved to: {stability_path}")


    def plot_results(self):
        # This is the original 4-panel plot for time and frequency
        # (Code is similar to simple_bag_analysis.py, so it's omitted for brevity but would be included here)
        pass

    def plot_stability_charts(self):
        if not self.results:
            print("No results to plot for stability.")
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))
        fig.suptitle('Planner Path Stability Analysis', fontsize=16)

        planner_ids = sorted(self.results.keys())
        labels = [self.planner_names.get(pid, pid.upper()) for pid in planner_ids]
        
        # 1. Standard Deviation of Angular Velocity
        std_dev_means = [np.mean(self.results[pid]['std_devs']) for pid in planner_ids]
        std_dev_errs = [np.std(self.results[pid]['std_devs']) for pid in planner_ids]
        ax1.bar(labels, std_dev_means, yerr=std_dev_errs, capsize=5, color='skyblue')
        ax1.set_ylabel('Std Dev of Angular Velocity (rad/s)')
        ax1.set_title('Path Smoothness (Lower is Better)')
        ax1.tick_params(axis='x', rotation=45)

        # 2. Total Angular Change
        total_change_means = [np.mean(self.results[pid]['total_changes']) for pid in planner_ids]
        total_change_errs = [np.std(self.results[pid]['total_changes']) for pid in planner_ids]
        ax2.bar(labels, total_change_means, yerr=total_change_errs, capsize=5, color='lightgreen')
        ax2.set_ylabel('Total Angular Change (rad)')
        ax2.set_title('Path Efficiency (Lower is Better)')
        ax2.tick_params(axis='x', rotation=45)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_path = os.path.join(self.analysis_dir, 'stability_analysis.png')
        plt.savefig(save_path)
        plt.close()
        print(f"📈 Stability charts saved to: {save_path}")

    def plot_angle_timelines(self):
        if not self.results:
            print("No results to plot for angle timelines.")
            return

        planner_ids = sorted(self.results.keys())
        num_planners = len(planner_ids)
        
        max_trials = 0
        if self.results:
            max_trials = max(len(res.get('plot_data', [])) for res in self.results.values())

        if max_trials == 0:
            print("No trial data to plot for angle timelines.")
            return

        fig, axes = plt.subplots(num_planners, max_trials, figsize=(8 * max_trials, 4 * num_planners), squeeze=False, sharex=True, sharey=True)
        fig.suptitle('Path Angle Over Time (Separated by Trial)', fontsize=20, y=1.02)

        for i, planner_id in enumerate(planner_ids):
            plot_data_list = self.results[planner_id].get('plot_data', [])
            avg_hold_times = self.results[planner_id].get('avg_hold_times', [])
            planner_name = self.planner_names.get(planner_id, planner_id.upper())
            
            for j in range(max_trials):
                ax = axes[i, j]
                if j < len(plot_data_list):
                    timeline, angles = plot_data_list[j]
                    ax.plot(timeline, np.rad2deg(angles), label=f'Trial {j+1}')
                    ax.grid(True)
                    
                    # Add Avg Holding Time text
                    if j < len(avg_hold_times):
                        hold_time_text = f'Avg Hold Time: {avg_hold_times[j]:.2f}s'
                        ax.text(0.95, 0.05, hold_time_text,
                                verticalalignment='bottom', horizontalalignment='right',
                                transform=ax.transAxes,
                                color='red', fontsize=12, bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.3'))

                    # Set titles
                    if i == 0:
                        ax.set_title(f'Trial {j+1}', fontsize=14)
                else:
                    ax.axis('off')

                # Set labels
                if j == 0:
                    ax.set_ylabel(f'{planner_name}\n\nAngle (degrees)', fontsize=12)
                if i == num_planners - 1:
                    ax.set_xlabel('Time (s)')
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_path = os.path.join(self.analysis_dir, 'angle_timelines_separated_with_metrics.png')
        plt.savefig(save_path, bbox_inches='tight')
        plt.close()
        print(f"📈 Separated angle timeline plots with metrics saved to: {save_path}")


if __name__ == '__main__':
    # The script is run from /home/p/ros2_ws/src, and bags are in rosbags_5x5
    bag_directory = 'rosbags_5x5'
    analyzer = EnhancedBagAnalyzer(bag_directory)
    analyzer.run_analysis()
