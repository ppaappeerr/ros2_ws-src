import matplotlib.pyplot as plt
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import math
import argparse
from transforms3d.euler import quat2euler
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from scipy.signal import welch

def read_rosbag(bag_file_path, topic_name):
    """Reads messages from a specific topic in a rosbag file (mcap format)."""
    storage_options = StorageOptions(uri=bag_file_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if topic_name not in type_map:
        print(f"Topic {topic_name} not found in {bag_file_path}")
        return []

    msg_type_str = type_map[topic_name]
    msg_type = get_message(msg_type_str)

    messages = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            messages.append((timestamp, msg))
            
    return messages

def analyze_path_stability(ax, all_data):
    """
    Plots path angle over time for stability analysis.
    (Data is pre-processed and trimmed by the caller)
    """
    ax.set_title('Path Stability Analysis (Trimmed)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Path Angle (degrees)')
    ax.grid(True)

    for data in all_data:
        std_dev = np.std(data['angles'])
        ax.plot(data['times'], data['angles'], label=f"{data['label']} (StdDev: {std_dev:.2f})")
    ax.legend()

def analyze_smoothness(all_data, set_num):
    """
    Performs advanced smoothness analysis using Angular Velocity and Power Spectral Density.
    """
    print("\n--- Performing Advanced Smoothness Analysis ---")
    
    # --- 1. Angular Velocity Analysis ---
    fig_vel, (ax_vel, ax_box) = plt.subplots(2, 1, figsize=(12, 10), gridspec_kw={'height_ratios': [3, 2]})
    
    all_velocities = []
    labels = []
    
    for data in all_data:
        # Calculate angular velocity
        d_angle = np.diff(data['angles'])
        d_time = np.diff(data['times'])
        angular_velocity = np.divide(d_angle, d_time, out=np.zeros_like(d_angle), where=d_time!=0)
        
        rms_velocity = np.sqrt(np.mean(angular_velocity**2))
        print(f"'{data['label']}' | RMS Angular Velocity: {rms_velocity:.2f} deg/s")

        ax_vel.plot(data['times'][:-1], angular_velocity, label=f"{data['label']} (RMS: {rms_velocity:.2f} deg/s)")
        all_velocities.append(angular_velocity)
        labels.append(data['label'])

    ax_vel.set_title(f'Angular Velocity Analysis (Test Set {set_num})')
    ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('Angular Velocity (degrees/s)')
    ax_vel.grid(True)
    ax_vel.legend()

    ax_box.boxplot(all_velocities, vert=False, labels=labels)
    ax_box.set_title('Distribution of Angular Velocities')
    ax_box.set_xlabel('Angular Velocity (degrees/s)')
    ax_box.grid(True)
    
    fig_vel.tight_layout()
    vel_filename = f"smoothness_angular_velocity_set_{set_num}.png"
    fig_vel.savefig(vel_filename)
    print(f"Saved {vel_filename}")

    # --- 2. Power Spectral Density Analysis ---
    fig_psd, ax_psd = plt.subplots(figsize=(12, 7))
    
    for data in all_data:
        # Ensure constant sampling rate by interpolation
        fs = 10  # Sampling frequency of 10 Hz
        time_interp = np.arange(data['times'][0], data['times'][-1], 1/fs)
        angles_interp = np.interp(time_interp, data['times'], data['angles'])
        
        # Calculate PSD
        freqs, psd = welch(angles_interp, fs=fs, nperseg=128)
        
        # Calculate power in high-frequency band (> 1 Hz)
        high_freq_mask = freqs > 1
        high_freq_power = np.trapz(psd[high_freq_mask], freqs[high_freq_mask])
        print(f"'{data['label']}' | High-Frequency Power (>1Hz): {high_freq_power:.4f}")

        ax_psd.semilogy(freqs, psd, label=f"{data['label']} (HF Power: {high_freq_power:.4f})")

    ax_psd.set_title(f'Power Spectral Density (Test Set {set_num})')
    ax_psd.set_xlabel('Frequency (Hz)')
    ax_psd.set_ylabel('Power / Frequency (dB/Hz)')
    ax_psd.grid(True)
    ax_psd.legend()
    
    fig_psd.tight_layout()
    psd_filename = f"smoothness_power_spectrum_set_{set_num}.png"
    fig_psd.savefig(psd_filename)
    print(f"Saved {psd_filename}")


def analyze_tilt_robustness(fig, bag_file, pipeline_name):
    """Plots path angle vs. IMU pitch for tilt robustness analysis."""
    # ... (This function remains unchanged)
    imu_topic = '/imu/data'
    path_topic = '/safe_path_vector_3d' if '3d' in pipeline_name.lower() else '/safe_path_vector'

    imu_data = read_rosbag(bag_file, imu_topic)
    path_data = read_rosbag(bag_file, path_topic)

    if not imu_data or not path_data:
        print(f"Skipping tilt analysis for {pipeline_name}: Missing data.")
        return

    path_stamps, path_msgs = zip(*path_data)
    path_times = np.array([(t - path_stamps[0]) / 1e9 for t in path_stamps])
    path_angles = np.array([math.degrees(math.atan2(msg.vector.y, msg.vector.x)) for msg in path_msgs])

    imu_stamps, imu_msgs = zip(*imu_data)
    imu_times = np.array([(t - path_stamps[0]) / 1e9 for t in imu_stamps])
    pitches = []
    for msg in imu_msgs:
        q = msg.orientation
        _, pitch, _ = quat2euler([q.w, q.x, q.y, q.z])
        pitches.append(math.degrees(pitch))
    pitches = np.array(pitches)

    ax1 = fig.add_subplot()
    ax1.set_title(f'Tilt Robustness: {pipeline_name}')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Path Angle (degrees)', color='tab:blue')
    ax1.plot(path_times, path_angles, color='tab:blue', label='Path Angle')
    ax1.tick_params(axis='y', labelcolor='tab:blue')
    ax1.grid(True)
    ax2 = ax1.twinx()
    ax2.set_ylabel('IMU Pitch (degrees)', color='tab:orange')
    ax2.plot(imu_times, pitches, color='tab:orange', label='IMU Pitch', linestyle='--')
    ax2.tick_params(axis='y', labelcolor='tab:orange')
    fig.legend(loc="upper right", bbox_to_anchor=(1,1), bbox_transform=ax1.transAxes)

def analyze_path_accuracy(ax, all_data, ground_truth_angle=0):
    """
    Analyzes path accuracy against a ground truth.
    Assumes a straight corridor scenario (ground truth = 0 degrees).
    """
    print("\n--- Performing Path Accuracy Analysis ---")
    ax.set_title('Path Accuracy Analysis (Error from Ground Truth)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Absolute Error (degrees)')
    ax.grid(True)

    for data in all_data:
        # Calculate absolute error from ground truth
        absolute_error = np.abs(data['angles'] - ground_truth_angle)
        
        # Calculate Mean Absolute Error (MAE)
        mae = np.mean(absolute_error)
        print(f"'{data['label']}' | Mean Absolute Error (MAE): {mae:.2f} degrees")

        ax.plot(data['times'], absolute_error, label=f"{data['label']} (MAE: {mae:.2f})")
    
    ax.legend()

def main():
    parser = argparse.ArgumentParser(description="Analyze ROS2 bags for optical cane project based on a test set number.")
    parser.add_argument('--test_set', type=str, required=True, help='The test set to analyze (e.g., "1" for all, or "1_1", "1_2" for individual bags).')
    
    args = parser.parse_args()
    set_str = args.test_set
    base_path = '.'
    bag_files = []
    labels = []

    all_labels = ["Pipeline1_2D_Projection", "Pipeline2_Pure_2D", "Pipeline3_3D_Corridor"]

    if '_' in set_str:
        # Individual bag analysis (e.g., "1_1")
        parts = set_str.split('_')
        set_num = parts[0]
        pipeline_num = int(parts[1])
        
        bag_files.append(f"{base_path}/test_{set_num}_p{pipeline_num}/test_{set_num}_p{pipeline_num}_0.mcap")
        labels.append(all_labels[pipeline_num - 1])
        
        # Tilt analysis is only meaningful for the specific pipeline being analyzed
        tilt_bag_file = bag_files[0]
        tilt_label = labels[0]
    else:
        # Full set comparison (e.g., "1")
        set_num = set_str
        bag_files = [
            f"{base_path}/test_{set_num}_p1/test_{set_num}_p1_0.mcap",
            f"{base_path}/test_{set_num}_p2/test_{set_num}_p2_0.mcap",
            f"{base_path}/test_{set_num}_p3/test_{set_num}_p3_0.mcap"
        ]
        labels = all_labels
        
        # For full comparison, default tilt analysis to pipeline 3
        tilt_bag_file = bag_files[2]
        tilt_label = labels[2]

    print(f"--- Starting Analysis for Test Set {set_str} ---")

    # --- Pre-processing: Load and Trim Data ---
    all_data = []
    for bag_file, label in zip(bag_files, labels):
        topic_name = '/safe_path_vector_3d' if '3d' in label.lower() else '/safe_path_vector'
        path_data = read_rosbag(bag_file, topic_name)
        if not path_data:
            print(f"ERROR: No data found for topic in {bag_file}. Skipping.")
            continue
        timestamps, msgs = zip(*path_data)
        start_time = timestamps[0]
        times = np.array([(t - start_time) / 1e9 for t in timestamps])
        angles = np.array([math.degrees(math.atan2(msg.vector.y, msg.vector.x)) for msg in msgs])
        all_data.append({'label': label, 'times': times, 'angles': angles})

    if not all_data:
        print("No valid data found for this test set. Exiting.")
        return

    if len(all_data) > 1:
        min_duration = min(d['times'][-1] for d in all_data)
        print(f"Trimming all datasets to the shortest duration: {min_duration:.2f} seconds.")
        for data in all_data:
            mask = data['times'] <= min_duration
            data['times'] = data['times'][mask]
            data['angles'] = data['angles'][mask]

    # --- Stability Plot ---
    fig1, ax1 = plt.subplots(figsize=(12, 7))
    ax1.set_title(f'Path Stability Analysis (Test Set {set_str})')
    analyze_path_stability(ax1, all_data)
    output_filename_stability = f"path_stability_analysis_set_{set_str}.png"
    fig1.savefig(output_filename_stability)
    print(f"Saved {output_filename_stability}")

    # --- Advanced Smoothness Analysis ---
    analyze_smoothness(all_data, set_str)

    # --- Path Accuracy Plot ---
    fig_acc, ax_acc = plt.subplots(figsize=(12, 7))
    analyze_path_accuracy(ax_acc, all_data)
    output_filename_accuracy = f"path_accuracy_analysis_set_{set_str}.png"
    fig_acc.savefig(output_filename_accuracy)
    print(f"Saved {output_filename_accuracy}")

    # --- Tilt Plot ---
    fig2 = plt.figure(figsize=(12, 7))
    analyze_tilt_robustness(fig2, tilt_bag_file, tilt_label)
    output_filename_tilt = f"tilt_robustness_{tilt_label}_set_{set_str}.png"
    fig2.savefig(output_filename_tilt)
    print(f"Saved {output_filename_tilt}")

    plt.show()

if __name__ == '__main__':
    main()
