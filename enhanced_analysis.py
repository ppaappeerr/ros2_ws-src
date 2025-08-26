import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import re
import math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from scipy.signal import welch
from transforms3d.euler import quat2euler

# ==============================================================================
# ROSBAG READING UTILITY
# ==============================================================================

def read_rosbag_topic(bag_file_path, topic_name):
    """Reads all messages from a specific topic in a rosbag file."""
    try:
        storage_options = StorageOptions(uri=str(bag_file_path), storage_id='mcap')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        if topic_name not in type_map:
            return []

        msg_type = get_message(type_map[topic_name])
        messages = []
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(data, msg_type)
                messages.append((timestamp, msg))
        return messages
    except Exception as e:
        print(f"Error reading bag file {bag_file_path}: {e}")
        return []

# ==============================================================================
# METRIC CALCULATION FUNCTIONS
# ==============================================================================

def calculate_common_metrics(bag_path):
    """Calculates metrics applicable to all planner types."""
    path_data = read_rosbag_topic(bag_path, '/safe_path_vector')
    if not path_data:
        return {}

    timestamps, msgs = zip(*path_data)
    start_time = timestamps[0]
    times = np.array([(t - start_time) / 1e9 for t in timestamps])
    
    # Convert angles to degrees, adjusting for -X front
    angles = np.array([math.degrees(math.atan2(msg.vector.y, msg.vector.x)) for msg in msgs])
    
    metrics = {}
    # Stability
    metrics['path_angle_std'] = np.std(angles)

    # Smoothness
    d_angle = np.diff(angles)
    d_time = np.diff(times)
    angular_velocity = np.divide(d_angle, d_time, out=np.zeros_like(d_angle), where=d_time!=0)
    metrics['angular_velocity_rms'] = np.sqrt(np.mean(angular_velocity**2))

    # Centrality Bias (Deviation from -X axis, which is 180 or -180 degrees)
    # We use 180 degrees and calculate the shortest angular distance
    angles_rad = np.deg2rad(angles)
    target_rad = np.pi
    abs_diff = np.abs(np.arctan2(np.sin(angles_rad - target_rad), np.cos(angles_rad - target_rad)))
    metrics['centrality_bias_mae'] = np.rad2deg(np.mean(abs_diff))

    # High-Frequency Noise (PSD)
    fs = 20  # Assume a reasonable sampling frequency (e.g., 20 Hz)
    if len(times) > 1 and times[-1] > 0:
        fs = len(times) / times[-1] # More robust calculation of avg sampling rate
    
    time_interp = np.arange(times[0], times[-1], 1/fs)
    if len(time_interp) > 1:
        angles_interp = np.interp(time_interp, times, angles)
        freqs, psd = welch(angles_interp, fs=fs, nperseg=min(len(angles_interp), 128))
        high_freq_mask = freqs > 1.0  # Jitter is typically > 1Hz
        if np.any(high_freq_mask):
            metrics['psd_hf_noise'] = np.trapz(psd[high_freq_mask], freqs[high_freq_mask])
        else:
            metrics['psd_hf_noise'] = 0
    else:
        metrics['psd_hf_noise'] = 0


    return metrics

def calculate_ftg_metrics(bag_path):
    """Calculates metrics specific to the Follow-the-Gap planner."""
    # Placeholder: This would involve parsing /ftg_gaps MarkerArrays
    # For now, we return dummy data.
    return {'avg_gap_width': np.nan, 'avg_gap_depth': np.nan}

def calculate_heightmap_metrics(bag_path):
    """Calculates metrics specific to the HeightMap planner."""
    pillar_data = read_rosbag_topic(bag_path, '/height_pillars')
    if not pillar_data:
        return {'drop_pillar_count': 0, 'obstacle_pillar_count': 0}
    
    # In MarkerArray, each marker is in the .markers list
    markers = [marker for _, msg in pillar_data for marker in msg.markers]
    
    # Filter for DROP pillars (Yellow to Red, check green component near zero)
    drop_pillars = [m for m in markers if m.color.g < 0.1 and m.color.r > 0.5]
    # Filter for OBSTACLE pillars (Orange to Crimson, check blue component near zero)
    obstacle_pillars = [m for m in markers if m.color.b < 0.1 and m.color.r > 0.5]

    return {
        'drop_pillar_count': len(drop_pillars),
        'obstacle_pillar_count': len(obstacle_pillars)
    }

# ==============================================================================
# PLOTTING AND REPORTING
# ==============================================================================

def generate_plots(df, output_dir):
    """Generates and saves various comparison plots."""
    if df.empty:
        print("DataFrame is empty, skipping plot generation.")
        return

    print(f"Generating plots in {output_dir}...")
    output_dir.mkdir(exist_ok=True)

    # Set plot style
    sns.set_theme(style="whitegrid")

    # Plot each metric
    metrics_to_plot = [
        'path_angle_std', 'angular_velocity_rms', 
        'centrality_bias_mae', 'psd_hf_noise',
        'drop_pillar_count', 'obstacle_pillar_count'
    ]
    
    for metric in metrics_to_plot:
        if metric not in df.columns or df[metric].isnull().all():
            continue
        
        plt.figure(figsize=(12, 8))
        sns.boxplot(data=df, x='mode', y=metric, order=['A', 'B', 'C', 'D', 'E'])
        sns.stripplot(data=df, x='mode', y=metric, order=['A', 'B', 'C', 'D', 'E'], color=".25", alpha=0.6)
        plt.title(f'Comparison of {metric.replace("_", " ").title()}')
        plt.ylabel(metric)
        plt.xlabel('Planner Mode')
        
        filename = output_dir / f"plot_{metric}_comparison.png"
        plt.savefig(filename)
        plt.close()
        print(f"Saved {filename}")

def generate_report(df, output_dir):
    """Generates a summary markdown report."""
    if df.empty:
        print("DataFrame is empty, skipping report generation.")
        return
        
    print(f"Generating report in {output_dir}...")
    report_path = output_dir / "analysis_report.md"
    
    summary = df.groupby('mode').agg(['mean', 'std', 'min', 'max']).round(3)
    
    with open(report_path, 'w') as f:
        f.write("# Analysis Report\n\n")
        f.write(f"Analysis of {len(df)} runs from directory: `{output_dir.resolve()}`\n\n")
        f.write("## Summary Statistics\n\n")
        f.write(summary.to_markdown())
        f.write("\n\n## Plots\n\n")
        for plot_file in sorted(output_dir.glob("plot_*.png")):
            f.write(f"### {plot_file.stem.replace('_', ' ').title()}\n")
            f.write(f"![{plot_file.name}]({plot_file.name})\n\n")
            
    print(f"Saved {report_path}")

# ==============================================================================
# MAIN EXECUTION
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Enhanced analysis script for the Optical Cane project.")
    parser.add_argument(
        '--input_dir', 
        type=str, 
        required=True, 
        help="Directory containing the rosbag files (e.g., 'results/2025-08-23/raw_bags')."
    )
    parser.add_argument(
        '--output_dir', 
        type=str, 
        default=None,
        help="Directory to save plots and reports. Defaults to a new folder inside the input directory."
    )
    args = parser.parse_args()

    input_path = Path(args.input_dir)
    if not input_path.is_dir():
        print(f"Error: Input directory not found at {input_path}")
        return

    if args.output_dir:
        output_path = Path(args.output_dir)
    else:
        output_path = input_path.parent / "analysis_results"
    output_path.mkdir(exist_ok=True)

    # Discover bag files and parse metadata
    all_results = []
    bag_files = sorted(list(input_path.glob('**/*.mcap')))
    print(f"Found {len(bag_files)} bag files to analyze.")

    for bag_file in bag_files:
        match = re.search(r'bag_\d+_\*([A-E])(\d+)', bag_file.name)
        if not match:
            print(f"Skipping file with non-standard name: {bag_file.name}")
            continue
        
        mode, run = match.groups()
        run = int(run)
        
        print(f"--- Analyzing Mode {mode}, Run {run} ({bag_file.name}) ---")
        
        # Calculate metrics
        metrics = {'mode': mode, 'run': run}
        metrics.update(calculate_common_metrics(bag_file))
        
        if mode == 'D':
            metrics.update(calculate_ftg_metrics(bag_file))
        elif mode == 'E':
            metrics.update(calculate_heightmap_metrics(bag_file))
            
        all_results.append(metrics)

    # Convert to DataFrame for easier analysis
    df = pd.DataFrame(all_results)
    
    # Save raw data to CSV
    csv_path = output_path / "metrics_summary.csv"
    df.to_csv(csv_path, index=False)
    print(f"Saved raw metrics to {csv_path}")

    # Generate visual outputs
    generate_plots(df, output_path)
    generate_report(df, output_path)
    
    print("\nAnalysis complete.")
    print(f"Results are in: {output_path.resolve()}")


if __name__ == '__main__':
    main()
