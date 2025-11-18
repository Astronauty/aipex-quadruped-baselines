#!/usr/bin/env python3
"""
Parse ROS2 log file to extract GRF (Ground Reaction Force) data from MPC solver
and create a CSV file and visualization chart.
"""

import re
import csv
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

def parse_grf_from_log(log_file_path):
    """
    Parse GRF data from ROS2 log file.
    Returns a list of dictionaries with timestamp and GRF values.
    """
    grf_data = []
    
    with open(log_file_path, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i]
        
        # Look for "GRF at k = 0:" pattern
        if "GRF at k = 0:" in line:
            # Extract timestamp from the line
            # Format: timestamp [node] [INFO] [timestamp] [node]: GRF at k = 0:
            timestamp_match = re.search(r'^(\d+\.\d+)', line)
            if timestamp_match:
                timestamp = float(timestamp_match.group(1))
                
                # Read the next 3 lines which contain the GRF matrix
                # Row 1: x forces (FL, FR, RL, RR)
                # Row 2: y forces (FL, FR, RL, RR)
                # Row 3: z forces (FL, FR, RL, RR)
                if i + 3 < len(lines):
                    row1 = lines[i + 1].strip()
                    row2 = lines[i + 2].strip()
                    row3 = lines[i + 3].strip()
                    
                    # Extract numbers from each row
                    # Remove the timestamp prefix if present
                    row1_clean = re.sub(r'^\d+\.\d+\s+\[.*?\]\s*', '', row1)
                    row2_clean = re.sub(r'^\d+\.\d+\s+\[.*?\]\s*', '', row2)
                    row3_clean = re.sub(r'^\d+\.\d+\s+\[.*?\]\s*', '', row3)
                    
                    # Parse the 4 values from each row
                    x_forces = [float(x) for x in row1_clean.split()]
                    y_forces = [float(y) for y in row2_clean.split()]
                    z_forces = [float(z) for z in row3_clean.split()]
                    
                    if len(x_forces) == 4 and len(y_forces) == 4 and len(z_forces) == 4:
                        grf_entry = {
                            'timestamp': timestamp,
                            'FL_x': x_forces[0],
                            'FR_x': x_forces[1],
                            'RL_x': x_forces[2],
                            'RR_x': x_forces[3],
                            'FL_y': y_forces[0],
                            'FR_y': y_forces[1],
                            'RL_y': y_forces[2],
                            'RR_y': y_forces[3],
                            'FL_z': z_forces[0],
                            'FR_z': z_forces[1],
                            'RL_z': z_forces[2],
                            'RR_z': z_forces[3],
                        }
                        grf_data.append(grf_entry)
                
                i += 4  # Skip the 3 matrix rows
            else:
                i += 1
        else:
            i += 1
    
    return grf_data

def create_csv(grf_data, output_csv_path):
    """Write GRF data to CSV file."""
    if not grf_data:
        print("No GRF data found!")
        return
    
    fieldnames = ['timestamp', 'FL_x', 'FR_x', 'RL_x', 'RR_x', 
                  'FL_y', 'FR_y', 'RL_y', 'RR_y', 
                  'FL_z', 'FR_z', 'RL_z', 'RR_z']
    
    with open(output_csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(grf_data)
    
    print(f"CSV file created: {output_csv_path}")
    print(f"Total GRF entries: {len(grf_data)}")

def create_charts(grf_data, output_chart_path):
    """Create visualization charts for GRF data."""
    if not grf_data:
        print("No GRF data to plot!")
        return
    
    df = pd.DataFrame(grf_data)
    
    # Normalize timestamps to start from 0
    df['time_relative'] = df['timestamp'] - df['timestamp'].min()
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 4, figsize=(16, 12))
    fig.suptitle('Ground Reaction Forces (GRF) from MPC Solver', fontsize=16, fontweight='bold')
    
    # Plot x, y, z forces for each foot
    feet = ['FL', 'FR', 'RL', 'RR']
    components = ['x', 'y', 'z']
    
    for foot_idx, foot in enumerate(feet):
        for comp_idx, comp in enumerate(components):
            ax = axes[comp_idx, foot_idx]
            col_name = f'{foot}_{comp}'
            
            ax.plot(df['time_relative'], df[col_name], linewidth=1.5)
            ax.set_title(f'{foot} - {comp.upper()} Force', fontweight='bold')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Force (N)')
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_chart_path, dpi=300, bbox_inches='tight')
    print(f"Chart saved: {output_chart_path}")
    
    # Also create a combined plot showing all z-forces (vertical forces)
    fig2, ax2 = plt.subplots(figsize=(12, 6))
    for foot in feet:
        ax2.plot(df['time_relative'], df[f'{foot}_z'], label=f'{foot}', linewidth=2)
    ax2.set_title('Vertical Ground Reaction Forces (Z-component)', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Force (N)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    z_force_chart_path = str(output_chart_path).replace('.png', '_vertical_forces.png')
    plt.savefig(z_force_chart_path, dpi=300, bbox_inches='tight')
    print(f"Vertical forces chart saved: {z_force_chart_path}")

def main():
    log_file_path = Path("/home/abien/.ros/log/2025-11-18-11-23-39-499094-abien-Alienware-m16-R2-11395/launch.log")
    output_dir = Path("/home/abien/ros2_ws/src/aipex-quadruped-baselines/src/convex_mpc")
    
    if not log_file_path.exists():
        print(f"Error: Log file not found at {log_file_path}")
        return
    
    print(f"Parsing log file: {log_file_path}")
    grf_data = parse_grf_from_log(log_file_path)
    
    if not grf_data:
        print("No GRF data found in log file!")
        return
    
    # Create CSV file
    csv_path = output_dir / "grf_outputs.csv"
    create_csv(grf_data, csv_path)
    
    # Create charts
    chart_path = output_dir / "grf_charts.png"
    create_charts(grf_data, chart_path)
    
    print(f"\nSummary:")
    print(f"  - Total samples: {len(grf_data)}")
    print(f"  - Time range: {grf_data[0]['timestamp']:.3f} to {grf_data[-1]['timestamp']:.3f}")
    print(f"  - Duration: {grf_data[-1]['timestamp'] - grf_data[0]['timestamp']:.3f} seconds")

if __name__ == "__main__":
    main()

