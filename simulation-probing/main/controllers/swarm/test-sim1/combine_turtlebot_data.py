#!/usr/bin/env python3
"""
Script to combine all TurtleBot test data from JSON files in the root directory.
Reads JSON files and combines them into a single CSV format grouped by test run.
"""
import os
import json
import pandas as pd
from datetime import datetime
import glob

def load_turtlebot_data_from_root():
    """Load JSON data for all turtlebots from numbered folders."""
    # Use the directory of this script as the root
    root_dir = os.path.dirname(__file__)
    
    all_test_data = {}  # Will store data grouped by test_run
    
    # Find all folders that are just numbers
    numbered_folders = []
    for item in os.listdir(root_dir):
        item_path = os.path.join(root_dir, item)
        if os.path.isdir(item_path) and item.isdigit():
            numbered_folders.append(item)
    
    # Sort folders numerically
    numbered_folders.sort(key=int)
    
    if not numbered_folders:
        print("Warning: No numbered folders found")
        return all_test_data
    
    print(f"Found numbered folders: {numbered_folders}")
    
    # Go through all numbered folders
    for folder_num in numbered_folders:
        folder_path = os.path.join(root_dir, folder_num)
        
        print(f"Processing folder {folder_num}...")
        
        # Each folder should contain 3 JSON files for the 3 turtlebots
        for robot_name in ['TurtleBot3Burger_1', 'TurtleBot3Burger_2', 'TurtleBot3Burger_3']:
            json_file = os.path.join(folder_path, f"{robot_name}.json")
            if os.path.exists(json_file):
                with open(json_file, 'r') as f:
                    data = json.load(f)
                    test_run = folder_num  # Use folder name as test_run
                    
                    if test_run not in all_test_data:
                        all_test_data[test_run] = {}
                    
                    all_test_data[test_run][robot_name] = data
            else:
                print(f"Warning: {json_file} not found")
    
    return all_test_data

def combine_all_test_data():
    """Combine data from all test runs into a single DataFrame."""
    all_data = []
    
    # Load all test data grouped by test run
    test_data = load_turtlebot_data_from_root()
    
    # Sort test runs numerically
    sorted_test_runs = sorted(test_data.keys(), key=lambda x: int(x) if str(x).isdigit() else float('inf'))
    
    for test_run in sorted_test_runs:
        print(f"Processing test run {test_run}...")
        
        turtlebot_data = test_data[test_run]
        
        # Get all unique keys across all turtlebots (excluding test_run)
        all_keys = set()
        for robot_data in turtlebot_data.values():
            all_keys.update(robot_data.keys())
        all_keys.discard('test_run')  # Remove test_run from keys
        
        # Create row for this test run
        row = {'test_run': test_run}
        
        # Add columns for each turtlebot and each key
        for robot_name in ['TurtleBot3Burger_1', 'TurtleBot3Burger_2', 'TurtleBot3Burger_3']:
            robot_data = turtlebot_data.get(robot_name, {})
            for key in sorted(all_keys):
                column_name = f"{robot_name}_{key}"
                row[column_name] = robot_data.get(key, '')
        
        all_data.append(row)
    
    return pd.DataFrame(all_data)

def create_summary_report():
    """Create a summary report of all test runs."""
    df = combine_all_test_data()
    
    if df.empty:
        print("No test data found!")
        return
    
    # Save combined data to CSV
    output_file = "combined_turtlebot_data.csv"
    df.to_csv(output_file, index=False)
    print(f"Combined data saved to {output_file}")
    
    # Print summary statistics
    print(f"\nSummary:")
    print(f"Total test runs: {len(df)}")
    print(f"Columns: {len(df.columns)}")
    print(f"Test runs processed: {', '.join(df['test_run'].astype(str))}")
    
    # Show sample of data
    print(f"\nSample data (first 5 rows):")
    print(df.head().to_string())
    
    # Create timing analysis if timing data exists
    timing_columns = [col for col in df.columns if any(timing_key in col for timing_key in 
                     ['object_detected_start', 'consensus', 'path_finding', 'path_following'])]
    
    if timing_columns:
        print(f"\nTiming Analysis:")
        timing_df = df[['test_run'] + timing_columns]
        print(timing_df.to_string())

def create_individual_robot_reports():
    """Create separate reports for each robot."""
    # Load all test data
    test_data = load_turtlebot_data_from_root()
    
    for robot_name in ['TurtleBot3Burger_1', 'TurtleBot3Burger_2', 'TurtleBot3Burger_3']:
        robot_data = []
        
        # Sort test runs numerically
        sorted_test_runs = sorted(test_data.keys(), key=lambda x: int(x) if str(x).isdigit() else float('inf'))
        
        for test_run in sorted_test_runs:
            if robot_name in test_data[test_run]:
                data = test_data[test_run][robot_name].copy()
                data['robot_name'] = robot_name
                robot_data.append(data)
        
        if robot_data:
            robot_df = pd.DataFrame(robot_data)
            output_file = f"{robot_name}_data.csv"
            robot_df.to_csv(output_file, index=False)
            print(f"Individual data for {robot_name} saved to {output_file}")

if __name__ == "__main__":
    print("TurtleBot Test Data Combiner")
    print("=" * 40)
    
    # Create combined report
    create_summary_report()
    
    # Create individual robot reports
    create_individual_robot_reports()
    
    print("\nData combination complete!") 