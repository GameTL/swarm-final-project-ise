# TurtleBot Test Data Collection and Analysis

This directory contains the test data collection and analysis system for the swarm robotics project.

## How it works

### Data Collection
- Each time you run `swarm.py`, it automatically increments a test run counter
- Test data is saved as JSON files in the root directory (same level as swarm.py)
- Each robot (TurtleBot3Burger_1, TurtleBot3Burger_2, TurtleBot3Burger_3) overwrites its JSON file with new data
- The test run number is embedded within each JSON file

### File Structure
```
controllers/swarm/
├── swarm.py
├── test_counter.yaml
├── TurtleBot3Burger_1.json  # Latest test data
├── TurtleBot3Burger_2.json  # Latest test data
├── TurtleBot3Burger_3.json  # Latest test data
└── test/
    ├── combine_turtlebot_data.py
    ├── requirements.txt
    └── README.md
```

## Data Analysis

### Running the Analysis Script
To combine and analyze test data:

```bash
cd test
python combine_turtlebot_data.py
```

The script will:
1. Read JSON files from the parent directory (root)
2. Extract test run numbers from within each JSON file
3. Group data by test run number
4. Generate combined CSV reports

### Output Files
The script generates several output files in the `test/` directory:

1. **combined_turtlebot_data.csv**: Combined data from all test runs and all robots
   - Columns: test_run, TurtleBot3Burger_1_<key>, TurtleBot3Burger_2_<key>, TurtleBot3Burger_3_<key>

2. **Individual robot CSV files**:
   - TurtleBot3Burger_1_data.csv
   - TurtleBot3Burger_2_data.csv
   - TurtleBot3Burger_3_data.csv

### Data Keys Collected
The system automatically collects timing data for various robot states:
- `object_detected_start`: When object detection begins
- `consensus`: Consensus phase timing
- `path_finding`: Path planning timing
- `path_following`: Path execution timing
- `idle`: Idle state timing
- `task_conflict`: Task conflict resolution timing
- `task_successful`: Successful task completion timing
- `reassign`: Task reassignment timing
- `random_movement`: Random movement timing
- `test_run`: Test run number (automatically added)

## Usage Example

1. Run your swarm simulation multiple times
2. Each run will overwrite the JSON files with new data (including updated test run number)
3. Run the analysis script to extract and combine historical data
4. Analyze the CSV files to understand robot behavior patterns

**Note:** Since JSON files are overwritten each run, the analysis script extracts test run information from the current files. For comprehensive historical analysis, you may want to manually backup JSON files between runs.

## Requirements

Make sure you have the required dependencies:
```bash
pip install -r requirements.txt
``` 