from datetime import datetime
import json
class DataCollector:
    def __init__(self, robot_name, test_run_number):
        self.data : dict[str, datetime] = dict()
        self.robot_name = robot_name
        self.test_run_number = test_run_number
        
        # Save JSON files in root directory like before
        self.data_file = f"{robot_name}.json"
    
    def collect_data(self, key: str, value: datetime):
        self.data[key] = value
        
    
    def save_data(self):
        # Convert datetime objects to strings for JSON serialization
        serializable_data = {}
        for key, value in self.data.items():
            if isinstance(value, datetime):
                serializable_data[key] = value.isoformat()
            else:
                serializable_data[key] = str(value)
        
        # Add test run number to the data
        serializable_data['test_run'] = self.test_run_number
        
        with open(self.data_file, "w") as f:
            json.dump(serializable_data, f, indent=2)

if __name__ == "__main__":
    data_collector = DataCollector("test", 1)
    data_collector.collect_data("test", datetime.now())
    data_collector.save_data()