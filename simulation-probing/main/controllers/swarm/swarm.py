import cv2
import ast
import json
import asyncio
import numpy as np
import random
import yaml
import os
from rich.pretty import pprint
from controller import Robot, Camera, Motor, Display, Supervisor
from swarmtools import FormationMaster
from swarmtools import ObjectDetector
from swarmtools import Communicator
from swarmtools import Driver

PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}


""" 
NOTES:
Uncomment line 252 in driver.py to enable live plotting of the robot's path + waypoints, disable live plotting will allow for FASTER SIMULATION
"""

from datetime import datetime

def increment_test_counter():
    """Read, increment, and save the test run counter."""
    counter_file = "test_counter.yaml"
    
    # Read current counter value
    if os.path.exists(counter_file):
        with open(counter_file, 'r') as f:
            data = yaml.safe_load(f)
            current_count = data.get('test_runs', 0)
    else:
        current_count = 0
    
    # Increment counter
    new_count = current_count + 1
    
    # Save updated counter
    with open(counter_file, 'w') as f:
        yaml.dump({'test_runs': new_count}, f)
    
    print(f"[TEST_COUNTER] Test run #{new_count}")
    return new_count

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

class SwarmMember:
    def __init__(self, test_run_number, mode=0, verbose=False):
        # Instantiate the robot & big objects
        
        
        self.robot = Robot()
        # For Data Collection
        self.data_collector = DataCollector(self.robot.getName(), test_run_number)
        self.timestep = int(self.robot.getBasicTimeStep())
        self.verbose = verbose
        self.object_detector = ObjectDetector(self.robot)
        self.communicator = Communicator(self.robot)
        self.driver = Driver(self.robot)
        self.tick = 0
        # Computer vision
        self.detected_flag = False

        # Retrieve robot parameters
        self.robot_name = self.robot.getName()
        self.mode = mode
        self.priority_queue = PRIORITY_LIST
        self.communicator.robot_entries[self.robot_name] = (
            self.driver.robot_position["x"],
            self.driver.robot_position["y"],
            self.driver.robot_position["theta"],
            self.driver.robot_position["x"],
            self.driver.robot_position["y"],
            self.driver.robot_position["theta"],
        )

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.path = None
        self.status_prev = None
        self.status = None
        self.reassign_flag = False


    def print_position(self):
        print(f"[helper]({self.robot.getName()}) Robot X position: {self.driver.robot_position['x']:6.3f}    Robot Y position: {self.driver.robot_position['y']:6.3f}    Robot Theta position: {self.driver.robot_position['theta']:6.3f}")

    def path_finding(self):
        print(f"[path_finding]({self.robot.getName()}) calculating...")
        self.data_collector.collect_data("path_finding", str(datetime.now()))
        paths_json = self.formation_object()

        self.communicator.broadcast_message("[path_following]", paths_json)

        if self.path == None:
            self.path = self.communicator.path
        if self.verbose:
            print(f"[{self.status}]{self.robot_name}: {self.path}")  # big print
        
        paths = ast.literal_eval(paths_json)
        if self.robot_name in paths.keys():
            self.path = paths.get(self.robot_name, "")
        self.communicator.path = self.path # Sync with communicator

    def random_movement_find(self):
        
        print(f"{self.priority_queue} from {self.robot_name}")
        while self.robot.step(self.timestep) != -1:
            # Check for incoming messages
            status = self.communicator.listen_to_message()
            if status != None:
                self.status = status
            if self.status_prev != self.status or self.verbose:
                print(f"[{self.status}]({self.robot.getName()}) CHANGED")

            if self.object_detector.detect() and not self.detected_flag:
                self.data_collector.collect_data("object_detected_start", str(datetime.now()))
                # * As a member that found the object becomes the master.
                print(
                    f"[object_detected]({self.robot.getName()}) found cylinder @ {cylinder_position}"
                )
                # self.status = "path_finding"  
                self.status = "consensus"  
                self.driver.stop()

            # print(self.robot.getName(),f'{status=}')
            if self.status == "idle":
                self.data_collector.collect_data("idle", str(datetime.now()))
                self.driver.stop()

            elif self.status == "consensus" and not self.detected_flag:
                self.data_collector.collect_data("consensus", str(datetime.now()))
                self.detected_flag = True  # detect once and top
                self.task_master = self.robot_name
                self.communicator.task_master = self.robot_name

                print(f"[consensus]({self.robot.getName()}) waiting consensus...")
                self.communicator.broadcast_message("[task]", cylinder_position)

            elif self.status == "path_finding" and self.task_master == self.robot_name:
                self.data_collector.collect_data("path_finding", str(datetime.now()))
                # Used only by the TaskMaster
                self.path_finding()
                self.status = "path_following"
                # self.status = "idle"

            elif self.status == "path_following":
                self.data_collector.collect_data("path_following", str(datetime.now()))
                list_waypoint =  list(self.communicator.path.values())
                
                # Sampling
                self.driver.sorted_waypoints = list(self.communicator.path.values())[::45]
                # self.driver.sorted_waypoints = []
                self.driver.sorted_waypoints.append(list_waypoint[-1])
                print(f"[path_printing_reduced]({self.robot_name}) {self.driver.sorted_waypoints}")
                # if self.robot_name != "TurtleBot3Burger_1":
                #     quit()

                # self.driver.stop()
                if self.path != "":
                    # self.driver.move_forward()
                    # self.driver.simple_follow_path(self.communicator.path)
                    # for i in self.driver.sorted_waypoints:
                    #     print(i)
                    
                    self.driver.pid_path_follow()
                    # self.driver.anti_clockwise_spin()
                    quit()
                    # self.driver.stop()
                self.status = "idle"
                #     print(
                #         f"[path_following]({self.robot.getName()}) Making my way downtown, walking fast"
                #     )
                # else:
                #     print(f"[path_following]({self.robot.getName()}) FUCK")

                # break

            elif self.status == "task":
                if self.detected_flag:
                    print(f"[task_conflict]({self.robot.getName()})")
                    self.data_collector.collect_data("task_conflict", str(datetime.now()))
                    self.communicator.broadcast_message("[task_conflict]", self.priority_queue)
                else:
                    print(f"[task_successful]({self.robot.getName()})")
                    self.task_master = self.communicator.task_master
                    self.data_collector.collect_data("task_successful", str(datetime.now()))
                    self.communicator.broadcast_message("[task_successful]", self.task_master)

                self.status = "idle"

            elif self.status == "reassign" and not self.reassign_flag:
                self.data_collector.collect_data("reassign", str(datetime.now()))
                task_master = self.priority_queue.pop(0)
                if task_master == self.robot_name:
                    self.path_finding()
                    self.status = "path_finding"
                    self.path = self.communicator.path

                    if self.path != "":
                        # self.driver.move_forward()
                        self.driver.simple_follow_path(self.path)
                        # self.driver.anti_clockwise_spin()
                        quit()
                        # self.driver.stop()
                    self.status = "idle"
                else:
                    self.task_master = task_master
                    self.communicator.task_master = task_master
                    self.communicator.broadcast_message("[task_successful]", self.task_master)
                    self.status = "idle"
                self.priority_queue.append(task_master)
                self.reassign_flag = True

            else:
                self.data_collector.collect_data("random_movement", str(datetime.now()))
                self.driver.move_along_polynomial() # option for driving 1
                # self.driver.move_forward() # option for driving 2
                self.communicator.send_position(
                    robot_position={
                        "x": self.driver.robot_position["x"],
                        "y": self.driver.robot_position["y"],
                        "theta": self.driver.robot_position["theta"],
                    }
                )


            self.communicator.robot_entries[self.robot_name] = (
                self.driver.robot_position["x"],
                self.driver.robot_position["y"],
                self.driver.robot_position["theta"],
            )

    def formation_object(self):
        if self.detected_flag:
            coords = self.communicator.robot_entries.copy()
            for robot_name in coords:
                coords[robot_name] = list(
                    map(lambda x: round(x, 2), coords[robot_name])
                )
            # print(f"[path_finding]({self.robot_name}): current coords={coords}")

            #! Obstacles are in a list format e.g. [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]; should be input from map so leaving it empty for now
            self.formationer = FormationMaster(
                robot=self.robot,
                current_coords=coords,
                object_coords=(cylinder_position["x"], cylinder_position["y"]),
                obstacles=[],
                debug_level=0,
            )
            self.formationer.calculate_target_coords()
            self.formationer.plan_paths()

            paths = json.dumps(self.formationer.paths)
            self.path = json.loads(paths)[self.robot_name]

            return paths
        else:
            # print(
            #     f"[path_finding]({self.robot.getName()}) LISTENING for {cylinder_position}"
            # )
            return None
        


def main():
    import threading
    import traceback
    class bcolors:
        RED_FAIL       = '\033[91m'
        GRAY_OK        = '\033[90m'
        GREEN_OK       = '\033[92m'
        YELLOW_WARNING = '\033[93m'
        BLUE_OK        = '\033[94m'
        MAGENTA_OK     = '\033[95m'
        CYAN_OK        = '\033[96m'
        ENDC           = '\033[0m'
        BOLD           = '\033[1m'
        ITALIC         = '\033[3m'
        UNDERLINE      = '\033[4m'
    
    # Increment test run counter
    test_run_number = increment_test_counter()
        
    member = SwarmMember(test_run_number)
    localisation_service = threading.Thread(target=member.driver.run_odometry_service)

    if member.driver.check_encoder_not_null_and_init():
        localisation_service.start()
        try:
            member.random_movement_find()
        except:
            tb_str = traceback.format_exc()
            print(bcolors.RED_FAIL + tb_str + bcolors.ENDC)
            member.driver.alive = False
            member.data_collector.save_data()
            quit()
        member.driver.alive = False


main()