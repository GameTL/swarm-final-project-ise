from controller import Robot, Camera, Motor, Display, Supervisor
import json
from rich.pretty import pprint
from collections import defaultdict
import time
import ast

EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]

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

class Communicator:
    def __init__(self, robot: Robot, mode=0, verbose=False):
        self.verbose = verbose
        # setting up
        self.robot : Robot = robot

        self.timestep = 64
        self.name = self.robot.getName()
        self.mode = mode
        self.robot_entries = {}
        
        self.emitter = self.robot.getDevice(EMITTER_DEVICE_NAME) # sending info using webots
        self.receiver = self.robot.getDevice(RECEIVER_DEVICE_NAME) # receiving info using webots
        self.receiver.enable(self.timestep)

        self.message_interval = MESSAGE_INTERVAL
        self.time_tracker = 0
        self.priority_queue = PRIORITY_LIST.copy()  # Make a copy to allow modifications

        self.object_coordinates = {}
        self.task_master = ""
        self.path = None
        self.message_id = 0
        
        # Consensus handling - merged from real-world robot
        self.taskmaster_claims = []
        self.consensus_reached = False
        self.obstacle_coords = []  # Should know from lidar/environment

    def listen_to_message(self) -> None | str:
        """ 
d        listen for ['[probe]', '[object_detected]', '[task]', '[task_conflict]', '[task_successful]', '[consensus_ack]']
        """
        # Receive messages from other robots and print
        while self.receiver.getQueueLength() > 0:  
            received_message = self.receiver.getString()
            if self.verbose: self.print_received_message(received_message)
            title, robot_id, message_id, content = json.loads(received_message)
            self.receiver.nextPacket()
            
            # Check for probing message
            if title == "[path_receiving]":
                return "path_receiving"
            elif title == "[probe]":
                self.robot_entries[robot_id] = content
            elif title == "[object_detected]":
                # Another robot detected the object - trigger consensus
                print(f"{bcolors.YELLOW_WARNING}[{self.name}](object_detected): Received OBJECT_DETECTED from {robot_id}{bcolors.ENDC}")
                self.object_coordinates = content
                self.consensus(robot_id)  # Trigger consensus with the sender as claimant
                return "consensus_received"
            elif title == "[task]":
                self.task_master = robot_id
                self.object_coordinates = content
                print(f"[{self.robot.getName()}](task): Object Detected from: {robot_id}@{content}; checking conflict...")
                return "task"
            elif title == "[task_conflict]":
                # Handle conflict - re-run consensus with new claimant
                print(f"{bcolors.RED_FAIL}[{self.name}](task_conflict): Conflict detected, claimant: {robot_id}{bcolors.ENDC}")
                self.consensus(robot_id)
                return "reassign"
            elif title == "[consensus_ack]":
                # Another robot acknowledged consensus
                print(f"{bcolors.GREEN_OK}[{self.name}](consensus_ack): {robot_id} acknowledged taskmaster: {content}{bcolors.ENDC}")
                if not self.consensus_reached:
                    self.task_master = content
                    self.consensus_reached = True
                return None  # Don't change state, just acknowledgment
            elif title == "[path_following]":
                paths = ast.literal_eval(content)
                if self.name in paths.keys():
                    self.path = paths.get(self.name, "")
                    return "path_following"
                else:
                    self.mode = 2
                    return "idle"
            elif title == "[task_successful]":
                if self.task_master == self.name:
                    return "path_finding"
                else:
                    return "idle"
            else:
                if self.verbose:
                    print(f"[{self.name}](unknown_message): {title}")
            
        return None 
    
    def broadcast_message(self, title: str, content):
        # Send the message
        message = json.dumps([title, self.name, self.message_id, content])
        if self.verbose:
            print(f"[{self.robot.getName()}](broadcast_message): {message}")
        self.emitter.send(message)
        self.message_id += 1

    def send_position(self, robot_position):
        # Broadcast the message
        self.broadcast_message("[probe]", (robot_position["x"], robot_position["y"], robot_position["theta"]))
        # Reset the timer
        self.time_tracker = 0

    def print_received_message(self, msg):
        print(f"[{self.name}](helper): {msg}")

    def consensus(self, new_claimant: str) -> str:
        """
        Handle consensus when one or more robots detect an object.
        Uses priority queue and vote counting for conflict resolution.
        
        Merged from real-world robot implementation.
        
        Args:
            new_claimant: The robot claiming to be the taskmaster
            
        Returns:
            The determined taskmaster robot name
        """
        if new_claimant == self.name:
            print(f"{bcolors.CYAN_OK}[{self.name}](consensus): Claiming taskmaster.{bcolors.ENDC}")

        self.taskmaster_claims.append(new_claimant)

        # Get unique claims
        unique_claims = set(self.taskmaster_claims)
        print(f"{bcolors.YELLOW_WARNING}[{self.name}](consensus): Current claims: {self.taskmaster_claims}, Unique: {unique_claims}{bcolors.ENDC}")

        if len(unique_claims) == 1:
            # Everyone agrees on the same taskmaster
            self.task_master = list(unique_claims)[0]
        else:
            # Multiple claimants - check who is highest in priority queue
            for robot in self.priority_queue:
                if robot in unique_claims:
                    self.task_master = robot
                    break
            
            # Count votes if still mismatched (fallback mechanism)
            vote_count = defaultdict(int)
            for claim in self.taskmaster_claims:
                vote_count[claim] += 1

            # Find robot with most votes
            most_voted = max(vote_count, key=vote_count.get)
            if vote_count[most_voted] > len(self.taskmaster_claims) // 2:
                self.task_master = most_voted

        # Push taskmaster to back in the priority queue (round-robin for next time)
        if self.task_master in self.priority_queue:
            self.priority_queue.remove(self.task_master)
            self.priority_queue.append(self.task_master)

        self.consensus_reached = True
        print(f"{bcolors.GREEN_OK}[{self.name}](consensus): Consensus reached: {self.task_master} is the taskmaster.{bcolors.ENDC}")
        
        # Broadcast acknowledgment
        self.broadcast_message("[consensus_ack]", self.task_master)
        
        # Clear claims after consensus
        self.taskmaster_claims.clear()
        
        return self.task_master

    def object_detected(self, object_coords: dict = None):
        """
        Utility function for robot to call when object is detected.
        This initiates the consensus process and broadcasts to other robots.
        
        Merged from real-world robot implementation.
        
        Args:
            object_coords: Dictionary with 'x', 'y', 'theta' of detected object
        """
        if object_coords:
            self.object_coordinates = object_coords
            
        print(f"{bcolors.BLUE_OK}[{self.name}](object_detected): Object detected at {self.object_coordinates}, initiating consensus...{bcolors.ENDC}")
        
        # First, claim taskmaster for self
        self.consensus(self.name)
        
        # Broadcast to other robots that object was detected
        self.broadcast_message("[object_detected]", self.object_coordinates)
        
        return self.task_master

    def is_taskmaster(self) -> bool:
        """
        Check if this robot is the current taskmaster.
        """
        return self.consensus_reached and self.task_master == self.name
    
    def reset_consensus(self):
        """
        Reset consensus state for a new detection cycle.
        """
        self.taskmaster_claims.clear()
        self.consensus_reached = False
        self.task_master = ""
        