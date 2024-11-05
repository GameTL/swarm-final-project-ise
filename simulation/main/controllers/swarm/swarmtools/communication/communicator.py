from controller import Robot, Camera, Motor, Display, Supervisor
import json
from rich.pretty import pprint
import time
import ast

EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot1", "TurtleBot2"]

class Communicator:
    def __init__(self, robot: Robot, mode=0, verbose=True):
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

        self.object_coordinates = {}
        self.task_master = ""
        self.path = None
        self.message_id = 0

    def listen_to_message(self) -> None | str:
        """ 
        listen for ['[probe]', '[object_detected]', '[task]', '[task_conflict]', '[task_successful]']
        """
        # Receive messages from other robots and print
        return_status = None
        while self.receiver.getQueueLength() > 0:  
            time.sleep(0.001)
            received_message = self.receiver.getString()
            if self.verbose: self.print_received_message(received_message)
            title, robot_id, message_id, content = json.loads(received_message)
            
            # print(f"{self.robot.getName()} received a message from {robot_id}; message ID: {message_id}")
            # Check for probing message
            if title == "[path_receiving]":
                return_status = "path_receiving"
            elif title == "[probe]":
                self.robot_entries[robot_id] = content
            elif title == "[object_detected]":
                retreturn_status = "idle" 
            elif title == "[task]":
                self.task_master = robot_id
                self.object_coordinates = content
                print(f"[task]({self.robot.getName()}) Object Detected from: {robot_id}@{content}; checking conflict...")
                return_status = "task"
            elif title == "[task_conflict]":
                self.priority_list = content
                self.task_master = self.priority_list[0]
            elif title == "[path_following]":
                paths = ast.literal_eval(content)
                if self.name in paths.keys():
                    self.path = paths.get(self.name, "")
                    return_status = "path_following"
                else:
                    self.mode = 2
                    return_status = "idle"
            elif title == "[task_successful]":
                return_status = "path_finding"
            else:
                print("x")
            
            self.receiver.nextPacket()
        return return_status 
    
    def broadcast_message(self, title: str, content):
        # Send the message
        message = json.dumps([title, self.name, self.message_id, content])
        if self.verbose:
            print(f"[broadcast_message]({self.robot.getName()}) {message}")
        self.emitter.send(message)
        self.message_id += 1

    def send_position(self, robot_position):
        # Broadcast the message
        self.broadcast_message("[probe]", (robot_position["x"], robot_position["y"], robot_position["theta"]))
        # Reset the timer
        self.time_tracker = 0

    def print_received_message(self, msg):
        print(f"[helper]({self.name}) {msg}")
        