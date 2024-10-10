from controller import Robot, Emitter, Receiver, GPS
import json

# Declare constants
GPS_DEVICE_NAME = "gps"
EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot1", "TurtleBot2", "TurtleBot3", "TurtleBot4", "TurtleBot5"]


class SwarmMember:
    def __init__(self, mode=0):
        # Instantiate the robot
        self.robot = Robot()

        # Retrieve robot parameters
        self.timestep = 64
        self.name = self.robot.getName()
        self.mode = mode
        self.robot_entries = {}
        self.priority_list = PRIORITY_LIST

        # Enable sensory devices
        self.gps = self.robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)
        
        self.emitter = self.robot.getDevice(EMITTER_DEVICE_NAME)
        self.receiver = self.robot.getDevice(RECEIVER_DEVICE_NAME)
        self.receiver.enable(self.timestep)

        # Define an interval for sending messages
        self.message_interval = MESSAGE_INTERVAL
        self.time_tracker = 0

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.count = 0 #! For testing

    def run(self):
        # Main flow
        while self.robot.step(self.timestep) != -1:
            self.time_tracker += self.timestep

            if self.mode == 0:
                # Probing mode
                if self.time_tracker >= self.message_interval:
                    self.send_position()

                    #! For testing - Assume detection
                    if self.count >= 3 and (self.name == "TurtleBot2" or self.name == "TurtleBot3"):
                        self.object_coordinates = (3.0, 4.0, 5.0)
                        self.count = 0
                    self.count += 1

                self.check_detection()
                
                # Check robot entries
                entries_modified = self.listen_message()
                if entries_modified:
                    print(self.name)
                    print(self.robot_entries)
                 
            elif self.mode == 1:
                # checking concensus
                if self.check_colliding_master():
                    print("Task master conflict found, appointing new task master..")
                    self.broadcast_message("[TaskConflict]", json.dumps(self.priority_list))
                    self.task_master = self.priority_list[0]
                self.broadcast_message("[TaskSucessful]", "")
                self.mode = 2

            elif self.mode == 2:
                # Consensus confirm
                if self.time_tracker >= self.message_interval:
                    print(f"{self.name} consensus found")
                    print(f"Taskmaster: {self.task_master}")
                    print(f"Coordinates: {self.object_coordinates}")
                    self.time_tracker = 0

    def check_detection(self):
        if len(self.object_coordinates) != 0:
            # Object detected
            self.mode = 1
            # self.task_master = self.name
            self.broadcast_message("[Task]", f"{self.object_coordinates}")
            
    def check_colliding_master(self):
        return self.name != self.task_master
            
    def broadcast_message(self, title: str, content: str):
        # Send the message
        message = f"{title};{self.name};{content}"
        self.emitter.send(message.encode("utf-8"))
    
    def send_position(self):
        # Get current position
        current_x, current_y, current_z = self.gps.getValues()
        # Broadcast the message
        self.broadcast_message("[Probe]", f"[{current_x}, {current_y}, {current_z}]")
        # Reset the timer
        self.time_tracker = 0

    def listen_message(self):
        entries_modified = False

        # Receive messages from other robots and print
        while self.receiver.getQueueLength() > 0:
            received_message = self.receiver.getString()
            message_components = received_message.split(";")
            
            # Check for probing message
            if message_components[0] == "[Probe]":
                self.robot_entries[message_components[1]] = message_components[2]
                entries_modified = True
            elif message_components[0] == "[Task]":
                self.task_master = message_components[1]
                self.object_coordinates = tuple([float(elem.replace("(", "").replace(")", "").replace(" ", "")) for elem in message_components[2].split(",")])
            elif message_components[0] == "[TaskConflict]":
                self.priority_list = json.loads(message_components[2])
                self.task_master = self.priority_list[0]
            elif message_components[0] == "[TaskSucessful]":
                self.mode = 2
            
            self.receiver.nextPacket()

        # True if robot entries are modified
        return entries_modified

if __name__ == "__main__":
    robot = SwarmMember()
    robot.run()
