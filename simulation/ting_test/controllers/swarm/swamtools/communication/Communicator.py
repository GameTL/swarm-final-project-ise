import json

EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot1", "TurtleBot2"]

class Communicator:
    def __init__(self, robot, mode=0):
        # setting up
        self.robot = robot

        self.timestep = 64
        self.name = self.robot.getName()
        self.mode = mode
        self.robot_entries = {}
        self.priority_list = PRIORITY_LIST
        
        self.emitter = self.robot.getDevice(EMITTER_DEVICE_NAME)
        self.receiver = self.robot.getDevice(RECEIVER_DEVICE_NAME)
        self.receiver.enable(self.timestep)

        self.message_interval = MESSAGE_INTERVAL
        self.time_tracker = 0

        self.object_coordinates = ()
        self.task_master = ""
        self.count = 0 

    def listen_message(self):
        # listening incoming message.
        entries_modified = False

        # Receive messages from other robots and print
        while self.receiver.getQueueLength() > 0:
            received_message = self.receiver.getString()
            title, robot_id, content = json.loads(received_message)
            
            # Check for probing message
            if title == "[Probe]":
                self.robot_entries[robot_id] = content
                entries_modified = True
            elif title == "[Task]":
                self.task_master = robot_id
                self.object_coordinates = content
            elif title == "[TaskConflict]":
                self.priority_list = content
                self.task_master = self.priority_list[0]
            elif title == "[TaskSucessful]":
                self.mode = 2
            
            self.receiver.nextPacket()

        # True if robot entries are modified
        return entries_modified
    
    def broadcast_message(self, title: str, content):
        # Send the message
        message = json.dumps([title, self.name, content])
        self.emitter.send(message)

    def send_position(self, robot_position):
        # Broadcast the message
        self.broadcast_message("[Probe]", (robot_position["x"], robot_position["y"], robot_position["theta"]))
        # Reset the timer
        self.time_tracker = 0