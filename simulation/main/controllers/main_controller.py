from controller import Robot, Emitter, Receiver, GPS
import json
import numpy as np
import math

# Declare constants
# Communication
GPS_DEVICE_NAME = "gps"
EMITTER_DEVICE_NAME = "emitter"
RECEIVER_DEVICE_NAME = "receiver"
MESSAGE_INTERVAL = 2000 # ms
PRIORITY_LIST = ["TurtleBot1", "TurtleBot2", "TurtleBot3", "TurtleBot4", "TurtleBot5"]

# SLAM
TIME_STEP = 64
MAX_SPEED = 6.28
MAP_WIDTH = 250
MAP_HEIGHT = 220
RESOLUTION = 0.1  # 10 cm per grid cell

class SwarmMember:
    def __init__(self, robot, mode=0):
        # Instantiate the robot
        self.robot = robot

        # Retrieve robot parameters
        self.timestep = TIME_STEP
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

        # SLAM
        self.left_motor = self.robot.getDevice("left_wheel_motor")
        self.right_motor = self.robot.getDevice("right_wheel_motor")

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_encoder = self.robot.getDevice("left_wheel_sensor")
        self.right_encoder = self.robot.getDevice("right_wheel_sensor")
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)

        self.lidar = self.robot.getDevice("lidar_sensor")
        self.lidar.enable(TIME_STEP)

        self.map = np.zeros((MAP_HEIGHT, MAP_WIDTH))

        self.robot_x, self.robot_y, self.robot_theta = 0.0, 0.0, 0.0  # Initial position


    def run(self):
        # Main flow

        # Initialize previous encoder values
        prev_left_encoder = self.left_encoder.getValue()
        prev_right_encoder = self.right_encoder.getValue()

        # Wait until valid encoder values are available
        while math.isnan(prev_left_encoder) or math.isnan(prev_right_encoder):
            print("Waiting for valid encoder values...")
            self.robot.step(TIME_STEP)  # Step the simulation until we get valid readings
            prev_left_encoder = self.left_encoder.getValue()
            prev_right_encoder = self.right_encoder.getValue()

        print(f"Valid Initial Left Encoder: {prev_left_encoder}, Valid Initial Right Encoder: {prev_right_encoder}")

        while self.robot.step(self.timestep) != -1:
            self.time_tracker += self.timestep

            # Perform SLAM first
            left_pos = self.left_encoder.getValue()
            right_pos = self.right_encoder.getValue()

            self.robot_x, self.robot_y, self.robot_theta, prev_left_encoder, prev_right_encoder = self.update_odometry(
                left_pos, right_pos, prev_left_encoder, prev_right_encoder, self.robot_x, self.robot_y, self.robot_theta
            )
            
            if self.time_tracker % 100 == 0 and self.time_tracker > 0:
                print(f"{self.name} X position: {self.robot_x}")

            lidar_data = self.lidar.getRangeImage()

            self.update_map(lidar_data, self.robot_x, self.robot_y, self.robot_theta)

            self.left_motor.setVelocity(0.5 * MAX_SPEED)
            self.right_motor.setVelocity(0.5 * MAX_SPEED)

            # Check communication
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
                # checking consensus
                if self.check_colliding_master():
                    print("Task master conflict found, appointing new task master..")
                    self.broadcast_message("[TaskConflict]", self.priority_list)
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

            else:
                print("I'm fucked")

            if self.time_tracker % 100 == 0 and self.time_tracker > 0:
                print(f"{self.name} printing map")
                self.print_map()
                # print("LIDAR Values:", lidar_data)  

    def check_detection(self):
        if len(self.object_coordinates) != 0:
            # Object detected
            self.mode = 1
            # self.task_master = self.name
            self.broadcast_message("[Task]", self.object_coordinates)
            
    def check_colliding_master(self):
        return self.name != self.task_master
            
    def broadcast_message(self, title: str, content):
        # Send the message
        message = json.dumps([title, self.name, content])
        self.emitter.send(message)
    
    def send_position(self):
        # Get current position
        current_x, current_y, current_z = self.gps.getValues()
        # Broadcast the message
        self.broadcast_message("[Probe]", (current_x, current_y, current_z))
        # Reset the timer
        self.time_tracker = 0

    def listen_message(self):
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
    
    # SLAM functions
    def update_odometry(self, left_encoder_value, right_encoder_value, prev_left_encoder, prev_right_encoder, prev_x, prev_y, prev_theta):
        wheel_radius = 0.033  
        wheel_base = 0.160  

        delta_left = (left_encoder_value - prev_left_encoder) * wheel_radius
        delta_right = (right_encoder_value - prev_right_encoder) * wheel_radius

        delta_center = (delta_left + delta_right) / 2
        delta_theta = (delta_right - delta_left) / wheel_base

        new_x = prev_x + delta_center * math.cos(prev_theta)
        new_y = prev_y + delta_center * math.sin(prev_theta)
        new_theta = prev_theta + delta_theta

        return new_x, new_y, new_theta, left_encoder_value, right_encoder_value

    def update_map(self, lidar_data, robot_x, robot_y, robot_theta):
        map_center_x = MAP_WIDTH // 2
        map_center_y = MAP_HEIGHT // 2

        for i, distance in enumerate(lidar_data):
            if distance == float('inf') or distance < 0.0 or distance > 10.0:
                continue  

            angle = robot_theta + i * (2 * math.pi / len(lidar_data))

            obstacle_x = robot_x + distance * math.cos(angle)
            obstacle_y = robot_y + distance * math.sin(angle)

            grid_x = int(obstacle_x / RESOLUTION) + map_center_x
            grid_y = int(obstacle_y / RESOLUTION) + map_center_y

            if self.time_tracker % 100 == 0 and self.time_tracker > 0:
                print(f"Obstacle Global Position: ({obstacle_x}, {obstacle_y}) -> Grid ({grid_x}, {grid_y})")

            if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                self.map[grid_x][grid_y] = 1  # Mark cell as occupied
                if self.time_tracker % 100 == 0 and self.time_tracker > 0:
                    print(f"Obstacle marked at ({grid_x}, {grid_y})")  
            else:
                if self.time_tracker % 100 == 0 and self.time_tracker > 0:
                    print(f"Obstacle out of bounds: ({grid_x}, {grid_y})")  

    def print_map(self):
        print("SLAM Map:")
        for i in range(MAP_HEIGHT):  # Loop over rows (Y-axis)
            for j in range(MAP_WIDTH):  # Loop over columns (X-axis)
                if self.map[i][j] == 1:
                    print("#", end="")  # Obstacle
                else:
                    print(".", end="")  
            print()

if __name__ == "__main__":
    robot = Robot()
    swarm_member = SwarmMember(robot)
    swarm_member.run()
