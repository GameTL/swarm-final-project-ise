# my_controller controller.

# Import necessary classes
from controller import Robot
import numpy as np
import math
import asyncio
import time
from rich.pretty import pprint
MAX_SPEED = 6.28
MAP_WIDTH = 250
MAP_HEIGHT = 220
RESOLUTION = 0.1  # 10 cm per grid cell
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
ENCODER_RESOLUTION = 2048  # Example value; replace with your actual encoder resolution

GPS_DEVICE_NAME = "gps"
class Localisation:
    def __init__(self, robot):
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())  # Ensure timestep is an integer

        # Initialize robot position using float64 for higher precision
        self.robot_position = {
            "x": np.float64(0.0),
            "y": np.float64(0.0),
            "theta": np.float64(0.0)
        }

        # Encoder
        self.left_encoder = self.robot.getDevice("left_wheel_sensor")
        self.right_encoder = self.robot.getDevice("right_wheel_sensor")
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        
        # Store previous encoder values as float64
        self.prev_left_encoder = np.float64(self.left_encoder.getValue())
        self.prev_right_encoder = np.float64(self.right_encoder.getValue())
        
        # Lidar
        self.lidar = self.robot.getDevice("lidar_sensor")
        self.lidar.enable(self.timestep)
        self.map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.float64)
        
        self.gps = robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)


    def check_encoder_not_null(self):
        # Wait until valid encoder values are available
        print(f"[localisation]({self.robot.getName()}) Waiting for encoder != nan")
        while math.isnan(self.prev_left_encoder) or math.isnan(self.prev_right_encoder):
            self.robot.step(self.timestep)  # Step the simulation until we get valid readings
            self.prev_left_encoder = self.left_encoder.getValue()
            self.prev_right_encoder = self.right_encoder.getValue()

        print(
            f"[localisation]({self.robot.getName()}) Valid Initial Left Encoder: {self.prev_left_encoder}, Valid Initial Right Encoder: {self.prev_right_encoder}"
        )
        return True
    
    # SLAM functions
    # l & r encoder value, using contsants
    # output new x,y, here are sensor values give me location
    def update_odometry(self):
        temp_left = self.left_encoder.getValue()
        temp_right = self.right_encoder.getValue()
        delta_left = (temp_left - self.prev_left_encoder) * WHEEL_RADIUS
        delta_right = (temp_right - self.prev_right_encoder) * WHEEL_RADIUS
        self.prev_left_encoder = temp_left
        self.prev_right_encoder = temp_right

        # print(f"Delta Left: {delta_left}, Delta Right: {delta_right}")

        delta_center = (delta_left + delta_right) / 2
        delta_theta = (delta_right - delta_left) / WHEEL_BASE

        # print(f"Delta Center: {delta_center}, Delta Theta: {delta_theta}")

        self.robot_position["x"] = self.robot_position["x"] + delta_center * math.cos(
            self.robot_position["theta"]
        )
        self.robot_position["y"] = self.robot_position["y"] + delta_center * math.sin(
            self.robot_position["theta"]
        )
        self.robot_position["theta"] = self.robot_position["theta"] + delta_theta
    
    def update_odometry_o1(self):
        # Get current encoder values as float64
        temp_left = np.float64(self.left_encoder.getValue())
        temp_right = np.float64(self.right_encoder.getValue())
        
        # Calculate change in encoder values (radians)
        delta_left = temp_left - self.prev_left_encoder
        delta_right = temp_right - self.prev_right_encoder
        
        # Update previous encoder values
        self.prev_left_encoder = temp_left
        self.prev_right_encoder = temp_right
        
        # Compute delta_center and delta_theta using float64
        delta_center = (delta_left + delta_right) / 2.0 * WHEEL_RADIUS
        delta_theta = (delta_right - delta_left) * WHEEL_RADIUS / WHEEL_BASE
        
        # Update orientation first
        self.robot_position["theta"] += delta_theta
        
        # Normalize theta to be within [-pi, pi]
        self.robot_position["theta"] = (self.robot_position["theta"] + np.pi) % (2 * np.pi) - np.pi
        
        # Calculate average orientation
        avg_theta = self.robot_position["theta"] - delta_theta / 2.0
        
        # Update position using the average orientation
        self.robot_position["x"] += delta_center * np.cos(avg_theta)
        self.robot_position["y"] += delta_center * np.sin(avg_theta)
    
    def update_odometry_service(self):
        # when encoder is live then trigger the set
        self.robot_position["x"], self.robot_position["y"], current_z = (
            self.gps.getValues()
        )  # init the coords even when using wheel odom
        print(f"[localisaton]({self.robot.getName()}) INIT WITH GPS AT: Robot X position: {self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")
        pprint(self.robot_position)
        while True:
            self.update_odometry_o1()
        
    


    # Calulate the Lidar infomation
    def update_map(self):
        map_center_x = MAP_WIDTH // 2
        map_center_y = MAP_HEIGHT // 2

        for i, distance in enumerate(self.lidar.getRangeImage()):
            if distance == float("inf") or distance < 0.0 or distance > 10.0:
                continue

            angle = self.robot_position["theta"] + i * (2 * math.pi / len(self.lidar.getRangeImage()))

            obstacle_x = self.robot_position["x"] + distance * math.cos(angle)
            obstacle_y = self.robot_position["y"] + distance * math.sin(angle)

            grid_x = int(obstacle_x / RESOLUTION) + map_center_x
            grid_y = int(obstacle_y / RESOLUTION) + map_center_y

            # print(f"Obstacle Global Position: ({obstacle_x}, {obstacle_y}) -> Grid ({grid_x}, {grid_y})")

            if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                # ? not global?????
                self.map[grid_x][grid_y] = 1  # Mark cell as occupied
                # print(f"Obstacle marked at ({grid_x}, {grid_y})")
            else:
                pass
                # print(f"Obstacle out of bounds: ({grid_x}, {grid_y})")
                
                

    def print_map(self):
        print("SLAM Map:")
        for i in range(MAP_HEIGHT):  # Loop over rows (Y-axis)
            for j in range(MAP_WIDTH):  # Loop over columns (X-axis)
                if self.map[i][j] == 1:
                    print("#", end="")  # Obstacle
                else:
                    print(".", end="")
            print()

