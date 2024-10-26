# my_controller controller.

# Import necessary classes
from controller import Robot
import numpy as np
import math
from utils import Communicator

TIME_STEP = 64
MAX_SPEED = 6.28
MAP_WIDTH = 250
MAP_HEIGHT = 220
RESOLUTION = 0.1  # 10 cm per grid cell
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160


class Localisation:
    def __init__(self, robot):
        self.robot = robot
        self.left_motor = self.robot.getDevice("left_wheel_motor")
        self.right_motor = self.robot.getDevice("right_wheel_motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.robot_position = {"x": 0.0, "y": 0.0, "theta": 0.0}

        # Encoder
        self.left_encoder = self.robot.getDevice("left_wheel_sensor")
        self.right_encoder = self.robot.getDevice("right_wheel_sensor")
        self.left_encoder.enable(TIME_STEP)
        self.right_encoder.enable(TIME_STEP)
        self.prev_left_encoder = self.left_encoder.getValue()
        self.prev_right_encoder = self.right_encoder.getValue()

        # Lidar
        self.lidar = self.robot.getDevice("lidar_sensor")
        self.lidar.enable(TIME_STEP)
        self.map = np.zeros((MAP_HEIGHT, MAP_WIDTH))

    def check_encoder_not_null(self):
        # Wait until valid encoder values are available
        while math.isnan(self.prev_left_encoder) or math.isnan(self.prev_right_encoder):
            print("Waiting for valid encoder values...")
            self.robot.step(TIME_STEP)  # Step the simulation until we get valid readings
            self.prev_left_encoder = self.left_encoder.getValue()
            self.prev_right_encoder = self.right_encoder.getValue()

        print(
            f"Valid Initial Left Encoder: {self.prev_left_encoder}, Valid Initial Right Encoder: {self.prev_right_encoder}"
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

