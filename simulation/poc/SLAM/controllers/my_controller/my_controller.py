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

robot = Robot()

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_encoder = robot.getDevice("left_wheel_sensor")
right_encoder = robot.getDevice("right_wheel_sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

lidar = robot.getDevice("lidar_sensor")
lidar.enable(TIME_STEP)

map = np.zeros((MAP_HEIGHT, MAP_WIDTH))

c = Communicator(robot)

robot_position = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
}


# SLAM functions
def update_odometry(left_encoder_value, right_encoder_value, prev_left_encoder, prev_right_encoder, prev_x, prev_y, prev_theta):
    wheel_radius = 0.033  
    wheel_base = 0.160  

    delta_left = (left_encoder_value - prev_left_encoder) * wheel_radius
    delta_right = (right_encoder_value - prev_right_encoder) * wheel_radius

    #print(f"Delta Left: {delta_left}, Delta Right: {delta_right}")

    delta_center = (delta_left + delta_right) / 2
    delta_theta = (delta_right - delta_left) / wheel_base

    #print(f"Delta Center: {delta_center}, Delta Theta: {delta_theta}")

    new_x = prev_x + delta_center * math.cos(prev_theta)
    new_y = prev_y + delta_center * math.sin(prev_theta)
    new_theta = prev_theta + delta_theta

    #print(f"New X: {new_x}, New Y: {new_y}, New Theta: {new_theta}")

    return new_x, new_y, new_theta, left_encoder_value, right_encoder_value

def update_map(lidar_data):
    #print(f"LIDAR Data: {lidar_data}")

    map_center_x = MAP_WIDTH // 2
    map_center_y = MAP_HEIGHT // 2

    for i, distance in enumerate(lidar_data):
        if distance == float('inf') or distance < 0.0 or distance > 10.0:
            continue  

        angle = robot_position["theta"] + i * (2 * math.pi / len(lidar_data))

        obstacle_x = robot_position["x"] + distance * math.cos(angle)
        obstacle_y = robot_position["y"] + distance * math.sin(angle)

        grid_x = int(obstacle_x / RESOLUTION) + map_center_x
        grid_y = int(obstacle_y / RESOLUTION) + map_center_y

        #print(f"Obstacle Global Position: ({obstacle_x}, {obstacle_y}) -> Grid ({grid_x}, {grid_y})")

        if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
            map[grid_x][grid_y] = 1  # Mark cell as occupied
            #print(f"Obstacle marked at ({grid_x}, {grid_y})")  
        else:
            pass
            #print(f"Obstacle out of bounds: ({grid_x}, {grid_y})")  

def print_map():
    print("SLAM Map:")
    for i in range(MAP_HEIGHT):  # Loop over rows (Y-axis)
        for j in range(MAP_WIDTH):  # Loop over columns (X-axis)
            if map[i][j] == 1:
                print("#", end="")  # Obstacle
            else:
                print(".", end="")  
        print() 

# Initialize previous encoder values
prev_left_encoder = left_encoder.getValue()
prev_right_encoder = left_encoder.getValue()

# Wait until valid encoder values are available
while math.isnan(prev_left_encoder) or math.isnan(prev_right_encoder):
    print("Waiting for valid encoder values...")
    robot.step(TIME_STEP)  # Step the simulation until we get valid readings
    prev_left_encoder = left_encoder.getValue()
    prev_right_encoder = right_encoder.getValue()

print(f"Valid Initial Left Encoder: {prev_left_encoder}, Valid Initial Right Encoder: {prev_right_encoder}")

# Main loop
step_count = 0
while robot.step(TIME_STEP) != -1:
    left_pos = left_encoder.getValue()
    right_pos = right_encoder.getValue()

    #print(f"Current Left Encoder: {left_pos}, Current Right Encoder: {right_pos}")

    robot_position["x"], robot_position["y"], robot_position["theta"], prev_left_encoder, prev_right_encoder = update_odometry(
        left_pos, right_pos, prev_left_encoder, prev_right_encoder, robot_position["x"], robot_position["y"], robot_position["theta"])

    print(f"Robot X position: {robot_position['x']}")
    c.send_position(robot_position)

    lidar_data = lidar.getRangeImage()

    update_map(lidar_data)

    left_motor.setVelocity(0.5 * MAX_SPEED)
    right_motor.setVelocity(0.5 * MAX_SPEED)
    step_count += 1
    if step_count % 100 == 0:
        print(f"Printing data at step {step_count}")
        print_map()  
        print("LIDAR Values:", lidar_data)  
