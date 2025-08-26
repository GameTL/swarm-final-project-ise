# my_controller controller.

# Import necessary classes
from controller import Robot
import numpy as np
import math
from utils import Communicator
from utils import Localisation

TIME_STEP = 64
MAX_SPEED = 6.28
MAP_WIDTH = 250
MAP_HEIGHT = 220
RESOLUTION = 0.1  # 10 cm per grid cell
WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.160
SPEED_PERCENTAGE = 1.0

class SwarmMember:
    def __init__(self, mode=0):
        self.robot = Robot()
        
        self.comms = Communicator(self.robot)
        self.localisation = Localisation(self.robot)
        
        self.left_motor = self.robot.getDevice("left_wheel_motor")
        self.right_motor = self.robot.getDevice("right_wheel_motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.robot_position = self.localisation.robot_position




def main():
    member = SwarmMember()

    if member.localisation.check_encoder_not_null():
        # Main loop
        step_count = 0
        while member.robot.step(TIME_STEP) != -1:
            member.localisation.update_odometry()
            print(f"Robot X position: {member.robot_position['x']:6.3f}    Robot Y position: {member.robot_position['y']:6.3f}    Robot Theta position: {member.robot_position['theta']:6.3f}")
            member.localisation.update_map()
            member.left_motor.setVelocity(SPEED_PERCENTAGE * MAX_SPEED)
            member.right_motor.setVelocity(SPEED_PERCENTAGE * MAX_SPEED)
            step_count += 1
            if step_count % 100 == 0:  # time in webots, every 100 steps of the robot???!~
                print(f"Printing data at step {step_count}")
                member.localisation.print_map()
            if step_count > 64:
                member.right_motor.setVelocity(0.5 * MAX_SPEED)


main()
