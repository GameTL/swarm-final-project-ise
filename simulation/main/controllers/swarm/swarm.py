from controller import Robot, Camera, Motor, Display, Supervisor
import cv2
import numpy as np
from swarmtools.vision.object_detector import ObjectDetector
from swarmtools.communication.communicator import Communicator
from swarmtools.navigation.follower import RandomPolynomialFollower
from swarmtools.navigation.formation_dict import FormationMaster

import asyncio

# Declare constants
MAX_SPEED = 2
GPS_DEVICE_NAME = "gps"
PRIORITY_LIST = ["robot1", "robot2", "robot3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}


class SwarmMember:
    def __init__(self, mode=0):
        # Instantiate the robot & big objects

        self.robot = Robot()
        self.object_detector = ObjectDetector(self.robot)
        self.communicator = Communicator(self.robot)
        

        # while self.robot.step(self.timestep) != -1:
            

        self.position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        # Computer vision
        self.detected_flag = False
        # Retrieve robot parameters
        # self.timestep = 64
        self.timestep = int(self.robot.getBasicTimeStep())
        self.name = self.robot.getName()
        self.mode = mode
        self.priority_queue = PRIORITY_LIST

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        
        # testing for sim 
        self.follower = RandomPolynomialFollower(self,degree=3, timestep=self.timestep)
        
        # Init motors
        self.leftMotor = self.robot.getDevice("left wheel motor")
        self.rightMotor = self.robot.getDevice("right wheel motor")
        self.leftMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        # Enable sensory devices
        self.gps = self.robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)

        print(f"{self.robot.getName()}: finding yellow object")

    # motion
    def move_forward(self):
        self.leftMotor.setVelocity(MAX_SPEED)
        self.rightMotor.setVelocity(MAX_SPEED)

    def move_backward(self):
        self.leftMotor.setVelocity(-MAX_SPEED)
        self.rightMotor.setVelocity(-MAX_SPEED)

    def left_spin(self):
        self.leftMotor.setVelocity(-0.7 * MAX_SPEED)
        self.rightMotor.setVelocity(0.7 * MAX_SPEED)

    def right_spin(self):
        self.leftMotor.setVelocity(0.7 * MAX_SPEED)
        self.rightMotor.setVelocity(-0.7 * MAX_SPEED)

    def stop(self):
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

    def random_movement_find(self):
        while self.robot.step(self.timestep) != -1:
            self.position["x"], self.position["y"], current_z = self.gps.getValues()
            status = (
                self.communicator.listen_to_message()
            )  # check for incoming messages
            # print(self.robot.getName(),f'{status=}')
            if status == "stop":
                self.stop()
                break
            elif status == "task":
                print(self.position["x"], self.position["y"])
                self.stop()
                break
            elif self.object_detector.detect() and not self.detected_flag:
                self.communicator.broadcast_message("[Task]", cylinder_position)
                print(
                    f"[ObjectDetected]@{self.robot.getName()}:found cylinder @ {cylinder_position}"
                )
                self.detected_flag = True  # detect once and top
                self.stop()
                break
            else:
                self.communicator.send_position(
                robot_position={"x": self.position["x"], "y": self.position["y"], "theta": 0.0}
            )
            self.follower.move_along_polynomial()
            # self.leftMotor.setVelocity(MAX_SPEED * 0.5)
            # self.rightMotor.setVelocity(MAX_SPEED)
    def formation_object(self):
        if self.detected_flag:
            print(
                        f"[Formation]@{self.robot.getName()}: Path planning for {cylinder_position}"
                    )
            # if self.communicator.task_master == self.robot.getName(): # if the task master
            # print(f'this is the shit I have to deal with {self.communicator.robot_entries=}')
            coords = self.communicator.robot_entries.copy()
            coords[self.robot.getName()] = (self.position["x"],self.position["y"])
            self.formationer = FormationMaster(
                current_coords= coords,
                object_coords=(cylinder_position["x"],cylinder_position["y"]),
                verbose=True
            )
            self.formationer.calculate_target_coords()
            # self.formationer.plan_paths()
            # else:
            #     self.stop()
            #      # program stops here
            
            
        


async def main():
    # task1 = asyncio.create_task(listening())
    # while 1:
    member = SwarmMember()
    member.random_movement_find()
    member.formation_object()


asyncio.run(main())
