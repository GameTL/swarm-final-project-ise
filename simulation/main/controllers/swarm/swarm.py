import cv2
import json
import asyncio
import numpy as np
from controller import Robot, Camera, Motor, Display, Supervisor
from swarmtools.navigation.follower import RandomPolynomialFollower
from swarmtools.navigation.formation import FormationMaster
from swarmtools import ObjectDetector
from swarmtools import Communicator
from swarmtools import Localisation


# Declare constants
MAX_SPEED = 2
GPS_DEVICE_NAME = "gps"
# GPS = True
GPS = False
PRIORITY_LIST = ["robot1", "robot2", "robot3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}


class SwarmMember:
    def __init__(self, mode=0):
        # Instantiate the robot & big objects

        self.robot = Robot()
        self.object_detector = ObjectDetector(self.robot)
        self.communicator = Communicator(self.robot)
        
        if GPS:
            self.robot_position = {"x": 0.0, "y": 0.0, "theta": 0.0}
        else:
            self.localisation = Localisation(self.robot)
            self.robot_position = self.localisation.robot_position

        # Computer vision
        self.detected_flag = False

        # Retrieve robot parameters
        self.timestep = int(self.robot.getBasicTimeStep())
        self.name = self.robot.getName()
        self.mode = mode
        self.priority_queue = PRIORITY_LIST
        self.communicator.robot_entries[self.name] = (self.robot_position["x"], self.robot_position["y"], self.robot_position["theta"])

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.path = None
        self.status = None
        
        # testing for sim 
        self.follower = RandomPolynomialFollower(self,degree=3, timestep=self.timestep)
        
        # Init motors
        self.leftMotor = self.robot.getDevice("left_wheel_motor")
        self.rightMotor = self.robot.getDevice("right_wheel_motor")
        self.leftMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        # Enable sensory devices
        self.gps = self.robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)

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
            # print(f"[Localisation]({self.robot.getName()}) Robot X position: {self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")
            # Check for incoming messages
            status = self.communicator.listen_to_message()
            if status != None:
                print(status)
                self.status = status
            print(f"{self.name=} {self.status=}")

            # print(self.robot.getName(),f'{status=}')
            if self.status == "stop":
                self.stop()
                break
            elif self.status == "path":
                if self.detected_flag:
                    print(f"{self.name} calculating formation")
                    paths = self.formation_object()

                    self.communicator.broadcast_message("[Path]", paths)
            
                if self.path == None:
                    self.path = self.communicator.path
                
                self.status = "task"
                print(f"[Path] {self.name}: {self.path}")
            elif self.status == "task":
                self.stop()
                break
            elif self.object_detector.detect() and not self.detected_flag:
                self.communicator.broadcast_message("[Task]", cylinder_position)
                print(
                    f"[ObjectDetected]@{self.robot.getName()}:found cylinder @ {cylinder_position}"
                )
                self.detected_flag = True  # detect once and top
                self.status = "path"
                self.stop()
            else:
                self.communicator.send_position(
                robot_position={"x": self.robot_position["x"], "y": self.robot_position["y"], "theta": self.robot_position["theta"]}
            )
            
            self.follower.move_along_polynomial()
            
            if GPS:
                self.robot_position["x"], self.robot_position["y"], current_z = self.gps.getValues()
            else:
                self.localisation.update_odometry()
            # self.leftMotor.setVelocity(MAX_SPEED * 0.5)
            # self.rightMotor.setVelocity(MAX_SPEED)
            
            self.communicator.robot_entries[self.name] = (self.robot_position["x"], self.robot_position["y"], self.robot_position["theta"])

    def formation_object(self):
        if self.detected_flag:
            coords = self.communicator.robot_entries.copy()
            for robot_name in coords:
                coords[robot_name] = list(map(lambda x: round(x, 1), coords[robot_name]))
            print(f"[Formation]@{self.name}: {coords}")

            self.formationer = FormationMaster(
                current_coords=coords,
                object_coords=(cylinder_position["x"], cylinder_position["y"]),
                verbose=False
            )
            self.formationer.calculate_target_coords()
            self.formationer.plan_paths()

            paths = json.dumps(self.formationer.paths)
            self.path = json.loads(paths)[self.name]

            return paths
        else:
            print(f"[Formation]({self.robot.getName()}): LISTENING for {cylinder_position}")
            
            return None


async def main():
    # task1 = asyncio.create_task(listening())
    # while 1:
    member = SwarmMember()
    if member.localisation.check_encoder_not_null():
        # when encoder is live then trigger the set 
        member.robot_position["x"], member.robot_position["y"], current_z = member.gps.getValues() # init the coords even when using wheel odom
    # if 1:
        # Create tasks for the asynchronous SLAM functions
        # odometry_task = asyncio.create_task(member.update_odometry())  #$
        # map_task = asyncio.create_task(member.update_map())  #$
        
        member.random_movement_find()
        
        # await asyncio.gather(odometry_task, map_task)  #$

asyncio.run(main())
