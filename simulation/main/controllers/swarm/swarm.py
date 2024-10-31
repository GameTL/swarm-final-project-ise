import cv2
import ast
import json
import asyncio
import numpy as np
from rich.pretty import pprint
from controller import Robot, Camera, Motor, Display, Supervisor
from swarmtools import FormationMaster
from swarmtools import ObjectDetector
from swarmtools import Communicator
from swarmtools import Localisation
from swarmtools import Driver


# GPS = True
GPS = False

PRIORITY_LIST = ["robot1", "robot2", "robot3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}


class SwarmMember:
    def __init__(self, mode=0, verbose=False):
        # Instantiate the robot & big objects

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.verbose = verbose
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
        self.name = self.robot.getName()
        self.mode = mode
        self.priority_queue = PRIORITY_LIST
        self.communicator.robot_entries[self.name] = (
            self.robot_position["x"],
            self.robot_position["y"],
            self.robot_position["theta"],
        )

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.path = None
        self.status_prev = None
        self.status = None

        # testing for sim
        self.driver = Driver(
            robot=self.robot, robot_position=self.robot_position, localisation=self.localisation
        )
    def print_position(self):
        print(f"[helper]({self.robot.getName()}) Robot X position: {self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")
        
    def random_movement_find(self):
        while self.robot.step(self.timestep) != -1:
            
            # Check for incoming messages
            status = self.communicator.listen_to_message()
            if status != None:
                self.status = status
            if self.status_prev != self.status or self.verbose:
                print(f"[{self.status}]({self.robot.getName()})")

            # print(self.robot.getName(),f'{status=}')
            if self.status == "idle":
                self.driver.stop()
                break
            elif self.status == "path_finding":
                # Used only by the TaskMaster
                if self.detected_flag:
                    print(f"[path_finding]({self.robot.getName()}) calculating...")
                    paths_json = self.formation_object()

                    self.communicator.broadcast_message("[path_following]", paths_json)

                if self.path == None:
                    self.path = self.communicator.path
                if self.verbose:
                    print(f"[{self.status}]{self.name}: {self.path}")  # big print
                
                paths = ast.literal_eval(paths_json)
                if self.name in paths.keys():
                    self.path = paths.get(self.name, "")
                self.communicator.path = paths # for themselves just in case
                # self.status = "path_following" #TODO the taskmaster is fucked
                self.status = "idle"
            #
            elif self.status == "path_following":
                self.driver.stop()
                self.status = "idle"
                # path = self.communicator.path
                # if path != "":
                #     self.driver.simple_follow_path(path)
                #     self.driver.stop()
                #     # pprint(path)
                #     print(
                #         f"[path_following]({self.robot.getName()}) Making my way downtown, walking fast"
                #     )
                # else:
                #     print(f"[path_following]({self.robot.getName()}) FUCK")

                break
            elif self.object_detector.detect() and not self.detected_flag:
                # * As a member that found the object becomes the master.
                print(
                    f"[object_detected]({self.robot.getName()}) found cylinder @ {cylinder_position}"
                )
                self.detected_flag = True  # detect once and top
                self.status = "path_finding"  #
                self.driver.stop()
            elif self.status == "task":
                self.driver.stop()
                break
            else:
                self.communicator.send_position(
                    robot_position={
                        "x": self.robot_position["x"],
                        "y": self.robot_position["y"],
                        "theta": self.robot_position["theta"],
                    }
                )

            # self.driver.move_forward()
            self.driver.move_along_polynomial()
            self.print_position() # for Debu
            if GPS:
                self.robot_position["x"], self.robot_position["y"], current_z = (
                    self.driver.gps.getValues()
                )
            else:
                # self.localisation.update_odometry()
                self.localisation.update_odometry_o1()
            # self.leftMotor.setVelocity(MAX_SPEED * 0.5)
            # self.rightMotor.setVelocity(MAX_SPEED)

            self.communicator.robot_entries[self.name] = (
                self.robot_position["x"],
                self.robot_position["y"],
                self.robot_position["theta"],
            )

    def formation_object(self):
        if self.detected_flag:
            coords = self.communicator.robot_entries.copy()
            for robot_name in coords:
                coords[robot_name] = list(
                    map(lambda x: round(x, 1), coords[robot_name])
                )
            # print(f"[path_finding]({self.name}): current coords={coords}")

            self.formationer = FormationMaster(
                robot=self.robot,
                current_coords=coords,
                object_coords=(cylinder_position["x"], cylinder_position["y"]),
                verbose=False,
            )
            self.formationer.calculate_target_coords()
            self.formationer.plan_paths()

            paths = json.dumps(self.formationer.paths)
            self.path = json.loads(paths)[self.name]

            return paths
        else:
            print(
                f"[path_finding]({self.robot.getName()}) LISTENING for {cylinder_position}"
            )
            return None


def main():
    import threading
    # task1 = asyncio.create_task(listening())
    # while 1:
    member = SwarmMember()
    localisation_service = threading.Thread(target=member.localisation.update_odometry_service)
    if member.localisation.check_encoder_not_null():
        localisation_service.start()
        
        # if 1:
        # Create tasks for the asynchronous SLAM functions
        # odometry_task = asyncio.create_task(member.update_odometry())  #$
        # map_task = asyncio.create_task(member.update_map())  #$

        member.random_movement_find()

        # await asyncio.gather(odometry_task, map_task)  #$


main()