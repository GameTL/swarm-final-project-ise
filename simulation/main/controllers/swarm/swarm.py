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
from swarmtools import Driver


# GPS = True
GPS = False
PRIORITY_LIST = ["TurtleBot3Burger_1", "TurtleBot3Burger_2", "TurtleBot3Burger_3"]

cylinder_position = {"x": 0.75, "y": -0.25, "theta": 0.0}


class SwarmMember:
    def __init__(self, mode=0, verbose=False):
        # Instantiate the robot & big objects

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.verbose = verbose
        self.object_detector = ObjectDetector(self.robot)
        self.communicator = Communicator(self.robot)
        self.driver = Driver(self.robot)
        self.tick = 0

        # Computer vision
        self.detected_flag = False

        # Retrieve robot parameters
        self.name = self.robot.getName()
        self.mode = mode
        self.priority_queue = PRIORITY_LIST
        self.communicator.robot_entries[self.name] = (
            self.driver.robot_position["x"],
            self.driver.robot_position["y"],
            self.driver.robot_position["theta"],
        )

        # Detection parameters
        self.object_coordinates = ()
        self.task_master = ""
        self.path = None
        self.status_prev = None
        self.status = None
        self.reassign_flag = False


    def print_position(self):
        print(f"[helper]({self.robot.getName()}) Robot X position: {self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")

    def path_finding(self):
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
        self.communicator.path = self.path # Sync with communicator

    def random_movement_find(self):
        print(f"{self.priority_queue} from {self.name}")
        while self.robot.step(self.timestep) != -1:
            # Check for incoming messages
            status = self.communicator.listen_to_message()
            if status != None:
                self.status = status
            if self.status_prev != self.status or self.verbose:
                print(f"[{self.status}]({self.robot.getName()}) CHANGED")

            if self.object_detector.detect() and not self.detected_flag:
                # * As a member that found the object becomes the master.
                print(
                    f"[object_detected]({self.robot.getName()}) found cylinder @ {cylinder_position}"
                )
                # self.status = "path_finding"  
                self.status = "consensus"  
                self.driver.stop()

            # print(self.robot.getName(),f'{status=}')
            if self.status == "idle":
                self.driver.stop()

            elif self.status == "consensus" and not self.detected_flag:
                self.detected_flag = True  # detect once and top
                self.task_master = self.name
                self.communicator.task_master = self.name

                print(f"[consensus]({self.robot.getName()}) waiting consensus...")
                self.communicator.broadcast_message("[task]", cylinder_position)

            elif self.status == "path_finding" and self.task_master == self.name:
                # Used only by the TaskMaster
                self.path_finding()
                self.status = "path_following"
                # self.status = "idle"

            elif self.status == "path_following":
                self.path = self.communicator.path
                # print(f"[path_printing]({self.name}) {self.path}")

                # self.driver.stop()
                if self.path != "":
                    # self.driver.move_forward()
                    self.driver.simple_follow_path(self.path)
                    # self.driver.anti_clockwise_spin()
                    quit()
                    # self.driver.stop()
                self.status = "idle"
                #     print(
                #         f"[path_following]({self.robot.getName()}) Making my way downtown, walking fast"
                #     )
                # else:
                #     print(f"[path_following]({self.robot.getName()}) FUCK")

                # break

            elif self.status == "task":
                if self.detected_flag:
                    print(f"[task_conflict]({self.robot.getName()})")
                    self.communicator.broadcast_message("[task_conflict]", self.priority_queue)
                else:
                    print(f"[task_successful]({self.robot.getName()})")
                    self.task_master = self.communicator.task_master
                    self.communicator.broadcast_message("[task_successful]", self.task_master)

                self.status = "idle"

            elif self.status == "reassign" and not self.reassign_flag:
                task_master = self.priority_queue.pop(0)
                if task_master == self.name:
                    self.path_finding()
                    self.status = "path_finding"
                    self.path = self.communicator.path

                    if self.path != "":
                        # self.driver.move_forward()
                        self.driver.simple_follow_path(self.path)
                        # self.driver.anti_clockwise_spin()
                        quit()
                        # self.driver.stop()
                    self.status = "idle"
                else:
                    self.task_master = task_master
                    self.communicator.task_master = task_master
                    self.communicator.broadcast_message("[task_successful]", self.task_master)
                    self.status = "idle"
                self.priority_queue.append(task_master)
                
                self.reassign_flag = True

            else:
                self.driver.move_along_polynomial()
                self.communicator.send_position(
                    robot_position={
                        "x": self.robot_position["x"],
                        "y": self.robot_position["y"],
                        "theta": self.robot_position["theta"],
                    }
                )

            #* if threading is used, the following code should be used
            # if GPS:
            #     self.robot_position["x"], self.robot_position["y"], current_z = (
            #         self.driver.gps.getValues()
            #     )
            # else:
            #     # self.localisation.update_odometry()
            #     self.localisation.update_odometry_o1()
            # self.leftMotor.setVelocity(MAX_SPEED * 0.5)
            # self.rightMotor.setVelocity(MAX_SPEED)

            self.communicator.robot_entries[self.name] = (
                self.driver.robot_position["x"],
                self.driver.robot_position["y"],
                self.driver.robot_position["theta"],
            )

    def formation_object(self):
        if self.detected_flag:
            coords = self.communicator.robot_entries.copy()
            for robot_name in coords:
                coords[robot_name] = list(
                    map(lambda x: round(x, 2), coords[robot_name])
                )
            # print(f"[path_finding]({self.name}): current coords={coords}")

            #! Obstacles are in a list format e.g. [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]; should be input from map so leaving it empty for now
            self.formationer = FormationMaster(
                robot=self.robot,
                current_coords=coords,
                object_coords=(cylinder_position["x"], cylinder_position["y"]),
                obstacles=[],
                debug_level=0,
            )
            self.formationer.calculate_target_coords()
            self.formationer.plan_paths()

            paths = json.dumps(self.formationer.paths)
            self.path = json.loads(paths)[self.name]

            return paths
        else:
            # print(
            #     f"[path_finding]({self.robot.getName()}) LISTENING for {cylinder_position}"
            # )
            return None


def main():
    import threading
    import traceback
    class bcolors:
        RED_FAIL       = '\033[91m'
        GRAY_OK        = '\033[90m'
        GREEN_OK       = '\033[92m'
        YELLOW_WARNING = '\033[93m'
        BLUE_OK        = '\033[94m'
        MAGENTA_OK     = '\033[95m'
        CYAN_OK        = '\033[96m'
        ENDC           = '\033[0m'
        BOLD           = '\033[1m'
        ITALIC         = '\033[3m'
        UNDERLINE      = '\033[4m'
        
    member = SwarmMember()
    localisation_service = threading.Thread(target=member.driver.run_odometry_service)

    if member.driver.check_encoder_not_null_and_init():
        localisation_service.start()
        try:
            member.random_movement_find()
        except:
            tb_str = traceback.format_exc()
            print(bcolors.RED_FAIL + tb_str + bcolors.ENDC)
            member.driver.alive = False
            quit()
        member.driver.alive = False


main()
