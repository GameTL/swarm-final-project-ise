#!/usr/bin/env python3
""" 
Run the following before 
ros2 launch /home/orin_nano/swarm-final-project-ise/ros2_ws/launch/jeff_bringup_launch.py
>>> [INFO] [launch]: All log files can be found below /home/orin_nano/.ros/log/2025-04-18-19-06-37-354274-jetson1-3994
>>> [INFO] [launch]: Default logging verbosity is set to INFO
>>> [INFO] [x_drive_controller-1]: process started with pid [3995]
>>> [INFO] [sllidar_node-2]: process started with pid [3997]
>>> [sllidar_node-2] [INFO] [1744977997.534736341] [sllidar_node]: SLLidar running on ROS2 package SLLidar.ROS2 SDK Version:1.0.1, SLLIDAR SDK Version:2.1.0
>>> [sllidar_node-2] [INFO] [1744977997.555858183] [sllidar_node]: SLLidar S/N: D80BE1F7C2E399D7C2E592F128DF4110
>>> [sllidar_node-2] [INFO] [1744977997.555996369] [sllidar_node]: Firmware Ver: 1.02
>>> [sllidar_node-2] [INFO] [1744977997.556016306] [sllidar_node]: Hardware Rev: 18
>>> [sllidar_node-2] [INFO] [1744977997.557819761] [sllidar_node]: SLLidar health status : 0
>>> [sllidar_node-2] [INFO] [1744977997.557853268] [sllidar_node]: SLLidar health status : OK.
>>> [sllidar_node-2] [INFO] [1744977997.734668013] [sllidar_node]: current scan mode: DenseBoost, sample rate: 32 Khz, max_distance: 40.0 m, scan frequency:10.0 Hz, 
if not change the port form 1 to 0 and 0 to 1 

>>> Run Bringup
ros2 launch '/home/orin_nano/swarm-final-project-ise/ros2_ws/launch/jeff_bringup_launch.py'
>>> Run Teleop
ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args 
>>> Run this script 
python '/home/orin_nano/swarm-final-project-ise/ros2_ws/src/collective_transport/collective_transport/collective_transport_camera_odom.py'
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor          # NEW
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from datetime import datetime
import time
import threading 
import traceback
import math
import cv2
import numpy as np
from rich.pretty import pprint
# from .submodules.dynamixel_class import DynamixelInterface
import os
import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from geometry_msgs.msg import Pose2D, Twist

from submodules.p2p_communication.communicator import Communicator
from submodules.object_detection.object_detection_ycbcr_class import CVMeasure
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
# Get the environment variable
ROBOT_ID: str = str(os.environ.get('ROBOT_ID'))
fake = False
stop_msg = Twist()

THRESHOLD_X_POSITION = 0.005 # 3 cm
THRESHOLD_Y_POSITION = 0.005 # 3 cm
THRESHOLD_THETA_POSITION = 1 # 3 cm
MAX_WAIT = 10.0  # seconds to wait for peer acknowledgment

communicator = Communicator(identifier="1", suppress_output=False)

# communicator.start_periodic_broadcast(interval_sec=1)
# Create a class to manage shared ROS subscribers and publishers
class ROSManager(Node):
    def __init__(self, node_name='state_machine_node'):
        super().__init__('minimal_subscriber')

        self.latest_robot_pose = Pose2D() # Continously update the lastest prosition
        
        self.robot_pose_sub = self.create_subscription(
            Pose2D,                   # Message type
            '/robot_pose',             # Topic name
            self.robot_pose_callback,  # Callback function
            2)                       # QoS profile (queue size)
            
        self.robot_pose_sub  # prevent unused variable warning
        # Set up publishers 
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 2)
        self.spinned = False

        
        # Start a non-blocking spin in a separate thread
        self.spin_thread = threading.Thread(target=self._spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
    def _spin_node(self):
        while rclpy.ok() and not self.spinned:
            rclpy.spin_once(self, timeout_sec=0.5)
            self.spinned = True
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
        # print(f"new {msg}")
        
    def get_latest_pose(self):
        return self.latest_robot_pose
        
    def publish_cmd_vel(self, twist_msg):
        self.cmd_vel_pub.publish(twist_msg)
        
    def shutdown(self):
        self.node.destroy_node()
if not rclpy.ok():
    rclpy.init()
ros_manager = ROSManager()

def camera_check(camera_id="/dev/video0"):
    """ 
    Check if the camera is present
    """
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        return False
    ret, frame = cap.read()
    cap.release()
    return ret and frame is not None
    
    
#### END Special Function
class Init(State):
    def __init__(self) -> None:
        """
        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "end"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        print(bcolors.YELLOW_WARNING + f"Executing state Init" + bcolors.ENDC)
        print(f"Robot ID: {str(ROBOT_ID)}")

        if camera_check():
            print(bcolors.YELLOW_WARNING + f"Checking Camera and lights" + bcolors.ENDC)
        else:
            print(bcolors.RED_FAIL + f"ERROR: Camera or lights check failed." + bcolors.ENDC)
            return "end" 
        
        DEMO_TIME = 1
        time.sleep(DEMO_TIME) # for showing and filming
        
        print("Assuming the theta = 0 globally")
        return "outcome1"
    
class Ready(State):

    def __init__(self) -> None:
        super().__init__(["outcome1", "end"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        # DATA COLLECTION: Ready State Start
        communicator.data_collector.collect_data("ready_state_start", str(datetime.now()))
        print(bcolors.YELLOW_WARNING + f"Executing state Ready" + bcolors.ENDC)
        while True:
            if communicator.header != '':
                print(f"{communicator.header=}")
            if communicator.header == "START_EXPERIMENT":
                print(bcolors.GREEN_OK + "Received START_EXPERIMENT signal. Proceeding..." + bcolors.ENDC)
                return "outcome1"
            time.sleep(0.01)
        
class SeekObject(State):
    def __init__(self) -> None:
            
        super().__init__(["outcome1","loop", "end"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        # DATA COLLECTION: Object Detection Start
        communicator.data_collector.collect_data("object_detection_start", str(datetime.now()))
        # if not hasattr(self, "cv_class"):/
        self.cv_class = CVMeasure(cv_window=False)
        # self.cv_class = CVMeasure(cv_window=True)
        print(bcolors.YELLOW_WARNING + f"Executing state SeekObject" + bcolors.ENDC)
        
        # print(f"{self.cv_class.cylinder_detection=}")
        
        while 1:
            if ROBOT_ID == "1":
                rob_pose = [1.00, 1.00, 0.00] # For experiment 
            else: # ROBOT == 2
                rob_pose = [0.00, 0.00, 0.00] # For experiment 
            if communicator.header == "PATH":  # received the path from master
                print(bcolors.YELLOW_WARNING + f"RECEIVED PATH HEADER" + bcolors.ENDC)
                return "outcome2"  # IdleSlave
            
            detection_info = self.cv_class.cylinder_detection # NEED lidar to work
            
            # --- Simplified Validation ---
            is_valid_detection = False
            if isinstance(detection_info, list) and len(detection_info) >= 3:
                # STOP MSG
                # Check if all of the first three elements are finite numbers
                if all(math.isfinite(val) for val in detection_info[:3]):
                    is_valid_detection = True
            # --- End Simplified Validation ---

            if is_valid_detection:
                time.sleep(0.01) # simulating stopping
                print(f"{detection_info=}")
                print(f"Found an object stopping.. Validating detection data...")
                
                # Since we've validated, no need for the extra checks here,
                # but you might want to log the valid data explicitly if needed.
                print(f"Valid detection data received: {detection_info[:3]}")
                
                break # Exit the loop as valid data is found

        if isinstance(detection_info, list) and len(detection_info) >= 3:
            # Check if all of the first three elements are finite numbers
            twist_msg = Twist()
            # STOP MSG
            if all(math.isfinite(val) for val in detection_info[:3]):
                print(bcolors.BLUE_OK + f"Found a Detection at {detection_info=}" + bcolors.ENDC) 
                
                blackboard["object_info"] = detection_info
                object_d     = detection_info[0]
                object_w     = detection_info[1]
                object_theta = detection_info[2]
                
                # communicator.object_coords = [0.0, 0.0] # assume
                # communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]] #
                
                object_pose = [ # robot pose  + the distance xy of the object relative to robot pose
                    rob_pose[0] + (object_d)*math.cos(math.radians(rob_pose[2] + object_theta)) * 0.8,
                    rob_pose[1] + (object_d)*math.sin(math.radians(rob_pose[2] + object_theta)) * 0.8
                ]
                # Real Results
                # communicator.object_coords = object_pose
                # Testing Results
            communicator.object_coords = [0.5,0.5]
            print(bcolors.BLUE_OK + f"Object Calculated: \n\trobot_pose {rob_pose}\n\tobject_pose {communicator.object_coords }" + bcolors.ENDC)
            communicator.obstacle_coords = [] # assume
            communicator.object_detected()
            communicator.cleanup() # To clear waypoints and orientation
                    # return "end" #FoundObjectHost
            return "outcome1" #FoundObjectHost
        

class PathFollowing(State):
    """
    - Motor     : Rotating Stationary, slow speed
    - CV        : On
    - Lidar     : Off
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Outcomes:
            outcome: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "end"])
        self.counter = 0
        
        self.linear_kp = 1
        self.linear_ki = 0 
        self.linear_kd = 0

        self.angular_kp = 1
        self.angular_ki = 0 
        self.angular_kd = 0

        
    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        # DATA COLLECTION: Path Following Start
        communicator.data_collector.collect_data("path_following", str(datetime.now()))
        print(bcolors.YELLOW_WARNING + f"Executing state PathFollowing" + bcolors.ENDC)
        pprint(communicator.command)
        pprint(communicator.orientation)

        
        # STOP MSG
        time.sleep(1)
        return "outcome1"
"""
New stuff here
"""
class EndingExperiment(State):
    """
    Both robots have reached the object and acknowledge gripping it.
    """
    def __init__(self) -> None:
        super().__init__(["outcome1", "end"])

    def execute(self, blackboard: Blackboard) -> str:
        print(bcolors.YELLOW_WARNING + "Executing EndingExperiment state: Ending Experiment" + bcolors.ENDC)
        # broadcast that this robot has gripped the object
        communicator.broadcast("END_EXPERIMENT", communicator.object_coords)
        # wait for peer to acknowledge grip
        start = time.time()
        while time.time() - start < MAX_WAIT:
            if getattr(communicator, "header", None) == "END_EXPERIMENT":
                break
            time.sleep(0.1)
        else:
            return "end"            
        print(bcolors.GREEN_OK + "Both robots have acknowledged ending experiment" + bcolors.ENDC)
        print(bcolors.GREEN_OK + "Clean shutdown completed." + bcolors.ENDC)
        print(bcolors.GREEN_OK + "Saving data..." + bcolors.ENDC)
        print(bcolors.GREEN_OK + f"{communicator.data_collector.data} | test_run: {communicator.data_collector.test_run_number}" + bcolors.ENDC)
        communicator.data_collector.save_data()
        return "outcome1"

              
def main(args=None):
    
    
    # Set ROS 2 loggers
    set_ros_loggers()
    
    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state("Init", Init(), transitions={"outcome1": "Ready","end": "outcome4"})
    sm.add_state("Ready", Ready(), transitions={"outcome1": "SeekObject","end": "outcome4"})
    
    sm.add_state("SeekObject", SeekObject(), transitions={"outcome1": "PathFollowing","loop": "SeekObject","end": "outcome4"})
    # tell arrvied via comms  EndingExperiment
    # click in place 
    # move 45 degrees towards the center 
    
    # sm.add_state("IdleSlave", IdleSlave(), transitions={"outcome1": "BAR","end": "outcome4"}) # TaskMaster
    # sm.add_state("FoundObject", FoundObject(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    # sm.add_state("PathPlanning", PathPlanning(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    
    sm.add_state("PathFollowing", PathFollowing(), transitions={"outcome1": "EndingExperiment","end": "outcome4"}) #Slave
    
    sm.add_state("EndingExperiment", EndingExperiment(), transitions={"outcome1": "outcome4","end": "outcome4"})
    # sm.add_state("DiagonalTransport", DiagonalTransport(), transitions={"outcome1": "outcome4", "end":"outcome4"})

    
    # Start listening thread
    server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
    server_thread.start()

    try:
        print("State Machine starting")
        outcome = sm()
        print(f"State Machine finished with outcome: {outcome}")
        
        if outcome == "outcome4":
            print(bcolors.GREEN_OK + "Experiment completed successfully. Cleaning up..." + bcolors.ENDC)
            # Stop all motion
            ros_manager.publish_cmd_vel(stop_msg)
            time.sleep(0.5)  # Give time for the stop command to be processed
            
            # Clean up ROS and other resources
            if rclpy.ok():
                rclpy.shutdown()
            
            print(bcolors.GREEN_OK + "Clean shutdown completed." + bcolors.ENDC)
            print(bcolors.GREEN_OK + "Saving data..." + bcolors.ENDC)
            print(bcolors.GREEN_OK + f"{communicator.data_collector.data} | test_run: {communicator.data_collector.test_run_number}" + bcolors.ENDC)
            communicator.data_collector.save_data()
            os._exit(0)  # Force clean exit
            
    except KeyboardInterrupt:
        print(bcolors.YELLOW_WARNING + "\nProcess interrupted by user. Cleaning up..." + bcolors.ENDC)
        time.sleep(1)
        # Stop all motion
        ros_manager.publish_cmd_vel(stop_msg)
        time.sleep(1)
        if sm.is_running():
            sm.cancel_state()
    
    # Wait for server thread to complete
    server_thread.join(timeout=2.0)  # Add timeout to prevent hanging

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()
        
    print(bcolors.RED_FAIL + "Exiting due to error or interruption" + bcolors.ENDC)
    os._exit(1)  # Exit with error code

if __name__ == '__main__':
    main()