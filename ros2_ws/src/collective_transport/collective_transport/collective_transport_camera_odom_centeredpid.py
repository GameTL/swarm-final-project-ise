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
import numpy as np
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
from submodules.cam_odom.cam_odom_client import CamOdomClient
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
ROBOT_ID: int = int(os.environ.get('ROBOT_ID'))
fake = False
stop_msg = Twist()

stop_msg.linear.x   = 0.0
stop_msg.linear.y   = 0.0
stop_msg.linear.z   = 0.0
stop_msg.angular.x  = 0.0
stop_msg.angular.y  = 0.0
stop_msg.angular.z  = 0.0

""" Location of the robot in the arena
 ^ 
 | 
_____________
|          2|
|           |
|           |
|1          |
_____________ 
"""

MAX_CMD_VEL = 0.05 # 5cm per second
communicator = Communicator(identifier=ROBOT_ID)
try:
    cam_odom = CamOdomClient(robot_id=3, debug=False)
    cam_odom.connect()
except Exception as e:
    print(e)
    quit()
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

        
        # Start a non-blocking spin in a separate thread
        self.spin_thread = threading.Thread(target=self._spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
        
    def _spin_node(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
            
    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg
        # print(f"new {msg}")
        
    def get_latest_pose(self):
        return self.latest_robot_pose
        
    def publish_cmd_vel(self, twist_msg):
        self.cmd_vel_pub.publish(twist_msg)
        
    def shutdown(self):
        self.node.destroy_node()
        
class Init(State):
    """
    - Motor     : Stop, read the encoder check odom if it's ok, (Check if Thread exist, Start Threading)
    - CV        : Stop, check if camera is present, then close
    - Lidar     : Running, run the LiDAR node 
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self, ros_manager: ROSManager) -> None:
        """
        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "end"])
        self.ros_manager = ros_manager
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO(bcolors.YELLOW_WARNING + f"Executing state Init" + bcolors.ENDC)
        yasmin.YASMIN_LOG_INFO(f"Robot ID: {str(ROBOT_ID)}")
        # time.sleep(1) 
        
        yasmin.YASMIN_LOG_INFO("Assuming the theta = 0 globally")
        return "outcome1"

class MoveStartPos(State):
    """
    - Motor     : Moving Diagonally
    - CV        : Stop
    - Lidar     : Stop
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self, ros_manager: ROSManager) -> None:
        """
        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "end"])
        self.ros_manager = ros_manager
        self.goal_reached = False
        self.linear_kp = 1

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data struc
        re for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO(bcolors.YELLOW_WARNING + f"Executing state MoveStartPos" + bcolors.ENDC)
        
        # print(self.ros_manager.get_latest_pose())
        
        if ROBOT_ID == 1:
            # sub to robot_pos
            rob_pose = self.ros_manager.get_latest_pose()
            # goal_pose = np.array([rob_pose.x + 0.5, 
            #              rob_pose.y + 0.5,
            #              rob_pose.theta + 0])
            goal_pose = np.array([rob_pose.x + 0, 
                         rob_pose.y + 0,
                         rob_pose.theta + 0])
            yasmin.YASMIN_LOG_INFO(f"{goal_pose=}, {rob_pose=}")
            while True:
                rob_pose = self.ros_manager.get_latest_pose()
                
                # print(f'{rob_pose=}')
                err_pose = np.array([rob_pose.x, rob_pose.y, 0]) - goal_pose
                if (err_pose[0]**2 + err_pose[1]**2) > 0.005:
                    vel = [
                        self.linear_kp * err_pose[0],
                        self.linear_kp * err_pose[1]
                    ]
                    twist_msg = Twist()
                    [twist_msg.linear.x, twist_msg.linear.y] = vel
                    self.ros_manager.publish_cmd_vel(twist_msg)
                else:
                    break
        return "outcome1"

class AtStartPos(State):
    """
    Wait for all robot to reach this state --> move to SeekObject
    - Motor     : Stop
    - CV        : Stop
    - Lidar     : Running
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self, ros_manager: ROSManager) -> None:
        super().__init__(["outcome1", "end"])
        # self.ros_manager = ros_manager
        # self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO(bcolors.YELLOW_WARNING + f"Executing state AtStartPos" + bcolors.ENDC)
        # TODO check other robot if at the place 
        #* for testing 1 robot ---> bypass
        # return "end"
        return "outcome1"
        
class SeekObject(State):
    """
    - Motor     : Rotating Stationary, slow speed
    - CV        : On
    - Lidar     : Off
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self, ros_manager: ROSManager) -> None:
            
        super().__init__(["outcome1","loop", "end"])
        self.ros_manager = ros_manager

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        # if not hasattr(self, "cv_class"):/
        self.cv_class = CVMeasure(cv_window=False)
        # self.cv_class = CVMeasure(cv_window=True)
        yasmin.YASMIN_LOG_INFO(bcolors.YELLOW_WARNING + f"Executing state SeekObject" + bcolors.ENDC)
        
        # TODO Sub to the object detection
        # TODO Pub twist msg to keep rotating. 
        yasmin.YASMIN_LOG_INFO(f"{self.ros_manager.get_latest_pose()=}")
        # yasmin.YASMIN_LOG_INFO(f"{self.cv_class.cylinder_detection=}")
        
        # while 1:
        #     # yasmin.YASMIN_LOG_INFO(f"{self.cv_class.cylinder_detection=}")
        #     if communicator.header == "PATH":  # received the path from master
        #         return "outcome2"  # IdleSlave
            
        #     detection_info = self.cv_class.cylinder_detection
            
        #     # --- Simplified Validation ---
        #     is_valid_detection = False
        #     if isinstance(detection_info, list) and len(detection_info) >= 3:
        #         self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        #         # Check if all of the first three elements are finite numbers
        #         if all(math.isfinite(val) for val in detection_info[:3]):
        #             is_valid_detection = True
        #     # --- End Simplified Validation ---

        #     if is_valid_detection:
        #         twist_msg = Twist()
        #         # self.ros_manager.publish_cmd_vel(twist_msg)
        #         yasmin.YASMIN_LOG_INFO(f"{detection_info=}")
        #         yasmin.YASMIN_LOG_INFO(f"Found an object stopping.. Validating detection data...")
                
        #         # Since we've validated, no need for the extra checks here,
        #         # but you might want to log the valid data explicitly if needed.
        #         yasmin.YASMIN_LOG_INFO(f"Valid detection data received: {detection_info[:3]}")
                
        #         break # Exit the loop as valid data is found

        #     else:
        #         # If validation failed (either not a list, too short, or contained non-finite values in the first 3)
        #         # yasmin.YASMIN_LOG_INFO(f"Invalid or incomplete detection data: {detection_info}. Sending rotating twist msg.")
        #         if fake:
        #             pass
        #         else:
        #             twist_msg = Twist()
        #             twist_msg.angular.z = 0.55
        #             self.ros_manager.publish_cmd_vel(twist_msg)
        #             pass
        #         # time.sleep(0.1)  # around 10hz
        # detection_info = self.cv_class.cylinder_detection # do smth
        # if isinstance(detection_info, list) and len(detection_info) >= 3:
        #     # Check if all of the first three elements are finite numbers
        #     twist_msg = Twist()
        #     self.ros_manager.publish_cmd_vel(twist_msg)
        #     if all(math.isfinite(val) for val in detection_info[:3]):
        #         yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Found a Detection at {detection_info=}" + bcolors.ENDC)
        
        #         blackboard["object_info"] = detection_info
        #         object_theta = detection_info[0]
        #         object_d     = detection_info[1]
        #         object_w     = detection_info[2]
                
        #         detection_info = self.ros_manager.get_latest_pose
        #         rob_pose = self.ros_manager.get_latest_pose()
        #         # communicator.object_coords = [0.0, 0.0] # assume
        #         # communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]] #
                
        #         object_pose = [ # robot pose  + the distance xy of the object relative to robot pose
        #             rob_pose.x + (object_d+object_w/2)*math.cos(math.radians(rob_pose.theta + object_theta)),
        #             rob_pose.y + (object_d+object_w/2)*math.sin(math.radians(rob_pose.theta + object_theta))
        #         ]
        #         communicator.object_coords = object_pose
        #         yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Object Calculated: \n\trobot_pose {rob_pose}\n\tobject_pose {object_pose}" + bcolors.ENDC)
        #         communicator.obstacle_coords = [] # assume
        #         # return "end" #FoundObjectHost
        return "outcome1" #FoundObjectHost
        return "loop"
        

class PathFollowing(State):
    """
    - Motor     : Rotating Stationary, slow speed
    - CV        : On
    - Lidar     : Off
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self, ros_manager: ROSManager) -> None:
        """
        Outcomes:
            outcome: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "end"])
        self.ros_manager = ros_manager
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
        #### Special Function
        class PID:
            def __init__(self, kp=1, ki=0, kd=0, min=-0.5, max=0.5, deadzone_limit=0.4, clamped=True):
                self.kp = kp
                self.ki = ki
                self.kd = kd
                self.clamped = clamped
                self.min = min
                self.max = max
                self.diff_err = np.float64(0.0)
                self.sum_err = np.float64(0.0)
                self.prev_err : np.float64 = np.float64(0.0)
                self.prev_time : float = time.time_ns()
            def calculate(self, err) -> np.float64:
                err = np.float64(err) # ensure it's float64
                # Make sure the err is negetive an positive otherwise the Err Sum will runaway
                curr_time = time.time_ns()
                delta_t = np.float64(((curr_time - self.prev_time)/ 10**9))
                self.sum_err += err * delta_t
                self.diff_err = (err - self.prev_err)/delta_t
                out = self.kp *  err + self.ki  * self.sum_err + self.kd * self.diff_err
                
                if -self.activation_threshold < out < self.activation_threshold:
                    out = np.sign(out) * self.activation_threshold
                    
                self.prev_time = curr_time
                self.prev_err = err
                if self.clamped:
                    return max(min(out, self.max), self.min)
                else:
                    return out

            def reset(self):
                self.sum_err = 0
                self.prev_err : np.float64 = np.float64(0.0)
                self.prev_time : float = time.time_ns()
                
        def globaltorobottf(global_x_vel, global_y_vel):
            sin_cos_45 = 0.70710678118
            out_x = (-global_x_vel*sin_cos_45 + global_y_vel*sin_cos_45 )
            out_y = (global_x_vel*sin_cos_45 + global_y_vel*sin_cos_45 )
            return out_x, out_y
        #### END Special Function
        
        yasmin.YASMIN_LOG_INFO(bcolors.YELLOW_WARNING + f"Executing state PathFollowing" + bcolors.ENDC)
        communicator.object_coords # x, y, theta
        #### Specs SI Units
        target_x = 0.5 #m
        target_y = 0.5 #m
        target_theta = 45 #degress
        threshold_x_position = 0.03 # 3 cm
        threshold_y_position = 0.03 # 3 cm
        while True:
            #* 1. THETA-movement 45 degree
            yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Moving in Theta-Direction" + bcolors.ENDC)
            twist_msg = Twist()
            # print(f'{cam_odom.current_position=}')
            # print(abs(45 - cam_odom.current_position["theta"]))
            err_theta = target_theta - cam_odom.current_position["theta"]
            if err_theta < -180 or err_theta > 180:
                err_theta = -(err_theta % 180) # tell it to go other way, "it's closer the other way"
            pid_theta = PID(kp=0.05, ki=0, kd=0, min=-0.5, max=0.5, deadzone_limit=0.3, clamped=True)
            while (abs(err_theta)) > 2: # err less than 2 degree, paired with the delay of the odom 2 degree is perfect
                print(f'{cam_odom.current_position=}')
                twist_msg.angular.z = pid_theta.calculate(err_theta)
                self.ros_manager.publish_cmd_vel(twist_msg)
            self.ros_manager.publish_cmd_vel(Twist()) # STOP MSG
            time.sleep(0.5)
            yasmin.YASMIN_LOG_INFO(bcolors.GREEN_OK + f"arrvied at theta= {cam_odom.current_position["theta"]}" + bcolors.ENDC)
            time.sleep(3)
            
            #* 2. Move along the x-axis first 
            yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Moving in global X-Direction" + bcolors.ENDC)
            twist_msg = Twist() # reset the twist msg
            pid_x = PID(kp=0.5, ki=0, kd=0, min=-0.5, max=0.5, deadzone_limit=0.3, clamped=True)
            while True:
                err_x = target_x - cam_odom.current_position["x"]
                # print(f'{err_x=}')
                # print(f'{cam_odom.current_position=}')
                # print(f'{twist_msg=}')
                if (abs(err_x)) < threshold_x_position:
                    break
                out_x = pid_x.calculate(err_x) # err_position --> Velocity
                twist_msg.linear.x, twist_msg.linear.y =  globaltorobottf(out_x, 0)
                self.ros_manager.publish_cmd_vel(twist_msg)
                time.sleep(0.01)
            self.ros_manager.publish_cmd_vel(Twist()) # STOP MSG
            self.sleep(0.5)
            yasmin.YASMIN_LOG_INFO(bcolors.GREEN_OK + f"arrvied at x= {cam_odom.current_position["x"]}" + bcolors.ENDC)
            time.sleep(2)
            
            #* 3. Move along the y-axis second 
            yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Moving in global Y-Direction" + bcolors.ENDC)
            twist_msg = Twist() # reset the twist msgs
            pid_y = PID(kp=0.5, ki=0, kd=0, min=-0.5, max=0.5, deadzone_limit=0.3, clamped=True)
            while True:
                err_y = target_y - cam_odom.current_position["y"]
                # print(f'{err_y=}')
                # print(f'{cam_odom.current_position=}')
                # print(f'{twist_msg=}')
                if (abs(err_y)) < threshold_y_position:
                    break
                out_y = pid_y.calculate(err_y) # err_position --> Velocity
                twist_msg.linear.x, twist_msg.linear.y =  globaltorobottf(0, out_y)
                self.ros_manager.publish_cmd_vel(twist_msg)
                time.sleep(0.01)
            self.ros_manager.publish_cmd_vel(Twist()) # STOP MSG
            time.sleep(0.5)
            yasmin.YASMIN_LOG_INFO(bcolors.GREEN_OK + f"arrvied at y= {cam_odom.current_position["y"]}" + bcolors.ENDC)
            time.sleep(2)
            """ 
            cmd_vel direction
            -0.5   +0.5    +x
            +0.5   -0.5    -x
            +0.5   +0.5    +y
            -0.5   -0.5    +y
            """
            quit()
        # goal2 = [
            # (1.78, -1.05),
            # (1.78, -0.25),
            # (1.05, -0.25, 3.14),
        # ]
        
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        time.sleep(1)
        return "outcome1"


              
def main(args=None):
    global cam_odom
    

    yasmin.YASMIN_LOG_INFO("yasmin_frfr")
    if not rclpy.ok():
        rclpy.init(args=args)
    

    ros_manager = ROSManager()
    
    # Set ROS 2 loggers
    set_ros_loggers()
    
    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state("Init", Init(ros_manager), transitions={"outcome1": "MoveStartPos","end": "outcome4"})
    sm.add_state("MoveStartPos", MoveStartPos(ros_manager), transitions={"outcome1": "AtStartPos","end": "outcome4"})
    sm.add_state("AtStartPos", AtStartPos(ros_manager), transitions={"outcome1": "SeekObject","end": "outcome4"})
    
    sm.add_state("SeekObject", SeekObject(ros_manager), transitions={"outcome1": "PathFollowing","loop": "SeekObject","end": "outcome4"})
    # sm.add_state("IdleSlave", IdleSlave(), transitions={"outcome1": "BAR","end": "outcome4"}) # TaskMaster
    # sm.add_state("FoundObject", FoundObject(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    # sm.add_state("PathPlanning", PathPlanning(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    
    sm.add_state("PathFollowing", PathFollowing(ros_manager), transitions={"outcome1": "AtObject","end": "outcome4"}) #Slave
    
    sm.add_state("AtObject", AtStartPos(ros_manager), transitions={"outcome1": "outcome4","end": "outcome4"})

    
    # Start listening thread
    server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
    server_thread.start()

    try:
        yasmin.YASMIN_LOG_INFO("State Machine starting")
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        time.sleep(1)
        ros_manager.publish_cmd_vel(stop_msg)
        ros_manager.publish_cmd_vel(stop_msg)
        ros_manager.publish_cmd_vel(stop_msg)
        time.sleep(1)
        if sm.is_running():
            sm.cancel_state()
    server_thread.join()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()