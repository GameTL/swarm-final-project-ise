#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
import time
import threading 
import traceback
import math
# from .submodules.dynamixel_class import DynamixelInterface
import os
import cv2
import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from geometry_msgs.msg import Pose2D, Twist

from submodules.p2p_communication.communicator import Communicator

from submodules.object_detection import object_detection
# Get the environment variable
ROBOT_ID: int = int(os.environ.get('ROBOT_ID'))
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
# Define the FooState class, inheriting from the State class
class Init(State):
    """
    - Motor     : Stop, read the encoder check odom if it's ok, (Check if Thread exist, Start Threading)
    - CV        : Stop, check if camera is present, then close
    - Lidar     : Running, run the LiDAR node 
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to track the number of executions of this state.
    """

    def __init__(self) -> None:
        """
        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0
        self.publisher_ = self.create_publisher(Pose2D, 'topic', 10)


    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state Init")
        yasmin.YASMIN_LOG_INFO(f"Robot ID: {str(ROBOT_ID)}")
        time.sleep(1) 
        
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

    def __init__(self) -> None:
        """
        Outcomes:
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.lastest_robot_pose = []
        self.goal_reached = False
        
        self.robot_pose_sub = rclpy.node.create_subscription(
            Pose2D,                   # Message type
            'robot_pose',                  # Topic name
            self.robot_pose_callback,      # Callback function
            10)                            # QoS profile (queue size)
            # Create publisher for velocity commands
        self.cmd_vel_pub = rclpy.node.create_publisher(
            Twist,                         # Message type
            'cmd_vel_nav',                     # Topic name
            10)                            # QoS profile
            
    def robot_pose_callback(self, msg):
        self.lastest_robot_pose = msg.pose

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state MoveStartPos")
        
        if ROBOT_ID == 1:
            # sub to robot_pos
            goal_pose = [self.lastest_robot_pose.x + 0.5, 
                         self.lastest_robot_pose.y + 0.5,
                         self.lastest_robot_pose.theta + 0]
            while True:
                err_pose = self.lastest_robot_pose - goal_pose
                if (err_pose[0]**2 + err_pose[1]**2) > 0.005:
                    vel = [
                        self.kp * err_pose[0],
                        self.kp * err_pose[1]
                    ]
                    twist_msg = Twist()
                    [twist_msg.linear.x, twist_msg.linear.y] = vel
                    self.cmd_vel_pub.publish(twist_msg)
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

    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state AtStartPos")
        # TODO check other robot if at the place 
        #* for testing 1 robot ---> bypass
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

    def __init__(self) -> None:
        def detection_worker():
            # containst
            self.cylinder_detection = object_detection.detect_yellow_cylinder(self.lidar_reader_node, pub=True)
            #baaaaa
            
        super().__init__(["outcome1", "outcome2"])
        camera_id = "/dev/video0"
        video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        video_capture.set(cv2.CAP_PROP_FPS, 30)
        self.counter = 0
        self.lidar_reader_node = object_detection.LidarReader()
        self.lidar_thread = threading.Thread(target=rclpy.spin, args=(self.node), daemon=True)
        
        
        self.detection_thread = threading.Thread(target=detection_worker)
        self.detection_thread.daemon = True
        self.robot_pose_sub = rclpy.node.create_subscription(
            Pose2D,                   # Message type
            'robot_pose',                  # Topic name
            self.robot_pose_callback,      # Callback function
            10)                            # QoS profile (queue size)
    def robot_pose_callback(self, msg):
        self.lastest_robot_pose = msg.pose

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state SeekObject")
        
        # TODO Sub to the object detection
        # TODO Pub twist msg to keep rotating. 
        self.lidar_thread.start()
        self.detection_thread.start()
        
        while self.cylinder_detection == None: # 
                if communicator.header == "PATH": # received the path from master
                    self.lidar_reader_node.destroy_node()
                    return "outcome2" # IdleSlave
                twist_msg = Twist()
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.001
                self.cmd_vel_pub.publish(twist_msg)
                time.sleep(0.1) # around 10hz
                break
        # send stop twist
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
        detection_info = self.cylinder_detection # do smth
        object_theta = float(detection_info["relative_angle"] )
        object_d= float(detection_info["distance"])
        object_w= float(detection_info["width"])
        
        detection_info = self.lastest_robot_pose
        last_pose =[self.lastest_robot_pose.x, 
        self.lastest_robot_pose.y,
        self.lastest_robot_pose.theta + 0]
        # communicator.object_coords = [0.0, 0.0] # assume
        # communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]] # assume
        
        object_pose = [ # robot pose  + the distance xy of the object relative to robot pose
            last_pose[0] + (object_d+object_w/2)*math.cos(math.radians(last_pose[2] + object_theta)),
            last_pose[1] + (object_d+object_w/2)*math.sin(math.radians(last_pose[2] + object_theta))
        ]
        communicator.object_coords = object_pose
        yasmin.YASMIN_LOG_INFO(f"Object Calculated: \n\trobot_pose {last_pose}\n\tobject_pose {object_pose}")
        communicator.obstacle_coords = [] # assume
        self.lidar_reader_node.destroy_node()
        self.lidar_thread.join()
        return "outcome1" #FoundObjectHost
        
        

# class FoundObjectHost(State):
#     """
#     - Motor     : Rotating Stationary, slow speed
#     - CV        : On
#     - Lidar     : Off
#     - Odometry  : Check Wheel Movement (Threading)
    
#     Attributes:
#         counter (int): Counter to track the number of executions of this state.
#     """

#     def __init__(self) -> None:
#         """
#         Outcomes:
#             outcome1: Indicates the state should continue to the Bar state.
#             outcome2: Indicates the state should finish execution and return.
#         """
#         super().__init__(["outcome1", "outcome2"])

#     def execute(self, blackboard: Blackboard) -> str:
#         """
#         Executes the logic for the Foo state.
#         Args: blackboard (Blackboard): The shared data structure for states.
#         Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
#         Raises: Exception: May raise exceptions related to state execution.
#         """
#         yasmin.YASMIN_LOG_INFO("Executing state ")
#         # time.sleep(3)  # Simulate work by sleeping

#         #  TODO rotate the robot untill the cylinder is centered
#         communicator.object_coords = [0.75, -0.25] # assume
#         communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]] # assume
        
#         communicator.object_detected() # activate consensus send to all with master nomination. check if master. 
#         # communicator would broadcast PATH
#         return "outcome1"
        
        

# class IdleSlave(State):
#     """
#     - Motor     : Stop, On
#     - CV        : Off
#     - Lidar     : Off
#     - Odometry  : Check Wheel Movement (Threading)
    
#     Attributes:
#         counter (int): Counter to track the number of executions of this state.
#     """

#     def __init__(self) -> None:
#         """
#         Outcomes:
#             outcome1: Indicates the state should continue to the Bar state.
#             outcome2: Indicates the state should finish execution and return.
#         """
#         super().__init__(["outcome1", "outcome2"])
#         self.counter = 0

#     def execute(self, blackboard: Blackboard) -> str:
#         """
#         Executes the logic for the Foo state.
#         Args: blackboard (Blackboard): The shared data structure for states.
#         Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
#         Raises: Exception: May raise exceptions related to state execution.
#         """
#         yasmin.YASMIN_LOG_INFO("Executing state FOO")
#         # time.sleep(3)  # Simulate work by sleeping

#         if self.counter < 3:
#             self.counter += 1
#             blackboard["foo_str"] = f"Counter: {self.counter}"
#             return "outcome1"
#         else:
#             return "outcome2"

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
            outcome1: Indicates the state should continue to the Bar state.
            outcome2: Indicates the state should finish execution and return.
        """
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0
        
        self.linear_kp = 1
        self.linear_ki = 0 
        self.linear_kd = 0

        self.angular_kp = 1
        self.angular_ki = 0 
        self.angular_kd = 0

        
        self.robot_pose_sub = rclpy.node.create_subscription(
            Pose2D,                   # Message type
            'robot_pose',                  # Topic name
            self.robot_pose_callback,      # Callback function
            10)                            # QoS profile (queue size)
        self.cmd_vel_pub = rclpy.node.create_publisher(
            Twist,                         # Message type
            'cmd_vel_nav',                     # Topic name
            10)                            # QoS profile
            
    def robot_pose_callback(self, msg):
        self.lastest_robot_pose = msg.pose
        
    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state PathFollowing")
        
        #* [path_planning](jetson2) Assigning jetson1 [1.78, -1.05] -> (1.05, -0.25, 3.14)
        
        twist_msg = Twist()
        
        paths = [
            (0.0, 0.5),
            (0.5, 0.5),
            (0.5, 0.5, 3.14),
        ]
        for path in paths:
            sum_e = [0,0]
            de_dt = [0,0]
            
            prev_err_pose = [0,0]
            prev_t = time.time_ns()
            # path following to th point
            while True:
                err_pose = self.lastest_robot_pose - path
                if (err_pose[0]**2 + err_pose[1]**2) > 0.005:
                    t = time.time_ns()
                    delta_t = (t-prev_t)*10**9
                    prev_t = int(t)
                    sum_e += err_pose * delta_t
                    de_dt = (err_pose - prev_err_pose)/delta_t
                    
                    vel = [
                        self.linear_kp * err_pose[0] + self.linear_ki * sum_e[0] + self.linear_kd * de_dt[0],
                        self.linear_kp * err_pose[1] + self.linear_ki * sum_e[1] + self.linear_kd * de_dt[1]
                    ]
                    [twist_msg.linear.x, twist_msg.linear.y] = vel
                    yasmin.YASMIN_LOG_INFO(f"\terr_pose {err_pose}\n\ttwist_pose {vel}")
                    self.cmd_vel_pub.publish(twist_msg)
                else:
                    break
            
        # rotation to point the gripper
        sum_e = 0
        de_dt = 0
        
        prev_err_angle = 0
        prev_t = time.time_ns()
        # path following to th point
        angle = 45
        while True:
            err_angle = self.lastest_robot_pose[2] - angle
            if (err_angle**2) > 0.005:
                t = time.time_ns()
                delta_t = (t-prev_t)*10**9
                prev_t = int(t)
                sum_e += err_angle * delta_t
                de_dt = (err_angle - prev_err_angle)/delta_t
                
                vel = self.linear_kp * err_angle + self.linear_ki * sum_e + self.linear_kd * de_dt,
                [twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z] = [0,0,vel]
                yasmin.YASMIN_LOG_INFO(f"\terr_angle {err_angle}\n\ttwist_pose {vel}")
                self.cmd_vel_pub.publish(twist_msg)
            else:
                break

        # goal2 = [
            # (1.78, -1.05),
            # (1.78, -0.25),
            # (1.05, -0.25, 3.14),
        # ]
        
        return "outcome1"
        
        
def main(args=None):
    yasmin.YASMIN_LOG_INFO("yasmin_frfr")
    

    
    rclpy.init(args=args)
    # Set ROS 2 loggers
    set_ros_loggers()
    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state("Init", Init(), transitions={"outcome1": "MoveStartPos","end": "outcome4"})
    sm.add_state("MoveStartPos", MoveStartPos(), transitions={"outcome1": "AtStartPos","end": "outcome4"})
    sm.add_state("AtStartPos", AtStartPos(), transitions={"outcome1": "SeekObject","end": "outcome4"})
    
    sm.add_state("SeekObject", SeekObject(), transitions={"outcome1": "FoundObject","outcome2": "IdleSlave","end": "outcome4"})
    # sm.add_state("IdleSlave", IdleSlave(), transitions={"outcome1": "BAR","end": "outcome4"}) # TaskMaster
    # sm.add_state("FoundObject", FoundObject(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    # sm.add_state("PathPlanning", PathPlanning(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    
    sm.add_state("PathFollowing", PathFollowing(), transitions={"outcome1": "AtObject","end": "outcome4"}) #Slave
    
    sm.add_state("AtObject", AtStartPos(), transitions={"outcome1": "outcome4","end": "outcome4"})

    YasminViewerPub("yasmin_demo", sm)
    

    # Start listening thread
    server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
    server_thread.start()

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    server_thread.join()

    # Shutdown ROS 2 if it's running
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()