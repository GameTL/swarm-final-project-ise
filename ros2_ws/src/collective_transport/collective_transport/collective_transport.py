#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
import time
import threading 
import traceback
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

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
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
            'cmd_vel',                     # Topic name
            10)                            # QoS profile
            
    def robot_pose_callback(self, msg):
        self.lastest_robot_pose = msg.pose

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state MoveStartPos")
        
        #TODO Move Diagonally 0.5 x and 0.5 y from the wall 
        
        if ROBOT_ID == 1:
            # sub to robot_pos
            goal_pose = [self.lastest_robot_pose.x + 0.5, 
                         self.lastest_robot_pose.y + 0.5,
                         self.lastest_robot_pose.theta + 0]
            while True:
                err_pose = goal_pose - goal_pose
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
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        
        # TODO Sub to the object detection
        # TODO Pub twist msg to keep rotating. 

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
        
        

class FoundObject(State):
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

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state ")
        # time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
        

class PathPlanning(State):
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

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        # time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
        

class IdleSlave(State):
    """
    - Motor     : Stop, On
    - CV        : Off
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

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        # time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"

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

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.

        Args:
            blackboard (Blackboard): The shared data structure for states.

        Returns:
            str: The outcome of the execution, which can be "outcome1" or "outcome2".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        # time.sleep(3)  # Simulate work by sleeping

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"
        
        
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
    sm.add_state("IdleSlave", IdleSlave(), transitions={"outcome1": "BAR","end": "outcome4"}) # TaskMaster
    sm.add_state("FoundObject", FoundObject(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    sm.add_state("PathPlanning", PathPlanning(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    
    sm.add_state("PathFollowing", PathFollowing(), transitions={"outcome1": "AtObject","end": "outcome4"}) #Slave
    
    sm.add_state("AtObject", AtStartPos(), transitions={"outcome1": "outcome4","end": "outcome4"})

    YasminViewerPub("yasmin_demo", sm)
    
    # Execute the FSM
    communicator = Communicator()

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