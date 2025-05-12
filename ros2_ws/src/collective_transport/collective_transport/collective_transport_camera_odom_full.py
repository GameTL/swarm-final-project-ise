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
ROBOT_ID: str = str(os.environ.get('ROBOT_ID'))
fake = False
stop_msg = Twist()

stop_msg.linear.x   = 0.0
stop_msg.linear.y   = 0.0
stop_msg.linear.z   = 0.0
stop_msg.angular.x  = 0.0
stop_msg.angular.y  = 0.0
stop_msg.angular.z  = 0.0

THRESHOLD_X_POSITION = 0.005 # 3 cm
THRESHOLD_Y_POSITION = 0.005 # 3 cm
THRESHOLD_THETA_POSITION = 1 # 3 cm

# wait for the clicking
MAX_WAIT = 10.0  # seconds to wait for peer acknowledgment
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
home_coords =  {"1" :[0.8,0.8,315], "2" : [0.2,0.2,315], "3" : [0.2,0.8,315]}
linear_pid_dict =  {
    "1" :{
        "kp" :  4,
        "ki" :  6.0,
        "kd" :  3,
        "clamped" :  True,
        "min" :  -0.8,
        "max" :  0.8,
        "deadzone_limit" : 0.2}, 
    "2" :{
        "kp" :  1.5,
        "ki" :  6.0,
        "kd" :  0.3,
        "clamped" :  True,
        "min" :  -0.8,
        "max" :  0.8,
        "deadzone_limit" : 0.4}}

angular_pid_dict =  {
    "1" :{ # with new wheels
        "kp" :  0.04,
        "ki" :  0.000,
        "kd" :  0.01,
        "clamped" :  True,
        "min" :  -0.9,
        "max" :  0.9,
        "deadzone_limit" : 0.35}, 
    "2" :{ # robocup wheels K_u = 0.05, T_u - 3.5
        "kp" :  0.02, # 0.02 also works well for P only
        "ki" :  0.0085714286, # (0.54* 0.025)/0.35 
        "kd" :  0.0,
        "clamped" :  True,
        "min" :  -0.9,
        "max" :  0.9,
        "deadzone_limit" : 0.45}
                     }

""" 
OLD
linear_pid_dict =  {
    "1" :{
        "kp" :  0.3,
        "ki" :  6.0,
        "kd" :  0.1,
        "clamped" :  True,
        "min" :  -0.7,
        "max" :  0.7,
        "deadzone_limit" : 0.2}, 
    "2" :{
        "kp" :  1,
        "ki" :  0.1,
        "kd" :  0.3,
        "clamped" :  True,
        "min" :  -0.5,
        "max" :  0.5,
        "deadzone_limit" : 0.4}}

angular_pid_dict =  {
    "1" :{ # with new wheels
        "kp" :  1,
        "ki" :  0,
        "kd" :  0,
        "clamped" :  True,
        "min" :  -0.9,
        "max" :  0.9,
        "deadzone_limit" : 0.4}, 
    "2" :{ # robocup wheels
        "kp" :  0.3,
        "ki" :  0.1,
        "kd" :  0.3,
        "clamped" :  True,
        "min" :  -0.9,
        "max" :  0.9,
        "deadzone_limit" : 0.52}
                     }
"""





MAX_CMD_VEL = 0.05 # 5cm per second
try:
    if int(ROBOT_ID) == 1:
        cam_odom = CamOdomClient(robot_id="3", debug=False)
    else:
        cam_odom = CamOdomClient(robot_id=ROBOT_ID, debug=False)
        
    cam_odom.connect()
except Exception as e:
    print(e)
    quit()
# while True:
time.sleep(1)
print([cam_odom.current_position["x"], cam_odom.current_position["y"], cam_odom.current_position["theta"]])
communicator = Communicator(identifier=ROBOT_ID, odom_obj=cam_odom, suppress_output=False)
# Start listening thread
server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
server_thread.start()

communicator.start_periodic_broadcast(interval_sec=1)
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
    def stop_motors(self):
        time.sleep(0.1)
        msg =  Twist()
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.1)
        self.cmd_vel_pub.publish(msg)
        time.sleep(0.1)
        
        
    def shutdown(self):
        self.node.destroy_node()
if not rclpy.ok():
    rclpy.init()
ros_manager = ROSManager()
#### Special Function
class PID:
    def __init__(self, kp=1, ki=0, kd=0, min=-0.5, max=0.5, deadzone_limit=0.4, clamped=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.clamped = clamped
        self.min = min
        self.max = max
        self.deadzone_limit = deadzone_limit
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
        out = self.kp *  err
        if -self.deadzone_limit < out < self.deadzone_limit:
            out = np.sign(out) * self.deadzone_limit
        out += self.ki  * self.sum_err + self.kd * self.diff_err
            
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
                
def globaltorobottf_at315(global_x_vel, global_y_vel):
    """ 
    cmd_vel direction
    0.5    +0.5    +x
    -0.5   -0.5    -x
    -0.5   +0.5    +y
    +0.5   -0.5    -y
    """
    sin_cos_315 = 0.70710678118
    out_x = (global_x_vel*sin_cos_315 - global_y_vel*sin_cos_315 )
    out_y = (+global_x_vel*sin_cos_315 + global_y_vel*sin_cos_315 )
    return out_x, out_y
def globaltorobottf(global_x_vel, global_y_vel, current_theta):
    current_theta_rad = np.radians(current_theta)
    out_x = (-global_x_vel*np.cos(current_theta_rad) + global_y_vel*np.sin(current_theta_rad) )
    out_y = (global_x_vel*np.sin(current_theta_rad) + global_y_vel*np.cos(current_theta_rad) )
    return out_x, out_y
import time
import numpy as np
import datetime # Added for timestamp in filename
import plotly.graph_objects as go # Added for plotting
def movetotheta(target_theta):
    # --- Logging setup ---
    log_times = []
    log_errors = []
    start_time = time.time()
    # --- End Logging setup ---
    print(bcolors.BLUE_OK + f"Moving in Theta-Direction; Goal: {target_theta=}" + bcolors.ENDC)
    twist_msg = Twist()
    pid_theta = PID(kp=angular_pid_dict[ROBOT_ID]["kp"], ki=angular_pid_dict[ROBOT_ID]["ki"], kd=angular_pid_dict[ROBOT_ID]["kd"], min=angular_pid_dict[ROBOT_ID]["min"], max=angular_pid_dict[ROBOT_ID]["max"], deadzone_limit=angular_pid_dict[ROBOT_ID]["deadzone_limit"], clamped=angular_pid_dict[ROBOT_ID]["clamped"])
    while True: # err less than 2 degree, paired with the delay of the odom 2 degree is perfect
        err_theta = target_theta - cam_odom.current_position["theta"]
        
        # Gemini way 
        # --- STANDARD Angle Wrap-Around Normalization ---
        if err_theta > 180:
            err_theta -= 360
        elif err_theta < -180:
            err_theta += 360
        # --- End Normalization ---
        # My way
        # if err_theta < -180 or err_theta > 180:
        #     err_theta = np.sign(err_theta) * (-1) * (err_theta % 180) # tell it to go other way, "it's closer the other way"
        if (abs(err_theta)) < THRESHOLD_THETA_POSITION:
        # if time.time() - start_time > 30:
            ros_manager.publish_cmd_vel(Twist()) # STOP MSG
            ## LOGGINGGG
            # # Generate filename
            # timestamp_str = datetime.datetime.now().strftime('%y%m%d%H%M%S')
            # filename = f"theta_error_log_{timestamp_str}.png" # Save as PNG
            # fig = go.Figure()
            # fig.add_trace(go.Scatter(x=log_times, y=log_errors, mode='lines+markers', name='Theta Error'))

            # fig.update_layout(
            #     title=f'Theta Error vs Time (Target: {target_theta} deg)',
            #     xaxis_title='Time (s)',
            #     yaxis_title='Error Theta (degrees)',
            #     legend_title='Legend'
            # )

            # # Save the plot
            # fig.write_image(filename)
            # print(f"saving {filename}")
            # quit()
            ## END LOGGINGGG 
            break
                # --- Log data ---
                
        elapsed_time = time.time() - start_time
        log_times.append(elapsed_time)
        log_errors.append(err_theta)
        # --- End Log data ---
        
        # print(f'{cam_odom.current_position=}')
        out_theta = pid_theta.calculate(err_theta)
        twist_msg.angular.z = out_theta
        print(f"\r Moving...| err_theta={err_theta:>6.2f} | out_theta={out_theta:>6.2f} | odom_pos: [{float(cam_odom.current_position['x']):>6.2f} {float(cam_odom.current_position['y']):>6.2f} {float(cam_odom.current_position['theta']):>6.2f}] |", end='', flush=True)
        # print(f"\r {err_theta=}; {out_theta=}; {cam_odom.current_position=}", end='', flush=True)
        # print(bcolors.BLUE_OK + f"{err_theta=},{out_theta=}, currpos={cam_odom.current_position}" + bcolors.ENDC)
        ros_manager.publish_cmd_vel(twist_msg)
        time.sleep(0.01)
        
    print()
    ros_manager.publish_cmd_vel(Twist()) # STOP MSG
    time.sleep(0.25)
    ros_manager.publish_cmd_vel(Twist()) # STOP MSG
    print(bcolors.GREEN_OK + f"arrvied at theta= {cam_odom.current_position['theta']}" + bcolors.ENDC)
    time.sleep(1.5)
    
def movealongx315(target_x):
    print(bcolors.BLUE_OK + f"Moving in global X-Direction; Goal: {target_x=}" + bcolors.ENDC)
    twist_msg = Twist() # reset the twist msg
    pid_x = PID(kp=linear_pid_dict[ROBOT_ID]["kp"], ki=linear_pid_dict[ROBOT_ID]["ki"], kd=linear_pid_dict[ROBOT_ID]["kd"], min=linear_pid_dict[ROBOT_ID]["min"], max=linear_pid_dict[ROBOT_ID]["max"], deadzone_limit=linear_pid_dict[ROBOT_ID]["deadzone_limit"], clamped=linear_pid_dict[ROBOT_ID]["clamped"])
    
    while True:
        err_x = target_x - cam_odom.current_position["x"]
        # print(f'{err_x=}')
        # print(f'{cam_odom.current_position=}')
        # print(f'{twist_msg=}')
        if (abs(err_x)) < THRESHOLD_X_POSITION:
            break
        out_x = pid_x.calculate(err_x) # err_position --> Velocity
        print(f"\r Moving...| err_x={err_x:>6.2f} | out_x={out_x:>6.2f} | odom_pos: [{float(cam_odom.current_position['x']):>6.2f} {float(cam_odom.current_position['y']):>6.2f} {float(cam_odom.current_position['theta']):>6.2f}] |", end='', flush=True)
        twist_msg.linear.x, twist_msg.linear.y =  globaltorobottf_at315(out_x, 0)
        ros_manager.publish_cmd_vel(twist_msg)
        time.sleep(0.05)
    print()
    ros_manager.publish_cmd_vel(Twist()) # STOP MSG
    time.sleep(0.25)
    ros_manager.publish_cmd_vel(Twist()) # STOP MSG
    print(bcolors.GREEN_OK + f"arrvied at x= {cam_odom.current_position['x']}" + bcolors.ENDC)
    time.sleep(1.5)

def movealongy315(target_y):
    print(bcolors.BLUE_OK + f"Moving in global Y-Direction; Goal: {target_y=}" + bcolors.ENDC)
    twist_msg = Twist() # reset the twist msgs
    pid_y = PID(kp=linear_pid_dict[ROBOT_ID]["kp"], ki=linear_pid_dict[ROBOT_ID]["ki"], kd=linear_pid_dict[ROBOT_ID]["kd"], min=linear_pid_dict[ROBOT_ID]["min"], max=linear_pid_dict[ROBOT_ID]["max"], deadzone_limit=linear_pid_dict[ROBOT_ID]["deadzone_limit"], clamped=linear_pid_dict[ROBOT_ID]["clamped"])
    while True:
        err_y = target_y - cam_odom.current_position["y"]
        # print(f'{err_y=}')
        # print(f'{cam_odom.current_position=}')
        # print(f'{twist_msg=}')
        if (abs(err_y)) < THRESHOLD_Y_POSITION:
            break
        out_y = pid_y.calculate(err_y) # err_position --> Velocity
        print(f"\r Moving...| err_y={err_y:>6.2f} | out_y={out_y:>6.2f} | odom_pos: [{float(cam_odom.current_position['x']):>6.2f} {float(cam_odom.current_position['y']):>6.2f} {float(cam_odom.current_position['theta']):>6.2f}] |", end='', flush=True)
        twist_msg.linear.x, twist_msg.linear.y =  globaltorobottf_at315(0, out_y)
        ros_manager.publish_cmd_vel(twist_msg)
        time.sleep(0.01)
    print()
    ros_manager.publish_cmd_vel(Twist()) # STOP MSG
    time.sleep(0.25)
    print(bcolors.GREEN_OK + f"arrvied at y= {cam_odom.current_position['y']}" + bcolors.ENDC)
    time.sleep(1.5)
    
def decode_coords(goals, x, y):
    task = []
    ws_coord = [x,y]
    for info in goals: 
        dx = info[0] - ws_coord[0]
        dy = info[1] - ws_coord[1]
        if abs(dx) > abs(dy):
            task.append(['x', info[0]])
            ws_coord[0] = info[0]
        else:
            task.append(['y', info[1]])
            ws_coord[1] = info[1]
        if len(info) == 3: # lastinfo
            task.append(['theta', info[2]])
    return task
#### END Special Function
class Init(State):
    """
    - Motor     : Stop, read the encoder check odom if it's ok, (Check if Thread exist, Start Threading)
    - CV        : Stop, check if camera is present, then close
    - Lidar     : Running, run the LiDAR node 
    - Odometry  : Check Wheel Movement (Threading)
    
    Attributes:
        counter (int): Counter to    track the number of executions of this state.
    """

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
        
        DEMO_TIME = 1
        time.sleep(DEMO_TIME) # for showing and filming
        
        print("Assuming the theta = 0 globally")
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
        super().__init__(["outcome1","outcome2", "end"])
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
        print(bcolors.YELLOW_WARNING + f"Executing state MoveStartPos" + bcolors.ENDC)
        
        # print(self.ros_manager.get_latest_pose())
        
        home_pose = home_coords[ROBOT_ID]
            
        # goal_pose = np.array([rob_pose.x + 0, 
        #                 rob_pose.y + 0,
        #                 rob_pose.theta + 0])
        
        home_goal = [
            # (cam_odom.current_position['x'], cam_odom.current_position['y']),
            (home_pose[0], cam_odom.current_position['y']), # move along the x 
            (home_pose[0], home_pose[1]), # move along the y &  ensure theta
            (home_pose[0], home_pose[1], home_pose[2]), # move along the y &  ensure theta
        ]
        ######## START HOMING
        print(bcolors.BLUE_OK + f"{home_goal=}" + bcolors.ENDC)
        commands = decode_coords(home_goal, communicator.current_coords[0],communicator.current_coords[1])
        print(bcolors.BLUE_OK + f"{commands=}" + bcolors.ENDC)
        movetotheta(target_theta=315)
        print(bcolors.BLUE_OK + f"Moving Independantly from the swarms" + bcolors.ENDC)
        for command, target in commands:
            print(f'{command=}')
            if command == 'x':
                movealongx315(target)
            elif command == 'y':
                movealongy315(target)
            elif command == 'theta':
                movetotheta(target)
            else:
                pass
        print(bcolors.GREEN_OK + f"FINSIHED CENTERING>>>>>>>........." + bcolors.ENDC)
        ######### HOMING
        print(bcolors.YELLOW_WARNING + f"{communicator.header=}" + bcolors.ENDC)
        if communicator.header == "" or communicator.header == "COORDINATES": # starting
            return "outcome1"
        else:
            return "outcome2"

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
        super().__init__(["outcome1", "end"])
        # self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        print(bcolors.YELLOW_WARNING + f"Executing state AtStartPos" + bcolors.ENDC)
        ######### CHECKING OTHER ROBOT LOCATION
        # while True:
        #     print(f'{communicator.coords_dict=}')
        #     if len(communicator.coords_dict) == 1:
        #         break
        #     else:
        #         print(bcolors.YELLOW_WARNING + f"Waiting for other robot to pair. Only see {len(communicator.coords_dict)}robot" + bcolors.ENDC)
        #         time.sleep(1)
        # # print(bcolors.BLUE_OK + f"Waiting for all to reach the location" + bcolors.ENDC)
        # while True:
        #     print(bcolors.BLUE_OK + f"see {len(communicator.coords_dict)}/{2} robot" + bcolors.ENDC)
        #     print(f'{communicator.coords_dict=}')
        #     time.sleep(0.01)
        #     num_robot = len(communicator.coords_dict)
        #     arrvied_counter = 0 
        #     for robot in communicator.coords_dict.keys():
        #         if abs(communicator.coords_dict[robot][0] - home_coords[robot][0]) < (THRESHOLD_X_POSITION * 2) :
        #             num_robot_arrvied +=1
        #         if abs(communicator.coords_dict[robot][1] - home_coords[robot][1]) < (THRESHOLD_Y_POSITION * 2) :
        #             num_robot_arrvied +=1
        #     if arrvied_counter == num_robot*2:
        #         break
        #     time.sleep(0.2)
        # print(bcolors.BLUE_OK + f"All robot have arrived at home " + bcolors.ENDC)
        # pprint(communicator.coords_dict)
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
            
        super().__init__(["outcome1","loop", "end"])

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
        print(bcolors.YELLOW_WARNING + f"Executing state SeekObject" + bcolors.ENDC)
        
        # TODO Sub to the object detection
        # TODO Pub twist msg to keep rotating. 
        # print(f"{self.cv_class.cylinder_detection=}")
        
        while 1:
            rob_pose = [cam_odom.current_position["x"], cam_odom.current_position["y"], cam_odom.current_position["theta"]]
        #     # print(f"{self.cv_class.cylinder_detection=}")
            if communicator.header == "PATH":  # received the path from master
                print(bcolors.YELLOW_WARNING + f"RECEIVED PATH HEADER" + bcolors.ENDC)
                return "outcome2"  # IdleSlave
            
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
        #         self.ros_manager.publish_cmd_vel(stop_msg)
        #         print(f"{detection_info=}")
        #         print(f"Found an object stopping.. Validating detection data...")
                
        #         # Since we've validated, no need for the extra checks here,
        #         # but you might want to log the valid data explicitly if needed.
        #         print(f"Valid detection data received: {detection_info[:3]}")
                
        #         break # Exit the loop as valid data is found

        #     # else:
        #         # If validation failed (either not a list, too short, or contained non-finite values in the first 3)
        #         # print(f"Invalid or incomplete detection data: {detection_info}. Sending rotating twist msg.")
        #             # twist_msg = Twist()
        #             # if int(ROBOT_ID) % 1: 
        #             #     twist_msg.angular.z = -0.6
        #             # else:
        #             #     twist_msg.angular.z = 0.6
                        
        #             # self.ros_manager.publish_cmd_vel(twist_msg)
        #         # time.sleep(0.1)  # around 10hz
        # if isinstance(detection_info, list) and len(detection_info) >= 3:
        #     # Check if all of the first three elements are finite numbers
        #     twist_msg = Twist()
        #     self.ros_manager.publish_cmd_vel(twist_msg)
        #     if all(math.isfinite(val) for val in detection_info[:3]):
        #         print(bcolors.BLUE_OK + f"Found a Detection at {detection_info=}" + bcolors.ENDC) 
        
        #         blackboard["object_info"] = detection_info
        #         object_d     = detection_info[0]
        #         object_w     = detection_info[1]
        #         object_theta = detection_info[2]
                
        #         # communicator.object_coords = [0.0, 0.0] # assume
        #         # communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]] #
                
        #         object_pose = [ # robot pose  + the distance xy of the object relative to robot pose
        #             rob_pose[0] + (object_d)*math.cos(math.radians(rob_pose[2] + object_theta)) * 0.8,
        #             rob_pose[1] + (object_d)*math.sin(math.radians(rob_pose[2] + object_theta)) * 0.8
        #         ]
        #         # Real Results
        #         # communicator.object_coords = object_pose
        #         # Testing Results
            communicator.object_coords = [0.5,0.5]
            print(bcolors.BLUE_OK + f"Object Calculated: \n\trobot_pose {rob_pose}\n\tobject_pose {communicator.object_coords }" + bcolors.ENDC)
            communicator.obstacle_coords = [] # assume
            communicator.object_detected()
            break
            
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

        
    def execute(self, blackboard: Blackboard) -> str:
        """
        Executes the logic for the Foo state.
        Args: blackboard (Blackboard): The shared data structure for states.
        Returns: str: The outcome of the execution, which can be "outcome1" or "outcome2".
        Raises: Exception: May raise exceptions related to state execution.
        """
        print(bcolors.YELLOW_WARNING + f"Executing state PathFollowing" + bcolors.ENDC)
        # pprint(communicator.command)
        # pprint(communicator.orientation)
        """ 
        >>> [(0.51, 0.45416666666666666), (0.8, 0.45416666666666666), (0.8, 0.5), (0.8, 0.5)]
        >>> 3.14"""
        _tmp = list(communicator.command[-1])
        _tmp.append(communicator.orientation)
        commands = communicator.command
        commands[-1] = tuple(_tmp)
        pprint(commands)
        commands = decode_coords(commands, communicator.current_coords[0], communicator.current_coords[1])
        pprint(commands)
        movetotheta(target_theta=315)
        print(bcolors.BLUE_OK + f"Moving to the planned commands" + bcolors.ENDC)
        for command, target in commands:
            print(f'{command=}')
            if command == 'x':
                movealongx315(target)
            elif command == 'y':
                movealongy315(target)
            elif command == 'theta':
                movetotheta(target)
            else:
                pass
        ros_manager.publish_cmd_vel(stop_msg) # STOP MSGk
        ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        time.sleep(1)
        communicator.cleanup() # To clear waypoints and orientation
        return "outcome1" # go ARRIVED POS
    
# ARRVIED 
# COLLECTIVE TRANSPORT 
#   # need to tune the speed of each robot
# MoveStartPos
# New stuff here
class AtObject(State):
    """
    Both robots have reached the object and acknowledge gripping it.
    """
    def __init__(self) -> None:
        super().__init__(["outcome1", "end"])

    def execute(self, blackboard: Blackboard) -> str:
        print(bcolors.YELLOW_WARNING + "Executing AtObject state: acknowledging object grip" + bcolors.ENDC)
        # broadcast that this robot has gripped the object
        communicator.broadcast("AT_OBJECT", communicator.object_coords)
        # wait for peer to acknowledge grip
        start = time.time()
        while time.time() - start < MAX_WAIT:
            if getattr(communicator, "header", None) == "AT_OBJECT":
                break
            time.sleep(0.1)
        else:
            self.get_logger().warn("Peer did not acknowledge in time")
            return "end"            
        print(bcolors.GREEN_OK + "Both robots have acknowledged object grip" + bcolors.ENDC)
        return "outcome1"

class ClickingObject(State):
    """
    Both robots have reached the object and acknowledge gripping it.
    """
    def __init__(self) -> None:
        super().__init__(["outcome1", "end"])

    def execute(self, blackboard: Blackboard) -> str:
        print(bcolors.YELLOW_WARNING + "Executing ClickingObject state: Clicking into the object " + bcolors.ENDC)
        # X-movement
        yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Moving in X-Direction" + bcolors.ENDC)
        start_time = time.time()
        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.linear.y = 0.0
        while (time.time() - start_time) < 2:
            self.ros_manager.publish_cmd_vel(twist_msg)
        yasmin.YASMIN_LOG_INFO(bcolors.BLUE_OK + f"Stopping" + bcolors.ENDC)
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        self.ros_manager.publish_cmd_vel(stop_msg) # STOP MSG
        time.sleep(1)
        return "outcome1"


class DiagonalTransport(State):
    """
    Executes collective diagonal transport using jacobian-based PI control.
    """

    def __init__(self) -> None:
        super().__init__(["outcome1", "end"])
        # === Diagonal collective-transport setup ===
        self.r       = 0.05
        self.L       = 0.2
        phi1         = np.deg2rad([45, 135, -135, -45])
        phi2         = phi1 + np.pi
        x1           = np.array([ self.L, -self.L, -self.L,  self.L])
        y1           = np.array([ self.L,  self.L, -self.L, -self.L])
        x2, y2       = -x1, -y1

        # build Jacobians
        def build_jacobian(phi, x, y):
            A = np.zeros((4,3))
            for i in range(4):
                A[i] = [
                    np.cos(phi[i]),
                    np.sin(phi[i]),
                    x[i]*np.sin(phi[i]) - y[i]*np.cos(phi[i])
                ]
            return A

        A1 = build_jacobian(phi1, x1, y1)
        A2 = build_jacobian(phi2, x2, y2)
        self.J1 = np.linalg.pinv(A1)
        self.J2 = np.linalg.pinv(A2)

        # PI controller gains
        K           = 1.0    # TODO: measure this
        self.Kp     = 8.0/K
        Ti          = 0.25
        self.Ki     = self.Kp / Ti

        # references
        self.phi_ref = -np.pi/4   # -45°
        self.v_diag  = 0.35
        self.dt      = 0.01
        self.e_int   = 0.0


    def execute(self, blackboard: Blackboard) -> str:
        print(bcolors.YELLOW_WARNING + "Executing DiagonalTransport state (pure diagonal)" + bcolors.ENDC)

        # --- PI yaw controller  ---
        theta = np.deg2rad(cam_odom.current_position["theta"])
        e = (self.phi_ref - theta + np.pi) % (2 * np.pi) - np.pi
        self.e_int += e * self.dt
        U = self.Kp * e + self.Ki * self.e_int
        wz = U

        # Desired chassis velocities (global –45°), no rotation
        vx = self.v_diag * np.cos(self.phi_ref)
        vy = self.v_diag * np.sin(self.phi_ref)

        # Build and publish a Twist → driver will spin only the two 315° wheels
        twist = Twist()
        twist.linear.x, twist.linear.y = globaltorobottf(vx, vy, cam_odom.current_position["theta"])
        twist.angular.z = 0.0
        ros_manager.publish_cmd_vel(twist)
        time.sleep(2.5)
        twist = Twist()
        ros_manager.publish_cmd_vel(twist)
        time.sleep(0.1)
        ros_manager.publish_cmd_vel(twist)
        

        return "outcome1"
    
# class DiagonalTransport(State):          Uses the move along functions
#     """
#     Executes collective transports in global x and y.
#     """
#     def __init__(self) -> None:
#         super().__init__(["outcome1", "end"])

#     def execute(self, blackboard: Blackboard) -> str:
#         print(bcolors.YELLOW_WARNING + "Executing DiagonalTransportAxes state (axis-based diagonal)" + bcolors.ENDC)
#         # Retrieve global object coordinates
#         target_x, target_y = communicator.object_coords[0], communicator.object_coords[1]
#         # First move along global X using the 315° wheel
#         movealongx315(target_x)
#         # Then move along global Y using the 315° wheel
#         movealongy315(target_y)
#         return "outcome1"



              
def main(args=None):
    # Set ROS 2 loggers
    set_ros_loggers()
    
    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state("Init", Init(), transitions={"outcome1": "MoveStartPos","end": "outcome4"})
    sm.add_state("MoveStartPos", MoveStartPos(), transitions={"outcome1": "AtStartPos","end": "outcome4"})
    sm.add_state("AtStartPos", AtStartPos(), transitions={"outcome1": "SeekObject","end": "outcome4"})
    
    sm.add_state("SeekObject", SeekObject(), transitions={"outcome1": "PathFollowing","loop": "SeekObject","end": "outcome4"})
    # tell arrvied via comms 
    # click in place 
    # move 45 degrees towards the center 
    
    # sm.add_state("IdleSlave", IdleSlave(), transitions={"outcome1": "BAR","end": "outcome4"}) # TaskMaster
    # sm.add_state("FoundObject", FoundObject(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    # sm.add_state("PathPlanning", PathPlanning(), transitions={"outcome1": "BAR","end": "outcome4"}) #Slave
    
    sm.add_state("PathFollowing", PathFollowing(), transitions={"outcome1": "AtObject","end": "outcome4"})
    
    sm.add_state("AtObject", AtObject(), transitions={"outcome1": "ClickingObject","end": "outcome4"})
    sm.add_state("ClickingObject", ClickingObject(), transitions={"outcome1": "DiagonalTransport","end": "outcome4"})
    sm.add_state("DiagonalTransport", DiagonalTransport(), transitions={"outcome1": "MoveStartPos", "end":"outcome4"})


    try:
        print("State Machine starting")
        outcome = sm()
        print(outcome)
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