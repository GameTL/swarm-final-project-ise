import time
import random
import numpy as np
import math
from rich.pretty import pprint
import matplotlib.pyplot as plt

MAX_SPEED = 6.28
MAP_WIDTH = 250
MAP_HEIGHT = 220
RESOLUTION = 0.1  # 10 cm per grid cell
WHEEL_RADIUS = 0.033
# WHEEL_RADIUS = 0.033
WHEEL_BASE = 0.18
GPS_DEVICE_NAME = "gps"

## for testing pid need to be moved
# Constants and Gains
theta_integral = 0.0  # Integral of the heading error
distance_integral = 0.0
# dt = 0.032  # Time step (adjust according to your simulation)
# dt = 0.1  # Time step (adjust according to your simulation)
# L = 0.16   # Wheelbase (distance between the wheels)
v = 1.0   # Linear velocity (adjust as needed)

plotting = "position" 
# plotting = "position_err_pid" 
# plotting = "angle_err_pid" 

class Driver:
    def __init__(self, robot):
        self.timestep = int(robot.getBasicTimeStep())
        self.robot = robot
        self.robot_name = robot.getName()
        self.robot_position = {
            "x": np.float64(0.0),
            "y": np.float64(0.0),
            "theta": np.float64(0.0),
            "imu_theta": np.float64(0.0),
            "imu_x": np.float64(0.0),
            "imu_y": np.float64(0.0),
            "imu_v_x": np.float64(0.0),
            "imu_v_y": np.float64(0.0),
        }
        self.alive = True

        # Init motors
        self.left_motor = robot.getDevice("left_wheel_motor")
        self.right_motor = robot.getDevice("right_wheel_motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

        # Encoder
        self.left_encoder = self.robot.getDevice("left_wheel_sensor")
        self.right_encoder = self.robot.getDevice("right_wheel_sensor")
        self.left_encoder.enable(self.timestep)
        self.right_encoder.enable(self.timestep)
        self.prev_left_encoder = np.float64(self.left_encoder.getValue())
        self.prev_right_encoder = np.float64(self.right_encoder.getValue())
        self.current_x = 0.0
        
        # IMU 
        self.accelerometer = self.robot.getDevice("accelerometer")
        self.accelerometer.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        self.imu_prev_w_z = np.float64(self.gyro.getValues()[2])
        self.imu_prev_a_x = np.float64(self.accelerometer.getValues()[0])
        self.imu_prev_a_y = np.float64(self.accelerometer.getValues()[1])
        self.time_prev = 0
        
        # PID config 
        if self.robot_name == "TurtleBot3Burger_3":
            self.sorted_waypoints = [(-0.3,-1.1),(-0.1,-0.65),(-0.1,-0.3),(0,0),(1,1)]
        else:
            self.sorted_waypoints = []
        
        self.waypoint_threshold = 0.02
        self.dt = 0.032  # Time step (adjust according to your simulation)
        self.v_linear = 2
        self.Kp_linear = 10
        self.Ki_linear = 0.1
        
        self.Kp_angular = 20
        self.Ki_angular = 0.1
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        
        
        # Enable sensory devices
        self.gps = robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)
        # Lidar
        self.lidar = self.robot.getDevice("lidar_sensor")
        self.lidar.enable(self.timestep)
        self.map = np.zeros((MAP_HEIGHT, MAP_WIDTH), dtype=np.float64)
        
        self.plot_data = []
        


        
    def get_pretty_position(self):
        # return f"[helper]({self.robot_name}) Robot X position:{self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}    Accelerometer: {self.accelerometer.getValues()}    Gyro: {self.gyro.getValues()}"
        return f"[helper]({self.robot_name}) Robot X position:{self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f} ||| X+IMU_THETA position:{self.robot_position['imu_x']:6.3f}    Y+IMU_THETA position: {self.robot_position['imu_y']:6.3f}    IMU Theta position: {self.robot_position['imu_theta']:6.3f}"

    # motion
    def move_forward(self, coeff=1):
        self.left_motor.setVelocity(coeff*MAX_SPEED)
        self.right_motor.setVelocity(coeff*MAX_SPEED)

    def move_backward(self):
        self.left_motor.setVelocity(-MAX_SPEED)
        self.right_motor.setVelocity(-MAX_SPEED)

    # Positive Theta
    def anti_clockwise_spin(self, coeff=0.5):
        self.left_motor.setVelocity(-coeff * MAX_SPEED)
        self.right_motor.setVelocity(coeff * MAX_SPEED)

    # Negetive Theta
    def clockwise_spin(self, coeff=0.5):
        self.left_motor.setVelocity(coeff * MAX_SPEED)
        self.right_motor.setVelocity(-coeff * MAX_SPEED)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def update_plot_position(self):
        self.ax.clear()
        self.ax.set_title(f'Robot Position in Real-Time {self.robot_name}')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        self.ax.plot(self.x_positions, self.y_positions, 'bo-',markersize=1)
        for waypoint in self.sorted_waypoints:
            self.ax.plot(waypoint[0], waypoint[1], 'ro', markersize=1)  # Red color for waypoints
        plt.draw()
        plt.pause(0.001)
        
    def update_plot_position_err_pid(self, time, distance_err):
        self.plot_data.append((time, distance_err))
        self.ax.clear()
        self.ax.set_title(f'Robot Position_PID in Real-Time {self.robot_name}')
        self.ax.set_ylabel('distance_err')
        self.ax.set_xlabel('Time')
        # self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(bottom=0)
        for data in self.plot_data:
            self.ax.plot(data[0], data[1], 'ro', markersize=1)  
        # self.ax.plot(self.plot_data[0], distance_err, 'bo-',markersize=1)
        plt.draw()
        plt.pause(0.01)
    
    def update_plot_angle_err_pid(self, time, angle_err):
        self.plot_data.append((time, angle_err))
        self.ax.clear()
        self.ax.set_title(f'Robot Position_PID in Real-Time {self.robot_name}')
        self.ax.set_ylabel('angle_err')
        self.ax.set_xlabel('Time')

        for data in self.plot_data:
            self.ax.plot(data[0], data[1], 'ro', markersize=1)  
        # self.ax.plot(self.plot_data[0], distance_err, 'bo-',markersize=1)
        plt.draw()
        plt.pause(0.01)
    
    def pi_controller(self, v_linear, current_vector, target_vector):
        """
        PI Controller for differential-drive robot to follow waypoints.
        """
        # 1. Compute Errors
        dx = target_vector[0] - current_vector[0]
        dy = target_vector[1] - current_vector[1]
        distance_error = math.hypot(dx, dy)
        if plotting == "position_err_pid":
            self.update_plot_position_err_pid(self.robot.getTime(), distance_error) 
        theta_desired = math.atan2(dy, dx)
        theta_error = theta_desired - current_vector[2]
        # Normalize theta_error to [-pi, pi]
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        if plotting == "angle_err_pid":
            self.update_plot_angle_err_pid(self.robot.getTime(), theta_error) 

        # 2. Update Integral Terms for Linear and Angular Velocities
        self.linear_integral += distance_error * self.dt
        self.angular_integral += theta_error * self.dt

        # 3. Compute Linear Velocity (v) using PI Controller
        v_linear = self.Kp_linear * distance_error + self.Ki_linear * self.linear_integral

        # 4. Compute Angular Velocity (u) using PI Controller
        v_pid = self.Kp_angular * theta_error + self.Ki_angular * self.angular_integral

        # 5. Compute Left and Right Wheel Velocities for Differential Drive
        v_left = v_linear - (v_pid * WHEEL_BASE/ 2)
        v_right = v_linear + (v_pid * WHEEL_BASE/ 2)

        # 6. Limit Motor Speeds
        v_left = max(min(v_left, MAX_SPEED), -MAX_SPEED)
        v_right = max(min(v_right, MAX_SPEED), -MAX_SPEED)

        return [v_left, v_right]
    
    # def pi_theta_Controller(self, theta_target, theta_current) -> float:
    #     """ 
    #     input: the current state & target state 
    #     output: velocity_pi 
    #     use V_vector = V_linear + V_PI"""
    #     pass
    # def test_pid_theta_controller():
    #     P_ = []
    #     self.robot_position["imu_theta"] # get the theta of the robot
    #     self.robot_position["y"] # get the theta of the robot
    #     self.robot_position["x"] # get the theta of the robot
        
    def pid_path_follow(self):
        #$ Initialization code here
        self.x_positions = []
        self.y_positions = []
        self.fig, self.ax = plt.subplots()
        self.ax.set_title(f'Robot Position in Real-Time {self.robot_name}')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(-2.5, 2.5)
        #$ Plot waypoints as red dots
        print(self.sorted_waypoints)
        for waypoint in self.sorted_waypoints:
            self.ax.plot(waypoint[0], waypoint[1], 'ro', markersize=2)  # Red color for waypoints
        plt.ion()  #$ Turn on interactive mode for live updates


        # Set motors to velocity control mode
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        # x_target = self.robot_position["x"]
        while self.robot.step(self.timestep) != -1:
            position_state = [
                self.robot_position["x"], 
                self.robot_position["y"], 
                self.robot_position["theta"]
                ]
            
            # # # Define target position
            # x_target = 0.0
            # y_target = -1.0
                        
            # Define target position based on current waypoint using pop
            if self.sorted_waypoints:
                target_vector = self.sorted_waypoints[0]
            else:
                # All waypoints have been reached; stop the robot
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                print("All waypoints reached. Stopping the robot.")
                self.alive = False
                self.stop()
                quit()
                break
            
            # Compute distance to the waypoint
            distance = math.hypot(target_vector[0] - position_state[0], target_vector[1] - position_state[1])
            
            # Check if waypoint is reached
            if distance < self.waypoint_threshold:
                print(f"Waypoint ({target_vector[0]}, {target_vector[1]}) reached.")
                self.sorted_waypoints.pop(0)  # Remove the reached waypoint
                continue
            
            # PID
            [v_left, v_right] = self.pi_controller(self.v_linear,position_state, target_vector=target_vector)
            
            # Set motor velocities
            self.left_motor.setVelocity(v_left)
            self.right_motor.setVelocity(v_right)
            print(self.get_pretty_position())
            #$ Place these updates in the main loop where needed:
            self.x_positions.append(position_state[0])
            self.y_positions.append(position_state[1])
            
            ##************************************************* 
            ##* UNCOMMENT THE FOLLOWING LINE TO UPDATE THE PLOT
            ##*************************************************  
            if plotting == "position":
                self.update_plot_position() 
            ##************************************************* 
            ##************************************************* 
            ##************************************************* 

                
    def simple_follow_path(self, path):
        # Constants
        ANGLE_THRESHOLD = 0.01  # radians
        DISTANCE_THRESHOLD = 0.01  # meters

        # Get the list of waypoints from the path
        print(f'{path=}')
        work_path = list(path.values())
        print("Original path:", work_path)

        # Function to sample waypoints
        def sample_waypoints(waypoints, sample_rate):
            if len(waypoints) <= 2:
                return waypoints  # Not enough waypoints to sample
            # Keep the first point
            sampled_waypoints = [waypoints[0]]
            # Sample the in-between points
            sampled_waypoints += waypoints[1:-1:sample_rate]
            # Keep the last point
            sampled_waypoints.append(waypoints[-1])
            return sampled_waypoints

        # Adjust the sample rate as needed
        sample_rate = 10  # Keep every 5th waypoint
        work_path = sample_waypoints(work_path, sample_rate)
        print("Sampled path:", work_path)

        # Initialize state variables
        state = "ROTATING_TO_WAYPOINT"
        target_x, target_y = work_path.pop(0)  # Start with the first waypoint
        # self.localisation.update_odometry()

        while self.robot.step(self.timestep) != -1:
            # Update odometry
            # self.localisation.update_odometry()
            current_x = self.robot_position["x"]
            current_y = self.robot_position["y"]
            current_theta = self.robot_position["theta"]

            # Calculate distance and angle to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance_to_target = np.hypot(dx, dy)
            angle_to_target = np.arctan2(dy, dx)
            angle_error = angle_to_target - current_theta
            angle_error = (angle_error + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-π, π]

            if state == "ROTATING_TO_WAYPOINT":
                if abs(angle_error) > ANGLE_THRESHOLD:
                    # Set motor speeds to rotate
                    if angle_error < 0:
                        self.clockwise_spin()
                    else:
                        self.anti_clockwise_spin()
                else:
                    self.stop()
                    state = "MOVING_TO_WAYPOINT"  # Transition to next state

            elif state == "MOVING_TO_WAYPOINT":
                if distance_to_target > DISTANCE_THRESHOLD:
                    # Move forward
                    self.move_forward()
                else:
                    self.stop()
                    print(f"[path_following]({self.robot.getName()}) Reached waypoint: ({target_x}, {target_y})")
                    # Check if there are more waypoints
                    if work_path:
                        target_x, target_y = work_path.pop(0)
                        state = "ROTATING_TO_WAYPOINT"  # Start over for next waypoint
                    else:
                        print(f"[path_following]({self.robot.getName()}) Robot X position:{self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")
                        print("Path following complete.")
                        break  # Exit the loop when done

            # Include any additional behaviors or idle actions here

        # Stop the robot after exiting the loop
        self.stop()

    def move_along_polynomial(self, degree=6):
        def evaluate_polynomial(x):
            """Evaluate the polynomial at a given x."""
            coeff = [random.uniform(-1, 1) for _ in range(degree + 1)]
            y = sum(coef * (x ** i) for i, coef in enumerate(coeff))
            return y
        
        """Make the robot follow the polynomial path."""
        # Increment x position
        self.current_x += 0.1  # Adjust step size as needed
        target_y = evaluate_polynomial(self.current_x)

        # Assume you have a GPS-like function to get the current position
        current_position = (self.gps.getValues()[0], self.gps.getValues()[2])
        target_position = (self.current_x, target_y)

        # Calculate direction to target
        direction_vector = (target_position[0] - current_position[0], 
                            target_position[1] - current_position[1])
        
        # Use direction to set wheel velocities
        if direction_vector[0] > 0:
            self.left_motor.setVelocity(MAX_SPEED * 0.8)
            self.right_motor.setVelocity(MAX_SPEED)
        elif direction_vector[0] < 0:
            self.left_motor.setVelocity(MAX_SPEED)
            self.right_motor.setVelocity(MAX_SPEED * 0.8)
        else:
            self.move_forward()

        # Adjust speed based on how far the robot is from the target
        distance = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        if distance < 0.1:  # If close to target, stop or slow down
            self.stop()

        # Print the polynomial equation for debugging
        # print(f"Polynomial: {' + '.join(f'{coef:.2f}*x^{i}' for i, coef in enumerate(self.coefficients))}")
        # print(f"Moving to target: ({self.current_x:.2f}, {target_y:.2f})")
    
    def check_encoder_not_null_and_init(self):
        # Wait until valid encoder values are available
        print(f"[localisation]({self.robot.getName()}) Waiting for encoder != nan")
        while math.isnan(self.prev_left_encoder) or math.isnan(self.prev_right_encoder) or math.isnan(self.imu_prev_w_z) or math.isnan(self.imu_prev_a_x) or math.isnan(self.imu_prev_a_y) :
            self.robot.step(self.timestep)  # Step the simulation until we get valid readings
            self.prev_left_encoder = self.left_encoder.getValue()
            self.prev_right_encoder = self.right_encoder.getValue()
            self.imu_prev_w_z = np.float64(self.gyro.getValues()[2])
            self.imu_prev_a_x = np.float64(self.accelerometer.getValues()[0])
            self.imu_prev_a_y = np.float64(self.accelerometer.getValues()[1])

        print(
            f"[localisation]({self.robot.getName()}) Valid Initial Left Encoder: {self.prev_left_encoder}, Valid Initial Right Encoder: {self.prev_right_encoder}"
        )
        # when encoder is live then trigger the set
        self.robot_position["x"], self.robot_position["y"], current_z = (
            self.gps.getValues()
        )  # init the coords even when using wheel odom
        self.robot_position["imu_x"], self.robot_position["imu_y"], current_z = (
            self.gps.getValues()
        ) 
        print(f"[localisaton]({self.robot.getName()}) INIT WITH GPS AT: Robot X position: {self.robot_position['x']:6.3f}    Robot Y position: {self.robot_position['y']:6.3f}    Robot Theta position: {self.robot_position['theta']:6.3f}")
        # pprint(_object=self.robot_position)
        return True
    
    def get_position_gps(self):
        """ return (x,y, theta_z) """
        return self.gps.getValues()
    
    # Writing to the robot_position
    def update_odometry_o1(self):
        # Get current encoder values as float64
        temp_left = np.float64(self.left_encoder.getValue())
        temp_right = np.float64(self.right_encoder.getValue())
        
        # Calculate change in encoder values (radians)
        delta_left = temp_left - self.prev_left_encoder
        delta_right = temp_right - self.prev_right_encoder
        
        # Update previous encoder values
        self.prev_left_encoder = temp_left
        self.prev_right_encoder = temp_right
        
        # Compute delta_center and delta_theta using float64
        delta_center = (delta_left + delta_right) / 2.0 * WHEEL_RADIUS
        delta_theta = (delta_right - delta_left) * WHEEL_RADIUS / WHEEL_BASE
        
        # Update orientation first
        self.robot_position["theta"] += delta_theta
        
        # Normalize theta to be within [-pi, pi]
        self.robot_position["theta"] = (self.robot_position["theta"] + np.pi) % (2 * np.pi) - np.pi
        
        # Calculate average orientation
        avg_theta = self.robot_position["theta"] - delta_theta / 2.0
        # avg_theta = self.robot_position["imu_theta"] 
        
        # Update position using the average orientation
        self.robot_position["x"] += delta_center * np.cos(avg_theta)
        self.robot_position["y"] += delta_center * np.sin(avg_theta)
        self.robot_position["imu_x"] += delta_center * np.cos(self.robot_position["imu_theta"])
        self.robot_position["imu_y"] += delta_center * np.sin(self.robot_position["imu_theta"])
        
        #######################3 IMU
        current_time = self.robot.getTime()
        self.imu_prev_a_x = np.float64(self.accelerometer.getValues()[0])
        self.imu_prev_a_y = np.float64(self.accelerometer.getValues()[1])
        dt_i = current_time - self.time_prev
        
        # Read IMU data
        a_x_body = self.accelerometer.getValues()[0]
        a_y_body = self.accelerometer.getValues()[1]

        # Transform accelerations to global frame
        a_x_global = a_x_body * np.cos(self.robot_position["imu_theta"]) - a_y_body * np.sin(self.robot_position["imu_theta"])
        a_y_global = a_x_body * np.sin(self.robot_position["imu_theta"]) + a_y_body * np.cos(self.robot_position["imu_theta"])

        # Update velocity from acceleration
        self.robot_position["imu_v_x"] += a_x_global * dt_i
        self.robot_position["imu_v_y"] += a_y_global * dt_i

        # Update positions from velocity
        # self.robot_position["imu_x"]  += self.robot_position["imu_v_x"] * dt_i
        # self.robot_position["imu_y"]  += self.robot_position["imu_v_y"] * dt_i
        self.robot_position["imu_theta"] += (self.gyro.getValues()[2]) * (dt_i) * 2
        
        self.time_prev = current_time
    
    def run_odometry_service(self):
        self.robot_position["x"], self.robot_position["y"], current_z = self.get_position_gps()
        while self.alive:
            # print(self.get_pretty_position())
            # self.robot_position["x"], self.robot_position["y"], current_z = self.get_position_gps()
            self.time_prev = self.robot.getTime()
            self.update_odometry_o1()
            time.sleep(0.01)

    # Calulate the Lidar infomation
    def update_map(self):
        map_center_x = MAP_WIDTH // 2
        map_center_y = MAP_HEIGHT // 2

        for i, distance in enumerate(self.lidar.getRangeImage()):
            if distance == float("inf") or distance < 0.0 or distance > 10.0:
                continue

            angle = self.robot_position["theta"] + i * (2 * math.pi / len(self.lidar.getRangeImage()))

            obstacle_x = self.robot_position["x"] + distance * math.cos(angle)
            obstacle_y = self.robot_position["y"] + distance * math.sin(angle)

            grid_x = int(obstacle_x / RESOLUTION) + map_center_x
            grid_y = int(obstacle_y / RESOLUTION) + map_center_y

            # print(f"Obstacle Global Position: ({obstacle_x}, {obstacle_y}) -> Grid ({grid_x}, {grid_y})")

            if 0 <= grid_x < MAP_WIDTH and 0 <= grid_y < MAP_HEIGHT:
                # ? not global?????
                self.map[grid_x][grid_y] = 1  # Mark cell as occupied
                # print(f"Obstacle marked at ({grid_x}, {grid_y})")
            else:
                pass
                # print(f"Obstacle out of bounds: ({grid_x}, {grid_y})")
                
                

    def print_map(self):
        print("SLAM Map:")
        for i in range(MAP_HEIGHT):  # Loop over rows (Y-axis)
            for j in range(MAP_WIDTH):  # Loop over columns (X-axis)
                if self.map[i][j] == 1:
                    print("#", end="")  # Obstacle
                else:
                    print(".", end="")
            print()