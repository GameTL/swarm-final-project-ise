import random
import numpy as np
from rich.pretty import pprint

MAX_SPEED = 2
GPS_DEVICE_NAME = "gps"
class Driver:
    def __init__(self, robot, robot_position, localisation):
        self.timestep = int(robot.getBasicTimeStep())
        self.robot_position = robot_position
        self.current_x = 0  # Start at x = 0

        # Init motors
        self.leftMotor = robot.getDevice("left_wheel_motor")
        self.rightMotor = robot.getDevice("right_wheel_motor")
        self.leftMotor.setPosition(float("inf"))
        self.rightMotor.setPosition(float("inf"))
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        # Enable sensory devices
        self.gps = robot.getDevice(GPS_DEVICE_NAME)
        self.gps.enable(self.timestep)
        self.localisation = localisation

    # motion
    def move_forward(self, coeff=1):
        self.leftMotor.setVelocity(coeff*MAX_SPEED)
        self.rightMotor.setVelocity(coeff*MAX_SPEED)

    def move_backward(self):
        self.leftMotor.setVelocity(-MAX_SPEED)
        self.rightMotor.setVelocity(-MAX_SPEED)

    # Positive Theta
    def anti_clockwise_spin(self):
        self.leftMotor.setVelocity(-0.7 * MAX_SPEED)
        self.rightMotor.setVelocity(0.7 * MAX_SPEED)

    # Negetive Theta
    def clockwise_spin(self):
        self.leftMotor.setVelocity(0.7 * MAX_SPEED)
        self.rightMotor.setVelocity(-0.7 * MAX_SPEED)

    def stop(self):
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
    
    # def simple_follow_path(self, path):
        # # TODO NOT DONE
        # THETA_FACING_X = 0.0
        # THETA_FACING_Y = 90.0
        
        # self.localisation.update_odometry()
        # work_path = list(path.values())
        # pprint(work_path)
        # x, y  = self.robot_position["x"], self.robot_position["y"]
        # [step_x, step_y] = work_path.pop(0)
        # # print([step_x, step_y])
        # [err_x, err_y] = [step_x - x , step_y - y]
        # print([err_x, err_y])
        # x_prev, y_prev = self.robot_position["x"], self.robot_position["y"]
        
        # while True:
            
        #     # rotate untill theta is 0 facing x 
        #     err_theta_from_x = THETA_FACING_X - self.robot_position["theta"]
        #     if err_theta_from_x < -0.1:
        #         self.anti_clockwise_spin()
        #     elif err_theta_from_x > 0.1:
        #         self.clock_spin()
        #     else:
        #         self.move_forward()
            
        #     # elif err_theta_from_x > 0.1:
                
                
        #     # driving until the current x doesnt change
    def simple_follow_path(self, path):
        # Constants for desired orientations (in radians)
        THETA_FACING_X = 0.0  # Facing along positive x-axis
        THETA_FACING_Y = np.pi / 2  # Facing along positive y-axis # ***
        ANGLE_THRESHOLD = 0.05  # Threshold for angle comparison (radians)
        DISTANCE_THRESHOLD = 0.01  # Threshold for position comparison (meters)
        
        work_path = list(path.values())
        print("Path to follow:", work_path)
        
        while work_path:
            # Get the next waypoint from the path
            target_x, target_y = work_path.pop(0)
            print(f"Next waypoint: ({target_x}, {target_y})")
            
            # Move along the x-axis to target_x
            # First, orient the robot to face along the x-axis
            while True:
                self.localisation.update_odometry()
                current_theta = self.localisation.robot_position["theta"]
                err_theta = THETA_FACING_X - current_theta
                err_theta = (err_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
                
                if abs(err_theta) > ANGLE_THRESHOLD:
                    if err_theta < 0:
                        self.anti_clockwise_spin()
                    else:
                        self.clock_spin()
                else:
                    self.stop_motors()  # Stop rotation when aligned
                    break  # Exit the rotation loop
            
            # Move forward along x-axis until x-coordinate matches target_x
            while True:
                self.localisation.update_odometry()
                current_x = self.localisation.robot_position["x"]
                err_x = target_x - current_x
                
                if abs(err_x) > DISTANCE_THRESHOLD:
                    self.move_forward()
                else:
                    self.stop_motors()
                    break  # Reached target_x
            
            # Now orient the robot to face along the y-axis
            while True:
                self.localisation.update_odometry()
                current_theta = self.localisation.robot_position["theta"]
                err_theta = THETA_FACING_Y - current_theta
                err_theta = (err_theta + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]
                
                if abs(err_theta) > ANGLE_THRESHOLD:
                    if err_theta < 0:
                        self.anti_clockwise_spin()
                    else:
                        self.clock_spin()
                else:
                    self.stop_motors()
                    break  # Exit the rotation loop
            
            # Move forward along y-axis until y-coordinate matches target_y
            while True:
                self.localisation.update_odometry()
                current_y = self.localisation.robot_position["y"]
                err_y = target_y - current_y
                
                if abs(err_y) > DISTANCE_THRESHOLD:
                    self.move_forward()
                else:
                    self.stop_motors()
                    break  # Reached target_y
            
            # Proceed to the next waypoint in the path
            print(f"Reached waypoint: ({target_x}, {target_y})")
        
        # Stop the robot after reaching the final waypoint
        self.stop_motors()
        print("Path following complete.")

        
        
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
            self.leftMotor.setVelocity(MAX_SPEED * 0.8)
            self.rightMotor.setVelocity(MAX_SPEED)
        elif direction_vector[0] < 0:
            self.leftMotor.setVelocity(MAX_SPEED)
            self.rightMotor.setVelocity(MAX_SPEED * 0.8)
        else:
            self.move_forward()

        # Adjust speed based on how far the robot is from the target
        distance = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        if distance < 0.1:  # If close to target, stop or slow down
            self.stop()

        # Print the polynomial equation for debugging
        # print(f"Polynomial: {' + '.join(f'{coef:.2f}*x^{i}' for i, coef in enumerate(self.coefficients))}")
        # print(f"Moving to target: ({self.current_x:.2f}, {target_y:.2f})")
        
        
""" 

    [-0.29, 1.2],
│   [-0.28, 1.2],
│   [-0.27, 1.2],
│   [-0.26, 1.2],
│   [-0.25, 1.2],
│   [-0.24, 1.2],
│   [-0.23, 1.2],
│   [-0.22, 1.2],
│   [-0.21, 1.2],
│   [-0.2, 1.2],
│   [-0.19, 1.2],
│   [-0.18, 1.2],
│   [-0.17, 1.2],
│   [-0.16, 1.2],
│   [-0.15, 1.2],
│   [-0.14, 1.2],
│   [-0.13, 1.2],
│   [-0.12, 1.2],
│   [-0.11, 1.2],
│   [-0.1, 1.2],
│   [-0.09, 1.2],
│   [-0.08, 1.2],
│   [-0.07, 1.2],
│   [-0.06, 1.2],
│   [-0.05, 1.2],
│   [-0.04, 1.2],
│   [-0.03, 1.2],
│   [-0.02, 1.2],
│   [-0.01, 1.2],
│   [0.0, 1.2],
│   [0.01, 1.2],
│   [0.02, 1.2],
│   [0.03, 1.2],
│   [0.04, 1.2],
│   [0.05, 1.2],
│   [0.06, 1.2],

"""