#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from .....x_drive_controller.x_drive_controller.submodules.dcmotor_class import DCMotorInterface  # Import motor interface
import time
import numpy as np
import os


class OmniDriveObserver(Node):
    """ 
    Motor Configuration: 45 degrees to each x & y axis
    1A /  \ 2B
    4D \  / 3C
    """
    

    def __init__(self):
        super().__init__('omni_drive_observer')

        # Initialize Dynamixel Interface
        self.interface = DCMotorInterface()
        self.TICKS_PER_REV = 600  # to be changed
        self.WHEEL_RADIUS = 0.2854 #in metres
        self.LAST_ENCODER_VALUES = [0, 0, 0, 0]
        self.LAST_TIME = time.time()

        # Set a timer to continuously check motor states every 0.5s
        self.timer_period = 0.001  # Adjust as needed
        self.timer1 = self.create_timer(self.timer_period, self.read_motor_state)
        self.timer2 = self.create_timer(1, self.update_status)

        self.robot_velo = np.array([0, 0, 0])
        robot_id = int(os.environ.get('ROBOT_ID'))
        if robot_id == 1:
            self.robot_position = np.array([0, 0, 0])
        elif robot_id == 2:
            self.robot_position = np.array([0, 0, 0]) 
        elif robot_id == 3:
            self.robot_position = np.array([0, 0, 0])
        else:
            self.robot_position = np.array([0, 0, 0])
            
        self.position_publisher = self.create_publisher(Pose2D, 'robot_odom', 10)
    
    def update_status(self):
        print(f"Current Velo X: {self.robot_velo[0]}, Current Velo Y: {self.robot_velo[1]}, Current Velo Theta: {self.robot_velo[2]}"
            f" \n Current Posi X: {self.robot_position[0]}, Current Posi Y: {self.robot_position[1]}, Current Posi Theta: {self.robot_position[2]}")
        position_msg = Pose2D()
        position_msg.x = float(self.robot_position[0])
        position_msg.y = float(self.robot_position[1])
        position_msg.theta = float(self.robot_position[2])
        self.position_publisher.publish(position_msg)

        # self.robot_visual.update_position(self.robot_position[0], self.robot_position[1], self.robot_position[2])
    def call_back(self):
        current_ticks = [
            self.interface.get_encoder(1),
            self.interface.get_encoder(2),
            self.interface.get_encoder(3),
            self.interface.get_encoder(4)
        ]

        now = time.time()
        dt = now - self.LAST_TIME
        dticks = []
        for i in range(len(current_ticks)):
            delta = current_ticks[i] - self.LAST_ENCODER_VALUES[i]
            dticks.append(delta)
        self.LAST_ENCODER_VALUES = current_ticks
        self.LAST_TIME = now
        wheel_velocities = [
            (ticks / self.TICKS_PER_REV) * (2 * np.pi * self.WHEEL_RADIUS) / dt
            for ticks in dticks
        ]

        # Now convert wheel velocities → robot velocity using omni wheel kinematics
        vx, vy, omega = self.inverse_kinematics(wheel_velocities)
        return np.array([vx, vy, omega])  

    # def refresh_odometry(self):
    #     self.robot_velo = self.call_back()
    #     dt = self.timer_period
    #     velo = self.robot_velo
    #     if np.linalg.norm(velo) < 1e-4:
    #         return
    #     delta_posi = (velo * dt).reshape(-1)
    #     self.robot_position = (self.robot_position + delta_posi)

    def refresh_odometry(self):
        self.robot_velo, self.LAST_ENCODER_VALUES, self.LAST_TIME = self.interface.get_robot_velocity(self.LAST_ENCODER_VALUES, self.LAST_TIME)
        
        dt = self.timer_period
        velo = self.robot_velo
        if np.linalg.norm(velo) < 1e-4:
            return
        delta_posi = (velo * dt).reshape(-1)
        self.robot_position = (self.robot_position + delta_posi)

    def read_motor_state(self):
        """ Periodically reads motor states """
        try:
            # print("Reading motor state...")
            self.refresh_odometry()

        except Exception as e:
            self.get_logger().error(f"Error reading motor state: {e}")

def main(args=None):
    rclpy.init(args=args)
    omni_drive_observer = OmniDriveObserver()
    
    try:
        rclpy.spin(omni_drive_observer)
    except KeyboardInterrupt:
        pass
    finally:
        omni_drive_observer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
