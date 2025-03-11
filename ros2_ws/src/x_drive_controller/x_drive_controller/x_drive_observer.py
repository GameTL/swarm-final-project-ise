#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .submodules.dynamixel_class import DynamixelInterface  # Import motor interface
import time
import numpy as np

class XDriveObserver(Node):
    """ 
    Motor Configuration: 45 degrees to each x & y axis
    1A /  \ 2B
    4D \  / 3C
    """

    def __init__(self):
        super().__init__('x_drive_observer')

        # Initialize Dynamixel Interface
        self.interface = DynamixelInterface()

        # Set a timer to continuously check motor states every 0.5s
        self.timer_period = 0.01  # Adjust as needed
        self.timer = self.create_timer(self.timer_period, self.read_motor_state)

        self.robot_position = np.array([0, 0, 0]) # initial position

    def refresh_odometry(self, xDot, yDot, thetaDot):
        dt = self.timer_period
        velo = np.array([[xDot],
                         [yDot],
                         [thetaDot]])
        delta_posi = velo*dt
        self.robot_position = self.robot_position+delta_posi


    def read_motor_state(self):
        """ Periodically reads motor states """
        try:
            print("Reading motor state...")
            self.interface.call_back()
        except Exception as e:
            self.get_logger().error(f"Error reading motor state: {e}")

def main(args=None):
    rclpy.init(args=args)
    x_drive_observer = XDriveObserver()
    
    try:
        rclpy.spin(x_drive_observer)
    except KeyboardInterrupt:
        pass
    finally:
        x_drive_observer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
