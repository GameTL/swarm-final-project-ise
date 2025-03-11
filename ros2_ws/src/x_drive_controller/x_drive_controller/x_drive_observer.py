#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .submodules.dynamixel_class import DynamixelInterface  # Import motor interface
import time

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
