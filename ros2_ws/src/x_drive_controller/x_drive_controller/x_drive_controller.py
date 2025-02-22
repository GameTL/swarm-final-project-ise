#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
from .submodules.dynamixel_class import DynamixelInterface
import csv
import os
import numpy
import time
from math import degrees
# Config


class XDriveController(Node):
    """ 
    Motor Configuration: 45 degrees to each x & y axis
    1A /  \ 2B
    4D \  / 3C
    """
    def __init__(self):
        super().__init__('x_drive_controller')
        self.interface = DynamixelInterface()
        self.interface.disable_all_motors()
        
        # Create subscriber
        self.subscription = self.create_subscription( Twist,'cmd_vel', self.cmd_vel_callback,1)

    def cmd_vel_callback(self, msg):
        self.interface.drive(xDot=msg.linear.x, yDot=msg.linear.y, thetaDot=msg.angular.z)
        

def main(args=None):
    rclpy.init(args=args)
    x_drive_controller = XDriveController()
    
    try:
        rclpy.spin(x_drive_controller)
    except KeyboardInterrupt:
        pass
    finally:
        x_drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()