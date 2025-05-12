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


""" 
SAFE SPEED FOR LIDAR ODOM 
CMD_VEL
MAX_LINEARX_X = speed 0.08206928047885179
MAX_LINEARX_Y = speed 0.08206928047885179
ANGULAR_Z = turn 0.000357019954697317 
"""

class XDriveController(Node):
    """ 
    Motor Configuration: 45 degrees to each x & y axis
    1A /  \ 2B
    4D \  / 3C
    """
    def __init__(self):
        super().__init__('x_drive_controller')
        # Declare & Get parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        self.interface = DynamixelInterface(device_name=serial_port)
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