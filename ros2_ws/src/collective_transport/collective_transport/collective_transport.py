#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
from .submodules.dynamixel_class import DynamixelInterface
import os

# Get the environment variable
ROBOT_ID: int = int(os.environ.get('ROBOT_ID'))

# Print the value
print(f"Robot ID: {str(robot_id)}")

def main(args=None):
    # launch odom and localization of the ROBOT_ID
    # launch vision 
    # launch comms of ROBOT_ID
    rclpy.init(args=args)
    
    try:
        rclpy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        x_drive_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()