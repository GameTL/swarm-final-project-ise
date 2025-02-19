#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datetime import datetime
from dynamixel_class import DynamixelInterface
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
        # Get current timestamp
        # init_t = time.time_ns()
        # # print(f" got the msg at {init_t}")
        # timestamp = self.get_clock().now().to_msg().sec
        # # print(f" finished datarow at {(time.time_ns() - init_t)/1000000}ms")
        
        # # Prepare data row
        # data_row = [
        #     timestamp,
        #     msg.linear.x,
        #     msg.linear.y,
        #     msg.angular.z
        self.interface.drive(xDot=msg.linear.x, yDot=msg.linear.y, thetaDot=msg.angular.z)
        # angle = degrees(numpy.arctan2(msg.linear.x, msg.linear.y))
        # speed = 600
        # if angle < 0:
            # angle += 360
        # print(f'{msg.angular.z=}')
        # print(f'{angle=}')
        # print(f" finished angle at {(time.time_ns() - init_t)/1000000}ms")
        # self.interface.drive(angle=angle, speed=max(abs(msg.linear.x*speed), abs(msg.linear.y*speed)), ang_speed=int(msg.angular.z))
        # print(f" finished motor at {(time.time_ns() - init_t)/1000000}ms")
        # self.interface.forward()
        
        # # Write to CSV file
        # with open(self.log_filename, 'a', newline='') as f:
        #     writer = csv.writer(f)
        #     writer.writerow(data_row)
        
        # self.get_logger().debug('Logged x-drive data')
        #         """
        # Converts ROS cmd_vel (m/s and rad/s) into wheel commands in mm/s (int).
        
        # Parameters:
        # cmd_vel_x  : float - Linear velocity in x [m/s]
        # cmd_vel_y  : float - Linear velocity in y [m/s]
        # angular_z  : float - Angular velocity about z [rad/s]
        # rot_scale  : int   - Scaling factor for rotation (mm/s per rad/s)
        # max_command: int   - Maximum allowed command (absolute value)

        # Returns:
        # A tuple of four integers representing the wheel speeds (mm/s) for wheels:
        # wheel1, wheel2, wheel3, wheel4.
        # """
        # # Convert linear speeds from m/s to mm/s (1 m/s = 1000 mm/s)
        # vx_mm = int(round(cmd_vel_x * 1000))
        # vy_mm = int(round(cmd_vel_y * 1000))
        
        # # Fixed-point approximation for 1/√2 (~0.7071) scaled by 1000:
        # inv_sqrt2 = 707  # represents 0.707
        
        # # Compute the translational (x-y) contribution for each wheel.
        # # Instead of dividing by sqrt(2), we multiply by 707 and integer-divide by 1000.
        # wheel1_trans = (vx_mm + vy_mm) * inv_sqrt2 // 1000  # front left (45°)
        # wheel2_trans = (-vx_mm + vy_mm) * inv_sqrt2 // 1000  # front right (135°)
        # wheel3_trans = (vx_mm - vy_mm) * inv_sqrt2 // 1000  # back left (-45°)
        # wheel4_trans = (-vx_mm - vy_mm) * inv_sqrt2 // 1000  # back right (-135°)
        
        # # Compute the rotation contribution (in mm/s).
        # # For an X-drive a common pattern is to add the rotation term on wheels 1 and 4
        # # and subtract it on wheels 2 and 3.
        # rot_component = int(round(rot_scale * angular_z))
        
        # # Combine translation and rotation for each wheel:
        # wheel1 = wheel1_trans + rot_component
        # wheel2 = wheel2_trans - rot_component
        # wheel3 = wheel3_trans - rot_component
        # wheel4 = wheel4_trans + rot_component
        
        # # Optionally, clip the wheel speeds to the allowed range [-max_command, max_command]
        # wheel1 = max(-max_command, min(max_command, wheel1))
        # wheel2 = max(-max_command, min(max_command, wheel2))
        # wheel3 = max(-max_command, min(max_command, wheel3))
        # wheel4 = max(-max_command, min(max_command, wheel4))
        
        # return wheel1, wheel2, wheel3, wheel4
        
    # def tf_cmd_vel_2_dyanmixel(self, msg):
        

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