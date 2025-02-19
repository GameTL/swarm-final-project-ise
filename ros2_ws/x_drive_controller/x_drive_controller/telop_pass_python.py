# #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
import os 
import signal
import time
import math


#>>>>>>>CONFIGURATION>>>>>>> 
def linear(x):
    return x

def two_exponential(x):
    return (2**abs(x) - 1) * math.copysign(1, x)

def log2(x):
    return (math.log2(x) - 1) * math.copysign(1, x)

def x_power_two(x):
    return (x**2) * math.copysign(1, x)

def x_power_three(x):
    return x**3

trigger_curve_dict = {
    "linear": linear,
    "two_exponential": two_exponential,
    "log2": log2,
    "x_power_two": x_power_two,
    "x_power_three": x_power_three,
}
#<<<<<<<CONFIGURATION-SETUP<<<<<<< 

#>>>>>>>CONFIGURATION>>>>>>> 
# When using Ubuntu on Mac --> True
mac_keybind = True

# Set top speed
max_linear_x = float(0.31) #m/s
max_angular_z = float(2 * math.pi) #rad/s



class TeleopJoy(Node):
    
    def __init__(self,
                 mac_keybind = False):
        super().__init__('teleop_joy_gta_node')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_tf, 10)
        print(f"""
    max_linear_x    = {max_linear_x} m/s
    max_angular_z   = {max_angular_z} rad/s """)

    # def publish_zeros(self):
    #     rate = self.create_rate(10)
    #     while rclpy.ok():
    #             twist = Twist()
    #             twist.linear.x = 0.00
    #             twist.linear.y = 0.00
    #             twist.angular.z = 0.00
    #             self.cmd_vel_pub.publish(twist)
    #         rate.sleep()

    def joy_tf(self, data):
        x = float((data.axes[0])) # -1 to 1
        y = float((data.axes[1])) # -1 to 1
        theta = float((data.axes[2])) # -1 to 1
        twist = Twist()
        twist.linear.x = -x
        twist.linear.y = y
        twist.angular.z = theta

        self.cmd_vel_pub.publish(twist)

        # if data.buttons[0] or self.always_on:  # use self to reference instance variable
        #     self.idlepub = False  # use self here

        # else:
        #     self.idlepub = True  # use self here


def main(args=None):
    try:

        rclpy.init(args=args)
        teleop_joy_node = TeleopJoy()
        rclpy.spin(teleop_joy_node)
        teleop_joy_node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        try:
            print("KeyboardInterrupt: juiting..")
            pid = os.getpid()
            os.kill(pid, signal.SIGKILL)
                
        except ProcessLookupError:
            print(f"Process with id {pid} does not exist.")
        except PermissionError:
            print(f"You don't have permission to kill process with id {pid}.")
        time.sleep(0.1)
        exit()


if __name__ == '__main__':
    main()