import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Define constants
SPEED = 0.05 # m/s
TURN_SPEED = 0.5  # rad/s
STEP_DURATION = 0.1 # Time per step - second(s)
STOP_DURATION = 1.0 # second(s)

class TeleopPublisher(Node):
    def __init__(self, jetson_name, command_list):
        super().__init__('teleop_publisher_' + jetson_name)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_list = command_list

    def send_commands(self):
        for direction, steps in self.command_list:
            twist = Twist()

            # Set velocity based on direction
            if direction == "pos_x":
                twist.linear.x = SPEED
            elif direction == "neg_x":
                twist.linear.x = -SPEED
            elif direction == "pos_y":
                twist.linear.y = SPEED
            elif direction == "neg_y":
                twist.linear.y = -SPEED
            elif direction == "turn_ccw":
                twist.angular.z = TURN_SPEED
            elif direction == "turn_cw":
                twist.angular.z = -TURN_SPEED
            elif direction == "stop":
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                steps = int(STOP_DURATION / STEP_DURATION)
            
            self.get_logger().info(f"Publishing: {direction} for {steps} steps")
            # Publish velocity repeatedly for steps
            for _ in range(steps):
                self.publisher.publish(twist)
                time.sleep(STEP_DURATION)

            # Stop after movement
            self.stop_robot()

    def stop_robot(self):
        # Send a zero-velocity Twist message to stop the robot
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info("Stopping the robot.")

if __name__ == '__main__':
    rclpy.init()

    # Example movement commands for jetson1
    example_commands = [
        ('neg_x', 72), ('pos_y', 80), ('stop', 1)
    ]

    teleop_publisher = TeleopPublisher("jetson1", example_commands)
    teleop_publisher.send_commands()

    teleop_publisher.destroy_node()
    rclpy.shutdown()
