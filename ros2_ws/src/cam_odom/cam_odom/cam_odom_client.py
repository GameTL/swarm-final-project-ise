import socket
import json
import threading
import rclpy
import logging
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D, Twist
import sys
# Configure logging with ISO 8601 format
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(levelname)s][%(message)s]',
    datefmt='%H:%M:%S.%f'
)
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s][%(levelname)s][%(message)s]',
    datefmt='%H:%M:%S:$MS'
)

class CamOdomClient:
    def __init__(self, server_ip='192.168.0.103', port=12345, robot_id=None, debug=False):
        self.server_ip = server_ip
        self.port = port
        self.robot_id = str(robot_id) if robot_id is not None else None
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.current_position = {
            'x': 0.00,
            'y': 0.00,
            'theta': 0.00,
        }
        self._running = False
        self._buffer = ""
        self.debug = debug

    def connect(self):
        self.client.connect((self.server_ip, self.port))
        print("[CLIENT] Connected to position server")
        self._running = True
        threading.Thread(target=self._receive_loop, daemon=True).start()

    def _receive_loop(self):
        try:
            while self._running:
                data = self.client.recv(1024).decode()
                if not data:
                    break
                self._buffer += data
                while '\n' in self._buffer:
                    line, self._buffer = self._buffer.split('\n', 1)
                    try:
                        positions = json.loads(line)
                        # print(f"Messages received: {positions=}", end='', flush=True)
                        if self.robot_id in positions.keys():
                            self.current_position = positions[str(self.robot_id)]
                            if self.debug:
                                logging.info(f"(RECEIVED) {positions}")
                    except json.JSONDecodeError:
                        continue
        except Exception as e:
            print("[ERROR]", e)
        finally:
            self.client.close()
            print("Server Disconnected!")

    def get_position(self):
        return self.current_position

    def disconnect(self):
        self._running = False
        self.client.close()




class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('cam_odom_client')
        self.topic = '/robot_pose'
        self.publisher_ = self.create_publisher(Pose2D, self.topic, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.x = CamOdomClient(robot_id=3, debug=False)
        self.x.connect()
        self.msg = Pose2D()
        
        

    def timer_callback(self):
        # print(self.x.current_position)
        self.msg.x       = self.x.current_position["x"]
        self.msg.y       = self.x.current_position["y"]
        self.msg.theta   = self.x.current_position["theta"] 
        self.publisher_.publish(self.msg)
        self.get_logger().info('[%s|id:%s], x:%.2f, y:%.2f, theta:%.2f' % ('/robot_pose', self.x.robot_id,  self.msg.x,  self.msg.y,  self.msg.theta))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()