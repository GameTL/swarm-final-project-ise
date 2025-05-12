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
    def __init__(self, server_ip='192.168.0.150', port=12345, robot_id=None, debug=False):
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
        try:
            self.client.connect((self.server_ip, self.port))
        except OSError as e:
            raise ConnectionError(f"Failed to connect: {e}") from e
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
    @property
    def current_position_2d(self):
        return (self.current_position["x"], self.current_position["y"])

    def disconnect(self):
        self._running = False
        self.client.close()


def main(args=None):
    x = CamOdomClient(robot_id=3, debug=False)
    x.connect()
    x.current_position


if __name__ == '__main__':
    main()