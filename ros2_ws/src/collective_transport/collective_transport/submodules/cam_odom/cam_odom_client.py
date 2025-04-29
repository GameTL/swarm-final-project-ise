import socket
import json
import threading

class CamOdomClient:
    def __init__(self, server_ip='127.0.0.1', port=12345, robot_id=None):
        self.server_ip = server_ip
        self.port = port
        self.robot_id = str(robot_id) if robot_id is not None else None
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.current_position = None
        self._running = False
        self._buffer = ""

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
                        if self.robot_id is None or str(positions.get('id')) == self.robot_id:
                            self.current_position = positions
                            print("[RECEIVED]", positions)
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
