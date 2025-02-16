import json
import socket
import threading
import time

HOST_FILE_PATH = "./communication/hosts.json"
MAX_CONNECTIONS = 5
JETSON_NAME = "jetson1"
RETRY_ATTEMPTS = 3
RETRY_DELAY = 2  # seconds

class Communicator:
    def __init__(self, host_file_path: str, max_connections: int, debug=False):
        self.debug = debug
        self.max_connections = max_connections
        self.current_connections = {}
        self.peer_sockets = {}
        self.task_master = None
        self.message_id = 0

        self.load_host_info(host_file_path)
        self.priority_queue = list(self.host_info.keys())

    def load_host_info(self, host_file_path: str):
        try:
            with open(host_file_path, 'r') as host_file:
                self.host_info = json.load(host_file)
            if self.debug:
                print("[DEBUG] Host information loaded successfully.")
        except Exception as e:
            print(f"[ERROR] Error loading host file: {e}")
            self.host_info = {}

    def start_server(self, host_key: str):
        if host_key not in self.host_info:
            raise ValueError(f"[ERROR] No host configuration found for '{host_key}'")

        host_data = self.host_info[host_key]
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((host_data["ip"], host_data["port"]))
        server.listen(self.max_connections)
        if self.debug:
            print(f"[DEBUG] [{host_key}] Listening on {host_data['ip']}:{host_data['port']}")

        while True:
            try:
                client_socket, addr = server.accept()
                peer_name = self.identify_peer(addr[0])
                self.current_connections[addr] = peer_name
                if self.debug:
                    print(f"[{host_key}] Accepted connection from {peer_name} ({addr[0]}:{addr[1]})")
                client_handler = threading.Thread(target=self.handle_client, args=(client_socket, addr, peer_name), daemon=True)
                client_handler.start()
            except Exception as e:
                print(f"[ERROR] [{host_key}] Error accepting connection: {e}")

    def identify_peer(self, ip):
        for name, data in self.host_info.items():
            if data["ip"] == ip:
                return name
        return "Unknown"

    def connect_to_peer(self, peer_key: str):
        if peer_key not in self.host_info:
            print(f"[WARN] No peer configuration found for '{peer_key}'")
            return

        peer_data = self.host_info[peer_key]
        if peer_key in self.peer_sockets:
            if self.debug:
                print(f"[DEBUG] Reusing existing connection to {peer_key}.")
            return

        attempt = 0
        while attempt < RETRY_ATTEMPTS:
            try:
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5)
                client.connect((peer_data["ip"], peer_data["port"]))
                self.peer_sockets[peer_key] = client
                print(f"[CONN] Connected to {peer_key} ({peer_data['ip']}:{peer_data['port']})")
                return
            except Exception as e:
                print(f"[WARN] Failed to connect to {peer_key} (Attempt {attempt + 1}): {e}")
                attempt += 1
                time.sleep(RETRY_DELAY)

    def connect_to_peers(self):
        for host in self.host_info:
            if host != JETSON_NAME:
                self.connect_to_peer(host)

    def consensus(self, candidate_robot):
        if candidate_robot not in self.priority_queue:
            print(f"[WARN] {candidate_robot} is not in the priority queue.")
            return

        current_taskmaster = self.task_master

        if not current_taskmaster or self.priority_queue.index(candidate_robot) < self.priority_queue.index(current_taskmaster):
            self.task_master = candidate_robot
            self.broadcast_message("[taskmaster_agreement]", candidate_robot)
            print(f"[INFO] {candidate_robot} is now the taskmaster. Broadcasting agreement.")
        else:
            print(f"[INFO] Keeping {current_taskmaster} as taskmaster.")

    def listen_to_message(self, message):
        if not message.strip():
            print("Received an empty message. Ignoring.")
            return None

        try:
            if self.debug:
                print(f"[DEBUG] Processing message: {message}")

            title, robot_id, message_id, content = json.loads(message)

            if title == "[task]":
                self.consensus(robot_id)
                return "task"

            elif title == "[taskmaster_agreement]":
                self.task_master = content
                print(f"[INFO] Taskmaster globally set to {content}")

            return None
        except json.JSONDecodeError as e:
            print(f"Error processing message: Invalid JSON format ({e})")
        except Exception as e:
            print(f"Unexpected error processing message: {e}")
        return None

    def broadcast_message(self, title: str, content):
        message = json.dumps([title, JETSON_NAME, self.message_id, content])
        if self.debug:
            print(f"[DEBUG] [broadcast_message]({JETSON_NAME}) {message}")
        self.message_id += 1

        for peer_key, client in self.peer_sockets.items():
            try:
                client.sendall(message.encode('utf-8'))
                if self.debug:
                    print(f"[DEBUG] Message sent to {peer_key}")
            except Exception as e:
                print(f"[WARN] Lost connection to {peer_key}, attempting to reconnect.")
                del self.peer_sockets[peer_key]
                self.connect_to_peer(peer_key)

    def handle_client(self, client_socket, addr, peer_name):
        try:
            while True:
                message = client_socket.recv(1024)
                if not message:
                    print(f"[CONN] {peer_name} ({addr[0]}:{addr[1]}) disconnected.")
                    break
                response = self.listen_to_message(message.decode('utf-8'))
                if response:
                    client_socket.send(response.encode('utf-8'))
        except Exception as e:
            print(f"[ERROR] Error handling client {peer_name} ({addr[0]}:{addr[1]}): {e}")
        finally:
            client_socket.close()
            self.current_connections.pop(addr, None)
            if peer_name in self.peer_sockets:
                del self.peer_sockets[peer_name]
            print(f"[CONN] Connection with {peer_name} ({addr[0]}:{addr[1]}) closed.")

if __name__ == "__main__":
    communicator = Communicator(HOST_FILE_PATH, MAX_CONNECTIONS, debug=False)

    server_thread = threading.Thread(target=communicator.start_server, args=(JETSON_NAME,), daemon=True)
    server_thread.start()
    time.sleep(2)

    communicator.connect_to_peers()
    time.sleep(5)

    print(f"[MOCK] {JETSON_NAME} detected an object and is claiming taskmaster.")
    communicator.broadcast_message("[task]", JETSON_NAME)

    time.sleep(3)

    if communicator.task_master == JETSON_NAME:
        print(f"[MOCK] {JETSON_NAME} is now officially the taskmaster.")
    else:
        print(f"[MOCK] {JETSON_NAME} accepts {communicator.task_master} as the taskmaster.")

    while True:
        pass
    