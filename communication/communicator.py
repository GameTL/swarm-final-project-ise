import json
import socket
import threading
import time
import ast

HOST_FILE_PATH = "./communication/hosts.json"
MAX_CONNECTIONS = 5
JETSON_NAME = "jetson1"
RETRY_ATTEMPTS = 3
RETRY_DELAY = 2  # seconds
MESSAGE_INTERVAL = 2000  # ms

class Communicator:
    def __init__(self, host_file_path: str, max_connections: int, debug=False):
        self.debug = debug
        self.max_connections = max_connections
        self.current_connections = {}
        self.peer_sockets = {}
        self.object_coordinates = {}
        self.task_master = ""
        self.path = None
        self.message_id = 0

        self.load_host_info(host_file_path)
        self.priority_queue = list(self.host_info.keys())

    def load_host_info(self, host_file_path: str):
        """Loads the host information from the JSON file."""
        try:
            with open(host_file_path, 'r') as host_file:
                self.host_info = json.load(host_file)
            if self.debug:
                print("[DEBUG] Host information loaded successfully.")
        except Exception as e:
            print(f"[ERROR] Error loading host file: {e}")
            self.host_info = {}

    def start_server(self, host_key: str):
        """Starts the server for the specified host in the config file."""
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
        """Identifies the Jetson peer based on its IP."""
        for name, data in self.host_info.items():
            if data["ip"] == ip:
                return name
        return "Unknown"

    def connect_to_peer(self, peer_key: str):
        """Establishes a persistent connection to a peer."""
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
        """Establishes persistent connections to all known peers."""
        for host in self.host_info:
            if host != JETSON_NAME:
                self.connect_to_peer(host)

    def listen_to_message(self, message):
        """Processes incoming messages with error handling."""
        if not message.strip():
            print("Received an empty message. Ignoring.")
            return None

        try:
            if self.debug:
                print(f"[DEBUG] Processing message: {message}")
            title, robot_id, message_id, content = json.loads(message)
            if title == "[path_receiving]":
                return "path_receiving"
            elif title == "[probe]":
                self.object_coordinates[robot_id] = content
                print(f"[PROBE] {content}")
            elif title == "[task]":
                self.task_master = robot_id
                self.object_coordinates = content
                return "task"
            elif title == "[task_conflict]":
                return "reassign"
            elif title == "[path_following]":
                paths = ast.literal_eval(content)
                if JETSON_NAME in paths.keys():
                    self.path = paths.get(JETSON_NAME, "")
                    return "path_following"
                else:
                    return "idle"
            elif title == "[task_successful]":
                return "path_finding" if self.task_master == JETSON_NAME else "idle"
            else:
                print("Unknown message received.")
        except json.JSONDecodeError as e:
            print(f"Error processing message: Invalid JSON format ({e})")
        except Exception as e:
            print(f"Unexpected error processing message: {e}")
        return None

    def broadcast_message(self, title: str, content):
        """Broadcasts a message to all connected peers."""
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
        """Handles incoming messages from clients, detects disconnections."""
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
    communicator.broadcast_message("[probe]", "Hello to all Jetsons!")
