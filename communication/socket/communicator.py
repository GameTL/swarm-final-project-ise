import json
import time
import socket
import threading
from collections import defaultdict
from formation import FormationMaster
from translator import Translator

IDENTIFIER = "jetson1"
HOST_FP = "./communication/hosts.json"
MAX_CONNECTIONS = 5
TIMEOUT = 5
MAX_RETRIES = 3

class Communicator:
    def __init__(self, host_fp=HOST_FP, identifier=IDENTIFIER, max_connections=MAX_CONNECTIONS, suppress_output=True):
        # Initialize default attributes
        self.host_fp = host_fp
        self.identifier = identifier
        self.max_connections = max_connections
        self.taskmaster_claims = []
        self.suppress = suppress_output

        # For object detection and path planning
        self.current_coords = [0.0, 0.0]
        self.coords_dict = {}
        self.obstacle_coords = [] # Should know from lidar
        self.object_coords = () # Should know from lidar + camera
        self.taskmaster = ""
        
        # For movement
        self.command = []
        self.orientation = 0.0

        # Initialize host information and priority queue
        self.parse_json()

    def parse_json(self):
        try:
            with open(self.host_fp, mode="r") as hosts:
                self.host_info = json.load(hosts)
        except Exception as e:
            print(f"[ERROR] Error loading host information: {e}")
            self.host_info = {}

        self.priority_queue = list(self.host_info.keys())

    def comm_thread_spawner(self):
        if self.identifier not in self.host_info:
            raise ValueError(f"[ERROR] No host configuration found for '{self.identifier}'")

        host_data = self.host_info[self.identifier]
        server_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_fd.bind((host_data["ip"], host_data["port"]))
        server_fd.listen(self.max_connections)

        print(f"[INFO] {self.identifier} listening on {host_data['ip']}:{host_data['port']}")

        while True:
            try:
                client_fd, _ = server_fd.accept()
                client_thread = threading.Thread(target=self.handle_connection, args=(client_fd, ), daemon=True)
                client_thread.start()
            except Exception as e:
                print(f"[ERROR] [{self.identifier}] Error accepting connection: {e}")

    def handle_connection(self, client_fd):
        try:
            message = client_fd.recv(1024).decode("utf-8")
            if not message:
                return

            data = json.loads(message)
            header = data.get("header", "")
            sender = data.get("sender", "")
            content = data.get("content", "")

            if header == "OBJECT_DETECTED":
                print("[INFO] Object detected, stopping.") # TODO Replace with actual functionality
                self.consensus(sender)  # Trigger consensus function
            # elif header == "CONSENSUS_REACHED":
            #     print(f"[INFO] Consensus reached, taskmaster is {content}")
            elif header == "COORDINATES":
                # print(f"[INFO] Received {content} from {sender}")
                self.coords_dict[sender] = content
                self.coords_dict[self.identifier] = self.current_coords # Add its own coords

                if len(self.coords_dict) == len(self.priority_queue):
                    # print(f"({self.identifier}) {self.taskmaster}")
                    if self.identifier == self.taskmaster:
                        self.path_planning(
                            current_coords=self.coords_dict,
                            object_coords=self.object_coords,
                            obstacle_coords=self.obstacle_coords
                        )
                        self.taskmaster = ""

                        # Clear path planning params
                        # self.coords_dict = {}
                    
                    print(f"[COORDS] Current coords: {self.coords_dict}")
            elif header == "PATH":
                if not self.suppress:
                    print(f"[DEBUG] Received the paths from {sender}: {content}")

                try:
                    commands = json.loads(content)
                    # For robots that receive paths (non-taskmaster)
                    self.command = commands["waypoints"].get(self.identifier, [])
                    self.orientation = commands["orientation"].get(self.identifier, 0.0)

                    print(f"[PATH]({self.identifier}) Identified path: {self.command}; Orientation: {self.orientation}")

                except json.JSONDecodeError as e:
                    print(f"[ERROR] Failed to decode paths: {e}")

            # Send ACK back
            client_fd.sendall("ACK".encode("utf-8"))

            # Wait for SYNACK confirmation
            synack = client_fd.recv(1024).decode("utf-8")
            if synack == "SYNACK" and not self.suppress:
                print("[INFO] SYNACK received")

        except (json.JSONDecodeError, ConnectionError) as e:
            print(f"[ERROR] Error handling connection: {e}")

        finally:
            client_fd.close()

    def broadcast(self, header, content):
        message = json.dumps({
            "header": header,
            "sender": self.identifier,
            "content": content
        })

        for peer, peer_data in self.host_info.items():
            if peer == self.identifier:
                continue  # Skip self

            attempt = 0
            while attempt < MAX_RETRIES:
                try:
                    client_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    client_fd.settimeout(TIMEOUT)
                    client_fd.connect((peer_data["ip"], peer_data["port"]))

                    # Send the message
                    client_fd.sendall(message.encode("utf-8"))

                    # Wait for ACK
                    ack = client_fd.recv(1024).decode("utf-8")
                    if ack == "ACK":
                        # Send SYNACK confirmation
                        client_fd.sendall("SYNACK".encode("utf-8"))
                        if not self.suppress:
                            print(f"[INFO] {peer} acknowledged the message.")
                        break

                except (socket.timeout, ConnectionError):
                    print(f"[WARN] No ACK from {peer}, retrying ({attempt + 1}/{MAX_RETRIES})...")
                    attempt += 1
                    time.sleep(1)

                finally:
                    client_fd.close()

    def consensus(self, new_claimant):
        if new_claimant == self.identifier:
            print(f"[INFO] ({self.identifier}) Claiming taskmaster.")

        self.taskmaster_claims.append(new_claimant)

        # Get unique claims
        unique_claims = set(self.taskmaster_claims)

        if len(unique_claims) == 1:
            self.taskmaster = list(unique_claims)[0]  # Everyone agrees
        else:
            # Check who is highest in priority queue
            for robot in self.priority_queue:
                if robot in unique_claims:
                    self.taskmaster = robot
                    break
            
            # Count votes if still mismatched
            vote_count = defaultdict(int)
            for claim in self.taskmaster_claims:
                vote_count[claim] += 1

            # Find robot with most votes
            most_voted = max(vote_count, key=vote_count.get)
            if vote_count[most_voted] > len(self.taskmaster_claims) // 2:
                self.taskmaster = most_voted

        # Push taskmaster to back in the priority queue
        if self.taskmaster in self.priority_queue:
            self.priority_queue.remove(self.taskmaster)
            self.priority_queue.append(self.taskmaster)

        print(f"[INFO] Consensus reached: {self.taskmaster} is the taskmaster.")
        
        # Broadcast consensus to all robots
        # self.broadcast("CONSENSUS_REACHED", self.taskmaster)

        # Send own coordinates to the taskmaster
        if self.identifier == self.taskmaster:
            print(f"[INFO] {self.identifier} is the taskmaster, no need to send coords.")
        else:
            # Send coordinates to the taskmaster
            self.send_coords_to_taskmaster(self.taskmaster)

        # Clear claims after consensus
        self.taskmaster_claims.clear()

    def send_coords_to_taskmaster(self, taskmaster):
        taskmaster_data = self.host_info.get(taskmaster)
        if taskmaster_data:
            try:
                # Connect to the taskmaster
                taskmaster_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                taskmaster_fd.settimeout(TIMEOUT)
                taskmaster_fd.connect((taskmaster_data["ip"], taskmaster_data["port"]))

                # Prepare the message with coordinates
                message = json.dumps({
                    "header": "COORDINATES",
                    "sender": self.identifier,
                    "content": self.current_coords
                })

                # Send the coordinates to the taskmaster
                taskmaster_fd.sendall(message.encode("utf-8"))

                # Wait for ACK
                ack = taskmaster_fd.recv(1024).decode("utf-8")
                if ack == "ACK":
                    print(f"[INFO] Taskmaster {taskmaster} acknowledged the coordinates.")
                    
            except (socket.timeout, ConnectionError) as e:
                print(f"[ERROR] Error sending coordinates to {taskmaster}: {e}")
            finally:
                taskmaster_fd.close()

    def object_detected(self, message=""):
        """
        Utility function for robot to call when object is detected
        """
        self.consensus(self.identifier)
        communicator.broadcast("OBJECT_DETECTED", message) # TODO: Should be object and obstacle positions

    def path_planning(self, current_coords, object_coords, obstacle_coords, radius=0.3):
        print(f"[INFO] Received this set of current_coords: {current_coords}")
        
        # Initialize formation master
        self.formation_master = FormationMaster(
            self.identifier, 
            current_coords=current_coords, 
            object_coords=object_coords, 
            obstacles=obstacle_coords, 
            radius=radius
        )
        
        self.formation_master.calculate_target_coords()
        self.formation_master.plan_paths()

        paths = self.formation_master.paths

        # Convert paths to commands
        translator = Translator()

        #! Deprecated -- leave there for now for potential rollback
        # translator.calculate_commands(paths)
        # commands = translator.commands

        translator.sample_waypoints(paths)
        commands = translator.waypoints

        # Get your own path and orientation
        self.command = commands["waypoints"].get(self.identifier, [])
        self.orientation = commands["orientation"].get(self.identifier, 0.0)
        print(f"[PATH]({self.identifier}) Taskmaster's command: {self.command}; Orientation: {self.orientation}")

        # Broadcast commmands to every robot
        if not self.suppress:
            print(f"[DEBUG] {commands}")
        self.broadcast("PATH", json.dumps(commands))

    def cleanup(self):
        #! Clear computed path after moving (placeholder)
        self.command = []
        self.orientation = 0.0

    def start_periodic_broadcast(self, interval_sec=2):
        def broadcast_loop():
            last_time = time.time()
            while True:
                current_time = time.time()
                if current_time - last_time >= interval_sec:
                    # print("[COORDS] Broadcasting coordinates")
                    self.broadcast("COORDINATES", self.current_coords)
                    last_time = current_time
                time.sleep(0.05) # Light delay to avoid busy loop

        threading.Thread(target=broadcast_loop, daemon=True).start()

if __name__ == "__main__":
    communicator = Communicator()

    # For testing purpose
    communicator.current_coords = [1.78, -1.05]
    communicator.object_coords = [0.75, -0.25]
    communicator.obstacle_coords = [[-1, -1.4], [0.6, 0.3], [0.1, 1.67]]

    # Start listening thread
    server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
    server_thread.start()

    communicator.start_periodic_broadcast(interval_sec=4)

    while True:
        user_input = input("Press 'S' to claim taskmaster role: ").strip().upper()
        if user_input == "S":
            communicator.object_detected()
            communicator.cleanup() # To clear waypoints and orientation
        elif user_input == "Q":
            break
        else:
            communicator.current_coords[0] += 0.1 # Simulate robots moving in x-direction
            print(f"[DEBUG] Current position updated to: {communicator.current_coords}")

    server_thread.join()
    