import socket
import threading
import json
from aruco import Aruco
#########################
printout = False
#########################
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 12345      # Arbitrary non-privileged port

clients = []

def client_handler(conn, addr):
    print(f"[INFO] Robot connected from {addr}")
    while True:
        try:
            # Send the latest data to this client
            if latest_data:
                if printout:
                    print(f"[Sending]{latest_data}")
                conn.sendall((json.dumps(latest_data) + '\n').encode())
        except:
            print(f"[WARNING] Connection to {addr} lost.")
            clients.remove(conn)
            conn.close()
            break

def start_server():
    try:
        
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen()
        print(f"[SERVER] Listening on {HOST}:{PORT}")
    except OSError as e:
        print(e)
        quit()
    while True:
        conn, addr = server.accept()
        clients.append(conn)
        thread = threading.Thread(target=client_handler, args=(conn, addr), daemon=True)
        thread.start()

# Shared variable to hold the latest aruco.current_data
latest_data = {}

# Start the server in background
threading.Thread(target=start_server, daemon=True).start()