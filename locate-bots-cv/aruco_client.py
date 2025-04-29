import socket
import json

SERVER_IP = '192.168.0.150'  
PORT = 12345

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((SERVER_IP, PORT))
print("[CLIENT] Connected to position server")

try:
    buffer = ""
    while True:
        data = client.recv(1024).decode()
        buffer += data
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            try:
                positions = json.loads(line)
                print("[RECEIVED]", positions)  # You can now use this to navigate
            except json.JSONDecodeError:
                continue
except KeyboardInterrupt:
    client.close()

print("Server Disconnected!")
