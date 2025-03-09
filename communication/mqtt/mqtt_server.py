import paho.mqtt.client as mqtt
import json
import time

# MQTT Broker Configuration
BROKER = "mqtt.eclipseprojects.io"
PORT = 1883
TOPIC_DETECT = "swarm/object_detected"
TOPIC_STOP = "swarm/stop"
TOPIC_PATH = "swarm/path"
TOPIC_ACK = "swarm/ack"

# Dictionary to store paths for each robot
robot_paths = {}

# Callback for when a message is received
def on_message(client, userdata, msg):
    global robot_paths
    topic = msg.topic
    payload = json.loads(msg.payload.decode())
    
    if topic == TOPIC_DETECT:
        print("[Server] Object detected! Sending stop signal to all robots.")
        client.publish(TOPIC_STOP, json.dumps({"command": "STOP"}))
        time.sleep(1)  # Allow robots to stop
        determine_paths()
    
    elif topic == TOPIC_ACK:
        robot_id = payload["robot_id"]
        if robot_id in robot_paths:
            del robot_paths[robot_id]  # Remove acknowledged robot
            print(f"[Server] Robot {robot_id} acknowledged path.")
    
    if not robot_paths:
        print("[Server] All robots acknowledged. Resuming operations.")
        client.publish(TOPIC_STOP, json.dumps({"command": "RESUME"}))

# Function to determine paths (mock implementation)
def determine_paths():
    global robot_paths
    print("[Server] Determining new paths for robots...")
    robot_paths = {
        "robot_1": [10, 20],
        "robot_2": [30, 40]
    }
    for robot, path in robot_paths.items():
        client.publish(TOPIC_PATH, json.dumps({"robot_id": robot, "path": path}))
    print("[Server] Paths sent to robots.")

if __name__ == "__main__":
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)

    client.subscribe(TOPIC_DETECT)
    client.subscribe(TOPIC_ACK)

    print("[Server] Listening for messages...")
    client.loop_forever()
