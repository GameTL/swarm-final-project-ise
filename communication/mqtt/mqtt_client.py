import paho.mqtt.client as mqtt
import json
import threading
import time

# MQTT Configuration
BROKER = "mqtt.eclipseprojects.io"
PORT = 1883
TOPIC_DETECT = "swarm/object_detected"
TOPIC_STOP = "swarm/stop"
TOPIC_PATH = "swarm/path"
TOPIC_ACK = "swarm/ack"

ROBOT_ID = "robot_1"  # Unique ID for each Jetson

# State Variables
current_path = None
stopped = False

def on_message(client, userdata, msg):
    global current_path, stopped
    topic = msg.topic
    payload = json.loads(msg.payload.decode())
    
    if topic == TOPIC_STOP:
        command = payload["command"]
        if command == "STOP":
            print(f"[Robot {ROBOT_ID}] Stopping movement!")
            stopped = True
        elif command == "RESUME":
            print(f"[Robot {ROBOT_ID}] Resuming operations!")
            stopped = False
    
    elif topic == TOPIC_PATH and payload["robot_id"] == ROBOT_ID:
        current_path = payload["path"]
        print(f"[Robot {ROBOT_ID}] Received new path: {current_path}")
        acknowledge_path()

def acknowledge_path():
    print(f"[Robot {ROBOT_ID}] Acknowledging new path.")
    client.publish(TOPIC_ACK, json.dumps({"robot_id": ROBOT_ID}))

def object_detection_simulation():
    global stopped
    while True:
        time.sleep(5)  # Simulated object detection interval
        if not stopped:
            print(f"[Robot {ROBOT_ID}] Object detected! Sending alert.")
            client.publish(TOPIC_DETECT, json.dumps({"robot_id": ROBOT_ID}))

if __name__ == "__main__":
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)

    client.subscribe(TOPIC_STOP)
    client.subscribe(TOPIC_PATH)

    # Start object detection in a separate thread
    detection_thread = threading.Thread(target=object_detection_simulation, daemon=True)
    detection_thread.start()

    print(f"[Robot {ROBOT_ID}] Listening for commands...")
    client.loop_forever()
