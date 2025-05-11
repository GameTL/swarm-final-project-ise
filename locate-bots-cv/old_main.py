# PLESAE USE PYTHON 3.11 [dont work on 3.9 or 3.12]
import numpy as np
import cv2
import time
import threading
import queue
from aruco import Aruco
from aruco_visual import ArucoVisual  # Assuming this uses `turtle`
import datetime as dt
from aruco_server import latest_data
import sys

#########################
# gui = True
gui = False
printout = True
# printout = False
#########################
#insert the size of the frame here
MAP_SIZE = (1.45, 1.07)

prev_frame_time = 0
#* WINDOWS
# cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  # Open the camera
#* LINUX
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)      # ← new: V4L2 backend on Linux
if not cap.isOpened():                       #     bail out early if it fails
    raise RuntimeError("Could not open /dev/video0 – check index & permissions")
aruco = Aruco(MAP_SIZE)

# Thread-safe queue for communication between OpenCV and turtle
data_queue = queue.Queue()

# Function to update the turtle map in a separate thread
def update_turtle_map():
    global aruco_visual
    aruco_visual = ArucoVisual(640, 480)  # Initialize in the thread

    while True:
        try:
            data = data_queue.get(timeout=1)  # Wait for new data
            aruco_visual.update_map(data)  # Update turtle map
        except queue.Empty:
            pass  # No new data, continue looping

def record_data(df, timestemp):
    for key, values, in aruco.current_data.items():
        id = key
        x, y, theta = values
        data = {
            "timestamp": timestemp,
            "id": id,
            "x":x,
            "y": y,
            "theta": theta
        }
        df.loc[-1] = data

# Start turtle thread
if gui:
    turtle_thread = threading.Thread(target=update_turtle_map, daemon=True)
    turtle_thread.start()

counter = 0
fps = 0
try:
    while True: 
        # NEW  – overwrites the same line every pass (⏎ is carriage-return)
        ret, frame = cap.read()
        current_timestamp = dt.datetime.now()
        if not ret:
            continue  # Skip iteration if the frame is invalid

        detected_markers, recorded_data = aruco.aruco_detect(frame, current_timestamp)
        if detected_markers is None:
            detected_markers = frame.copy()

        # Send data to turtle thread
        data_queue.put(aruco.current_data)

        # FPS Calculation
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time else 0
        print(f"\rFrames: {counter:<10}, FPS: {str(int(fps)):<3}", end='', flush=True)
        prev_frame_time = new_frame_time

        # Overlay FPS text
        if gui:
            cv2.putText(frame, f"FPS: {int(fps)}", (7, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2), radius=5, color=(0, 0, 255), thickness=-1)
            cv2.imshow("Aruco Detection", frame)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break
        if cv2.waitKey(3) & 0xFF == ord('r'):
            aruco_visual.reset_map()

        if len(recorded_data) != 0:
            if printout:
                y = ""
                for _id in recorded_data.keys():
                    
                    x_str     = f"{recorded_data[_id]['x']:.2f}"
                    y_str     = f"{recorded_data[_id]['y']:.2f}"
                    theta_str = f"{recorded_data[_id]['theta']:.2f}"
                    y += f"\tid:{_id}, x:{x_str:<5}, y:{y_str:<3}, theta:{theta_str:<5};"
                print(y, end='', flush=True)
            latest_data.clear()
            latest_data.update(recorded_data)
        counter += 1

    cap.release()
    cv2.destroyAllWindows()
    if len(aruco.testing_data)!=0:
        print("saving data...")
        aruco.testing_data.to_csv(f"./result/{dt.datetime.today().strftime('%Y-%m-%d')}.csv", index=False)
except KeyboardInterrupt:
    print("\nInterrupted by user, shutting down gracefully.")
finally:
    cap.release()
    cv2.destroyAllWindows()