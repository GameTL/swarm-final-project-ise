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
import os

os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

#########################
# gui = True
gui = False
printout = False
# printout = False
#########################
#insert the size of the frame here
MAP_SIZE = (1.45, 1.07)

prev_frame_time = 0
#* WINDOWS
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Open the camera
#* LINUX
# cap = cv2.VideoCapture(2, cv2.CAP_V4L2)      # ← new: V4L2 backend on Linux

# 🎥 Camera setup
vid_fmt = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, vid_fmt)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

if not cap.isOpened():                       #     bail out early if it fails
    raise RuntimeError("Could not open /dev/video0 – check index & permissions")
aruco = Aruco(MAP_SIZE)

cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
vid_fmt = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, vid_fmt)

# Thread-safe queue for communication between OpenCV and turtle
data_queue = queue.Queue()

# Function to update the turtle map in a separate thread
def update_turtle_map():
    global aruco_visual
    aruco_visual = ArucoVisual(960, 720)  # Initialize in the thread

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

def crop_center(frame, target_width=960, target_height=720):
    h, w, _ = frame.shape
    x1 = w // 2 - target_width // 2
    y1 = h // 2 - target_height // 2
    x2 = x1 + target_width
    y2 = y1 + target_height
    return frame[y1:y2, x1:x2]

# Start turtle thread
if gui:
    turtle_thread = threading.Thread(target=update_turtle_map, daemon=True)
    turtle_thread.start()

fps = 0
frame_count = 0
start_time = time.time()
try:
    while True: 
        # NEW  – overwrites the same line every pass (⏎ is carriage-return)
        ret, frame = cap.read()
        current_timestamp = dt.datetime.now()
        if not ret:
            continue  # Skip iteration if the frame is invalid
        frame = crop_center(frame)
        detected_markers, recorded_data = aruco.aruco_detect(frame, current_timestamp)

        # Send data to turtle thread
        data_queue.put(aruco.current_data)

        frame_count += 1
        elapsed_time = time.time() - start_time  # 경과 시간
        if elapsed_time >= 1:
            fps = frame_count / elapsed_time  # FPS 계산
            start_time = time.time()  # 시간 초기화
            frame_count = 0
        print(f"\rFrames: {frame_count:<10}, FPS: {str(int(fps)):<3}", end='', flush=True)

        # Overlay FPS text
        if True:
            cv2.putText(frame, f"FPS: {int(fps)}", (7, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2), radius=5, color=(0, 0, 255), thickness=-1)
            cv2.imshow("Aruco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if cv2.waitKey(1) & 0xFF == ord('r'):
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