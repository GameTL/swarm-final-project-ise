import numpy as np
import cv2
import time
import threading
import queue
from aruco import Aruco
from aruco_visual import ArucoVisual  # Assuming this uses `turtle`
import datetime as dt
from aruco_server import latest_data

#insert the size of the frame here
MAP_SIZE = (1.45, 1.07)

prev_frame_time = 0
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Open the camera
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
turtle_thread = threading.Thread(target=update_turtle_map, daemon=True)
turtle_thread.start()

while True:
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
    prev_frame_time = new_frame_time

    # Overlay FPS text
    cv2.putText(frame, f"FPS: {int(fps)}", (7, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2), radius=5, color=(0, 0, 255), thickness=-1)

    cv2.imshow("Aruco Detection", frame)

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break
    if cv2.waitKey(3) & 0xFF == ord('r'):
        aruco_visual.reset_map()

    if recorded_data is not None:
        latest_data.clear()
        latest_data.update(recorded_data)

cap.release()
cv2.destroyAllWindows()
if len(aruco.testing_data)!=0:
    print("saving data...")
    aruco.testing_data.to_csv(f"./result/{dt.datetime.today().strftime('%Y-%m-%d')}.csv", index=False)