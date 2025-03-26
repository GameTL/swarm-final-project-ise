import numpy as np
import cv2
import time
import threading
import queue
from aruco import Aruco
from aruco_visual import ArucoVisual  # Assuming this uses `turtle`

prev_frame_time = 0
cap = cv2.VideoCapture(0)  # Open the camera
aruco = Aruco()

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

# Start turtle thread
turtle_thread = threading.Thread(target=update_turtle_map, daemon=True)
turtle_thread.start()

while True:
    ret, frame = cap.read()
    if not ret:
        continue  # Skip iteration if the frame is invalid

    detected_markers = aruco.aruco_detect(frame)
    if detected_markers is None:
        detected_markers = frame.copy()

    # Send data to turtle thread
    data_queue.put(aruco.current_data)

    # FPS Calculation
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time else 0
    prev_frame_time = new_frame_time

    # Overlay FPS text
    cv2.putText(detected_markers, f"FPS: {int(fps)}", (7, 70), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("Aruco Detection", detected_markers)

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
