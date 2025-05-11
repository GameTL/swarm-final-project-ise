import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

import cv2
import time

# 🎥 Camera setup
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
vid_fmt = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, vid_fmt)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

if not cap.isOpened():
    raise RuntimeError("❌ Could not open webcam")

# ℹ️ Print actual settings
print("Camera Configuration:")
print(f"   ▶ Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f} x {cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}")
print(f"   ▶ FPS (claimed): {cap.get(cv2.CAP_PROP_FPS):.2f}")
print(f"   ▶ Brightness: {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
print(f"   ▶ Exposure: {cap.get(cv2.CAP_PROP_EXPOSURE)}")

# ⏱️ Manual FPS tracking
fps = 0
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame grab failed")
        break

    frame_count += 1
    elapsed = time.time() - start_time
    if elapsed >= 1:
        fps = frame_count / elapsed
        frame_count = 0
        start_time = time.time()

    # 📺 Show FPS on screen
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("FPS Test - Logitech Webcam", frame)

    # Quit key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
