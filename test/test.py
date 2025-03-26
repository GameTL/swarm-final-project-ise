import numpy as np
import cv2
import time

ARUCO_DICT = {
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250
    
} 

prev_frame_time = 0
new_frame_time = 0


def aruco_display(corners, ids, rejected, image):
    if len(corners) > 0:
        
        ids = ids.flatten()
        
        for (markerCorner, markerID) in zip(corners, ids):
            
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
            cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
            
    return image

# img = cv2.imread('./marker/marker_with_bg.png', 1) 

cap = cv2.VideoCapture(0)  # Get the camera source

while True:
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_type = ["DICT_6X6_250"]
    for i in aruco_type:
        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[i])

        arucoParams = cv2.aruco.DetectorParameters()

        corners, ids, rejected = cv2.aruco.ArucoDetector(arucoDict, arucoParams).detectMarkers(gray)

        detected_markers = aruco_display(corners, ids, rejected, frame)

    new_frame_time = time.time() 
    fps = 1/(new_frame_time-prev_frame_time) 
    prev_frame_time = new_frame_time 
  
    # converting the fps into integer 
    fps = int(fps) 
  
    # converting the fps to string so that we can display it on frame 
    # by using putText function 
    fps = str(fps) 
  
    # putting the FPS count on the frame 
    cv2.putText(detected_markers, f"fps: {fps}", (7, 70), cv2.FONT_HERSHEY_SIMPLEX , 1, (0, 0, 0), 3, cv2.LINE_AA) 

    cv2.imshow("Image", detected_markers)
    
    key = cv2.waitKey(3) & 0xFF
    if key == ord('q'):  # Quit
            break


cap.release()
cv2.destroyAllWindows()