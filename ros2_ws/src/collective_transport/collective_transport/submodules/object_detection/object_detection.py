import cv2
import numpy as np
import rclpy
import threading
import csv
import os
from utils import LidarReader

window_title = "Yellow Cylinder Detection"

camera_id = "/dev/video0"

# Camera properties
frame_width = 1280
frame_center_x = frame_width // 2
horizontal_fov = 78 #67.19710030243834
fx = 918.297472

#variation factor: 0.0344768746 * distance**2 + -0.1910787309*distance + 1.2288527891

# LiDAR properties
lidar_min_rad = -3.1241390705108643
lidar_max_rad = 3.1415927410125732
angle_increment = 0.0019344649044796824

video_capture = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
video_capture.set(cv2.CAP_PROP_FPS, 30)

def map_cam_to_lidar(angle_360):
    lidar_angle_rad = lidar_max_rad - (angle_360 / 360) * (lidar_max_rad - lidar_min_rad)
    return lidar_angle_rad

def get_data(node, distance, center_x, bbox_width, x, w):
    real_object_width = 0.2  # Example width in meters, you can modify this based on your needs
    raw_angle_diff, angle_diff = find_angle_diff(node, x, x+w)
    return [frame_width, distance, center_x, w, x, raw_angle_diff, angle_diff, bbox_width, real_object_width]  

def find_angle_diff(lidar_node, left, right):
    relative_left = ((left - frame_center_x) / frame_width) * horizontal_fov
    left_360 = (relative_left + 360) % 360
     
    relative_right = ((right - frame_center_x) / frame_width) * horizontal_fov
    right_360 = (relative_right + 360) % 360
     
    raw_angle_diff = abs(np.radians(right_360 - left_360))
     
    if raw_angle_diff > np.pi:
        angle_diff = np.pi*2 - raw_angle_diff
        print(angle_diff)
    elif raw_angle_diff < - np.pi:
        angle_diff = np.pi*2 +  raw_angle_diff
        print(angle_diff)
    else:
        angle_diff = raw_angle_diff
        print(angle_diff)
     
    return raw_angle_diff, angle_diff
 
def find_width(lidar_node, distance, left, right):
    _, angle = find_angle_diff(lidar_node, left, right)
    width = 2*( 0.0344768746 * distance**2 + -0.1910787309*distance + 1.2288527891 ) * np.tan(angle/2) * distance
    return width
   
def detect_yellow_cylinder(lidar_node):
    if video_capture.isOpened():
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            
            while True:
                _, frame = video_capture.read()
                
                if frame is None:
                    continue

                # Convert frame to HSV
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Define range for yellow color
                lower_yellow = np.array([20, 130, 50])  # Adjust as needed
                upper_yellow = np.array([30, 255, 255])  # Adjust as needed
                
                # Threshold the HSV image to get only yellow colors
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # Find contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                object_center_x = None
                valid_bbox_found = False
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 1000:  # Adjust minimum area as needed
                        x, y, w, h = cv2.boundingRect(cnt)
                        
                        # Check if the center of the bounding box is within 500px from the left and right edges
                        if x + w // 2 >= 600 and (x + w // 2) <= (frame_width - 600):
                            object_center_x = x + w // 2
                            object_width = w
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                            valid_bbox_found = True  # Valid bounding box found

                if valid_bbox_found and object_center_x is not None:
                    #---------------------------------------------------------------------------------------

                    #              status: "Object Found"
                    #              then stop moving

                    #---------------------------------------------------------------------------------------
                    relative_angle = ((object_center_x - frame_center_x) / frame_width) * horizontal_fov
                    angle_360 = (relative_angle + 360) % 360
                    lidar_angle_rad = map_cam_to_lidar(angle_360)
                    distance = lidar_node.get_distance(lidar_angle_rad)
                    width = np.round(find_width(node, distance, x, x+w),2)

                    #---------------------------------------------------------------------------------------

                    #              send info -> distance, width, relative_angle

                    #---------------------------------------------------------------------------------------

                    # Display information
                    text = f"{distance:.2f}m, {lidar_angle_rad:.2f} rad, w: {width}m"
                    cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                # Display result
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break
                
                keyCode = cv2.waitKey(30) & 0xFF
                if keyCode == 27 or keyCode == ord('q'):  # Exit on ESC or 'q'
                    break

        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == "__main__":
    rclpy.init()
    node = LidarReader()
    
    # Spin the node in a separate thread so it keeps receiving LiDAR data
    lidar_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    lidar_thread.start()
    
    try:
        detect_yellow_cylinder(node)
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
