import cv2
import numpy as np
import rclpy
import threading
import csv
import os
from utils import LidarReader

window_title = "Yellow Cylinder Detection"

camera_id = "/dev/video0"

'''
Adjust color -> line 110 and 111
----------------------------------------------------------------------------------------------------------------------
BLUE
                lower_yellow = np.array([0, 100, 140])  # Adjust as needed
                upper_yellow = np.array([255, 255, 255])  # Adjust as needed
----------------------------------------------------------------------------------------------------------------------
BLUE
                lower_yellow = np.array([100, 140, 50])  # Adjust as needed
                upper_yellow = np.array([255, 255, 100])  # Adjust as needed

'''

# Camera properties
frame_width = 1280
frame_center_x = frame_width // 2
horizontal_fov = 78 #67.19710030243834
fx = 918.297472

# LiDAR properties
lidar_min_rad = -3.1241390705108643
lidar_max_rad = 3.1415927410125732
angle_increment = 0.0019344649044796824

# CSV File Setup
csv_filename = "data20_near_2.csv"
if not os.path.exists(csv_filename):
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        #(lidar_node, distance, object_center_x, object_width, x, raw_angle_diff, angle_diff, predicted_width)
        writer.writerow(["frame_width", "distance", "center_x", "w", "x", "raw_angle_diff", "angle_diff", "predicted_width"])

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
'''
def cal_width(distance):
    width = -0.0000431152 * distance**2 + 0.0011380118 * distance + 0.0000393178
    return width
'''
def find_angle_diff(lidar_node, left, right):
    relative_left = ((left - frame_center_x) / frame_width) * horizontal_fov
    left_360 = (relative_left + 360) % 360
    #left_angle_rad = map_cam_to_lidar(left_360)
     
    relative_right = ((right - frame_center_x) / frame_width) * horizontal_fov
    right_360 = (relative_right + 360) % 360
    #right_angle_rad = map_cam_to_lidar(right_360)
     
    raw_angle_diff = abs(np.radians(right_360 - left_360))
     
    if raw_angle_diff > np.pi:
        angle_diff = np.pi*2 - raw_angle_diff
        #print(angle_diff)
    elif raw_angle_diff < - np.pi:
        angle_diff = np.pi*2 +  raw_angle_diff
        #print(angle_diff)
    else:
        angle_diff = raw_angle_diff
        #print(angle_diff)
     
    return raw_angle_diff, angle_diff

'''
old version  ->  w = distance * 2 / cos(angle_diff/2)
def find_width(lidar_node, distance, left, right):
    _, angle = find_angle_diff(lidar_node, left, right)
    width = 2*( 0.0310178270 * distance**2 + -0.1685466068*distance + 1.1966356720 ) * np.tan(angle/2) * distance
    return width
    
    
after edit calculation (1st formular)
width = 2 * distance * np.tan(angle/2) / ( 1 - np.tan(angle/2) ) * (0.9527768982 / (1 + np.exp(-7.5788891181 * (distance - 0.0134318745))))
'''  

def find_width(lidar_node, distance, left, right):
    _, angle = find_angle_diff(lidar_node, left, right)
    #r = distance / np.cos(angle/2)
    #w_prime = np.sqrt( (2*(r**2)) - (2*(r**2)*np.cos(angle)) / 2 )
    width = 2 * distance * np.tan(angle/2) / ( 1 - np.tan(angle/2) ) * (0.9467576549 / (1 + np.exp(-8.9297158914 * (distance - 0.0317558291))))
    print(width)
    return width

def calculate_lidar_distance(object_center_x, frame_center_x, frame_width, horizontal_fov, map_cam_to_lidar, lidar_node):
    relative_angle = ((object_center_x - frame_center_x) / frame_width) * horizontal_fov
    angle_360 = (relative_angle + 360) % 360
    lidar_angle_rad = map_cam_to_lidar(angle_360)
    distance = lidar_node.get_distance(lidar_angle_rad)
    return distance
    
def detect_yellow_cylinder(lidar_node):
    if video_capture.isOpened():
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            
            while True:
                _, frame = video_capture.read()
                
                if frame is None:
                    continue

                # Convert frame to HSV
                ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
                
                # Define range for yellow color
                lower_yellow = np.array([100, 140, 50])  # Adjust as needed
                upper_yellow = np.array([255, 255, 100])  # Adjust as needed
                
                # Threshold the HSV image to get only yellow colors
                mask = cv2.inRange(ycrcb, lower_yellow, upper_yellow)
                
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

                if valid_bbox_found and object_center_x is not None :
                    '''
                    --------------------------------------------------------------------------------------------------------------------------
                    status  ->  ObjectFound()
                    stop motor  -> stop moving for 1-2 sec, make sure the robot is completely stationary
                    then start measurement
                    --------------------------------------------------------------------------------------------------------------------------
                    '''
                    relative_angle = ((object_center_x - frame_center_x) / frame_width) * horizontal_fov
                    angle_360 = (relative_angle + 360) % 360
                    lidar_angle_rad = map_cam_to_lidar(angle_360)
                    distance = lidar_node.get_distance(lidar_angle_rad)
                    width = np.round(find_width(node, distance, x, x+w),2)

                    '''
                    --------------------------------------------------------------------------------------------------------------------------
                    send out
                    -> angle_360
                    -> width
                    -> distance
                    --------------------------------------------------------------------------------------------------------------------------
                    '''
                    
                    '''
                    #node, distance, center_x, bbox_width, x, w
                    data = get_data(lidar_node, distance, object_center_x, object_width, x, w)
                    with open(csv_filename, mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(data)
                    '''
                    
                    # Display information
                    text = f"distance: {distance:.2f} m, angle: {angle_360:.2f} deg, width: {width} m,"
                    cv2.putText(frame, text, (10, 720-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    #print(distance, object_center_x, object_width, width)

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
        
