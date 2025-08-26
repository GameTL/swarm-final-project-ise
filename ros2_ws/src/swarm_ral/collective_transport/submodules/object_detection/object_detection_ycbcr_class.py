#* Make sure to run this first 
# ros2 launch sllidar_ros2 sllidar_s3_launch.py 'serial_port:=/dev/ttyUSB1'
import cv2
import numpy as np
import rclpy
import threading
import csv
import os
try:
    from .utils.lidar_reader import LidarReader
except:
    pass
try:
    from utils.lidar_reader import LidarReader
except:
    pass

window_title = "Yellow Cylinder Detection"


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

class CylinderDetection(tuple):
    def __new__(cls, distance: float, width: float, angle360: float):
        return tuple.__new__(cls, (distance, width, angle360))

    def __init__(self, distance: float, width: float, angle360: float):
        self.distance: float = distance
        self.width: float = width
        self.angle360: float = angle360


class CVMeasure:
    def __init__(self, camera_id="/dev/video0", cv_window=False):
        # Camera properties
        self.cv_window = cv_window
        self.camera_id = camera_id
        self._frame_width = 1280
        self._frame_center_x = self._frame_width // 2
        self._horizontal_fov = 78 #67.19710030243834
        self._fx = 918.297472
    
        # LiDAR properties
        self._lidar_min_rad = -3.1241390705108643
        self._lidar_max_rad = 3.1415927410125732
        self._angle_increment = 0.0019344649044796824
    
        # CSV File Setup
        csv_filename = "data20_near_2.csv"
        if not os.path.exists(csv_filename):
            with open(csv_filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                #(lidar_node, distance, object_center_x, object_width, x, raw_angle_diff, angle_diff, predicted_width)
                writer.writerow(["self._frame_center_x", "distance", "center_x", "w", "x", "raw_angle_diff", "angle_diff", "predicted_width"])

        self._video_capture = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        self._video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
        self._video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self._video_capture.set(cv2.CAP_PROP_FPS, 30)

        if not rclpy.ok():
            rclpy.init()
        self.lidar_node = LidarReader()
            
        # Spin the node in a separate thread so it keeps receiving LiDAR data
        lidar_thread = threading.Thread(target=rclpy.spin, args=(self.lidar_node,), daemon=True)
        lidar_thread.start()
        
        self.cylinder_detection = None
        cylinder_thread = threading.Thread(target=self.detect_yellow_cylinder, daemon=True)
        cylinder_thread.start()
        print("CV Init and Running..")
    def destroy_node(self):
        self.lidar_node.destroy_node()
        
    def map_cam_to_lidar(self, angle360):
        lidar_angle_rad = self._lidar_max_rad - (angle360 / 360) * (self._lidar_max_rad - self._lidar_min_rad)
        return lidar_angle_rad

    # def get_data(self, node, distance, center_x, bbox_width, x, w):
    #     real_object_width = 0.2  # Example width in meters, you can modify this based on your needs
    #     raw_angle_diff, angle_diff = self.find_angle_diff( x, x+w)
    #     return [self._frame_center_x, distance, center_x, w, x, raw_angle_diff, angle_diff, bbox_width, real_object_width]  
    '''
    def cal_width(distance):
        width = -0.0000431152 * distance**2 + 0.0011380118 * distance + 0.0000393178
        return width
    '''
    def find_angle_diff(self, left, right):
        relative_left = ((left - self._frame_center_x) / self._frame_width) * self._horizontal_fov
        left_360 = (relative_left + 360) % 360
        #left_angle_rad = map_cam_to_lidar(left_360)
        
        relative_right = ((right - self._frame_center_x) / self._frame_width) * self._horizontal_fov
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

    def find_width(self, distance, left, right):
        if distance is None:
            return None
        
        _, angle = self.find_angle_diff( left, right)
        #r = distance / np.cos(angle/2)
        #w_prime = np.sqrt( (2*(r**2)) - (2*(r**2)*np.cos(angle)) / 2 )
        width = 2 * distance * np.tan(angle/2) / ( 1 - np.tan(angle/2) ) * (0.9467576549 / (1 + np.exp(-8.9297158914 * (distance - 0.0317558291))))
        # print(width)
        return width

    # def calculate_lidar_distance(self, object_center_x):
    #     relative_angle = ((object_center_x - self._frame_center_x) / self._frame_center_x) * self._horizontal_fov
    #     angle360 = (relative_angle + 360) % 360
    #     lidar_angle_rad = self.map_cam_to_lidar(angle360)
    #     distance = self.lidar_node.get_distance(lidar_angle_rad)
    #     return distance
        
    def detect_yellow_cylinder(self):
        if self._video_capture.isOpened():
            try:
                if self.cv_window:
                    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                
                while True:
                    _, frame = self._video_capture.read()
                    
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
                            if x + w // 2 >= 600 and (x + w // 2) <= (self._frame_width - 600):
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
                        relative_angle = ((object_center_x - self._frame_center_x) / self._frame_width) * self._horizontal_fov
                        angle360 = (relative_angle + 360) % 360
                        lidar_angle_rad = self.map_cam_to_lidar(angle360)
                        try:
                            distance = self.lidar_node.get_distance(lidar_angle_rad)
                        except Exception as e:
                            print(f"Error getting distance: {e}")
                            distance = None
                            
                        width = self.find_width( distance, x, x+w)
                        if width is not None:
                            width = np.round(width,2)
                            '''
                            --------------------------------------------------------------------------------------------------------------------------
                            send out
                            -> angle360
                            -> width
                            -> distance
                            --------------------------------------------------------------------------------------------------------------------------
                            '''
                            result = [distance, width, angle360]
                            self.cylinder_detection = result
                            # print(f'{self.cylinder_detection=}')
                            '''
                            #node, distance, center_x, bbox_width, x, w
                            data = get_data(lidar_node, distance, object_center_x, object_width, x, w)
                            with open(csv_filename, mode='a', newline='') as file:
                                writer = csv.writer(file)
                                writer.writerow(data)
                            '''
                            
                            # Display information
                            if self.cv_window:
                                text = f"distance: {distance:.2f} m, angle: {angle360:.2f} deg, width: {width} m,"
                                cv2.putText(frame, text, (10, 720-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                                #print(distance, object_center_x, object_width, width)
                            



                    # Display result
                    if self.cv_window:
                        if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                            cv2.imshow(window_title, frame)
                        else:
                            break
                    
                        keyCode = cv2.waitKey(30) & 0xFF
                        if keyCode == 27 or keyCode == ord('q'):  # Exit on ESC or 'q'
                            break

            finally:
                self._video_capture.release()
                if self.cv_window:
                    cv2.destroyAllWindows()
        else:
            print("Unable to open camera")

if __name__ == "__main__":
    cv_class = CVMeasure(cv_window=True)
    
    try:
        cv_class.detect_yellow_cylinder()
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
    finally:
        rclpy.shutdown()
        
