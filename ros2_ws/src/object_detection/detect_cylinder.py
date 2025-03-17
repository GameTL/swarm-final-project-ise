import cv2
import numpy as np
import rclpy
import threading
from utils import LidarReader


class YellowCylinderDetection:
    def __init__(self, camera_id="/dev/video0", frame_width=1280, frame_height=720, horizontal_fov=67.85781564399836, fx=918.297472):
        # Camera properties
        self.camera_id = camera_id
        self.frame_width = frame_width
        self.frame_center_x = frame_width // 2
        self.frame_height = frame_height
        self.horizontal_fov = horizontal_fov
        self.fx = fx
        
        # LiDAR properties
        self.lidar_min_rad = -3.1241390705108643
        self.lidar_max_rad = 3.1415927410125732
        self.angle_increment = 0.0019344649044796824
        
        # Initialize video capture
        self.video_capture = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        self.video_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.video_capture.set(cv2.CAP_PROP_FPS, 30)

        # Window title for display
        self.window_title = "Yellow Cylinder Detection"

    def map_cam_to_lidar(self, angle_360):
        lidar_angle_rad = self.lidar_max_rad - (angle_360 / 360) * (self.lidar_max_rad - self.lidar_min_rad)
        return lidar_angle_rad

    def find_width(self, lidar_node, left, right):
        relative_left = ((left - self.frame_center_x) / self.frame_width) * self.horizontal_fov
        left_360 = (relative_left + 360) % 360
        left_angle_rad = self.map_cam_to_lidar(left_360)
        left_distance = lidar_node.get_distance(left_angle_rad)

        relative_right = ((right - self.frame_center_x) / self.frame_width) * self.horizontal_fov
        right_360 = (relative_right + 360) % 360
        right_angle_rad = self.map_cam_to_lidar(right_360)
        right_distance = lidar_node.get_distance(right_angle_rad)

        raw_angle_diff = abs(right_angle_rad - left_angle_rad)

        if raw_angle_diff > np.pi:
            angle_diff = np.pi * 2 - raw_angle_diff
        elif raw_angle_diff < -np.pi:
            angle_diff = np.pi * 2 + raw_angle_diff
        else:
            angle_diff = raw_angle_diff

        return np.sqrt(left_distance**2 + right_distance**2 - 2 * left_distance * right_distance * np.cos(angle_diff))

    def trig_angle(self, center_angle, left, right):
        relative_left = ((left - self.frame_center_x) / self.frame_width) * self.horizontal_fov
        left_360 = (relative_left + 360) % 360
        left_angle_rad = self.map_cam_to_lidar(left_360)

        relative_right = ((right - self.frame_center_x) / self.frame_width) * self.horizontal_fov
        right_360 = (relative_right + 360) % 360
        right_angle_rad = self.map_cam_to_lidar(left_360)

        raw_left_diff = abs(center_angle - left_angle_rad)
        if raw_left_diff > np.pi:
            left_diff = 2 * np.pi - raw_left_diff
        elif raw_left_diff < -np.pi:
            left_diff = 2 * np.pi + raw_left_diff
        else:
            left_diff = raw_left_diff

        raw_right_diff = abs(center_angle - right_angle_rad)
        if raw_right_diff > np.pi:
            right_diff = 2 * np.pi - raw_right_diff
        elif raw_right_diff < -np.pi:
            right_diff = 2 * np.pi + raw_right_diff
        else:
            right_diff = raw_right_diff

        return left_diff, right_diff

    def detect_yellow_cylinder(self, lidar_node):
        if self.video_capture.isOpened():
            try:
                cv2.namedWindow(self.window_title, cv2.WINDOW_AUTOSIZE)

                while True:
                    _, frame = self.video_capture.read()

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

                    for cnt in contours:
                        area = cv2.contourArea(cnt)
                        if area > 1000:  # Adjust minimum area as needed
                            x, y, w, h = cv2.boundingRect(cnt)
                            object_center_x = x + w // 2
                            object_width = w
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

                    if object_center_x is not None:
                        relative_angle = ((object_center_x - self.frame_center_x) / self.frame_width) * self.horizontal_fov
                        angle_360 = (relative_angle + 360) % 360
                        lidar_angle_rad = self.map_cam_to_lidar(angle_360)
                        distance = lidar_node.get_distance(lidar_angle_rad)
                        width = w / self.fx * distance
                        left_angle_diff, right_angle_diff = self.trig_angle(lidar_angle_rad, x, x + w)
                        text = f"distance: {distance:.3f}, width: {width:.3f}, {w}"
                        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    # Display result
                    if cv2.getWindowProperty(self.window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(self.window_title, frame)
                    else:
                        break

                    keyCode = cv2.waitKey(30) & 0xFF
                    if keyCode == 27 or keyCode == ord('q'):  # Exit on ESC or 'q'
                        break

            finally:
                self.video_capture.release()
                cv2.destroyAllWindows()
        else:
            print("Unable to open camera")


if __name__ == "__main__":
    rclpy.init()
    node = LidarReader()

    # Spin the node in a separate thread so it keeps receiving LiDAR data
    lidar_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    lidar_thread.start()

    # Create an instance of the YellowCylinderDetection class and start the detection
    detector = YellowCylinderDetection()
    detector.detect_yellow_cylinder(node)

    node.destroy_node()
    rclpy.shutdown()
