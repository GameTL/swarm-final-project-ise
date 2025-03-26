import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.subscription 
        self.latest_scan = None

    def lidar_callback(self, msg):
        self.latest_scan = msg  # Store the latest LiDAR data
        print(self.latest_scan)
        
    def find_angle(x, frame_width):   #cx is frame_width
        cx = frame_width/2
        cam_fov_rad = np.deg2rad(69.1674136)
        angle_offset = (x - cx) / cx * (cam_fov_rad/2)
        lidar_angle_rad = np.pi / 2 - angle_offset
        return lidar_angle_rad

    def get_distance(self, object_x, frame_width):
        if self.latest_scan is None:
            #print('None')
            return None, None

        # Extract LiDAR parameters
        angle_min = self.latest_scan.angle_min
        angle_max = self.latest_scan.angle_max
        angle_increment = self.latest_scan.angle_increment
        distances = np.array(self.latest_scan.ranges)
        
        lidar_angle_rad = self.find_angle(object_x, frame_width)

        angle_index = int((lidar_angle_rad - angle_min) / angle_increment)

        # Get distance at the computed index
        if 0 <= angle_index < len(distances):
            distance = distances[angle_index]
            return np.degrees(lidar_angle_rad), distance
        return None, None
