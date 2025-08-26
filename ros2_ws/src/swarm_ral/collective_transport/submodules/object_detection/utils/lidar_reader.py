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
        self.latest_scan = None

    def lidar_callback(self, msg):
        self.latest_scan = msg  # Store the latest LiDAR data
    
    def all_distances(self):
        return np.array(self.latest_scan.ranges)

    def get_distance(self, rad):
        if self.latest_scan is None:
            return None
        
        angle_min = np.radians(self.latest_scan.angle_min)
        angle_max = np.radians(self.latest_scan.angle_max)
        angle_increment = np.radians(self.latest_scan.angle_increment)
        distances = np.array(self.latest_scan.ranges)
        
        lidar_angle = rad
        
        if not (self.latest_scan.angle_min <= lidar_angle <= self.latest_scan.angle_max):
            print("out of LiDAR Range")
            return None
        
        index = int((lidar_angle - self.latest_scan.angle_min) / self.latest_scan.angle_increment)
        
        if 0 <= index < len(distances):
            return distances[index]
        
        return None 
        
    '''    
    ------------------------------------------------------------------------------------------------------------   
    sensor_msgs.msg.LaserScan(
         header=std_msgs.msg.Header(
              stamp=builtin_interfaces.msg.Time(sec=1741612612, nanosec=692294058), frame_id='laser'), 
              angle_min=-3.1241390705108643, 
              angle_max=3.1415927410125732, 
              angle_increment=0.0019344649044796824, 
              time_increment=2.9911951060057618e-05, 
              scan_time=0.09688480943441391, 
              range_min=0.15000000596046448, 
              range_max=40.0
    ------------------------------------------------------------------------------------------------------------   
    '''   

        
    '''
    
    SPARE
    
    
    def find_lidar_index(self, box_x_center, frame_width):
        cx = frame_width / 2
        cam_fov_rad = np.deg2rad(69.1674136)
        angle_offset = (box_x_center - cx) / cx * (cam_fov_rad / 2)
        lidar_angle_rad = np.pi / 2 - angle_offset
        
        # Convert LiDAR angle to index
        if self.latest_scan:
            angle_min = self.latest_scan.angle_min
            angle_increment = self.latest_scan.angle_increment
            index = int((lidar_angle_rad - angle_min) / angle_increment)
            return index, np.degrees(lidar_angle_rad)
        return None, None
    '''

