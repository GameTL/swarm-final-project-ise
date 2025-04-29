import cv2
import numpy as np
import pandas as pd
import json

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50
} 

class Aruco():
    def __init__(self, map_size, aruco_dict = ARUCO_DICT):
        self.aruco_dict = aruco_dict
        self.aruco_type = self.aruco_dict.keys()
        self.current_data = dict()
        self.map_size = map_size
        self.testing_data = pd.DataFrame(columns=["id", "timestamp", "x", "y", "theta"])
        '''
        self.communicator = Communicator() #create communicator object
        with open("../ros2_ws/src/collective_transport/collective_transport/collective_transport/submodules/p2p_communication/hosts.json", "r") as f:
            hosts_data = json.load(f)
        self.marker_to_robot = {
            int(key): value for key, value in hosts_data.items() if key.isdigit()
        } #maps my_marker_id to robot_address
        '''
        
    def aruco_detect(self, frame, current_timestamp):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for i in self.aruco_type:
            arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[i])

            arucoParams = cv2.aruco.DetectorParameters()

            corners, ids, rejected = cv2.aruco.ArucoDetector(arucoDict, arucoParams).detectMarkers(gray)
            detected_markers, recorded_data = self.aruco_display(corners, ids, rejected, frame, current_timestamp)
        return detected_markers, recorded_data

    def aruco_display(self, corners, ids, rejected, image, current_timestamp):
        recorded_data = dict()
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 0, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 0, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 0, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 0, 0), 2)

                dy = (topRight[0] - topLeft[0])/image.shape[1]  # Difference in x
                dx = (topRight[1] - topLeft[1])/image.shape[0]  # Difference in y

                display_theta_rad = np.arctan2(dx, -dy)  # Get angle in radians
                display_theta_deg = np.degrees(display_theta_rad)-90 # Convert to degrees

                actual_theta_rad = np.arctan2(dx, dy)
                actual_theta_deg = 360-(np.degrees(display_theta_rad)+180)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 0), -1)

                topMiddle = (int((topRight[0]+topLeft[0])//2), int((topRight[1]+topLeft[1])//2))
                middleRight = (int((topRight[0]+bottomRight[0])//2), int((topRight[1]+bottomRight[1])//2))
                cv2.line(image, (cX, cY), topMiddle, (0, 0, 255), 2)
                cv2.line(image, (cX, cY), middleRight, (0, 255, 0), 2)
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 2)
                x = cX/image.shape[1]
                y = cY/image.shape[0]
                x_map = (x-0.5)*self.map_size[0]
                y_map = -(y-0.5)*self.map_size[0]
                print(f"[Inference] ArUco marker ID: {markerID} @ ({round(x_map, 4)}m, {round(y_map, 4)}m, {actual_theta_deg} degree)")
                self.current_data[str(markerID)] = (x, y, display_theta_deg) 
                recorded_data[str(markerID)] = {
                    "x":x,
                    "y": y,
                    "theta": actual_theta_deg
                }
                self.testing_data.loc[len(self.testing_data)] =  {
                    "id": str(markerID),
                    "timestamp": current_timestamp,
                    "x":x,
                    "y": y,
                    "theta": actual_theta_deg
                }
        return image, recorded_data
    
if __name__ == "__main__":
    aruco = Aruco()
