import cv2
import numpy as np

ARUCO_DICT = {
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250 
} 

class Aruco():
    def __init__(self, map_size , aruco_dict = ARUCO_DICT):
        self.aruco_dict = aruco_dict
        self.aruco_type = self.aruco_dict.keys()
        self.current_data = dict()
        self.map_size = map_size

    def aruco_detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for i in self.aruco_type:
            arucoDict = cv2.aruco.getPredefinedDictionary(self.aruco_dict[i])

            arucoParams = cv2.aruco.DetectorParameters()

            corners, ids, rejected = cv2.aruco.ArucoDetector(arucoDict, arucoParams).detectMarkers(gray)
            detected_markers = self.aruco_display(corners, ids, rejected, frame)
        return detected_markers

    def aruco_display(self, corners, ids, rejected, image):
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

                dx = topRight[0] - topLeft[0]  # Difference in x
                dy = topRight[1] - topLeft[1]  # Difference in y
                theta_rad = np.arctan2(dy, -dx)  # Get angle in radians
                theta_deg = np.degrees(theta_rad)-90 # Convert to degrees
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                x = cX/image.shape[1]
                y = cY/image.shape[0]
                x_map = (x-0.5)*self.map_size[0]
                y_map = -(y-0.5)*self.map_size[0]
                print(f"[Inference] ArUco marker ID: {markerID} @ ({x_map}, {y_map})")
                self.current_data[str(markerID)] = (x, y, theta_deg)
                
        return image
    
if __name__ == "__main__":
    aruco = Aruco()
