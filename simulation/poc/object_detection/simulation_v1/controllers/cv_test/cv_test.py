from controller import Robot, Camera, Motor, Display
import cv2
import numpy as np

MAX_SPEED = 2

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# RGB camera
camera = robot.getDevice("camera")
camera.enable(timestep)

# motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# display
display = robot.getDevice("display")
display_w = display.getWidth()
display_h = display.getHeight()
display.attachCamera(camera)

def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

#ADD THIS------------------------------------------------------------------------------------
# Known values (example):
REAL_OBJ_WIDTH = 0.15  # meters (real object width)
KNOWN_DISTANCE = 1.0  # meters (known distance)
KNOWN_WIDTH_IN_IMG = 24  # pixels (width in image at known distance)
#--------------------------------------------------------------------------------------------

# Calculate focal length
focal_length = (KNOWN_WIDTH_IN_IMG * KNOWN_DISTANCE) / REAL_OBJ_WIDTH

# main loop
while robot.step(timestep) != -1:
    display.setAlpha(0.0)
    display.fillRectangle(0,0,display_w, display_h)
    leftMotor.setVelocity(MAX_SPEED * 0.5)
    rightMotor.setVelocity(MAX_SPEED)
    
    # Get the image from the camera
    img = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Using HSV color space to segment the image with mask
    lower = np.array([70, 100, 0])
    upper = np.array([90, 255, 255])
    mask = cv2.inRange(img, lower, upper)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)
        largest_contour_center = cv2.moments(largest_contour)
        if largest_contour_center['m00'] > 0:
            cx = int(largest_contour_center['m10'] / largest_contour_center['m00'])
            cy = int(largest_contour_center['m01'] / largest_contour_center['m00'])
        
        # Bounding box - draw in the display 
        x, y, w, h = cv2.boundingRect(largest_contour)
        display.setAlpha(1.0)
        display.drawRectangle(x, y, w, h)
        
        d_left = x
        d_right = camera.getWidth() - (x + w)
                
        if abs(d_left - d_right) <= 1:
            stop()
#Add this ---------------------------------------------------------------------------------------------------------
        # Calculate the real distance using the focal length approach
        #distance from robot to outermost surface (max radius)
        real_distance = (REAL_OBJ_WIDTH * focal_length) / w
        print(f"Real Distance to the surface: {real_distance:.2f} meters")
        #print(w,h)
#------------------------------------------------------------------------------------------------------------------
