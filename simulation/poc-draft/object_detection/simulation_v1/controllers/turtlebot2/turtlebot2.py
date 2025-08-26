from controller import Robot, Camera, Motor, RangeFinder

MAX_SPEED = 1

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#RGB camera
camera = robot.getDevice("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

#Range Finder as Depth camera
depthCam = robot.getDevice("range-finder")
depthCam.enable(timestep)

#turtlebot
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

#motion
def move_forward():
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    
def move_backward():
    leftMotor.setVelocity(-MAX_SPEED)
    rightMotor.setVelocity(-MAX_SPEED)
     
def left_spin():
    leftMotor.setVelocity(-0.7 * MAX_SPEED)
    rightMotor.setVelocity(0.7 * MAX_SPEED)

def right_spin():
    leftMotor.setVelocity(0.7 * MAX_SPEED)
    rightMotor.setVelocity(-0.7 * MAX_SPEED)

def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

#use range-finder to move robots toward goal so we can see the depth clearer:
def move_to_object():
    while True:
        distance = depthCam.getRangeImage()[0]  # Assuming index 0 gives the distance directly in front
        print(f"Distance to object: {distance:.2f} meters")
        
        if distance > 0.2:
            move_forward() 
        elif distance < 0.2:
            move_backward()
        else:
            stop()  # Stop if at the desired distance (0.5 meters)
            print("Reached desired distance from object (0.5 meters).")
            break  # Exit the function once the desired distance is reached


#rotating around until yellow cube is found
#stop and face towards the target, the target has to be in the center of the frame
#get data -> object size and distance from the robot
#three robot must do the same
while robot.step(timestep) != -1:
    leftMotor.setVelocity(MAX_SPEED*2)
    rightMotor.setVelocity(MAX_SPEED)
    
    number_of_objects = camera.getRecognitionNumberOfObjects()
    objects = camera.getRecognitionObjects()
    #print(f"\nRecognized {number_of_objects} objects.")
    
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()
    
    for obj in objects:
      color = obj.getColors()
      r, g, b = color[0], color[1], color[2]  # RGB values
      #print(f"Color: {r}, {g}, {b}")
      
      if r == 1.0 and g == 1.0 and b == 0.0:
          #print("Yellow cube found! Stopping the robot.")
          #leftMotor.setVelocity(0)
          #rightMotor.setVelocity(0)
          image_position = obj.getPositionOnImage()
          #print('[goal found!!!]')
          if image_position[0] < 35:
             left_spin() #left spin
          elif image_position[0] > 35:
             right_spin() #right spin
          else:
              stop()
#print([obj.getPosition(),obj.getOrientation(), obj.getSize()])
   