# Requirements

This simulation utilises three turtlebot burger with 5 distinct targets in a room. A camera is fixed to each turtlebot

## Target Constraints
1. 3 distinct target objects (color, shape, size)
2. simple geometry
3. flat bottom
4. spaces between object is not too small (haven't decided) 

## Room Constraints
1. has walls - 4 sides
2. contain some other stationary objects e.g. a chair, a dog
3. the simulation is rectangular plane

## Key Milestone
1. Detect "A Yellow Box" and get the position/dimension from RGB camera and depth cam (range-finder), no stereo cam available in webots
    - currently use 2 separate sensors: RGB camera and range-finder as depth cam
    - will try YOLO3D (deep learning model) for object detection with either current sensors combination or Kinect camera
  
### SOON - Classifying objects 
  - Other Robots | Objects
  - Other Robots | Objects | Targets
  - Classify Targets based on
    1. based on color
    2. based on shape
    3. based on size
    4. based on type e.g. box, furniture, etc
  - 6 DoF Pose Estimation
