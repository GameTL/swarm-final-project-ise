from controller import Robot, Camera, Motor, Display, Supervisor
import cv2
import numpy as np
from swamtools.vision.ObjectDetector import ObjectDetector
from swamtools.communication.Communicator import Communicator
import asyncio

MAX_SPEED = 2
detected_flag = False

robot = Robot()
print(f"{robot.getName()}: finding yellow object")
robot_position = {
    "x": 0.0,
    "y": 0.0,
    "theta": 0.0
}
cylinder_position = {
    "x": 0.85,
    "y": -0.1,
    "theta": 0.0
}
# print(robot.__dict__)
timestep = int(robot.getBasicTimeStep())

object_detector = ObjectDetector(robot)
communicator = Communicator(robot) 

# motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# motion
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

def simple_movement():
    global detected_flag
    communicator.send_position(robot_position)
    
    # listen portion
    while robot.step(timestep) != -1:
        status = communicator.listen_to_message() # check for incoming messages 
        if status == "stop":
            stop()
            break
        if object_detector.detect() and not detected_flag:
            communicator.broadcast_message("[ObjectDetected]", (0, 0, 0))
            detected_flag = True # detect once and top
            stop()
            break
        
        leftMotor.setVelocity(MAX_SPEED*0.5)
        rightMotor.setVelocity(MAX_SPEED)



async def main():
    #task1 = asyncio.create_task(listening())
    # while 1:
    simple_movement()

asyncio.run(main())
