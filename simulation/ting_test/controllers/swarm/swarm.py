from controller import Robot, Camera, Motor, Display
import cv2
import numpy as np
from swamtools.vision.ObjectDetector import ObjectDetector
from swamtools.communication.Communicator import Communicator

MAX_SPEED = 2

robot = Robot()

timestep = int(robot.getBasicTimeStep())

object_detector = ObjectDetector(robot)

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

# main loop
while robot.step(timestep) != -1:
    leftMotor.setVelocity(MAX_SPEED*0.5)
    rightMotor.setVelocity(MAX_SPEED)
    if object_detector.detect():
        stop() 
            