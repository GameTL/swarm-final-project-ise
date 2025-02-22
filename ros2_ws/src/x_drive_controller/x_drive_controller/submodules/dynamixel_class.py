#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, tty, termios
from dynamixel_sdk import * # Uses Dynamixel SDK library
from math import sin, cos, pi, cos, radians, degrees
RADIANS45  = radians(45)
RADIANS135 = radians(135)
import numpy as np

mRAD2STEP = 0.085693
R = int(168/2) # (mm) Radius from the Robot_center to Wheel_center 
r = int(68/2) # (mm) Radius from the Robot_center to Wheel_center
V_motor_int_unsigned = [0,0,0,0]


# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 1.0 # Need this for AX-12

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
# DEVICENAME                  = '/dev/tty.usbmodem101'
# sudo chmod 777 /dev/tty.usbserial-AB0NRMLW # USE THIS FOR MAC & LINUX to allow port
# sudo chmod 777 /dev/tty.usbserial-AB0NRMLW # USE THIS FOR MAC & LINUX to allow port
PRINT = False
import platform
system = platform.system()
if system == "Darwin":
    print("The operating system is macOS.")
    DEVICENAME = '/dev/tty.usbserial-AB0NRMLW'  # Update this with your ports
elif system == "Linux":
    print("The operating system is Linux.")
    # DEVICENAME = '/dev/ttyUSB1'
    DEVICENAME = '/dev/ttyUSB1'

DISABLE = 0
ENABLE = 1
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_speed = [300, 800]         # Goal position

MAX_LINEAR_STEP = 330
MAX_ANGULAR_STEP = 213
STEP_LIMIT = MAX_LINEAR_STEP # MOVING SPEED LIMIT

#********* DYNAMIXEL Model definition *********
MY_DXL = 'AX-12'
DEFAULT_SPEED = 200

# Control table address
if MY_DXL == 'AX-12':
    # ADDR_TORQUE_ENABLE          = 64
    # ADDR_GOAL_POSITION          = 116
    # ADDR_PRESENT_POSITION       = 132
    ADDR_CW_ANGLE_LIMIT       = 6
    ADDR_CCW_ANGLE_LIMIT      = 8
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_MOVING_SPEED           = 32
    ADDR_TORQUE_LIMIT           = 34
    ADDR_PRESENT_POSITION       = 36
    DXL_MINIMUM_POSITION_VALUE  = 100   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000


# Class
class DynamixelInterface:
    def __init__(self, device_name=DEVICENAME, protocol_version=PROTOCOL_VERSION):
        self.portHandler    = PortHandler(device_name)            # Initialize PortHandler instance
        self.packetHandler  = PacketHandler(protocol_version)    # Initialize PacketHandler instance
        self.motors_id = [1,2,3,4]
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def write_register_only(self, dxl_id, address, value, size=2):
        if size == 1:
            self.packetHandler.write1ByteTxOnly(self.portHandler, dxl_id, address, value)
        elif size == 2:
            self.packetHandler.write2ByteTxOnly(self.portHandler, dxl_id, address, value)
        else:
            raise ValueError("Unsupported register size")
        return True
    
    def write_register(self, dxl_id, address, value, size=2):
        if size == 1:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, address, value)
        elif size == 2:
            result, error = self.packetHandler.write2ByteTxRx(self.portHandler, dxl_id, address, value)
        else:
            raise ValueError("Unsupported register size")
            
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
            return False
        elif error != 0:
            print("%s" % self.packetHandler.getRxPacketError(error))
            return False
        return True

    def read_register(self, dxl_id, address, size=2) -> int:
        """
        Read a register value from the Dynamixel
        size: 1 for 1 byte registers, 2 for 2 byte registers
        """
        if size == 1:
            value, result, error = self.packetHandler.read1ByteTxRx(self.portHandler, dxl_id, address)
        elif size == 2:
            value, result, error = self.packetHandler.read2ByteTxRx(self.portHandler, dxl_id, address)
        else:
            raise ValueError("Invalid size. Use 1 or 2")
            
        if result != COMM_SUCCESS:
            print(f"Failed to read address {address}: {self.packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"Error reading address {address}: {self.packetHandler.getRxPacketError(error)}")
        
        return value

    # Function to print key register values
    def print_motor_state(self, dxl_id):
        print("\n--- Motor State ---")
        print(f"ID: {self.read_register(dxl_id, 3, size=1)}")
        print(f"Baudrate: {self.read_register(dxl_id, 4, size=1)}")
        print(f"CW Angle Limit: {self.read_register(dxl_id, 6)}")
        print(f"CCW Angle Limit: {self.read_register(dxl_id, 8)}")
        print(f"Torque Enable: {self.read_register(dxl_id, ADDR_TORQUE_ENABLE, size=1)}")
        print(f"Moving Speed: {self.read_register(dxl_id, ADDR_MOVING_SPEED)}")
        print(f"Torque Limit: {self.read_register(dxl_id, ADDR_TORQUE_LIMIT)}")
        print(f"Present Position: {self.read_register(dxl_id, ADDR_PRESENT_POSITION)}")
        print(f"Present Speed: {self.read_register(dxl_id, 38)}")  # Speed is at address 38
        print(f"Present Load: {self.read_register(dxl_id, 40)}")
        print("----------------\n")
    
    def enable_all_motors(self):
        for dxl_id in self.motors_id:
            self.write_register(dxl_id, ADDR_TORQUE_ENABLE, ENABLE, 1)
        
    def disable_all_motors(self):
        for dxl_id in self.motors_id:
            self.write_register(dxl_id, ADDR_TORQUE_ENABLE, DISABLE, 1)
            
    def set_velocity_mode(self):
        for dxl_id in self.motors_id:
            self.write_register(dxl_id, ADDR_CW_ANGLE_LIMIT , 0, 2)
            self.write_register(dxl_id, ADDR_CCW_ANGLE_LIMIT, 0, 2)
        print("set to vel control")
            
    def set_torque(self, dxl_id, torque=500):
        # check 0 <= torque <= 1023
        if (torque < 0 or torque > 1023 or type(torque) != int):
            raise ValueError
        
        
        self.write_register(dxl_id, ADDR_TORQUE_LIMIT, torque, 2)
        if PRINT:
            print("torque: ", self.read_register(dxl_id ,ADDR_TORQUE_LIMIT, 2))
            
    def set_moving_speed(self, dxl_id, speed=0):
        # check 0 <= speed  <= 2047
        if (speed < 0 or speed > 2047 or type(speed) != int):
            raise ValueError(speed)
        
        self.write_register(dxl_id, ADDR_MOVING_SPEED, speed, 2)
        if PRINT:
            print("speed: ", self.read_register(dxl_id ,ADDR_MOVING_SPEED, 2))
        
    
    def testing_motors_moving_speed(self, speed1=300, speed2=1000):
        try:
            for dxl_id in self.motors_id:
                if PRINT:
                    print(dxl_id)
                self.write_register(dxl_id, ADDR_TORQUE_LIMIT, 500, 2)
            
            for dxl_id in self.motors_id:
                if PRINT:
                    print(dxl_id)
                # Write goal speed
                self.write_register(dxl_id, ADDR_MOVING_SPEED, speed1, 2)
                if PRINT:
                    print("speed: ", self.read_register(dxl_id ,ADDR_MOVING_SPEED, 2))
            time.sleep(4)
                
            for dxl_id in self.motors_id:
                if PRINT:
                    print(dxl_id)
                self.write_register(dxl_id, ADDR_MOVING_SPEED, speed2, 2)
                if PRINT:
                    print("speed: ", self.read_register(dxl_id, ADDR_MOVING_SPEED, 2))
            time.sleep(4)
            self.disable_all_motors()
        except KeyboardInterrupt:
            self.disable_all_motors()
            self.portHandler.closePort()
            
    def tf_speed(self, speed):
        # input -1023 to 1023 for easy 
        # output 0 to 2047, 1024 being neutal
        # return postive moving speed for memory address for both clockwise and anticlocker
        if int(speed) < 0: # if -ive
            return int(1024 - int(speed))
        return int(speed)
        
    def set_all_torque(self, torque=1023):
            self.set_torque(1, torque)
            self.set_torque(2, torque)
            self.set_torque(3, torque)
            self.set_torque(4, torque)
        
    def forward(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, speed)
            self.set_moving_speed(2, 1024+speed)
            self.set_moving_speed(3, 1024+speed)
            self.set_moving_speed(4, speed)
    
    def right(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, speed)
            self.set_moving_speed(2, speed)
            self.set_moving_speed(3, 1024+speed)
            self.set_moving_speed(4, 1024+speed)
    
    def backward(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, 1024+speed)
            self.set_moving_speed(2, speed)
            self.set_moving_speed(3, speed)
            self.set_moving_speed(4, 1024+speed)
    
    def left(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, 1024+speed)
            self.set_moving_speed(2, 1024+speed)
            self.set_moving_speed(3, speed)
            self.set_moving_speed(4, speed)
    
    def drive45(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, self.tf_speed(speed*1))
            self.set_moving_speed(2, self.tf_speed(0))
            self.set_moving_speed(3, self.tf_speed(-speed*1))
            self.set_moving_speed(4, self.tf_speed())
            
    # def drive(self, angle=45, speed=DEFAULT_SPEED, ang_speed=0 ): 
    #         speeds_trig = numpy.array( [+cos(radians(angle) - radians(RADIANS45)), +cos(radians(angle) - RADIANS135), -cos(radians(angle) - RADIANS45), -cos(radians(angle) - RADIANS135)]) * speed 
    #         speeds = [self.tf_speed(i) for i in speeds_trig]
    #         if PRINT:
    #             print(speeds)
    #         # print(f" finished cal at {(time.time_ns() - init_t)/1000000}ms")
    #         self.set_moving_speed(1, speeds[0])
    #         self.set_moving_speed(2, speeds[1])
    #         self.set_moving_speed(3, speeds[2])
    #         self.set_moving_speed(4, speeds[3])
    #         # print(f" finished transmit at {(time.time_ns() - init_t)/1000000}ms")
    def set_all_moving_speeds(self, speeds):
        for dxl_id, speed in zip(self.motors_id, speeds):
            self.write_register_only(dxl_id, ADDR_MOVING_SPEED, speed, 2)

    # def drive(self, angle=45, speed=0, ang_speed=0):
    #     # ang_speed * MAX_ANG
    #     ang_speed * 200
    #     # Precompute trigonometric values
    #     angle_rad = np.radians(angle)
    #     speeds_trig = np.array([
    #         +np.cos(angle_rad - RADIANS45),
    #         +np.cos(angle_rad - RADIANS135),
    #         -np.cos(angle_rad - RADIANS45),
    #         -np.cos(angle_rad - RADIANS135)
    #     ], dtype=np.float32)

    #     # Convert speeds to the correct format
    #     speeds = [self.tf_speed(int(s)) for s in speeds_trig]

    #     # Set all motor speeds at once
    #     self.set_all_moving_speeds(speeds)
    # def drive(self, angle=45, speed=DEFAULT_SPEED, ang_speed=0):
    #     # Precompute trigonometric values
    #     angle_rad = np.radians(angle)
    #     speeds_trig = (np.array([
    #         +np.cos(angle_rad - RADIANS45) ,
    #         +np.cos(angle_rad - RADIANS135),
    #         -np.cos(angle_rad - RADIANS45),
    #         -np.cos(angle_rad - RADIANS135)
    #     ]))* speed

    #     # Convert speeds to the correct format
    #     speeds = [self.tf_speed(int(s)) for s in speeds_trig]

    #     # Set all motor speeds at once
    #     self.set_all_moving_speeds(speeds)
    def drive(self, xDot, yDot, thetaDot):
        # print("----")
        if (not(xDot) or not(yDot)):
            magnitude_reducer = 1
        else:
            # print(xDot)
            # print(yDot)
            # print((abs(xDot)+abs(yDot)))
            # print(max(abs(xDot),abs(yDot)))
            magnitude_reducer = (abs(xDot)+abs(yDot))/max(abs(xDot),abs(yDot))
        # print(magnitude_reducer)
        # print("----")
        xDot       /= magnitude_reducer # (m/s)
        yDot       /= magnitude_reducer # (m/s) 
        Vin = np.array([[xDot],             # (mm/s)
                        [yDot],             # (mm/s)
                        [thetaDot]])*1000   # (mrad/s)
        arcJ = np.array([[-sin((5*pi)/4),  cos((5*pi)/4),  R],
                         [-sin((3*pi)/4),  cos((3*pi)/4),  R],
                         [-sin(pi/4),      cos(pi/4),      R],
                         [-sin((7*pi)/4),  cos((7*pi)/4),  R]]) / r
        Vomega = np.dot(arcJ,Vin)*2000 # (mrad/s)
        V_motor = Vomega*mRAD2STEP
        # V_motor_int = V_motor.astype
        
        for idx,num in enumerate(V_motor):
            # print(int(idx),num)
            
            V_motor_int_unsigned[idx] = (self.tf_speed(min(max(num,-STEP_LIMIT), STEP_LIMIT)))
        print(V_motor_int_unsigned)
        self.set_all_moving_speeds(V_motor_int_unsigned)
    
    def anticlockwise(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, 1024+speed,)
            self.set_moving_speed(2, 1024+speed,)
            self.set_moving_speed(3, 1024+speed,)
            self.set_moving_speed(4, 1024+speed,)
    
        
    def clockwise(self, speed=DEFAULT_SPEED):
            self.set_moving_speed(1, speed,)
            self.set_moving_speed(2, speed,)
            self.set_moving_speed(3, speed,)
            self.set_moving_speed(4, speed,)
    
            
    def circular_clockwise(self, speed=DEFAULT_SPEED):
        for i in range(180):
            self.drive(i*4)
            time.sleep(0.01)

if __name__ == "__main__":
    try:
        
        interface = DynamixelInterface()
        
        interface.disable_all_motors()
        interface.set_all_torque(1023)
        # self.set_velocity_mode()
        input("enter to start")
        # # self.testing_motors_moving_speed()
        # # Forward
        # # (2) +1 /  \ -1 (3)
        # # (1) +1 \  / -1 (4)
        # # while 1:
        # #     x = int(input("input:speed"))
        # #     print(self.set_moving_speed(2, speed=x, torque=500)) 
        while 1:
            # self.circular_clockwise(200, 200) 
            # self.drive(45, 200)
            # self.forward(int(input("inspeed: ")))
            interface.forward()
            # self.right()
            # time.sleep(1)
            # self.backward() 
            # time.sleep(1)
            # self.left()
            # time.sleep(1)
            # self.anticlockwise()
            # time.sleep(1)
            # self.clockwise()
            # time.sleep(1)
    except KeyboardInterrupt:
        interface.disable_all_motors()
        



""" 
# Control Table Address
ADDR_MODEL_NUMBER          = 0
ADDR_FIRMWARE_VERSION      = 2
ADDR_ID                   = 3
ADDR_BAUD_RATE            = 4
ADDR_RETURN_DELAY_TIME    = 5
ADDR_CW_ANGLE_LIMIT       = 6
ADDR_CCW_ANGLE_LIMIT      = 8
ADDR_TEMPERATURE_LIMIT    = 11
ADDR_MIN_VOLTAGE_LIMIT    = 12
ADDR_MAX_VOLTAGE_LIMIT    = 13
ADDR_MAX_TORQUE          = 14
ADDR_STATUS_RETURN_LEVEL  = 16
ADDR_ALARM_LED           = 17
ADDR_SHUTDOWN            = 18
ADDR_TORQUE_ENABLE       = 24
ADDR_LED                 = 25
ADDR_CW_COMPLIANCE_MARGIN = 26
ADDR_CCW_COMPLIANCE_MARGIN = 27
ADDR_CW_COMPLIANCE_SLOPE  = 28
ADDR_CCW_COMPLIANCE_SLOPE = 29
ADDR_GOAL_POSITION       = 30
ADDR_MOVING_SPEED        = 32
ADDR_TORQUE_LIMIT        = 34
ADDR_PRESENT_POSITION    = 36
ADDR_PRESENT_SPEED       = 38
ADDR_PRESENT_LOAD        = 40
ADDR_PRESENT_VOLTAGE     = 42
ADDR_PRESENT_TEMPERATURE = 43
ADDR_REGISTERED          = 44
ADDR_MOVING              = 46
"""