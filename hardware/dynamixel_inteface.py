#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, tty, termios
from dynamixel_sdk import * # Uses Dynamixel SDK library
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def write_register(packetHandler, portHandler, dxl_id, address, value, size=2):
    if size == 1:
        result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, address, value)
    elif size == 2:
        result, error = packetHandler.write2ByteTxRx(portHandler, dxl_id, address, value)
    else:
        raise ValueError("Unsupported register size")
        
    if result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(result))
        return False
    elif error != 0:
        print("%s" % packetHandler.getRxPacketError(error))
        return False
    return True

def read_register(packetHandler, portHandler, dxl_id, address, size=2) -> int:
    """
    Read a register value from the Dynamixel
    size: 1 for 1 byte registers, 2 for 2 byte registers
    """
    if size == 1:
        value, result, error = packetHandler.read1ByteTxRx(portHandler, dxl_id, address)
    elif size == 2:
        value, result, error = packetHandler.read2ByteTxRx(portHandler, dxl_id, address)
    else:
        raise ValueError("Invalid size. Use 1 or 2")
        
    if result != COMM_SUCCESS:
        print(f"Failed to read address {address}: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"Error reading address {address}: {packetHandler.getRxPacketError(error)}")
    
    return value

# Function to print key register values
def print_motor_state(packetHandler, portHandler, dxl_id):
    print("\n--- Motor State ---")
    print(f"ID: {read_register(packetHandler, portHandler, dxl_id, 3, size=1)}")
    print(f"Baudrate: {read_register(packetHandler, portHandler, dxl_id, 4, size=1)}")
    print(f"CW Angle Limit: {read_register(packetHandler, portHandler, dxl_id, 6)}")
    print(f"CCW Angle Limit: {read_register(packetHandler, portHandler, dxl_id, 8)}")
    print(f"Torque Enable: {read_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_ENABLE, size=1)}")
    print(f"Moving Speed: {read_register(packetHandler, portHandler, dxl_id, ADDR_MOVING_SPEED)}")
    print(f"Torque Limit: {read_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_LIMIT)}")
    print(f"Present Position: {read_register(packetHandler, portHandler, dxl_id, ADDR_PRESENT_POSITION)}")
    print(f"Present Speed: {read_register(packetHandler, portHandler, dxl_id, 38)}")  # Speed is at address 38
    print(f"Present Load: {read_register(packetHandler, portHandler, dxl_id, 40)}")
    print("----------------\n")


DISABLE = 0
ENABLE = 1

#********* DYNAMIXEL Model definition *********
MY_DXL = 'AX-12'


# Control table address
if MY_DXL == 'AX-12':
    # ADDR_TORQUE_ENABLE          = 64
    # ADDR_GOAL_POSITION          = 116
    # ADDR_PRESENT_POSITION       = 132
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT        = 34
    ADDR_PRESENT_POSITION       = 36
    DXL_MINIMUM_POSITION_VALUE  = 100   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 1.0 # Need this for AX-12

# Factory default ID of all DYNAMIXEL is 1
dxl_id                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
# DEVICENAME                  = '/dev/ttyUSB0'
# DEVICENAME                  = '/dev/tty.usbmodem101'
# sudo chmod 777 /dev/tty.usbserial-AB0NRMLW # USE THIS FOR MAC & LINUX to allow port
DEVICENAME = '/dev/tty.usbserial-AB0NRMLW'  # Update this with your port

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

index = 0
dxl_goal_speed = [300, 800]         # Goal position

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
for dxl_id in range(1,5):
    print(dxl_id)
    write_register(packetHandler, portHandler, dxl_id, ADDR_MOVING_SPEED, 0, 2)

# Use it in your main code:
try:
    
    while 1:
        for dxl_id in range(1,5):
            print(dxl_id)
            write_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_LIMIT, 500, 2)
        
        for dxl_id in range(1,5):
            print(dxl_id)
            write_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_LIMIT, 500, 2)
            # Write goal speed
            write_register(packetHandler, portHandler, dxl_id, ADDR_MOVING_SPEED, 200, 2)
            print("speed: ",read_register(packetHandler, portHandler, dxl_id ,ADDR_MOVING_SPEED, 2))
        time.sleep(4)
            
        for dxl_id in range(1,5):
            print(dxl_id)
            write_register(packetHandler, portHandler, dxl_id, ADDR_MOVING_SPEED, 800, 2)
            print("speed: ",read_register(packetHandler, portHandler, dxl_id, ADDR_MOVING_SPEED, 2))
        time.sleep(4)
except KeyboardInterrupt:
    for dxl_id in range(1,5):
        write_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_ENABLE, DISABLE, 1)
    # Close port
    portHandler.closePort()

# Disable Dynamixel Torque
for dxl_id in range(1,5):
    write_register(packetHandler, portHandler, dxl_id, ADDR_TORQUE_ENABLE, DISABLE, 1)

# Close port
portHandler.closePort()


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