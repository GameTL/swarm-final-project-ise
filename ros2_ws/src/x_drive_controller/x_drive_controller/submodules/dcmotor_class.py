import serial
import time
import numpy as np
from math import sin, cos, pi, cos, radians, degrees
RADIANS45  = radians(45)
RADIANS135 = radians(135)
from numpy.linalg import pinv

mRAD2STEP = 0.085693
R = int(168/2) # (mm) Radius from the Robot_center to Wheel_center 
r = int(68/2) # (mm) Radius from the Robot_center to Wheel_center
V_motor_int_unsigned = [0,0,0,0]

class DCMotorInterface:

    WHEEL_RADIUS = 0.03  # meters, example value; adjust as needed

    def __init__(self, port="/dev/ttyTHS1", baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
    
    def set_speed(self, motor_id:int, speed:int):
        cmd = f"SET_SPEED {motor_id} {speed}"
        self.ser.write(cmd.encode('utf-8'))
        ack = self.ser.readline().decode('utf-8').strip()
        if ack != "OK":
            print(f"Warning: No OK, got: {ack}")

    def set_target_rads(self, motor_id: int, rad_speeds: int):
        cmd = f"SET_RADS {motor_id} {rad_speeds}"
        self.ser.write(cmd.encode('utf-8'))
        ack = self.ser.readline().decode('utf-8').strip()
        if ack != "OK":
            print(f"Warning: No OK, got: {ack}")

    def set_target_rads(self, rad_speeds: list):
        """
        Send target wheel speeds in rad/s for all 4 motors together.

        rad_speeds: list of 4 floats [w1, w2, w3, w4]
        Motor 1 -> Front-Left
        Motor 2 -> Front-Right
        Motor 3 -> Rear-Right
        Motor 4 -> Rear-Left
        """
        if len(rad_speeds) != 4:
            raise ValueError("rad_speeds must be a list of 4 elements.")

        # Create the UART command
        cmd = f"SET_RADS {' '.join(str(r) for r in rad_speeds)}\n"
        self.ser.write(cmd.encode('utf-8'))

        # Read acknowledgment
        ack = self.ser.readline().decode('utf-8').strip()
        if ack != "OK":
            print(f"Warning: No OK, got: {ack}")
    
    def get_encoder(self, motor_id:int):
        cmd = f"GET_ENCODER DATA {motor_id}"
        self.ser.write(cmd.encode('utf-8'))
        response = self.ser.readline().decode('utf-8').strip()
        try:
            ticks = int(response)
            return ticks
        except ValueError:
            print(f"Invalid response: {response}")
            return None
    def get_wheel_angles(self):
        """
        Reads the current wheel angles in radians from the ESP32 for all 4 wheels.
        Returns a list of 4 floats.
        """
        angles = []
        for motor_id in range(1, 5):
            cmd = f"GET_ANGLE {motor_id}\n"
            self.ser.write(cmd.encode('utf-8'))
            response = self.ser.readline().decode('utf-8').strip()
            try:
                angle = float(response)
                angles.append(angle)
            except ValueError:
                print(f"Invalid angle response for motor {motor_id}: {response}")
                angles.append(0.0)
        return angles

    def get_robot_velocity(self, previous_angles, last_time):
        current_angles = self.get_wheel_angles()  # new method you must implement separately
        now = time.time()
        dt = now - last_time

        delta_theta = [curr - last for curr, last in zip(current_angles, previous_angles)]
        delta_s = [self.WHEEL_RADIUS * dtheta for dtheta in delta_theta]

        vx, vy, omega = self.inverse_kinematics(*delta_s)

        return np.array([vx, vy, omega]), current_angles, now
    
    def forward_kinematics(self, vx, vy, omega):
        # Robot parameters (adjust as needed)
        r = 0.03  # wheel radius (meters)
        R = 0.1   # distance from center to wheel (meters)

        # Convert robot velocity into individual wheel speeds using X-drive forward kinematics
        Vx = vx * 1000  # Convert m/s to mm/s if needed
        Vy = vy * 1000
        Wz = omega * 1000  # rad/s to mrad/s

        arcJ = np.array([
            [-np.sin(5 * np.pi / 4),  np.cos(5 * np.pi / 4),  R],
            [-np.sin(3 * np.pi / 4),  np.cos(3 * np.pi / 4),  R],
            [-np.sin(1 * np.pi / 4),  np.cos(1 * np.pi / 4),  R],
            [-np.sin(7 * np.pi / 4),  np.cos(7 * np.pi / 4),  R]
        ]) / r

        Vin = np.array([[Vx], [Vy], [Wz]])
        wheel_speeds = np.dot(arcJ, Vin).flatten()

        # Optionally clip and convert to int PWM values here
        wheel_speeds = np.clip(wheel_speeds, -255, 255).astype(int)

        for motor_id, speed in enumerate(wheel_speeds, start=1):
            self.set_speed(motor_id, int(speed))

    def inverse_kinematics(self, v1=0, v2=0, v3=0, v4=0):
        V_motor = np.array([[v1], [v2], [v3], [v4]])
        V_omega = V_motor / mRAD2STEP  # This converts back to rad/s or mm/s
        V_omega = V_omega / 2000
        inv_arcJ = pinv(np.array([[-sin((5*pi)/4),  cos((5*pi)/4),  R],
                                [-sin((3*pi)/4),  cos((3*pi)/4),  R],
                                [-sin(pi/4),      cos(pi/4),      R],
                                [-sin((7*pi)/4),  cos((7*pi)/4),  R]]) / r)
        Vin = np.dot(inv_arcJ, V_omega)  
        Vin = Vin / 1000  # Adjust for units properly
        return Vin[0], Vin[1], Vin[2]
    
    def close(self):
        self.ser.close()
