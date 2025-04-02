from controller import Robot
import math

TIME_STEP = 64
MAX_SPEED = 6.28

# Create the Robot instance
robot = Robot()

# Motors
left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Encoders
left_encoder = robot.getDevice("left_wheel_sensor")
right_encoder = robot.getDevice("right_wheel_sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# LIDAR
lidar = robot.getDevice("LDS-01")
lidar.enable(TIME_STEP)

# Odometry variables
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
WHEEL_RADIUS = 0.033   # meters
WHEEL_BASE   = 0.160   # meters

# Previous encoder placeholders
prev_left_enc = None
prev_right_enc = None

# State machine
FORWARD = 0
AVOID   = 1
current_state = FORWARD

# Distance thresholds
AVOID_DISTANCE = 0.50  # Start avoiding if < 0.5 m in front

# Speeds
forward_speed = 0.6 * MAX_SPEED  # move forward at 40% speed
turning_speed = 0.6 * MAX_SPEED  # turn in place at 60% speed

# 90° turn angle
TURN_ANGLE = math.pi / 2.0  # (radians) = 90 degrees

# We'll store the heading when we start turning
avoid_start_theta = None

# ----------------------------------------------------------------
# Helper: update_odometry()
# ----------------------------------------------------------------
def update_odometry(left_enc, right_enc, prev_left_enc, prev_right_enc, x, y, theta):
    """ differential-drive odometry."""
    delta_left = (left_enc - prev_left_enc) * WHEEL_RADIUS
    delta_right = (right_enc - prev_right_enc) * WHEEL_RADIUS

    delta_center = (delta_left + delta_right) / 2.0
    delta_theta  = (delta_right - delta_left) / WHEEL_BASE

    new_x = x + delta_center * math.cos(theta)
    new_y = y + delta_center * math.sin(theta)
    new_theta = theta + delta_theta

    return new_x, new_y, new_theta

# ----------------------------------------------------------------
# Helper function  tat ca;dfasj': angle_difference() to keep angles in [-pi, +pi]
# ----------------------------------------------------------------
def angle_difference(a, b):
    diff = a - b
    # Standard wrap to [-pi, +pi]
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff <= -math.pi:
        diff += 2.0 * math.pi
    return diff

# ----------------------------------------------------------------
# 1. Wait for valid encoder values once
# ----------------------------------------------------------------
while robot.step(TIME_STEP) != -1:
    l_val = left_encoder.getValue()
    r_val = right_encoder.getValue()
    if not math.isnan(l_val) and not math.isnan(r_val):
        prev_left_enc = l_val
        prev_right_enc = r_val
        print("Encoders initialized. Starting main loop...")
        break

# ----------------------------------------------------------------
# Main Control Loop
# ----------------------------------------------------------------
import os

# Open the .clf file for writing
log_file_path = "robot_log.clf"
if os.path.exists(log_file_path):
    os.remove(log_file_path)  

log_file = open(log_file_path, "w")

# Write the header to the .clf file
log_file.write("# message name [message contents] ipc timestamp ipc hostname logger_timestamp\n")
log_file.write("# message formats defined: PARAM SYNC ODOM FLASER TRUEPOS\n")

#ODOM data log
def log_odometry(file, timestamp, x, y, theta, linear_velocity, angular_velocity):
    file.write(
        f"ODOM {timestamp:.2f} {x:.6f} {y:.6f} {theta:.6f} "
        f"{linear_velocity:.6f} {angular_velocity:.6f} 0.000000 0.000000 host1 {timestamp:.6f}\n"
    )

#timestamp = robot.getTime()

#FLASER  DAta log
def log_flaser(
    file,
    num_rays,
    ranges,                
    end_val1, end_val2, end_val3,  
    end_val4, end_val5, end_val6,  
    big_timestamp,
    small_timestamp
):
    ranges_str = " ".join(f"{r:.2f}" for r in ranges)



    file.write(
        "FLASER "
        f"{num_rays} "
        f"{ranges_str} "   # the 180 range readings
        f"{end_val1:.6f} {end_val2:.6f} {end_val3:.6f} "
        f"{end_val4:.6f} {end_val5:.6f} {end_val6:.6f} "
        f"{big_timestamp:.6f} host1 {small_timestamp:.6f}\n"
    )




# Main Control Loop 
while robot.step(TIME_STEP) != -1:
    # Current timestamp 
    current_time = robot.getTime()

    # ---------------- Odometry Update ----------------
    current_left_enc = left_encoder.getValue()
    current_right_enc = right_encoder.getValue()
    
    # Calculate deltas using encoder readings
    delta_left = (current_left_enc - prev_left_enc) * WHEEL_RADIUS
    delta_right = (current_right_enc - prev_right_enc) * WHEEL_RADIUS

    robot_x, robot_y, robot_theta = update_odometry(
        current_left_enc, current_right_enc,
        prev_left_enc, prev_right_enc,
        robot_x, robot_y, robot_theta
    )

    prev_left_enc = current_left_enc
    prev_right_enc = current_right_enc
    

    # ---------------- LIDAR Read ----------------
    lidar_data = lidar.getRangeImage()
    lidar_data = [min(7.0, r) for r in lidar_data]  
    #reduced_lidar_data = lidar_data[:180]  
    num_lidar_rays = len(lidar_data)
    if num_lidar_rays >= 180:
        center_index = num_lidar_rays // 2  # Center of the scan
        half_width = 90  
        reduced_lidar_data = lidar_data[center_index - half_width : center_index + half_width]
    else:
        #print(f"Warning: LIDAR returned only {num_lidar_rays} readings, expected 180+")
        reduced_lidar_data = lidar_data[:180]  
    num_rays = len(reduced_lidar_data)
    front_index = num_rays // 2
    half_width = 10
    start_i = max(0, front_index - half_width)
    end_i = min(num_rays - 1, front_index + half_width)
    min_front_distance = min(reduced_lidar_data[start_i:end_i + 1])
    if robot.getTime() < 0.2:  
        print(f"First LIDAR scan: {lidar.getRangeImage()[:10]}")  


    # ---------------- State Machine ----------------
    if current_state == FORWARD:
        if min_front_distance < AVOID_DISTANCE:
            current_state = AVOID
            avoid_start_theta = robot_theta
        else:
            left_motor.setVelocity(forward_speed)
            right_motor.setVelocity(forward_speed)
    elif current_state == AVOID:
        turned_angle = abs(angle_difference(robot_theta, avoid_start_theta))
        if turned_angle >= TURN_ANGLE:
            current_state = FORWARD
        else:
            left_motor.setVelocity(-turning_speed)
            right_motor.setVelocity(turning_speed)
            
    # ---------------- Velocity Calculation ----------------
    # Calculate linear and angular velocities
    linear_velocity = (delta_left + delta_right) / 2.0 / (TIME_STEP / 1000.0)  # Convert TIME_STEP to seconds
    angular_velocity = (delta_right - delta_left) / WHEEL_BASE / (TIME_STEP / 1000.0)
    
    # ---------------- Logging ----------------
    # Log odometry (pose and velocities)
    log_odometry(log_file, current_time, robot_x, robot_y, robot_theta, linear_velocity, angular_velocity)
    #log_odometry(log_file, current_time, robot_x, robot_y, robot_theta)
    log_flaser(
        file=log_file,
        num_rays=num_rays,
        ranges=reduced_lidar_data,
        end_val1=robot_x, end_val2=robot_y, end_val3=robot_theta,
        end_val4=robot_x, end_val5=robot_y, end_val6=robot_theta,
        big_timestamp=current_time,
        small_timestamp=current_time
    )
    
    #print(f"Left Encoder: {current_left_enc}, Right Encoder: {current_right_enc}")
    #print(f"Delta Left: {delta_left}, Delta Right: {delta_right}")


    #Debug Output (Optional)
    #lidar_data = lidar.getRangeImage()
    #print(f"LIDAR Data: {lidar_data}")

    #print(f"[State: {current_state}] X={robot_x:.2f}, Y={robot_y:.2f}, "
          #f"Theta={robot_theta:.2f}, minFront={min_front_distance:.2f}"
         # f"Linear velocity = {linear_velocity:.2f}, Angular Velocity = {angular_velocity:.2f}")
     

# Close the .clf file after the loop ends
log_file.close()





