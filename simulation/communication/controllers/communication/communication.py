from controller import Robot, Emitter, Receiver, GPS

robot = Robot()
timestep = 64

# Print name
name = robot.getName()
print(f"Robot name: {name}")

# Get and enable GPS device
gps = robot.getDevice("gps")
gps.enable(timestep)

# Get Emitter and Receiver devices
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")

# Enable the receiver to listen for messages
receiver.enable(timestep)

# Define an interval for sending messages
message_interval = 2000
time_tracker = 0

while robot.step(timestep) != -1:
    # Implement time intervals
    time_tracker += timestep
    
    # Once time passes the interval
    if time_tracker >= message_interval:
        # Get the robot's current position
        current_position = gps.getValues()
        print(f"{name} position: {current_position}")
        
        # Send a message using the emitter
        message = f"{name} at {current_position[0]}, {current_position[1]}"
        emitter.send(message.encode('utf-8'))
        print(f"{name}'s emitter: {message}")
        
        # Reset the timer
        time_tracker = 0        

    # Receive messages from other robots
    while receiver.getQueueLength() > 0:
        received_message = receiver.getString()
        print(f"{name} received: {received_message}")
        receiver.nextPacket()
