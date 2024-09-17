from controller import Robot, Emitter, Receiver, GPS

class SwarmCommunicator:
    def __init__(self):
        # Instantiate the robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get and enable GPS device
        self.name = self.robot.getName()
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.current_position = self.gps.getValues()
        
        # Get Emitter and Receiver devices
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        
        self.message = ""
        
        # Enable the receiver to listen for messages
        self.receiver.enable(self.timestep)
        
        self.message_interval = 2000
        self.time_tracker = 0
        
    def run(self):
        """Main function to run the communication modules"""
        while self.robot.step(self.timestep) != -1:
            # Implement time intervals
            self.time_tracker += self.timestep
            
            # Once time passes the interval
            if self.time_tracker >= self.message_interval:
                # Get the robot's current position
                self.current_position = self.gps.getValues()
                print(f"{self.name} position: {self.current_position}")
                
                # Send a message using the emitter
                self.message = f"{self.name} at {self.current_position[0]}, {self.current_position[1]}"
                self.emitter.send(self.message.encode('utf-8'))
                print(f"{self.name}'s emitter: {self.message}")
                
                # Reset the timer
                self.time_tracker = 0    
                
            self.handle_message()
                    
    def handle_message(self):
        """Receive messages from other robots"""
        while self.receiver.getQueueLength() > 0:
            self.received_message = self.receiver.getString()
            print(f"{self.name} received: {self.received_message}")
            self.receiver.nextPacket()
        
if __name__ == "__main__":
    robot_communication_controller = SwarmCommunicator()
    robot_communication_controller.run()
