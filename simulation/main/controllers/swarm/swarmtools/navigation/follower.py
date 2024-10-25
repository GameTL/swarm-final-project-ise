import random
import numpy as np

MAX_SPEED = 2

class RandomPolynomialFollower:
    def __init__(self, swarm_member, degree=6, timestep=64):
        self.degree = degree
        self.coefficients = [random.uniform(-1, 1) for _ in range(degree + 1)]
        self.timestep = timestep
        self.current_x = 0  # Start at x = 0
        self.swarm_member = swarm_member  # Reference to SwarmMember object


    def evaluate_polynomial(self, x):
        """Evaluate the polynomial at a given x."""
        y = sum(coef * (x ** i) for i, coef in enumerate(self.coefficients))
        return y

    def move_along_polynomial(self):
        """Make the robot follow the polynomial path."""
        # Increment x position
        self.current_x += 0.1  # Adjust step size as needed
        target_y = self.evaluate_polynomial(self.current_x)

        # Assume you have a GPS-like function to get the current position
        current_position = (self.swarm_member.gps.getValues()[0], self.swarm_member.gps.getValues()[2])
        target_position = (self.current_x, target_y)

        # Calculate direction to target
        direction_vector = (target_position[0] - current_position[0], 
                            target_position[1] - current_position[1])
        
        # Use direction to set wheel velocities
        if direction_vector[0] > 0:
            self.swarm_member.leftMotor.setVelocity(MAX_SPEED * 0.8)
            self.swarm_member.rightMotor.setVelocity(MAX_SPEED)
        elif direction_vector[0] < 0:
            self.swarm_member.leftMotor.setVelocity(MAX_SPEED)
            self.swarm_member.rightMotor.setVelocity(MAX_SPEED * 0.8)
        else:
            self.move_forward()

        # Adjust speed based on how far the robot is from the target
        distance = np.sqrt(direction_vector[0] ** 2 + direction_vector[1] ** 2)
        if distance < 0.1:  # If close to target, stop or slow down
            self.swarm_member.stop()

        # Print the polynomial equation for debugging
        # print(f"Polynomial: {' + '.join(f'{coef:.2f}*x^{i}' for i, coef in enumerate(self.coefficients))}")
        # print(f"Moving to target: ({self.current_x:.2f}, {target_y:.2f})")