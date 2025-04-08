import turtle

class ArucoVisual():
    def __init__(self, width, height):
        self.wn = turtle.Screen()
        self.wn.bgcolor("white")

        # Set window size and world coordinates to center the screen
        self.width = width
        self.height = height
        self.wn.setup(width=width + 100, height=height + 100)  # Add padding
        self.wn.setworldcoordinates(-width // 2, -height // 2, width // 2, height // 2)
        
        self.turtle_dict = {}  # Stores robots
        self.text_turtle_dict = {}  # Stores text turtles

        self.reset_map_status = False
    
    def reset_map(self):
        self.reset_map_status = True

    def clear_map(self):
        for robot in self.turtle_dict.values():
            robot.clear()
            robot.hideturtle()
            del robot
        self.turtle_dict.clear()

        # Clear and remove all text turtles
        for text_turtle in self.text_turtle_dict.values():
            text_turtle.clear()
            text_turtle.hideturtle()
            del text_turtle
        self.text_turtle_dict.clear()
        self.reset_map_status = False

    def update_map(self, current_data):
        if self.reset_map_status:
            self.clear_map()
        if not current_data:
            return

        # Create a temporary list to store updates
        update_turtles = []

        # Iterate over the current data
        for robot_id, (x, y, theta) in current_data.items():
            x_scaled = (x - 0.5) * self.width  # Centering x
            y_scaled = (0.5 - y) * self.height  # Centering y (flipping y-axis)

            update_turtles.append((robot_id, x_scaled, y_scaled, theta))

        # Now apply the updates after iteration
        for robot_id, x_scaled, y_scaled, theta in update_turtles:
            # Update the robot position or create a new one if it doesn't exist
            if robot_id not in self.turtle_dict:
                # Create a new turtle for the robot
                self.turtle_dict[robot_id] = turtle.Turtle()
                self.turtle_dict[robot_id].speed(0)
                self.turtle_dict[robot_id].setheading(90)
                self.turtle_dict[robot_id].penup()  # Avoid drawing a line to start position
                self.turtle_dict[robot_id].setpos(x_scaled, y_scaled)
                self.turtle_dict[robot_id].pendown()
            else:
                # Move existing turtle
                self.turtle_dict[robot_id].goto(x_scaled, y_scaled)
                self.turtle_dict[robot_id].setheading(theta)

            # Create or update text turtle for ID
            if robot_id not in self.text_turtle_dict:
                self.text_turtle_dict[robot_id] = turtle.Turtle()
                self.text_turtle_dict[robot_id].hideturtle()
                self.text_turtle_dict[robot_id].penup()

            text_turtle = self.text_turtle_dict[robot_id]
            text_turtle.clear()  # Clear previous text before writing new
            text_turtle.goto(x_scaled, y_scaled + 10)  # Offset text above robot
            text_turtle.write(f"ID: {robot_id}", align="center", font=("Arial", 12, "bold"))