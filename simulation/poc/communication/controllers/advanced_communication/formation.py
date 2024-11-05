import math
import json
from rich.pretty import pprint


class FormationMaster:
    def __init__(self, robot, current_coords: dict, object_coords: tuple, obstacles: dict, radius=0.3, fineness=2, verbose=False):
        """ 
        current_coords: assumed that data are in this order [('TurtleBot3Burger_1', [1.3787268144635236, -0.38032573186548774, 0.0]), ('TurtleBot3Burger_2', [0.07806162991058642, 0.8007404816156147, 0.0])]
        current_coords: assumed that data are in this order [robot1,robot2,robot3] -> [5,6]
        """
        self.robot = robot
        
        # Retrieve parameters
        self.current_coords = current_coords
        self.member_list = list(self.current_coords.keys())
        self.member_count = len(self.member_list)
        self.object_coords = object_coords
        self.obstacles = list(map(lambda x: (round(x[0], fineness), round(x[1], fineness)), obstacles))
        
        # Configurations
        self.radius = radius
        self.fineness = fineness
        self.verbose = verbose
        self.go_around = False
        self.previous_movement = ""

        # Initialize calculation parameters
        self.target_coords = []
        # Schema:
        # {
        #   "robot1": {
        #       0: (x1, y1),
        #       1: (x2, y2),
        #   },
        #   "robot2": {
        #       0: (x1, y1),
        #       1: (x2, y2),
        #       2: (x3, y3),
        #   },
        # }
        self.paths = {}
        # Schema:
        # {
        #   0: [(x1, y1), (x2, y2)],
        #   1: [(x1, y1), (x2, y2), (x3, y3)],
        # }
        self.conflict_map = {}

    def calculate_target_coords(self):
        # Calculate target coords based on radius around the object coordinates
        internal_angle = 2 * math.pi / self.member_count
        for i in range(self.member_count):
            # x' = x + rcos(theta * i) 
            x = self.object_coords[0] + (self.radius * math.cos(internal_angle * i))
            # y' = y + rsin(theta * i)
            y = self.object_coords[1] + (self.radius * math.sin(internal_angle * i))

            self.target_coords.append((x, y))
        
        # Round to self.fineness digits
        self.target_coords = list(map(lambda x: (round(x[0], self.fineness), round(x[1], self.fineness)), self.target_coords))

        # Match closest start to target
        self.match_coords()

    def calculate_distance(self, current, end):
        return round(math.sqrt((current[0] - end[0]) ** 2 + (current[1] - end[1]) ** 2), self.fineness)
    
    def match_coords(self):
        print("# ===== Matching coordinates ===== #")
        distances = {}
        for member in self.member_list:
            # Compute distance from each robot to targets
            for target in self.target_coords:
                if member in distances:
                    distances[member].append(self.calculate_distance(self.current_coords[member][0:2], target))
                else:
                    distances[member] = [self.calculate_distance(self.current_coords[member][0:2], target)]
        
        # Finding the optimal matches
        self.matches = {}
        assigned_targets = set() 

        # Sort robots by minimum distance to targets to prioritize closest matches
        for member in sorted(distances.keys(), key=lambda m: min(distances[m])):
            # Find the closest unassigned target for this robot
            for i, _ in sorted(enumerate(distances[member]), key=lambda x: x[1]):
                if i not in assigned_targets:
                    self.matches[member] = i
                    assigned_targets.add(i)
                    break

        for member, order in self.matches.items():
            self.matches[member] = self.target_coords[order]
        
        pprint(f"Matches: {self.matches}")

    def plan_paths(self):
        print("# ===== Calculating paths ===== #")
        
        for (robot_id, target) in self.matches.items():
            start = self.current_coords[robot_id][0:2]
            end = target

            print(f"[path_planning](self) Assigning {robot_id} {start} -> {end}")
            path = self.path_planning_algorithm(start=start, end=end, verbose=self.verbose)
            
            self.paths[robot_id] = path

    def movement_x(self, movement_options, current_coords, end):
        difference_x = end[0] - current_coords[0]
        
        # Get closer by x-axis
        if difference_x > 0:
            # Append x + 0.01
            movement_options.append((round(current_coords[0] + 0.01, self.fineness), current_coords[1]))
            self.correct_x = False
        elif difference_x < 0:
            # Append x - 0.01
            movement_options.append((round(current_coords[0] - 0.01, self.fineness), current_coords[1]))
            self.correct_x = False
        else:
            # x already at correct position
            self.correct_x = True

    def movement_y(self, movement_options, current_coords, end):
        difference_y = end[1] - current_coords[1]
        
        # Get closer by y-axis
        if difference_y > 0:
            # Append y + 0.01
            movement_options.append((current_coords[0], round(current_coords[1] + 0.01, self.fineness)))
            self.correct_y = False
        elif difference_y < 0:
            # Append y - 0.01
            movement_options.append((current_coords[0], round(current_coords[1] - 0.01, self.fineness)))
            self.correct_y = False
        else:
            # y already at correct position
            self.correct_y = True

    def evaluate_movement_direction(self, current, selected):
        # print(f"{current=} {selected=}")
        difference_x = selected[0] - current[0]
        difference_y = selected[1] - current[1]
        if difference_x == 0 and difference_y == 0:
            # Staying in place
            return "0"
        elif difference_x > 0 and difference_y == 0:
            return "+x"
        elif difference_x < 0 and difference_y == 0:
            return "-x"
        elif difference_x == 0 and difference_y > 0:
            return "+y"
        elif difference_x == 0 and difference_y < 0:
            return "-y"
        else:
            return "ERROR"
        

    def path_planning_algorithm(self, start: tuple, end: tuple, verbose=False):
        # print(f'{start=} {end=}')
        current_coords = start
        step = 0
        path = {}
        self.correct_x = False
        self.correct_y = False

        while True:
            self.movement_options = []
            if not self.go_around:
                # Normal movement
                self.movement_x(
                    movement_options=self.movement_options, 
                    current_coords=current_coords, 
                    end=end
                )
                self.movement_y(
                    movement_options=self.movement_options, 
                    current_coords=current_coords, 
                    end=end
                )
            else:
                # If encounter an obstacle
                self.movement_y(
                    movement_options=self.movement_options, 
                    current_coords=current_coords, 
                    end=end
                )
                self.movement_x(
                    movement_options=self.movement_options, 
                    current_coords=current_coords, 
                    end=end
                )  

            if verbose: print(f"(helper)[DEBUG] Movement options before pruning: {self.movement_options}")

            for movement in self.movement_options:
                # Remove movement options that move towards an obstacle
                if movement in self.obstacles:
                    if verbose: print(f"(helper)[DEBUG] Obstacle found at: {movement}")
                    self.movement_options.remove(movement)
                    if len(self.movement_options) == 0:
                        # EDGE CASE: at correct x/y and finds obstacle
                        self.go_around = not self.go_around

                # Remove conflicting movement options if step has already been initialized
                if step in self.conflict_map:
                    if movement in self.conflict_map[step]:
                        if verbose: print(f"(helper)[DEBUG] Pruning conflicting step: {step}")
                        self.movement_options.remove(movement)

            if verbose: print(f"(helper)[DEBUG] Movement option(s) after pruning: {self.movement_options}")
            
            if verbose: print(f"(helper)[DEBUG] Correct x; {self.correct_x}, Correct y; {self.correct_y}")
            
            if len(self.movement_options) == 0:
                # Stay in place if no movement options
                path[step] = current_coords
            else:
                # 2 possibilities; 
                # if one movement option -> choose the remaining one
                # if 2 movement options -> choose the first one
                path[step] = self.movement_options[0]

            # Evaluate movement direction
            self.previous_movement = self.evaluate_movement_direction(
                current=current_coords, 
                selected=path[step]
            )
            if self.previous_movement == "ERROR":
                print("(helper)[ERROR] An unexpected error occurred")

            current_coords = path[step]
            if verbose: print(f"(helper)[DEBUG] Current coordinates: {current_coords}")
            
            if step in self.conflict_map:
                # Key exists
                self.conflict_map[step].append(current_coords)
            else:
                # Key doesn't exist yet
                self.conflict_map[step] = [current_coords]
            
            # if verbose: print(f"(helper)[DEBUG] Conflict map: {self.conflict_map}")

            step += 1
            
            if self.correct_x and self.correct_y:
                print("Path found")
                return path
            
            if self.previous_movement == "0":
                #! Testing
                print("Early breakout")
                break


if __name__ == "__main__":
    #! Example current positions (coordinates of the members)
    current_positions = {
        'TurtleBot3Burger_1': [1.78, -1.05, 0.20], 
        'TurtleBot3Burger_2': [-2.05, -1.40, 0.20], 
        'TurtleBot3Burger_3': [-0.35, 1.67, 0.50]
    }
    obstacles = [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]
    # obstacles = []
    name = 'TurtleBot3Burger_2'

    formation_master = FormationMaster(
        robot="", 
        current_coords=current_positions, 
        object_coords=(0.75, -0.25), 
        obstacles=obstacles, 
        radius=0.3, 
        verbose=False
    )
    formation_master.calculate_target_coords()
    formation_master.plan_paths()

    # print("Final Paths:")
    # for name, path in formation_master.paths.items():
    #     print(name, path)

    paths = json.dumps(formation_master.paths)
    loading = json.loads(paths)
    # pprint(loading)
    # pprint(f"{name}: {loading[name]}")
