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
        print(self.target_coords)

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
        
        print("Matches:", self.matches)

    def plan_paths(self):
        print("# ===== Calculating paths ===== #")
        # print(self.target_coords)
        # for (robot_id, target) in self.target_coords:
        #     print(robot_id, target)
        #     for current_robot_coords in self.current_coords_dict:
        #         print(securrent_robot_coords)

            # print(f"[path_planning](self) Assigning {robot_id} {self.current_coords_dict[robot_id][0:2]} -> {target}")
            # path = self.path_planning_algorithm(self.current_coords_dict[robot_id][0:2], target, verbose=self.verbose)
            # self.paths[robot_id] = path

    def path_planning_algorithm(self, start: tuple, end: tuple, verbose=False):
        # print(f'{start=} {end=}')
        current_coordinates = start
        step = 0
        path = {}
        correct_x = False
        correct_y = False

        while True:
            difference_x = end[0] - current_coordinates[0]
            difference_y = end[1] - current_coordinates[1]
            if verbose: print(f"(helper)[DEBUG] {difference_x=} {difference_y=}")

            movement_options = []
            # Get closer by x-axis
            if difference_x > 0:
                # Append x + 0.01
                movement_options.append((round(current_coordinates[0] + 0.01, self.fineness), current_coordinates[1]))
            elif difference_x < 0:
                # Append x - 0.01
                movement_options.append((round(current_coordinates[0] - 0.01, self.fineness), current_coordinates[1]))
            else:
                # x already at correct position
                correct_x = True
            
            # Get closer by y-axis
            if difference_y > 0:
                # Append y + 0.01
                movement_options.append((current_coordinates[0], round(current_coordinates[1] + 0.01, self.fineness)))
            elif difference_y < 0:
                # Append y - 0.01
                movement_options.append((current_coordinates[0], round(current_coordinates[1] - 0.01, self.fineness)))
            else:
                # y already at correct position
                correct_y = True

            if verbose: print(f"(helper)[DEBUG] Movement options before pruning: {movement_options}")

            for movement in movement_options:
                # Remove movement options that move towards an obstacle
                if movement in self.obstacles:
                    if verbose: print(f"(helper)[DEBUG] Obstacle found at: {movement}")
                    movement_options.remove(movement)
                # Remove conflicting movement options if step has already been initialized
                if step in self.conflict_map:
                    if movement in self.conflict_map[step]:
                        if verbose: print(f"(helper)[DEBUG] Pruning conflicting step: {step}")
                        movement_options.remove(movement)
            if verbose: print(f"(helper)[DEBUG] Movement option(s) after pruning: {movement_options}")
            
            if verbose: print(f"(helper)[DEBUG] Correct x; {correct_x}, Correct y; {correct_y}")
            
            if len(movement_options) == 0:
                # Stay in place if no movement options
                path[step] = current_coordinates
            else:
                # 2 possibilities; 
                # if one movement option -> choose the remaining one
                # if 2 movement options -> choose the first one
                path[step] = movement_options[0]
            current_coordinates = path[step]
            if verbose: print(f"(helper)[DEBUG] Current coordinates: {current_coordinates}")
            
            if step in self.conflict_map:
                # Key exists
                self.conflict_map[step].append(current_coordinates)
            else:
                # Key doesn't exist yet
                self.conflict_map[step] = [current_coordinates]
            
            # if verbose: print(f"(helper)[DEBUG] Conflict map: {self.conflict_map}")

            step += 1
            
            if correct_x and correct_y:
                return path


if __name__ == "__main__":
    #! Example current positions (coordinates of the members)
    current_positions = {'TurtleBot3Burger_1': [1.78, -1.05, 0.20], 'TurtleBot3Burger_2': [-2.05, -1.40, 0.20], 'TurtleBot3Burger_3': [-0.35, 1.67, 0.50]}
    obstacles = [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]
    name = 'TurtleBot3Burger_2'

    formation_master = FormationMaster(robot="", current_coords=current_positions, object_coords=(0.75, -0.25), obstacles=obstacles, radius=0.3, verbose=True)
    formation_master.calculate_target_coords()
    formation_master.plan_paths()

    # print("Final Paths:")
    # for name, path in formation_master.paths.items():
    #     print(name, path)

    paths = json.dumps(formation_master.paths)
    loading = json.loads(paths)
    # pprint(loading)
    # pprint(f"{name}: {loading[name]}")
