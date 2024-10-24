import math


class FormationMaster:
    def __init__(self, current_coords, object_coords, radius=2, verbose=False):
        # Retrieve parameters
        self.current_coords = current_coords
        self.member_count = len(self.current_coords)
        self.object_coords = object_coords
        self.radius = radius
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

        self.target_coords = [(round(x), round(y)) for (x, y) in self.target_coords]
        print(self.target_coords)

        # Check margins of error
        self.check_margin_of_error()

    def check_margin_of_error(self):
        print("# ===== Listing margins of error ===== #")
        if len(self.target_coords) == 0:
            print("Target coordinates not initialized")
            return
        
        for i, coord in enumerate(self.target_coords):
            distance_from_origin = math.sqrt((coord[0] - self.object_coords[0]) ** 2 + (coord[1] - self.object_coords[1]) ** 2)
            print(i, distance_from_origin)

    def plan_paths(self):
        print("# ===== Calculating paths ===== #")
        for i, target in enumerate(self.target_coords):
            print(f"Assigning robot{i + 1}'s path: {self.current_coords[i]} -> {target}")
            path = self.path_planning_algorithm(self.current_coords[i], target, verbose=self.verbose)
            #! Replace with robot name
            self.paths[f"robot{i + 1}"] = path

    def path_planning_algorithm(self, start: tuple, end: tuple, verbose=False):
        current_coordinates = start
        step = 0
        path = {}
        correct_x = False
        correct_y = False

        while current_coordinates != end:
            difference_x = end[0] - current_coordinates[0]
            difference_y = end[1] - current_coordinates[1]

            movement_options = []
            # Get closer by x-axis
            if difference_x > 0:
                # Append x + 1
                movement_options.append((current_coordinates[0] + 1, current_coordinates[1]))
            elif difference_x < 0:
                # Append x - 1
                movement_options.append((current_coordinates[0] - 1, current_coordinates[1]))
            else:
                # x already at correct position
                correct_x = True
            
            # Get closer by y-axis
            if difference_y > 0:
                # Append y + 1
                movement_options.append((current_coordinates[0], current_coordinates[1] + 1))
            elif difference_y < 0:
                # Append y - 1
                movement_options.append((current_coordinates[0], current_coordinates[1] - 1))
            else:
                # y already at correct position
                correct_y = True
            if verbose: print(f"Movement options before pruning: {movement_options}")

            if not correct_x or not correct_y:
                # Remove conflicting movement options if step has already been initialized
                if step in self.conflict_map:
                    movement_options = [option for option in movement_options if option not in self.conflict_map[step]]
                if verbose: print(f"Movement option(s) after pruning: {movement_options}")
                
                if verbose: print(f"Correct x; {correct_x}, Correct y; {correct_y}")
                
                if len(movement_options) == 0:
                    # Stay in place if no movement options
                    path[step] = current_coordinates
                else:
                    # 2 possibilities; 
                    # if one movement option -> choose the remaining one
                    # if 2 movement options -> choose the first one
                    path[step] = movement_options[0]
                current_coordinates = path[step]
                if verbose: print(current_coordinates)
                
                if step in self.conflict_map:
                    # Key exists
                    self.conflict_map[step].append(current_coordinates)
                else:
                    # Key doesn't exist yet
                    self.conflict_map[step] = [current_coordinates]
                
                if verbose: print(self.conflict_map)

                step += 1
        
        return path


if __name__ == "__main__":
    #! Example current positions (coordinates of the members)
    current_positions = [(0, 1), (1, 2), (0, 3)]

    formation_master = FormationMaster(current_positions, (4, 5), radius=2, verbose=False)
    formation_master.calculate_target_coords()
    formation_master.plan_paths()

    print("Final Paths:")
    for name, path in formation_master.paths.items():
        print(name, path)
