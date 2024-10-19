import math
from model import Node, a_star


class FormationMaster:
    def __init__(self, current_coords, object_coords, radius=2):
        # Retrieve parameters
        self.current_coords = current_coords
        self.member_count = len(self.current_coords)
        self.object_coords = object_coords
        self.radius = radius

        # Initialize calculation parameters
        self.target_coords = []
        self.paths = []
        self.conflict_map = set()

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

    def plan_paths(self, grid_size=10):
        print("# ===== Calculating paths ===== #")
        for i, target in enumerate(self.target_coords):
            start = Node(self.current_coords[i][0], self.current_coords[i][1])
            goal = Node(target[0], target[1])
            grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

            path = a_star(start, goal, grid, self.conflict_map)

            if path:
                self.paths.append(path)
                for point in path:
                    self.conflict_map.add((point[0], point[1]))
                print(f"Path for member {i}: {path}")
            else:
                print(f"No path found for member {i}")


if __name__ == "__main__":
    #! Example current positions (coordinates of the members)
    current_positions = [(0, 1), (1, 2), (2, 3)]

    formation_master = FormationMaster(current_positions, (4, 5), radius=2)
    formation_master.calculate_target_coords()
    formation_master.plan_paths(grid_size=10)

    print("Final Paths:", formation_master.paths)
