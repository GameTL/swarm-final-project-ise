import math


class FormationMaster:
    def __init__(self, current_coords, object_coords, member_count, radius=2):
        # Retrieve parameters
        self.current_coords = current_coords
        self.object_coords = object_coords
        self.member_count = member_count
        self.radius = radius

        # Initialize calculation parameters
        self.target_coords = []

    def calculate_target_coords(self):
        # Calculate target coords based on radius around the object coordinates
        internal_angle = 2 * math.pi / self.member_count
        for i in range(self.member_count):
            # x' = x + rcos(theta * i) 
            x = self.object_coords[0] + (self.radius * math.cos(internal_angle * i))
            # y' = y + rsin(theta * i)
            y = self.object_coords[1] + (self.radius * math.sin(internal_angle * i))

            self.target_coords.append((x, y))

        if not self.check_radius():
            print("Radius check failed")
        else:
            print("Radius checked")

        return

    def check_radius(self, margin_of_error=0.05, verbose=False):
        if len(self.target_coords) == 0:
            print("Target coordinates not initialized")
            return
        
        for i, coord in enumerate(self.target_coords):
            distance_from_origin = math.sqrt((coord[0] - self.object_coords[0]) ** 2 + (coord[1] - self.object_coords[1]) ** 2)
            
            if verbose:
                print(i, distance_from_origin)

            if math.fabs(distance_from_origin - self.radius) > margin_of_error:
                print(f"Target coordinates {i}: {self.target_coords[i]} failed with distance: {distance_from_origin}")
                return False
            
        return True


if __name__ == "__main__":
    formation_master = FormationMaster((0, 1), (3, 4), 5, radius=2)
    formation_master.calculate_target_coords()
    print(formation_master.target_coords)
