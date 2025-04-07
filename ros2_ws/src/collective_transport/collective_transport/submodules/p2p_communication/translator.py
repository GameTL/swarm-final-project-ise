from rich.pretty import pprint

from formation import FormationMaster # For running this file individually

class Translator:
    def __init__(self):
        self.commands = {}
        self.waypoints = {}

    def extract_orientation(self, paths):
        try:
            self.orientation = paths["orientation"]
            del paths["orientation"]
        except KeyError:
            print("Orientation has already been extracted")
        
        return paths

    def calculate_commands(self, paths):
        paths = self.extract_orientation(paths)
        # print(self.orientation)
        # print(paths)

        for jetson, path in paths.items():
            commands = []
            timestamps = sorted(path.keys(), key=int)

            prev_direction = None
            step_count = 0

            for i in range(len(timestamps) - 1):
                t1, t2 = timestamps[i], timestamps[i + 1]
                x1, y1 = path[t1]
                x2, y2 = path[t2]

                # Determine movement direction
                if x2 > x1:
                    direction = "pos_x"
                elif x2 < x1:
                    direction = "neg_x"
                elif y2 > y1:
                    direction = "pos_y"
                elif y2 < y1:
                    direction = "neg_y"
                else:
                    direction = "stop"

                # Group same direction movements
                if direction == prev_direction:
                    step_count += 1
                else:
                    if prev_direction is not None:
                        commands.append((prev_direction, step_count))
                    prev_direction = direction
                    step_count = 1  # Start counting new movement

            # Append the last movement
            if prev_direction is not None:
                commands.append((prev_direction, step_count))

            self.commands[jetson] = commands

            # Add orientation (in rad)
            # 0 is rightward in 2D
            self.commands[jetson].append(("turn", self.orientation[jetson]))

    def sample_waypoints(self, paths):
        paths = self.extract_orientation(paths)

        waypoints = {}
        for jetson, path in paths.items():
            corners = []
            timestamps = sorted(path.keys(), key=int)

            prev_direction = None

            # Include the first coordinates
            x_last, y_last = path[timestamps[0]]
            corners.append((x_last, y_last))

            for i in range(len(timestamps) - 1):
                t1, t2 = timestamps[i], timestamps[i + 1]
                x1, y1 = path[t1]
                x2, y2 = path[t2]

                # Determine movement direction
                if x2 > x1:
                    direction = "pos_x"
                elif x2 < x1:
                    direction = "neg_x"
                elif y2 > y1:
                    direction = "pos_y"
                elif y2 < y1:
                    direction = "neg_y"
                else:
                    direction = "stop"

                # Detect corners
                if prev_direction is not None and direction != prev_direction:
                    corners.append((x1, y1))

                if direction != "stop":
                    prev_direction = direction
                else:
                    # Record the same coordinates to show stops
                    corners.append((x1, y1)) 

            # Include the last coordinates
            #! Commented for now because creates dupes
            # x_last, y_last = path[timestamps[-1]]
            # corners.append((x_last, y_last))

            waypoints[jetson] = corners

        self.waypoints["waypoints"] = waypoints
        self.waypoints["orientation"] = self.orientation

if __name__ == "__main__":
    translator = Translator()

    current_positions = {
        'jetson1': [1.78, -1.05], 
        'jetson2': [-2.05, -1.40], 
        'jetson3': [-0.35, 1.67]
    }
    obstacles = [(-1, -1.4), (0.6, 0.3), (0.1, 1.67)]
    # obstacles = []
    name = 'jetson2'

    formation_master = FormationMaster(
        name=name, 
        current_coords=current_positions, 
        object_coords=(0.75, -0.25), 
        obstacles=obstacles, 
        radius=0.3, 
        debug_level=0
    )
    formation_master.calculate_target_coords()
    formation_master.plan_paths()
    
    paths = formation_master.paths
    # print(paths)

    # Process the path
    print("# ===== Translating to commands ===== #")
    translator.calculate_commands(paths)
    pprint(translator.commands)

    print("# ===== Sampling waypoints ===== #")
    translator.sample_waypoints(paths)
    pprint(translator.waypoints)

    #! Two possible approaches: clear commands after communicated (preferred), or initialize a new translator
    translator.commands.clear() 
