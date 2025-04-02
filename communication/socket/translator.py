import math
from rich.pretty import pprint

from formation import FormationMaster # For running this file individually

class Translator:
    def __init__(self):
        self.commands = {}

    def extract_orientation(self, paths):
        self.orientation = paths["orientation"]
        del paths["orientation"]

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

            angle_rad = self.orientation[jetson]
            if angle_rad > 0:
                self.commands[jetson].append(("turn_ccw", angle_rad))
            elif angle_rad < 0:
                self.commands[jetson].append(("turn_cw", math.fabs(angle_rad)))

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

    #! Two possible approaches: clear commands after communicated (preferred), or initialize a new translator
    translator.commands.clear() 
