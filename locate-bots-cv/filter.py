class AverageFilter:
    def __init__(self, alpha=0.5):
        self.smooth_dict=dict()
        self.alpha = alpha

    def filtering(self, id, curr_pose):
        if id not in self.smooth_dict.keys():
            self.smooth_dict[id] = curr_pose
        previous_smoothed_pose = self.smooth_dict[id]
        smoothed_pose = self.alpha * curr_pose + (1 - self.alpha) * previous_smoothed_pose
        self.smooth_dict[id] = smoothed_pose
        return smoothed_pose