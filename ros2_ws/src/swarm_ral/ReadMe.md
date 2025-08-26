# Flow of the experiment 
1. Init state: checking if the camera is attached 
2. Ready state: waiting for the self.communicator of each robot to receive a "start_object_detection" command/header 
3. Object Detection state:
   1. Turn on Camera using OpenCV
   2. Load the model 
   3. if detects trigger consesus
4. if detects an object, ask the other robot if they are detecting an object. If so, a potential collision may happens; Both check the priority queue on whos the boss then nominate a robot to do the path planning(fake). 
5. Path Planning state: taking Uniform(min_x, max_x, y) time
6. Path Following state: Experiment End