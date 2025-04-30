# Git Clone 
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/Adlink-ROS/rf2o_laser_odometry.git
git clone https://github.com/uleroboticsgroup/yasmin.git

# Information about this workspace 
we are using [YASMIN](https://github.com/uleroboticsgroup/yasmin) for the finite state machine
# Install the ros2_ws 
1. Makes sure ROS2 Humble is install
1. cd to `{your-dir}/swarm-final-project-ise/ros2_ws/`
1. use ros alias `colcon_build` or `colcon build --symlink-`
1. add `source ~/swarm-final-project-ise/ros2_ws/install/setup.bash` to `~/.bashrc`
1. check with bringups

# run 
sudo rosdep init # get the dependancies
rosdep update
colcon build --symlink-install or colcon_build



# how to make a package 
https://robotics.stackexchange.com/questions/97841/including-a-python-module-in-a-ros2-package


# Setup 
install from ros `swarm-final-project-ise/hardware/jetson_setup/setup_system.md`
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
ros2 run teleop_twist_keyboard teleop_twist_keyboard
sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-twist-mux

## LiDAR
For setting rf2o using CMAKE. First go the the CMakeLists.txt folder then run the following commands:
```bash
mkdir build
cd build
cmake ..
make
```

Make sure that the LiDAR is running, after that run this command in a new terminal:
```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```


# Run
## Drive 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run x_drive_controller x_drive_controller

## Lidar Odometry
<!-- ros2 launch sllidar_ros2 view_sllidar_s3_launch.py # to start the RPLidar Node -->
ros2 launch sllidar_ros2 view_sllidar_s3_launch.py 'serial_port:=/dev/ttyUSB1'
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node 

## Twist Mux
ros2 run twist_mux twist_mux --ros-args --params-file ./config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped