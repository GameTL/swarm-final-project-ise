# Install the ros2_ws 
1. Makes sure ROS2 Humble is install
1. cd to `{your-dir}/swarm-final-project-ise/ros2_ws/`
1. use ros alias `colcon_build` or `colcon build --symlink-`
1. add `source ~/swarm-final-project-ise/ros2_ws/install/setup.bash` to `~/.bashrc`
1. check with bringups

# run 
colcon build --symlink-install or colcon_build


# how to make a package 
https://robotics.stackexchange.com/questions/97841/including-a-python-module-in-a-ros2-package


# Setup 
install from ros `swarm-final-project-ise/hardware/jetson_setup/setup_system.md`
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
ros2 run teleop_twist_keyboard teleop_twist_keyboard
sudo apt-get install ros-humble-teleop-twist-keyboard

## LiDAR
For setting rf2o using CMAKE. First go the the CMakeLists.txt folder then run the following commands:
'''
mkdir build
cd build
cmake ..
make
'''

Make sure that the LiDAR is running, after that run this command in a new terminal:
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py


# Run
## Drive 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run x_drive_controller x_drive_controller
## Lidar Odometry
<!-- ros2 launch sllidar_ros2 view_sllidar_s3_launch.py # to start the RPLidar Node -->
ros2 launch sllidar_ros2 view_sllidar_s3_launch.py 'serial_port:=/dev/ttyUSB1'
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node 