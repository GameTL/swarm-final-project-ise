# Install the ros2_ws 
1. Makes sure ROS2 Humble is install
1. cd to `{your-dir}/swarm-final-project-ise/ros2_ws/`
1. use ros alias `colcon_build` or `colcon build --symlink-`
1. add `source ~/swarm-final-project-ise/ros2_ws/install/setup.bash` to `~/.bashrc`
1. check with bringups

# Start for Mehul 
run `ros2 launch sllidar_ros2 view_sllidar_s3_launch.py` to start the RPLidar Node


# Odom 

# run 
colcon build --symlink-install


# how to make a package 
https://robotics.stackexchange.com/questions/97841/including-a-python-module-in-a-ros2-package



# Setup 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
ros2 run teleop_twist_keyboard teleop_twist_keyboard
sudo apt-get install ros-humble-teleop-twist-keyboard

# Drive 
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run x_drive_controller x_drive_controller
# Lidar Odometry
ros2 launch sllidar_ros2 view_sllidar_s3_launch.py
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node 