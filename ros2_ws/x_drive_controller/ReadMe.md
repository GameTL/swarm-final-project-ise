# Compile 
cd ~/ros2_ws
colcon build --packages-select x_drive_controller
colcon build --symlink-install # or this one
source install/setup.bash

# Run 
ros2 run x_drive_controller x_drive_controller

## Check logs 
ls -lt x_drive_log_* | head -1  # shows the most recent log file
tail -f x_drive_log_20250204_123456.csv  # replace with your actual filename

## Testing 
```
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist 

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist 
"linear:
  x: 1.0
  y: 0.5
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 1
```


# joy 
ros2 run joy joy_node
ros2 topic echo /joy
ros2 run joy joy_enumerate_devices


# to drive 
ros2 run joy joy_node
ros2 run teleop_joy teleop_joy 

ros2 run teleop_twist_keyboard teleop_twist_keyboard 

ros2 run x_drive_controller x_drive_controller
