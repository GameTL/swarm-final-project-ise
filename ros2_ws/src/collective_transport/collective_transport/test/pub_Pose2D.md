# Publishing a Pose2D message using ros2 topic pub command
ros2 topic pub /robot_pose geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}"
ros2 topic pub /robot_pose geometry_msgs/msg/Pose2D "{x: 0.5, y: 0.5, theta: 0.0}"

# Publishing continuously at 10Hz
ros2 topic pub --rate 10 /robot_pose geometry_msgs/msg/Pose2D "{x: 1.0, y: 2.0, theta: 0.5}"
