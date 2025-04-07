from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the SLLIDAR S3'
    )
    
    mux_params_file_arg = DeclareLaunchArgument(
        'mux_params_file',
        default_value="./config/twist_mux.yaml",
        description='path to parameter file for twist_mux, from telop_keyboard, nav2'
    )

    return LaunchDescription([
        # Include the declared argument
        serial_port_arg,
        
        # SLLIDAR S3 node
        Node(
            # ros2 launch sllidar_ros2 view_sllidar_s3_launch.py 'serial_port:=/dev/ttyUSB1'
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}], #
            output='screen'
        ),
        
        # RF2O laser odometry node
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen'
        ),
        # telop keyboard (Twist) node
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop_twist_keyboard',
        #     # remapping=[('/cmd_vel','/cmd_vel_keyboard')],
        #     output='screen' # make this node visable on terminal
        # ),
        # Twist mux (Twist) node
        Node(
            # ros2 run twist_mux twist_mux --ros-args --params-file ./config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[LaunchConfiguration('mux_params_file')],
            remapping=('cmd_vel_out','cmd_vel'),
            output='screen' # make this node visable on terminal
        ),
        # Dyanmixel controller node
        Node(
            package='x_drive_controller',
            executable='x_drive_controller',
            name='x_drive_controller',
            output='screen' # make this node visable on terminal
        )
    ])



""" 
\e[1;32m - Bold Green
\e[1;31m - Bold Red
\e[1;34m - Bold Blue
\e[1;33m - Bold Yellow
\e[1;35m - Bold Magenta
\e[1;36m - Bold Cyan
\e[0m - Reset to default color

# SLLIDAR S3 node with blue output
Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
    parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
    output='screen',
    emulate_tty=True,
    prefix=['bash', '-c', 'echo -e "\\e[1;34m[LIDAR]:\\e[0m"; exec "$@"', '--']
),

# RF2O laser odometry node with red output
Node(
    package='rf2o_laser_odometry',
    executable='rf2o_laser_odometry_node',
    name='rf2o_laser_odometry',
    output='screen',
    emulate_tty=True,
    prefix=['bash', '-c', 'echo -e "\\e[1;31m[ODOMETRY]:\\e[0m"; exec "$@"', '--']
)

"""