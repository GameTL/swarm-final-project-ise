from ament_index_python.packages import get_package_share_directory
import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# run with teleop keyboard 
#*  ros2 run teleop_twist_keyboard  teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_keyboard


def generate_launch_description():
    
    default_config_topics = "../config/twist_mux.yaml"
    
    #* RPLIDAR VARS
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='1000000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='DenseBoost')


    return LaunchDescription([
        # Include the declared argument
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        
        # SLLIDAR S3 nodes
        # Node(
        #     #  ros2 launch sllidar_ros2 sllidar_s3_launch.py 'serial_port:=/dev/ttyUSB1'
        #     package='sllidar_ros2',
        #     executable='sllidar_s3_launch',
        #     name='sllidar_launch',
        #     parameters=[{'serial_port': LaunchConfiguration('serial_port')}], #
        #     output='screen'
        # ),
        
        # RF2O laser odometry node
        # Node(
        #     package='rf2o_laser_odometry',
        #     executable='rf2o_laser_odometry_node',
        #     name='rf2o_laser_odometry',
        #     # output='screen'
        # ),
        # telop keyboard (Twist) node
        # Node(
        #     package='teleop_twist_keyboard',
        #     executable='teleop_twist_keyboard',
        #     name='teleop_twist_keyboard',
        #     # remapping=[('/cmd_vel','/cmd_vel_keyboard')],
        #     output='screen' # make this node visable on terminal
        # ),
        
        # Twist mux (Twist) node
        # Node(
        #     # ros2 run twist_mux twist_mux --ros-args --params-file ./config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel
        #     package='twist_mux',
        #     executable='twist_mux',
        #     name='twist_mux',
        #     parameters=[LaunchConfiguration('config_topics')],
        #     # remapping=('cmd_vel_out','cmd_vel'),
        #     output='screen' # make this node visable on terminal
        # ),
        # Dyanmixel controller node
        Node(
            package='x_drive_controller',
            executable='x_drive_controller',
            name='x_drive_controller',
            output='screen' # make this node visable on terminal
        ),

        #* RPLIDAR LAUNCH PART
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen')
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