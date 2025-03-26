from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Name of the robot in the swarm'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose output'
    )
    
    include_hardware_arg = DeclareLaunchArgument(
        'include_hardware',
        default_value='false',
        description='Whether to include hardware launch files'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the SLLIDAR S3'
    )
    
    # Get the path to the jeff_bringup_launch.py file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    
    return LaunchDescription([
        # Include the declared arguments
        robot_name_arg,
        verbose_arg,
        include_hardware_arg,
        serial_port_arg,
        
        # Swarm State Machine node
        Node(
            package='swarm_control',
            executable='swarm_state_machine.py',
            name='swarm_state_machine',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'verbose': LaunchConfiguration('verbose')
            }],
            output='screen'
        ),
        
        # Wheel Odometry node
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }],
            output='screen'
        ),
        
        # Include the jeff_bringup_launch.py if include_hardware is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/jeff_bringup_launch.py']),
            condition=IfCondition(LaunchConfiguration('include_hardware')),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'mux_params_file': TextSubstitution(text='./config/twist_mux.yaml')
            }.items()
        ),
        
        # Object Detection node
        Node(
            package='swarm_control',
            executable='object_detector_node.py',
            name='object_detector',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }],
            output='screen'
        ),
        
        # Communication Bridge node
        Node(
            package='swarm_control',
            executable='communication_bridge_node.py',
            name='communication_bridge',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }],
            output='screen'
        ),
        
        # Path Planning node
        Node(
            package='swarm_control',
            executable='path_planning_node.py',
            name='path_planning',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }],
            output='screen'
        )
    ]) 