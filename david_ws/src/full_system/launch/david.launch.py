from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Path to david_srdf demo.launch.py
    david_demo_launch = os.path.join(
        get_package_share_directory('david_srdf'),
        'launch',
        'demo.launch.py'
    )

    return LaunchDescription([
        # api_control nodes
        Node(
            package='api_control',
            executable='command2motion_node',
            name='command2motion',
            output='screen'
        ),

        Node(
            package='api_control',
            executable='moveit2microros',
            name='moveit2microros',
            output='screen'
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(david_demo_launch)
        ),
        # voice commands in separate terminal, for clarity
        ExecuteProcess(
        cmd=[
            'gnome-terminal', 
            '--title=Voice Recognition',
            '--',
            'ros2', 'run', 'voice_control', 'voice_commands_node'
        ],
        output='screen'
    ),
        ExecuteProcess(
        cmd=[
            'gnome-terminal', 
            '--title=Micro-ROS agent',
            '--',
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '115200'
        ],
        output='screen'
    )
    ])
