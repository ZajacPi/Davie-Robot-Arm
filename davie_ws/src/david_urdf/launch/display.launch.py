# display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'david_urdf'

    pkg_share = get_package_share_directory(pkg_name)
    default_model_path = os.path.join(pkg_share, 'urdf', 'david_urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'urdf.rviz')

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot xacro file'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to RViz config file'
    )

    # Command to run xacro -> URDF (safe substitution)
    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        LaunchConfiguration('model')
    ])

    # robot_state_publisher: pass robot_description as a parameter
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_command, 'use_sim_time': False}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
