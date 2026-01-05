from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Node from package1
        Node(
            package="package1",
            executable="node1",
            name="node1",
            output="screen"
        ),

        # Node from package2
        Node(
            package="package2",
            executable="something_else",
            name="node2",
            output="screen"
        ),

        # Micro-ROS agent (package: micro_ros_agent)
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["udp4", "--port", "8888"],
            output="screen"
        ),

    ])
