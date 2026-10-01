from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ag_integrate",
                executable="integrate_node",
                name="integrate_node",
                output="screen",
            )
        ]
    )
