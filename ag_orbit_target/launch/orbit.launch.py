import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "ag_orbit_target"
    config = os.path.join(
        os.path.expanduser("~/ag_orbit_ws"),
        "src", pkg, "config", "orbit_params.yaml",
    )
    node = Node(
        package=pkg,
        executable="orbit_node",
        name="orbit_node",
        output="screen",
        parameters=[config],
        emulate_tty=True,
    )
    return LaunchDescription([node])
