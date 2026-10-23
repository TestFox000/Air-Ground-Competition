from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Run the Python serial_reader node (new map coordinate system).
    # Do NOT run any other node that publishes /odom/localization,
    # otherwise two publishers with different frames will interleave
    # messages and corrupt the pose used for navigation.
    serial_reader_node = Node(
        package="car_localization",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        serial_reader_node
    ])
