from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("px4_control"),
            "config",
            "params.yaml",
        ]),
        description="Full path to params.yaml for teleop node.",
    )

    params_file = LaunchConfiguration("params_file")

    # teleop must be launched in a real TTY (terminal) — output=screen so the
    # help banner is visible, but keystrokes still come from the launching shell.
    teleop_node = Node(
        package="px4_control",
        executable="px4_teleop_node",
        name="px4_teleop_node",
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([
        params_file_arg,
        teleop_node,
    ])
