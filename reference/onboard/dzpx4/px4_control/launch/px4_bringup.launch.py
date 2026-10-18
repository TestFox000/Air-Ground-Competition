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
        description="PX4 control parameter file.",
    )
    land_control_arg = DeclareLaunchArgument(
        "land_control",
        default_value="false",
        choices=["true", "false"],
        description="true uses PX4 NAV_LAND; false uses controlled descent.",
    )

    control_node = Node(
        package="px4_control",
        executable="px4_control_node",
        name="px4_control_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {"land_control": LaunchConfiguration("land_control")},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        land_control_arg,
        control_node,
    ])
