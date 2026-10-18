from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("px4_control"),
            "config",
            "params.yaml",
        ]),
    )
    land_control_arg = DeclareLaunchArgument(
        "land_control",
        default_value="false",
        choices=["true", "false"],
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("px4_control"),
            "launch",
            "px4_bringup.launch.py",
        ])),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "land_control": LaunchConfiguration("land_control"),
        }.items(),
    )

    return LaunchDescription([
        params_file_arg,
        land_control_arg,
        bringup,
    ])
