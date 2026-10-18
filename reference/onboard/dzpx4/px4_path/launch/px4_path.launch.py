from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_selected_mode(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    try:
        parameters = yaml.safe_load(Path(params_file).read_text(encoding="utf-8"))
        record_mode = parameters["px4_path_launch"]["ros__parameters"]["record_mode"]
    except (OSError, TypeError, KeyError, yaml.YAMLError) as error:
        raise RuntimeError(
            f"cannot read px4_path_launch.record_mode from '{params_file}': {error}"
        ) from error

    if not isinstance(record_mode, bool):
        raise RuntimeError(
            "px4_path_launch.record_mode must be a YAML boolean: true or false"
        )

    executable = "path_record_node" if record_mode else "path_follow_node"
    node_name = executable
    selected_mode = "record" if record_mode else "follow"
    return [
        LogInfo(msg=f"px4_path mode selected from YAML: {selected_mode}"),
        Node(
            package="px4_path",
            executable=executable,
            name=node_name,
            output="screen",
            parameters=[params_file],
        ),
    ]


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("px4_path"),
            "config",
            "params.yaml",
        ]),
        description="YAML file containing launch mode and all node parameters.",
    )

    return LaunchDescription([
        params_file_arg,
        OpaqueFunction(function=launch_selected_mode),
    ])
