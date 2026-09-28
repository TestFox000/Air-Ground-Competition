"""车端（地面车/香橙派）启动：链路接收 + 任务规划 + 敌我决策 + 裁判串口。

    ros2 launch ag_bringup ground.launch.py

底盘只需订阅 /mission/waypoints 并按格号行进，完成后向 /mission/task_done
发布格号（如 "C3"）或任务类型（如 "MINE"）即可推进下一段。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ag_bringup")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulate_referee",
                default_value="false",
                description="true = 不开串口，裁判状态由节点模拟（无硬件联调）",
            ),
            DeclareLaunchArgument(
                "enable_fire",
                default_value="true",
                description="false = 只瞄准不下发射击指令（调试用）",
            ),
            DeclareLaunchArgument(
                "simulate_chassis",
                default_value="true",
                description="true = 底盘按 eta 计时推进（未对接底盘协议时的默认）",
            ),
            DeclareLaunchArgument(
                "time_scale",
                default_value="10.0",
                description="底盘仿真快进倍速；实机用 1.0",
            ),
            Node(
                package="ag_bridge",
                executable="ground_link_node",
                name="ground_link_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ag_mission",
                executable="mission_node",
                name="mission_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ag_mission",
                executable="target_node",
                name="target_node",
                output="screen",
                parameters=[params, {"enable_fire": LaunchConfiguration("enable_fire")}],
            ),
            Node(
                package="ag_referee",
                executable="referee_node",
                name="referee_node",
                output="screen",
                parameters=[params, {"simulate": LaunchConfiguration("simulate_referee")}],
            ),
            Node(
                package="ag_chassis",
                executable="chassis_node",
                name="chassis_node",
                output="screen",
                parameters=[params, {"simulate": LaunchConfiguration("simulate_chassis"),
                                     "time_scale": LaunchConfiguration("time_scale")}],
            ),
        ]
    )
