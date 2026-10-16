"""每局一键启动：ros2 launch ag_integrate integrate.launch.py

开赛前的地图先验全部集中在 config/match_params.yaml 里改，不动代码。

模式（互斥，launch 参数 mode 二选一）：
  mode:=legacy （默认）integrate_node → 官方 path_executor（LEGACY_PATH=1 回退核）
  mode:=dzbrain          dzbrain_bridge → 官方 dzbrain 新核（/dzlink JSON 情报）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ag_integrate")
    params_file = os.path.join(share, "config", "match_params.yaml")
    mode = LaunchConfiguration("mode")
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode", default_value="legacy",
            description="决策核心选择：legacy（path_executor）或 dzbrain（官方新核）",
        ),
        Node(
            package="ag_integrate",
            executable="integrate_node",
            name="integrate_node",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'legacy'"])),
        ),
        Node(
            package="ag_integrate",
            executable="dzbrain_bridge",
            name="dzbrain_bridge",
            output="screen",
            parameters=[params_file],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'dzbrain'"])),
        ),
    ])
