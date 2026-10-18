"""每局一键启动：ros2 launch ag_integrate integrate.launch.py

开赛前地图先验全部集中在 config/match_params.yaml 里改，不动代码。

模式（互斥，launch 参数 mode 二选一）：
  mode:=legacy （默认）integrate_node → 官方 path_executor（LEGACY_PATH=1 回退核）
  mode:=dzbrain          dzbrain_bridge → 官方 dzbrain 新核（/dzlink JSON 情报）

respawn（2026-10-17 评审整改）：决策节点在 180s 的局里挂掉一次就等于弃权，
而它的回调链上有大量第三方输入（UDP 情报、ROS 话题、yaml）。宁可 2s 后
带全新状态重启（WorldModel 会重新累积、粘性否决从空开始），也不要死在场上。
注意：重启期间车上无人下发路线 —— dznav 侧的 link_timeout 会独立
把车刹停，两边配合起来才是安全的。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

#: 崩溃后多久重启。太短会在硬故障（如 yaml 语法错）上打出重启风暴。
RESPAWN_DELAY_S = 2.0


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
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            condition=IfCondition(PythonExpression(["'", mode, "' == 'legacy'"])),
        ),
        Node(
            package="ag_integrate",
            executable="dzbrain_bridge",
            name="dzbrain_bridge",
            output="screen",
            parameters=[params_file],
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            condition=IfCondition(PythonExpression(["'", mode, "' == 'dzbrain'"])),
        ),
    ])
