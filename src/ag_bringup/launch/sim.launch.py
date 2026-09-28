"""单机全流程仿真（无需无人机、无需串口、无需模型）：

    ros2 launch ag_bringup sim.launch.py

数据流：
    fake_intel_node ──▶ /link/field_intel ──▶ mission_node ──▶ /mission/waypoints
    vision_node(DummyBackend) ──▶ /perception/detections ──▶ target_node
    target_node ──▶ /gimbal/command ──▶ referee_node(simulate) ──▶ /referee/state

用途：上车前先在笔记本上跑通话题链路与状态机，验证
      「情报 -> 规划 -> 打击 -> 计分」是否闭环。
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
                "mode",
                default_value="progressive",
                description="模拟情报模式：full（一次给全）/ progressive（逐帧揭示+噪声）",
            ),
            Node(
                package="ag_bridge",
                executable="fake_intel_node",
                name="fake_intel_node",
                output="screen",
                parameters=[
                    params,
                    {
                        "mode": LaunchConfiguration("mode"),
                        # 直连车端，跳过 UDP
                        "intel_topic": "/link/field_intel",
                        # 同时喂一份假检测，让打击链路也能闭环
                        "publish_detections": True,
                    },
                ],
            ),
            Node(
                package="ag_perception",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[params, {"model_path": ""}],
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
                parameters=[params],
            ),
            Node(
                package="ag_referee",
                executable="referee_node",
                name="referee_node",
                output="screen",
                parameters=[params, {"simulate": True}],
            ),
            Node(
                package="ag_chassis",
                executable="chassis_node",
                name="chassis_node",
                output="screen",
                # 仿真里底盘快进：一段 eta 3 s 的路程在 0.03 s 内跑完
                parameters=[params, {"simulate": True, "time_scale": 100.0}],
            ),
        ]
    )
