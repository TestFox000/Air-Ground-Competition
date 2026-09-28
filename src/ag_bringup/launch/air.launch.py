"""机载端（无人机/机载电脑）启动：视觉推理 + 数传下发。

    ros2 launch ag_bringup air.launch.py model_path:=/models/best_new.rknn

无模型时会退化成 DummyBackend（输出恒 0），可先用来验证话题连通性。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ag_bringup")
    params = os.path.join(share, "config", "params.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="模型路径（.rknn / .onnx）；留空则使用 DummyBackend",
            ),
            DeclareLaunchArgument(
                "with_fake_intel",
                default_value="false",
                description="是否启动模拟情报源（无机载融合节点时用于联调）",
            ),
            Node(
                package="ag_perception",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[params, {"model_path": LaunchConfiguration("model_path")}],
            ),
            Node(
                package="ag_perception",
                executable="intel_fusion_node",
                name="intel_fusion_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ag_bridge",
                executable="air_link_node",
                name="air_link_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ag_bridge",
                executable="fake_intel_node",
                name="fake_intel_node",
                output="screen",
                parameters=[params],
                condition=IfCondition(LaunchConfiguration("with_fake_intel")),
            ),
        ]
    )
