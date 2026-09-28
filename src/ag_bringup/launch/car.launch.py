"""车端决策层启动（渡众 SmartMaster 实机专用）。

    ros2 launch ag_bringup car.launch.py                 # 默认 dry_run，只看不发
    ros2 launch ag_bringup car.launch.py dry_run:=false  # 真正发 /nav/waypoint

只启动「我们自己的那部分」：

    ground_link_node   收 UDP 情报 -> /link/field_intel
    car_node           情报 -> 任务排序 -> /nav/waypoint（车端 path_executor 执行）
    referee_node       接官方 dzjudgment 话题 -> /referee/state（可选）

车端的其它功能**不要在这里启动**，它们是渡众自带节点，按官方顺序单独起：

    ros2 launch dzactuator bringup_all.launch.py     # 底盘/IMU/里程计
    ros2 launch dzactuator judgment.launch.py        # 裁判串口
    ros2 launch dzactuator camera.launch.py          # /image_raw
    ros2 launch dzactuator det_node.launch.py        # RKNN 检测
    ros2 launch dzactuator dzrknn_dataprocess.launch.py   # 瞄准 + 开火
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
                "dry_run",
                default_value="true",
                description="true = 只计算路线不发命令（第一次上车必须先用它核对）",
            ),
            DeclareLaunchArgument(
                "enable_referee",
                default_value="true",
                description="false = 不启动 referee_node（例如还没起官方 dzjudgment）",
            ),
            DeclareLaunchArgument(
                "start_cell",
                default_value="N10",
                description="车被摆放的起点格号（必须与摆放位置一致）",
            ),
            DeclareLaunchArgument(
                "goal_cell",
                default_value="N11",
                description="⚠️ 车终点格号，现场量取后改这里",
            ),
            DeclareLaunchArgument(
                "publish_reset",
                default_value="true",
                description="false = 已经手动发过 reset，不要重复锚定里程计",
            ),
            Node(
                package="ag_bridge",
                executable="ground_link_node",
                name="ground_link_node",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="ag_car",
                executable="car_node",
                name="car_node",
                output="screen",
                parameters=[
                    params,
                    {
                        "dry_run": LaunchConfiguration("dry_run"),
                        "start_cell": LaunchConfiguration("start_cell"),
                        "goal_cell": LaunchConfiguration("goal_cell"),
                        "publish_reset": LaunchConfiguration("publish_reset"),
                    },
                ],
            ),
            Node(
                package="ag_referee",
                executable="referee_node",
                name="referee_node",
                output="screen",
                parameters=[params, {"source": "dz"}],
                condition=IfCondition(LaunchConfiguration("enable_referee")),
            ),
        ]
    )
