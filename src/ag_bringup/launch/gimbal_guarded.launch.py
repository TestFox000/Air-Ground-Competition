"""带**敌我闸门**的云台/瞄准启动（实机用这个，不要直接跑官方 launch）。

    ros2 launch ag_bringup gimbal_guarded.launch.py                    # 默认只打蓝靶（敌方）
    ros2 launch ag_bringup gimbal_guarded.launch.py enable_fire:=false  # 只瞄准、绝不发射

为什么要包一层（车端源码实测结论）：

    * `dzrknn_dataprocess`（官方瞄准节点）的开火条件里**没有敌我判断**：
        is_target_cls = (data[2] == 1) || (data[2] == 2)   // 蓝 红 都算靶
      也就是说它会**对红面（友方）靶也开火** —— 规则里每个计数 −5、最多 −15，
      而且会把 10 发弹药全打光。
    * 唯一看起来像闸门的 `/move_base/stop_signal`（注释写 "1 = allow shooting"）
      **只有发布者、没有订阅者**，是死代码。

所以链路改成：

    det_node ──/offset_center──▶ aim_guard_node ──/offset_center_guarded──▶ dzrknn_dataprocess
                                   │
                                   └─ 红面/非白名单类 → 替换成"无目标"帧

本 launch 做的事（**不改官方代码**，只用话题重映射）：

    1. 起 ag_car 的 aim_guard_node（过滤）
    2. 起官方 dzrknn_dataprocess，并把它的 `offset_center` 订阅重映射到过滤后的话题

⚠️ 启动后云台会立即开始左右扫描（官方行为），确保周围无人、有空间。
⚠️ 官方节点当前**未编译进厂家工作空间**，需要先跑 `tools/car_build_gimbal.sh`
   把它编到 /tmp/ag_ws，并先 `source /tmp/ag_ws/install/setup.bash`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    try:
        share = get_package_share_directory("ag_bringup")
        params = os.path.join(share, "config", "params.yaml")
    except Exception:                                   # noqa: BLE001
        params = None

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_fire",
                default_value="true",
                description="false = 闸门全关，只瞄准不发射（首次联调建议先 false）",
            ),
            DeclareLaunchArgument(
                "allow_classes",
                default_value="1",
                description="放行的类别（det_node 的编号：1=蓝靶=敌方，2=红靶=友方）；空=全放行（危险）",
            ),
            DeclareLaunchArgument(
                "max_shots",
                default_value="10",
                description="官方节点的全程弹药上限（车端硬上限就是 10）",
            ),
            # ① 过滤器
            Node(
                package="ag_car",
                executable="aim_guard_node",
                name="aim_guard_node",
                output="screen",
                parameters=[
                    {"input_topic": "/offset_center",
                     "output_topic": "/offset_center_guarded",
                     "class_blue": 1,
                     "class_red": 2,
                     "allow_classes": LaunchConfiguration("allow_classes"),
                     "enable_fire": LaunchConfiguration("enable_fire")},
                ] + ([params] if params else []),
            ),
            # ② 官方瞄准节点：订阅被重映射到过滤后的话题
            Node(
                package="dzrknn_dataprocess",
                executable="dzactuator_node",
                name="dzrknn_dataprocess",
                output="screen",
                emulate_tty=True,
                remappings=[("offset_center", "/offset_center_guarded")],
                parameters=[{"max_shots": LaunchConfiguration("max_shots")}],
            ),
        ]
    )
