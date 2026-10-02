"""integrate_node 启动文件：所有节点参数都可从命令行覆盖。

用法示例：
    ros2 launch ag_integrate integrate.launch.py
    ros2 launch ag_integrate integrate.launch.py dry_run:=false
    ros2 launch ag_integrate integrate.launch.py start_cell:=N10 goal_cell:=N11 \
        heading_bias_source:=udp
未传入的参数用节点默认值（见 integrate_node.py declare_parameter）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (参数名, 默认值, 说明) —— 默认值与 integrate_node.py 的 declare_parameter 保持一致。
PARAMETERS = [
    ("udp_port", "50000", "UDP 监听端口（dzuav 广播）"),
    ("waypoint_topic", "/nav/waypoint", "命令发布话题（官方 path_executor 消费）"),
    ("start_cell", "N10", "起点格号"),
    ("goal_cell", "N11", "终点格号"),
    ("start_heading", "y", "reset 断言朝向 (x/y)"),
    ("final_heading", "y", "终点朝向 (x/y)"),
    ("dry_run", "true", "true=只算不发（安全默认）"),
    ("publish_reset", "true", "是否先发 reset 再发路线"),
    ("min_mines", "1", "门控：最少确认雷数"),
    ("require_face", "true", "门控：是否要求靶面信息"),
    ("mid_boundary", "5", "中场穿越约束的行分界"),
    ("heading_bias_source", "none", "none=不启用外部朝向；udp=用无人机 0x4002 观测"),
    ("heading_bias", "0.0", "固定朝向补偿（弧度，手工标定用）"),
]


def generate_launch_description():
    actions = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in PARAMETERS
    ]
    # cross_gates 是字符串数组，launch 传数组语法麻烦，罕见改动直接改节点默认值。
    actions.append(
        Node(
            package="ag_integrate",
            executable="integrate_node",
            name="integrate_node",
            output="screen",
            parameters=[
                {name: LaunchConfiguration(name) for name, _, _ in PARAMETERS},
            ],
        )
    )
    return LaunchDescription(actions)
