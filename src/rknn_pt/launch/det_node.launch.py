#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视觉检测 det_node · 启动文件
================================================================
★ 参数集中在  ~/dzrobot/config/robot.yaml  的 /det_node 段。

顺带清理：原来的 launch 里传了 robot_color / tracking.* / show_results 三个参数，
但节点代码从来没有 declare 过它们 —— ROS 2 会【静默丢弃】未声明的覆盖项，
所以那三个参数一直是摆设，只会让人误以为"设了有用"。现已移除。
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

CONFIG_FILE = os.environ.get(
    'ROBOT_CONFIG',
    os.path.expanduser('~/dzrobot/config/robot.yaml'))


def generate_launch_description():
    if not os.path.exists(CONFIG_FILE):
        raise RuntimeError(
            '找不到配置中心文件: {}\n'
            '  请确认 ~/dzrobot/config/robot.yaml 存在。'.format(CONFIG_FILE))

    det_node = Node(
        package='det_node',
        executable='det_node',
        name='det_node',
        output='screen',
        parameters=[CONFIG_FILE],
    )

    return LaunchDescription([det_node])
