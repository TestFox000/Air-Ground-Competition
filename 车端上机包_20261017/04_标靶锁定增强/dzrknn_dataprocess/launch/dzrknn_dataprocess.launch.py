
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
云台自瞄节点 dzrknn_dataprocess · 启动文件
================================================================
★ 所有可调参数都在  ~/dzrobot/config/robot.yaml  里，本文件不再重复定义。

为什么不再把参数写在这里（历史教训）：
  ① 参数曾散落在三处 —— .h 的成员默认值 / .cpp 的 declare_parameter / 本文件的
     parameters 字典。改一处另两处不跟着变，曾导致"俯仰 -6° 改了完全没生效"。
  ② 曾用 LaunchConfiguration('xxx') 传参，它给出的是【字符串】'0.35'，
     而节点声明的是 double —— 类型不匹配会【静默失败】，参数根本没设上
     （实测 ros2 param get 返回 "Parameter not set"，而节点悄悄用了代码默认值）。
  ③ 本文件里还曾把两个参数挤压进注释行，导致 shot_cooldown_ms /
     target_confirm_frames 实际未传入。

改用 ROS 2 原生 params YAML 后：类型与取值都由 YAML 负责，只有一处真源，
且启动后可用  tools/healthcheck.sh  直接核对是否真的生效。
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

# 配置中心路径。可用环境变量覆盖，便于用另一套参数做 A/B 对比：
#   ROBOT_CONFIG=~/dzrobot/config/robot_tuned.yaml ros2 launch ...
CONFIG_FILE = os.environ.get(
    'ROBOT_CONFIG',
    os.path.expanduser('~/dzrobot/config/robot.yaml'))

NODE_NAME = 'dzrknn_dataprocess'


def generate_launch_description():
    if not os.path.exists(CONFIG_FILE):
        raise RuntimeError(
            '找不到配置中心文件: {}\n'
            '  请确认 ~/dzrobot/config/robot.yaml 存在，\n'
            '  或用 ROBOT_CONFIG=<路径> 指定另一份配置。'.format(CONFIG_FILE))

    node = Node(
        package='dzrknn_dataprocess',
        executable='dzactuator_node',
        name=NODE_NAME,
        output='screen',
        parameters=[CONFIG_FILE],   # ROS 2 按 YAML 原生类型装载本节点的参数段
    )

    return LaunchDescription([node])
