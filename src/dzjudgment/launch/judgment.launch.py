#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
射击驱动 dzjudgment 的 launch 文件。

=============================================================================
  这个节点是"开火"链路上最关键、也最容易被漏掉的一环
-----------------------------------------------------------------------------
  完整链路：
      dzrknn_dataprocess（云台自瞄，判定"瞄稳了"）
          │ 发布 /LaserShot_Command (UInt8 = 1)
          ▼
      dzjudgment（本节点）—— 上位机 ↔ 车载裁判/激光模块通讯
          │ 串口写 6 字节帧  B5 5B 03 21 01 25
          ▼
      激光发射模块 → 开火

  ★ 曾经的问题：本节点【根本没被启动过】。
    start_all.sh 只起了 dzrknn_dataprocess，导致
        ros2 topic info /LaserShot_Command
        → Publisher count: 3   Subscription count: 0
    云台算得好好的开火指令全进了空话题，激光永远不亮，且完全没有报错。
    参考《案例9：云台控制与射击》：dzshooting.launch.py 必须同时启动
        dzrknn_dataprocess（云台控制） + dzjudgment（上位机与车载裁判系统通讯）

  串口参数统一从配置中心 ~/dzrobot/config/robot.yaml 的 /dzjudgment 段读取，
  与其它节点保持同一套"唯一真源"约定。
=============================================================================
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

# 配置中心路径。可用环境变量覆盖，便于用另一套参数做 A/B 对比：
#   ROBOT_CONFIG=~/dzrobot/config/robot_tuned.yaml ros2 launch ...
CONFIG_FILE = os.environ.get(
    'ROBOT_CONFIG',
    os.path.expanduser('~/dzrobot/config/robot.yaml'))


def generate_launch_description():
    if not os.path.exists(CONFIG_FILE):
        raise RuntimeError(
            '找不到配置中心文件: {}\n'
            '  请确认 ~/dzrobot/config/robot.yaml 存在，\n'
            '  或用 ROBOT_CONFIG=<路径> 指定另一份配置。'.format(CONFIG_FILE))

    node = Node(
        package='dzjudgment',
        executable='dzjudgment',
        name='dzjudgment',
        output='screen',
        emulate_tty=True,
        parameters=[CONFIG_FILE],   # ROS 2 按 YAML 原生类型装载本节点的参数段
    )

    return LaunchDescription([node])
