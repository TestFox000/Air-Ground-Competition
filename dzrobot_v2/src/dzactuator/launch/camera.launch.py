#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机 usb_cam · 启动文件
================================================================
★ 参数集中在  ~/dzrobot/config/robot.yaml  的 /usb_cam_node 段，
  本文件不再重复定义，避免"改一处、另一处不跟着变"。
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

    cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        # ★ 相机进程意外退出时自动重启。
        #   实测过的故障：上一个相机进程刚被杀、/dev/video0 还没完全释放，
        #   新进程抢不到设备会直接 abort(exit -6) —— 表现就是"页面显示无信号"。
        #   respawn 让它 2 秒后自动再试，不必整趟重来。
        respawn=True,
        respawn_delay=2.0,
        parameters=[CONFIG_FILE],
    )

    return LaunchDescription([cam])
