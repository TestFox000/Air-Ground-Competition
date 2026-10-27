# uavlink.config —— 由 tools/uav_link.py 拆出（常量与配置（主机/环境/命令映射/限速表））。与原文件逐行一致（AST 校验过）。
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import socket
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

# ---------------------------------------------------------------- 现场契约（已核实，勿改）

ORIN_USER_DEFAULT = "nvidia"          # 机载默认用户（--orin-user 可改）
DZROS_ENV = "~/dzrobot/dzros.env"     # ROS 域号真源（默认 118），不 source 就跨不到机载域
ROS_SETUP = "/opt/ros/humble/setup.bash"
ORIN_WS_SETUPS = ("~/dzrobot/install/setup.bash", "~/dzpx4/install/setup.bash")
TMUX_SESSION = "agup"                 # orin_bringup.sh 编排的全栈会话
TMUX_WINDOWS = ("dzuav", "livox", "fastlio", "px4", "wdog", "odom", "imu",
                "control", "flight_wd", "path", "teleop")
ORIN_REPO = "~/dzuav_deploy/air_ground"   # Orin 上仓库路径（orin_deploy.py REMOTE_ROOT）
# 通道候选清单：话题名无法离线核实，采样器运行期自描述枚举后按子串匹配。
# ⚠️ 顺序敏感：attitude_target 必须在 attitude 前（setpoint 名字包含 attitude）。
TOPIC_CANDIDATES = (
    ("attitude_target", ("vehicle_attitude_setpoint", "attitude_target")),
    ("attitude", ("vehicle_attitude",)),
    ("rates", ("vehicle_angular_velocity",)),
    ("position", ("vehicle_local_position",)),
    ("battery", ("battery_status",)),
    ("status", ("vehicle_status",)),
    ("estimator", ("estimator_status",)),
    ("imu", ("sensor_combined",)),
)
# 每话题限速：attitude 10Hz、position 5Hz、status/其他 1Hz（status 另有 on-change 直通）
RATE_CAPS_HZ = {"attitude": 10.0, "position": 5.0}
RATE_DEFAULT_HZ = 1.0
# 命令映射表（数据驱动）：action → MAVLink VehicleCommand。
# cmd 编号：400=COMPONENT_ARM_DISARM 22=NAV_TAKEOFF 21=NAV_LAND
#          20=NAV_RETURN_TO_LAUNCH
# ctrl 字段 = px4_control ControlCommand 码（安全通道分路，M1）：
#   2=CMD_ARM（px4_control 的 ARM 自带"起飞到机载配置高度"）3=CMD_DISARM
#   4=CMD_LAND；None=无对应码（return：RTL 本身是安全动作；kill：最后手段
#   必须穿透一切状态机，恒直发）。采样器发现 /px4_control/command 且 ctrl
#   非 None 时优先走安全通道，找不到才直发 /fmu/in/vehicle_command。
# kill 与 flight_console 同语义：400 param1=0 param2=21196（强制断解锁=电机立即
# 停转）。不用 185 DO_FLIGHTTERMINATION——它受 CBRK_FLIGHTTERM 参数影响，
# 可能被降级成普通 failsafe 而非真正停转（2026-10-16 审查修正）。
COMMAND_MAP = {
    "arm":     {"cmd": 400, "params": {"param1": 1.0}, "ctrl": 2,
                "desc": "解锁（电机可转；走 px4_control 时=ARM+起飞到配置高度）"},
    "disarm":  {"cmd": 400, "params": {"param1": 0.0}, "ctrl": 3, "desc": "上锁"},
    "takeoff": {"cmd": 22, "params": {"param7": 2.0}, "ctrl": 2,
                "desc": "起飞到 --alt 米（param7，默认 2.0；走 px4_control 时高度"
                        "由机载 takeoff_height 参数定，--alt 不生效）"},
    "land":    {"cmd": 21, "params": {}, "ctrl": 4, "desc": "原地降落"},
    "return":  {"cmd": 20, "params": {}, "ctrl": None,
                "desc": "返航（Return-to-Launch，px4_control 无对应码，直发）"},
    "kill":    {"cmd": 400, "params": {"param1": 0.0, "param2": 21196.0},
                "ctrl": None,
                "desc": "急停（强制断解锁=电机立即停转，空中=坠机，仅撞人前止损；"
                        "恒直发穿透状态机）"},
}
# 栈生命周期动作（tmux / orin_bringup.sh，不走 ROS）
STACK_ACTIONS = ("stack-status", "stack-restart")
ALL_ACTIONS = tuple(COMMAND_MAP) + STACK_ACTIONS
COMMAND_TOPIC_HINT = "vehicle_command"        # 期望 /fmu/in/vehicle_command
SAFE_CHANNEL_HINT = "px4_control/command"     # px4_control 状态机防护的安全通道
# 安全闸门（对齐 flight_console.py 的强度分层）：
#   高风险（arm/takeoff/stack-restart/kill）必须显式 --yes——默认什么都不做；
#   kill 在 --yes 之外还要再输入全词 "kill" 确认（WORD_CONFIRM）：shell 历史上
#   `ctl land --yes` 与 `ctl kill --yes` 只差一个上翻键，倒计时挡不住肌肉记忆，
#   故 kill 移出倒计时、改全词确认（flight_console.py:619 同款强度）；
#   次高风险（disarm/land/return）免 --yes 但醒目警告 + 倒计时 3s。
YES_REQUIRED = {"arm", "takeoff", "stack-restart", "kill"}
WORD_CONFIRM = {"kill": "kill"}
COUNTDOWN_ACTIONS = {"disarm", "land", "return"}
COUNTDOWN_S = 3
SAMPLER_REMOTE = "/tmp/uav_sampler.py"
HOST_CACHE = Path("logs") / "uav_link" / "uav_link.local.env"   # 上次 --host 缓存
# ---------------------------------------------------------------- logs 取回日志

# 机载端暂存脚本（单行分号串，bash -lc 单引号包裹，内层不得有单引号）：
# dzuav jsonl 最近 5 个 + ROS log 最近 3 次打包 + tmux 各窗口 capture-pane。
_LOGS_STAGE_CMD = (
    "ST=/tmp/uav_logs_stage; rm -rf $ST; mkdir -p $ST; "
    "ls -t /tmp/ag_logs/*/dzuav-*.jsonl ~/ag_logs/*/dzuav-*.jsonl 2>/dev/null "
    "| head -5 | while read f; do cp \"$f\" $ST/ 2>/dev/null || true; done; "
    "if [ -d ~/.ros/log ]; then (cd ~/.ros/log && ls -t | head -3 "
    "| tar czf $ST/ros_log_recent.tgz -T -) 2>/dev/null || true; fi; "
    + " ".join(f"tmux capture-pane -pt {TMUX_SESSION}:{w} -S -300 "
               f"> $ST/tmux_{w}.txt 2>/dev/null || true;" for w in TMUX_WINDOWS)
    + " ls -la $ST"
)
