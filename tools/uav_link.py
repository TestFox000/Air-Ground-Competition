"""无人机地面站：看得见（遥测）、控得住（调控）、查得回（黑匣子 + 复盘）。

针对"飞控很不稳定，需要知道无人机正在发生什么"的现场需求：
  run     只读遥测 + 黑匣子：SSH 到机载电脑（Jetson Orin Nano），上传内嵌采样器
          （rclpy 自描述枚举话题），NDJSON 流回笔记本逐行落盘（黑匣子），
          2Hz 终端仪表盘 + 规则引擎实时告警（FAILSAFE/EKF/姿态发散/速率饱和/
          振动削波/低电/dzuav 静默/链路滞后）。Ctrl-C 优雅退出并写收尾统计。
  ctl     调控支配（全部显式闸门，默认什么都不做）：
          arm/takeoff/stack-restart/kill 必须 --yes；kill 在 --yes 之外还要
          再输入全词 "kill" 确认（对齐 flight_console.py 的双确认强度——shell
          历史上 `ctl land --yes` 与 `ctl kill --yes` 只差一个上翻键，3s 倒计时
          挡不住肌肉记忆回车）；disarm/land/return 免 --yes 但醒目警告 +
          3s 倒计时（--yes 跳过，倒计时内 Ctrl-C 友好中止）；stack-status 只读。
          命令通道分层（对齐 flight_console 设计）：采样器发现
          /px4_control/command 时 arm/disarm/takeoff/land 优先走 px4_control
          状态机防护的安全通道（空中拒 DISARM、地面才 ARM）；找不到安全通道
          才直发 /fmu/in/vehicle_command，且执行前打印醒目提示；kill 恒直发
          （最后手段必须穿透一切状态机）；return 无对应 ControlCommand 码，
          保持直发（RTL 本身是安全动作）。找不到任何命令通道时响亮失败
          （列出实际发现的话题），绝不盲发。
  logs    取回日志：dzuav jsonl、ROS log 近况、tmux 各窗口 capture-pane，
          打包到 logs/uav_link/<时间戳>/ 并列清单。
  report  离线复盘：读一个黑匣子 jsonl，按 严重/注意/信息 分级输出
          （armed 时间线、failsafe 事件、姿态/角速度统计、姿态-目标偏差、
          vibe clip、链路空洞、dzuav 健康摘要）。**双格式兼容**：除本工具
          run 落的采样格式（topic 键）外，也直接吃并行会话机载黑匣子
          tools/orin_flight_blackbox.py 的 flight_bb JSONL（ev 键判别，
          事件→分析维度映射见 report 节 FLIGHT_BB_* 常量与 build_report）。

现场架构（已核实，勿重查）：
  Orin 上官方栈 dzpx4：Livox MID360 → FASTLIO2 → PX4（MicroXRCEAgent DDS 桥，
  **不是裸 MAVLink**），另有我们的 dzuav 侦察进程；tools/orin_bringup.sh 用
  tmux 编排（会话 agup，窗口 dzuav/livox/fastlio/px4/wdog/odom/imu/control/
  flight_wd/path/teleop——wdog 是飞控链路看门狗、flight_wd 是机载自治 LAND
  守卫，恰是坠机复盘最要的证据窗口）。
  ROS 域号真源 ~/dzrobot/dzros.env（默认 118）——所有机载 ROS 命令必须先 source 它。
  PX4 经 XRCE 的话题通常是 /fmu/out/*，但具体话题名无法离线验证，采样器自描述发现。

用法:
  set AG_ORIN_PASS=******
  python tools/uav_link.py run --host 10.15.166.60            # 遥测 + 黑匣子
  python tools/uav_link.py ctl stack-status                   # 只读看栈状态
  python tools/uav_link.py ctl arm --yes                      # 解锁（高风险）
  python tools/uav_link.py ctl land                           # 3s 倒计时后降落
  python tools/uav_link.py ctl --print-caps                   # 看机载发现的命令通道
  python tools/uav_link.py logs                               # 取回日志打包
  python tools/uav_link.py report logs/uav_link/20261016_120000.jsonl

依赖: paramiko（仅笔记本端）；Orin 侧只用 rclpy + 标准库。
"""
from __future__ import annotations

# 2026-10-27 模块化拆分：实现移入 tools/uavlink/ 包（config/telemetry/safety/host/rules/blackbox/sampler_src/link/report/cli）。
# 本文件是对外稳定门面：CLI 用法、import uav_link 语义均不变；所有名字自包内再导出。

from uavlink.config import ALL_ACTIONS, COMMAND_MAP, COMMAND_TOPIC_HINT, COUNTDOWN_ACTIONS, COUNTDOWN_S, DZROS_ENV, HOST_CACHE, ORIN_REPO, ORIN_USER_DEFAULT, ORIN_WS_SETUPS, RATE_CAPS_HZ, RATE_DEFAULT_HZ, ROS_SETUP, SAFE_CHANNEL_HINT, SAMPLER_REMOTE, STACK_ACTIONS, TMUX_SESSION, TMUX_WINDOWS, TOPIC_CANDIDATES, WORD_CONFIRM, YES_REQUIRED, _LOGS_STAGE_CMD  # noqa: F401
from uavlink.telemetry import RateLimiter, TelemetryState, _float_or_none, _percentile, _wrap_deg, choose_command_route, choose_qos, classify_topic, estimator_flags_ok, find_command_channel, parse_sampler_line, quat_to_euler_deg, render_channel_failure, render_command_map, render_dashboard  # noqa: F401
from uavlink.safety import check_gate, clamp_alt, confirm_word, countdown  # noqa: F401
from uavlink.host import load_cached_host, probe_ssh, read_password, resolve_host, save_cached_host  # noqa: F401
from uavlink.rules import UavRuleEngine  # noqa: F401
from uavlink.blackbox import BlackboxWriter, read_blackbox  # noqa: F401
from uavlink.sampler_src import SAMPLER_SRC, sampler_launch_cmd  # noqa: F401
from uavlink.link import OrinLink, _connect, _exec_collect, _link_preflight, _sftp_get_tree, _summary_lines, _upload_sampler  # noqa: F401
from uavlink.report import FLIGHT_BB_KILL, FLIGHT_BB_NAV_NAMES, FLIGHT_BB_PAIR_WINDOW_S, FLIGHT_BB_SLAM_GAP_S, FLIGHT_BB_VC_NAMES, SEVERITY_LABEL, _SEVERITY_RANK, _fb_armed_at, _fb_close_slam_gap, _feed_flight_bb_row, _pair_fb_att_target, _pair_fb_pos_target, _row_t, _stat_line, build_findings, build_report, classify_blackbox_row, cmd_report, render_report  # noqa: F401
from uavlink.cli import _add_link_args, build_parser, cmd_ctl, cmd_logs, cmd_run, main  # noqa: F401

if __name__ == "__main__":
    raise SystemExit(main())
