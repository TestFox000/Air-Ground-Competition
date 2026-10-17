"""B9 决定性实验工具：验证车转弯/绕圈问题的根因与修复。

背景（docs/接手说明.md §3）：
    reset:N10,y 盲目断言车头朝 y(90°)。若真实朝向偏差 δ，纯追踪永远追不上
    目标 → 车绕半径约 0.43m 画圆。B9 实验就是控制变量验证这一点：
    把车**物理左转约 δ（实测 68.6°）**后，重发 reset + 一格命令，看车是否
    直线到达 —— 以此区分「摆位问题」还是「yaw 源头问题」。

用法（密码走 AG_CAR_PASS 环境变量，与 remote_run.py 一致）：
    set AG_CAR_PASS=******
    python tools/b9_test.py --host <车IP> topics              # 看车端话题现状
    python tools/b9_test.py --host <车IP> reset               # 发 reset:N10,y
    python tools/b9_test.py --host <车IP> goto N9             # 发一格命令（避开 K 列）
    python tools/b9_test.py --host <车IP> watch --duration 30 # 监听 /odom/localization 30 秒
    python tools/b9_test.py --host <车IP> full N9             # 完整实验：reset→等2s→goto→watch

注意：
    /odom/localization 与 /pursuitAngle 都是 BEST_EFFORT（接手说明 §5.4），
    本工具的 echo 命令已带 --qos-reliability best_effort，别用裸 ros2 topic echo。
    位置单位是**格**，不是米（历史报告偏大 2.86 倍的坑）。

依赖：pip install paramiko（仅本机）。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

try:
    import paramiko
except ImportError:  # pragma: no cover
    print("缺少 paramiko，请先执行: python -m pip install paramiko", file=sys.stderr)
    raise SystemExit(2)


WAYPOINT_TOPIC = "/nav/waypoint"
ODOM_TOPIC = "/odom/localization"      # 单位：格（block），不是米
PURSUIT_TOPIC = "/pursuitAngle"        # 转向角（BEST_EFFORT）


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    if not sys.stdin.isatty():
        raise SystemExit(
            "缺少车端密码：请 set AG_CAR_PASS=<密码> 或 --password <密码>"
        )
    import getpass
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def run_ssh(host: str, user: str, password: str, cmd: str, timeout: float = 30) -> tuple[int, str]:
    """执行一条远程命令，返回 (退出码, 输出)。"""
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password, timeout=15)
    try:
        _, stdout, stderr = client.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        code = stdout.channel.recv_exit_status()
        return code, (out + ("\n[stderr]\n" + err if err.strip() else "")).strip()
    finally:
        client.close()


def ros_cmd(cmd: str) -> str:
    """包一层 ROS 环境加载（车端默认没 source）。"""
    return f"source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/*/setup.bash; {cmd}"


# ---------------------------------------------------------------- 子命令

def cmd_topics(args) -> int:
    code, out = run_ssh(args.host, args.user, args.password,
                        ros_cmd("ros2 node list; echo ---; ros2 topic list"), timeout=20)
    print(out)
    return code


def cmd_reset(args) -> int:
    reset_cmd = f"reset:{args.cell},{args.heading}"
    print(f"[B9] 发送 reset: {reset_cmd!r} -> {WAYPOINT_TOPIC}")
    pub = (f"ros2 topic pub {WAYPOINT_TOPIC} std_msgs/msg/String "
           f'"data: \'{reset_cmd}\'" --once --timelimit 5')
    code, out = run_ssh(args.host, args.user, args.password, ros_cmd(pub), timeout=20)
    print(out if out else "(无输出，--once 模式正常)")
    print("[B9] reset 已发。车端 belief 已重置，可发 goto 命令。")
    return code


def cmd_goto(args) -> int:
    print(f"[B9] 发送一格命令: {args.target!r} -> {WAYPOINT_TOPIC}")
    pub = (f"ros2 topic pub {WAYPOINT_TOPIC} std_msgs/msg/String "
           f'"data: \'{args.target}\'" --once --timelimit 5')
    code, out = run_ssh(args.host, args.user, args.password, ros_cmd(pub), timeout=20)
    print(out if out else "(无输出，--once 模式正常)")
    print("[B9] 命令已发，观察车是否直线行驶。")
    return code


def cmd_watch(args) -> int:
    duration = args.duration
    print(f"[B9] 监听 {ODOM_TOPIC} {duration}s（BEST_EFFORT，单位=格）...")
    # 直接 echo 原始输出，行前缀时间戳便于事后分析
    echo = (f"timeout {duration} ros2 topic echo {ODOM_TOPIC} "
            f"--qos-reliability best_effort --field pose.pose.position")
    code, out = run_ssh(args.host, args.user, args.password, ros_cmd(echo), timeout=duration + 15)
    print(out if out.strip() else "(未收到任何消息：节点没起或话题名不对)")
    if not out.strip():
        print("[提示] 先确认车端节点在跑：python tools/b9_test.py --host ... topics")
    return 0


def cmd_full(args) -> int:
    """完整 B9 实验：reset → 2s → goto → watch → 汇报。"""
    reset_cmd = f"reset:{args.cell},{args.heading}"
    print("=" * 60)
    print(f"[B9 完整实验] 目标格: {args.target}")
    print(f"  前提：车已物理摆到与 reset 断言朝向一致（{args.heading} 方向）")
    print("=" * 60)

    print(f"\n[1/4] 发送 {reset_cmd!r}")
    pub = (f"ros2 topic pub {WAYPOINT_TOPIC} std_msgs/msg/String "
           f'"data: \'{reset_cmd}\'" --once --timelimit 5')
    run_ssh(args.host, args.user, args.password, ros_cmd(pub), timeout=20)

    print("[2/4] 等 2 秒让 path_executor 消费 reset ...")
    time.sleep(2)

    print(f"[3/4] 发送一格命令 {args.target!r}，车应开始移动")
    pub = (f"ros2 topic pub {WAYPOINT_TOPIC} std_msgs/msg/String "
           f'"data: \'{args.target}\'" --once --timelimit 5')
    run_ssh(args.host, args.user, args.password, ros_cmd(pub), timeout=20)

    print(f"[4/4] 监听 {ODOM_TOPIC} {args.duration}s（单位=格，BEST_EFFORT）")
    echo = (f"timeout {args.duration} ros2 topic echo {ODOM_TOPIC} "
            f"--qos-reliability best_effort --field pose.pose.position")
    code, out = run_ssh(args.host, args.user, args.password, ros_cmd(echo),
                        timeout=args.duration + 15)
    print(out if out.strip() else "(未收到任何位姿消息)")

    print("\n" + "=" * 60)
    print("[判定标准]")
    print("  ✅ 直线到达目标格附近并停下 → 根因确认=摆位偏差，物理对齐有效")
    print("  ❌ 仍然绕圈/大幅偏离     → 问题在 yaw 源头，需改定位流程")
    print("  （位置单位是格；行进效率=直线位移/轨迹长度，接近 1.0 为直线）")
    print("=" * 60)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="车端 IP（如 192.168.1.188）")
    parser.add_argument("--user", default="duzhong")
    parser.add_argument("--password", default=None, help="建议用 AG_CAR_PASS 环境变量")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("topics", help="查看车端节点与话题")

    p_reset = sub.add_parser("reset", help="发 reset 命令重置车端 belief")
    p_reset.add_argument("--cell", default="N10", help="reset 格号（默认 N10）")
    p_reset.add_argument("--heading", default="y", choices=["x", "y"],
                         help="断言朝向（默认 y）")

    p_goto = sub.add_parser("goto", help="发一格命令")
    p_goto.add_argument("target", help="目标格，如 N9（避开 K 列）")

    p_watch = sub.add_parser("watch", help="监听车位姿（BEST_EFFORT）")
    p_watch.add_argument("--duration", type=int, default=30, help="监听秒数（默认 30）")

    p_full = sub.add_parser("full", help="完整 B9 实验：reset→goto→watch→判定")
    p_full.add_argument("target", help="目标格，如 N9")
    p_full.add_argument("--cell", default="N10")
    p_full.add_argument("--heading", default="y", choices=["x", "y"])
    p_full.add_argument("--duration", type=int, default=30)

    args = parser.parse_args()
    args.password = read_password(args)

    handlers = {"topics": cmd_topics, "reset": cmd_reset, "goto": cmd_goto,
                "watch": cmd_watch, "full": cmd_full}
    return handlers[args.command](args)


if __name__ == "__main__":
    raise SystemExit(main())
