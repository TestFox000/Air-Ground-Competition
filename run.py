#!/usr/bin/env python3
"""一键运行入口。

本机：
    python run.py demo     # 默认：本机跑通全栈（不需要 ROS / 无人机 / 串口 / 模型）
    python run.py test     # 单元测试
    python run.py check    # 环境体检
    python run.py all      # test + demo + 复盘报告（改完代码跑这一个就够）
    python run.py log      # 看复盘报告：哪一步卡住、为什么、下一步做什么
    python run.py sim      # 有 ROS2 时启动 sim.launch.py

车端（需要 AG_CAR_PASS 环境变量，或按提示输入密码）：
    python run.py car-rehearse 172.23.116.188            # 上传+编译+干跑+取回+出报告
    python run.py car-fetch 172.23.116.188               # 只取回日志并出报告

demo 跑的是 **真实节点代码**（用 tests/ros_stub.py 顶替 rclpy），
话题接线、参数、回调状态机与实机完全一致，只是把硬件换成假数据。
Windows 上直接双击 run.bat 即可。

所有节点都会写黑匣子日志，跑完 `python run.py log` 就能看到结论。
"""

from __future__ import annotations

import argparse
import importlib
import json
import shutil
import socket
import subprocess
import sys
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent

# Windows 控制台默认是 GBK，中文输出会炸，这里强制 UTF-8 容错输出
for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def _setup_path() -> None:
    src = ROOT / "src"
    paths = [str(ROOT), str(ROOT / "tests"), str(src)]
    if src.is_dir():
        paths += [str(p) for p in sorted(src.iterdir()) if p.is_dir()]
    for p in paths:
        if p not in sys.path:
            sys.path.insert(0, p)


def _has_module(name: str) -> bool:
    try:
        importlib.import_module(name)
        return True
    except Exception:
        return False


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# ------------------------------------------------------------------ demo
def cmd_demo(args) -> int:
    _setup_path()
    import ros_stub  # noqa: F401  必须在导入节点前安装替身
    from std_msgs.msg import String  # noqa: E402

    from ag_bridge.air_link_node import AirLinkNode  # noqa: E402
    from ag_bridge.fake_intel_node import FakeIntelNode  # noqa: E402
    from ag_bridge.ground_link_node import GroundLinkNode  # noqa: E402
    from ag_chassis.chassis_node import ChassisNode  # noqa: E402
    from ag_mission.mission_node import MissionNode  # noqa: E402
    from ag_mission.target_node import TargetNode  # noqa: E402
    from ag_referee.referee_node import RefereeNode  # noqa: E402

    def build(overrides: dict, cls):
        ros_stub.set_overrides(**overrides)
        try:
            return cls()
        finally:
            ros_stub.clear_overrides()

    pa, pb = _free_port(), _free_port()

    fake = build(
        {
            "intel_topic": "/air/field_intel",
            "publish_detections": True,
            "detections_topic": "/perception/detections",
            "mode": args.mode,
            "noise_ratio": args.noise_ratio,
            "seed": args.seed,
        },
        FakeIntelNode,
    )
    air = build({"bind_port": pa, "peer_ip": "127.0.0.1", "peer_port": pb,
                 "intel_topic": "/air/field_intel"}, AirLinkNode)
    ground = build({"bind_port": pb, "peer_ip": "127.0.0.1", "peer_port": pa}, GroundLinkNode)
    mission = build({}, MissionNode)
    # 演示模式缩短开火冷却，否则一次演示要跑几十秒
    target = build({"fire_cooldown_s": 0.01}, TargetNode)
    referee = build({"simulate": True, "min_shot_interval_s": 0.01}, RefereeNode)
    # 底盘也是真节点：快进倍速让一段 3 s 的路在 0.03 s 内跑完
    chassis = build({"simulate": True, "time_scale": 100.0}, ChassisNode)

    print("=" * 66)
    print("空地协同全流程演示（真实节点代码 + ROS 替身，无需硬件）")
    print(f"  链路: 127.0.0.1:{pa} <-> {pb}   情报模式: {args.mode}   噪声: {args.noise_ratio}")
    print("=" * 66)

    last_intel_seq = -1
    last_mines_key: Optional[tuple] = None
    stable_run = 0
    last_cmd = None
    last_wp = None
    last_done = None
    finished = False
    departed = False
    step = 0

    for step in range(1, args.steps + 1):
        # 1) 机载端产出：情报帧 + 检测帧
        fake.tick()
        intel_msgs = fake.topic("/air/field_intel").published
        det_msgs = fake.topic("/perception/detections").published
        if not intel_msgs:
            break
        m = intel_msgs[-1]

        # 2) 机载链路下发（UDP）
        air.deliver("/air/field_intel", m)

        # 3) 车端链路接收并转发
        ground.tick()
        car_intel = ground.topic("/link/field_intel").published
        if car_intel and car_intel[-1].seq != last_intel_seq:
            last_intel_seq = car_intel[-1].seq
            mission.deliver("/link/field_intel", car_intel[-1])
            key = tuple(sorted(car_intel[-1].mines))
            stable_run = stable_run + 1 if key == last_mines_key else 1
            last_mines_key = key

        # 4) 敌我识别决策
        if det_msgs:
            target.deliver("/perception/detections", det_msgs[-1])
        gimbal = target.topic("/gimbal/command").published
        if gimbal and gimbal[-1] is not last_cmd:
            last_cmd = gimbal[-1]
            referee.deliver("/gimbal/command", last_cmd)
            if last_cmd.fire:
                print(f"  [{step:2d}] 开火: yaw={last_cmd.yaw_delta:.1f} "
                      f"pitch={last_cmd.pitch_delta:.1f} ({last_cmd.reason})")

        # 5) 裁判系统回填状态 -> 决策节点
        referee.tick()
        states = referee.topic("/referee/state").published
        if states:
            target.deliver("/referee/state", states[-1])

        # 6) 底盘（真实节点）：把路点交给它，它走完回报 task_done
        wps = mission.topic("/mission/waypoints").published
        if not wps:
            continue
        wp_msg = wps[-1]
        if not departed and stable_run < args.confirm_frames:
            if step % 3 == 0:
                print(f"  [{step:2d}] 侦察中：已确认 {len(last_mines_key or ())} 颗雷，"
                      f"等情报连续 {args.confirm_frames} 帧不变再出发（当前 {stable_run}）")
            continue
        if wp_msg is not last_wp:
            last_wp = wp_msg
            chassis.deliver("/mission/waypoints", wp_msg)
            wp = json.loads(wp_msg.data)
            if wp.get("done"):
                finished = True
                break
            print(f"  [{step:2d}] 出发 {wp['type']:<12} {wp.get('cell') or '':<4} "
                  f"+{wp['score']:<3} eta={wp['eta_s']:.1f}s")
            departed = True  # 一旦出发就持续执行，不再被侦察噪声打断

        # 底盘行进推进 + 回报
        chassis.tick()
        dones = chassis.topic("/mission/task_done").published
        if dones and dones[-1] is not last_done:
            last_done = dones[-1]
            mission.deliver("/mission/task_done", last_done)
            print(f"  [{step:2d}] 到位 {last_done.data}")

    # ---------------- 结算 ----------------
    plan = mission._plan
    score = mission.planner.estimate_score(plan)
    time_s = mission.planner.estimate_time(plan)
    st = referee.state
    strike_score = 5 * st.enemy_count - 5 * st.friend_count
    total = score - max(0, 15 - strike_score)

    print("-" * 66)
    print(f"规划任务数 {len(plan)}，已完成 {mission._index}")
    print(f"行驶耗时 {time_s:.1f} s / 180 s   行驶得分 {score}")
    print(f"开火 {st.shots} 发，敌计数 {st.enemy_count}/3，友计数 {st.friend_count}，余弹 {st.ammo}")
    print(f"打击得分 {strike_score}")
    print(f"预估总分 {total} / 85")
    print("=" * 66)

    if not finished:
        print("[未跑完] 步数不够，试试 --steps 60")
        return 1
    if st.friend_count > 0:
        print("[FAIL] 出现误击友方")
        return 1
    print("[PASS] 全流程跑通")
    return 0


# ------------------------------------------------------------------ test
def cmd_test(args) -> int:
    _setup_path()
    cmd = [sys.executable, "-m", "pytest", "tests", "-q"]
    # getattr：`all` 子命令没有 -k（它是复用 cmd_test 的），原来这里直接取 args.keyword
    # 会让 `python run.py all` 直接崩掉。
    keyword = getattr(args, "keyword", "")
    if keyword:
        cmd += ["-k", keyword]
    return subprocess.call(cmd, cwd=str(ROOT))


# ------------------------------------------------------------------ check
def cmd_check(_args) -> int:
    _setup_path()
    print(f"Python      : {sys.version.split()[0]} ({sys.executable})")
    ok = True

    for name, required, hint in (
        ("numpy", True, "pip install numpy"),
        ("pytest", False, "pip install pytest"),
        ("yaml", False, "pip install PyYAML（只影响离线脚本）"),
        ("cv2", False, "pip install opencv-python（标定/可视化）"),
        ("serial", False, "pip install pyserial（车端串口）"),
    ):
        has = _has_module(name)
        mark = "OK " if has else ("缺失" if required else "可选")
        if required and not has:
            ok = False
        print(f"{name:<11} : {mark}" + ("" if has else f"   -> {hint}"))

    git = shutil.which("git")
    print(f"git         : {git or '未安装 —— 强烈建议装（winget install Git.Git），不然改坏了回不去'}")

    ros2 = shutil.which("ros2")
    print(f"ros2        : {ros2 or '未安装（本机可跑 demo / test，不能跑 launch）'}")
    print(f"colcon      : {shutil.which('colcon') or '未安装'}")

    print("\n参数一致性检查:")
    rc = subprocess.call(
        [sys.executable, "-m", "pytest", "tests/test_params_consistency.py", "-q"],
        cwd=str(ROOT),
    )
    if rc != 0:
        ok = False

    print("\n结论: " + ("环境可用，继续 `python run.py demo`" if ok else "缺少必需依赖，请先安装"))
    return 0 if ok else 1


# ------------------------------------------------------------------ sim
def cmd_sim(args) -> int:
    if not shutil.which("ros2"):
        print("未检测到 ros2。本机无 ROS 时请先跑 `python run.py demo`（等价的全流程，不需要 ROS）。")
        return 1
    launch = ["ros2", "launch", "ag_bringup", "sim.launch.py"]
    if args.mode:
        launch += [f"mode:={args.mode}"]
    return subprocess.call(launch, cwd=str(ROOT))


def cmd_all(args) -> int:
    rc = cmd_test(args)
    if rc != 0:
        return rc
    print()
    rc = cmd_demo(args)
    if rc != 0:
        return rc
    # 顺手出复盘报告：改完代码跑 `run.py all` 就能同时看到"测试过了"和"这一局是怎么跑的"。
    # 报告本身失败（比如没生成日志）不影响 all 的结论。
    print()
    _run_report()
    return 0


# ------------------------------------------------------------------ 复盘 / 车端
def _run_report(log_dir=None, out=None, strict=False, as_json=False, session=None,
                all_=False, max_gap=None, timeline=None, list_only=False) -> int:
    """调 tools/log_report.py（放在子进程里跑，保证它的编码处理独立于本脚本）。"""
    argv = [sys.executable, str(ROOT / "tools" / "log_report.py")]
    for flag, val in (("--dir", log_dir), ("--session", session), ("--out", out)):
        if val:
            argv += [flag, str(val)]
    for flag, val in (("--max-gap", max_gap), ("--timeline", timeline)):
        if val is not None:
            argv += [flag, str(val)]
    if all_:
        argv.append("--all")
    if as_json:
        argv.append("--json")
    if strict:
        argv.append("--strict")
    if list_only:
        argv.append("--list")
    return subprocess.call(argv, cwd=str(ROOT))


def cmd_log(args) -> int:
    return _run_report(args.dir, args.out, args.strict, args.json,
                       args.session, args.all, args.max_gap, args.timeline, args.list)


def _remote(arg_list) -> int:
    """调 tools/remote_run.py（需要 paramiko；密码走 AG_CAR_PASS 或交互输入）。"""
    cmd = [sys.executable, str(ROOT / "tools" / "remote_run.py")]
    return subprocess.call(cmd + [str(a) for a in arg_list], cwd=str(ROOT))


def _fetch_and_report(args) -> int:
    print(f"[取回] {args.host}:{args.remote} -> {args.to}")
    rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "300",
                  "--get", args.remote, "--to", args.to])
    if rc != 0:
        print("取回失败。车连不上多半是网段变了 —— 见 docs/实机接口对照.md §9 的排查三步。",
              file=sys.stderr)
        return rc
    print()
    print("[复盘] 车端日志")
    return _run_report(args.to, args.out, args.strict)


def cmd_car_fetch(args) -> int:
    return _fetch_and_report(args)


def cmd_preflight(args) -> int:
    """上场前体检：把配置/环境里"最容易忘"的项自动查一遍。"""
    argv = []
    if args.params:
        argv += ["--params", args.params]
    if args.run_tests:
        argv.append("--run-tests")
    return subprocess.call(
        [sys.executable, str(ROOT / "tools" / "preflight_check.py")] + argv, cwd=str(ROOT))


def cmd_car_camera(args) -> int:
    """相机健康检查：官方 usb_cam 崩了以后整条链是静默的，任何视觉联调前先跑这个。"""
    return _remote(["--host", args.host, "--user", args.user, "--timeout", "300",
                    "--script", "tools/car_camera_check.sh",
                    "--out", "docs/probe/car_camera_check.txt"])


def cmd_car_rehearse(args) -> int:
    """上传 → 编译 → 车端全链路干跑 → 取回 → 出报告，一条命令走完。"""
    if args.build:
        print("[1/4] 上传源码并编译（几分钟）")
        rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "300",
                      "--put", "src", "--dest", f"{args.ws}/src"])
        if rc != 0:
            return rc
        rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "900",
                      "--script", "tools/car_build.sh", "--out", "docs/probe/car_build.txt"])
        if rc != 0:
            return rc
    else:
        print("[1/4] 跳过上传/编译（要一起做加 --build）")

    print(f"[2/4] 车端全链路干跑：不动车，dry_run，跑 {args.run_s}s")
    rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "600",
                  "--script", "tools/car_log_rehearsal.sh",
                  "--env", f"RUN_S={args.run_s}",
                  "--out", "docs/probe/car_log_rehearsal.txt"])
    if rc != 0:
        print("车端干跑失败。看 docs/probe/car_log_rehearsal.txt 里的节点日志尾部。",
              file=sys.stderr)
        return rc

    print("[3/4] 取回日志")
    rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "300",
                  "--get", args.remote, "--to", args.to])
    if rc != 0:
        return rc

    print("[4/4] 复盘报告")
    print()
    return _run_report(args.to, args.out, args.strict)


def build_parser() -> argparse.ArgumentParser:
    """单独抽出来是为了可测：测试会解析每个子命令并核对字段是否齐全。

    真实事故：`all` 子命令复用 `cmd_test`，但 `all` 没有 `-k/--keyword`，
    `python run.py all` 直接 AttributeError 崩掉（README 里还写着这条命令）。
    有了这个函数，`tests/test_run_cli.py` 就能直接拦下这类"少一个字段"的问题。
    """
    ap = argparse.ArgumentParser(description="air_ground 一键运行入口")
    sub = ap.add_subparsers(dest="cmd")

    def _demo_args(p):
        p.add_argument("--steps", type=int, default=60)
        p.add_argument("--confirm-frames", type=int, default=3,
                       help="情报连续多少帧不再变化才让车出发")
        p.add_argument("--mode", default="progressive", choices=["progressive", "full"])
        p.add_argument("--noise-ratio", type=float, default=0.2)
        p.add_argument("--seed", type=int, default=20260927)
        p.set_defaults(func=cmd_demo)

    p_demo = sub.add_parser("demo", help="本机跑通全栈（默认）")
    _demo_args(p_demo)

    p_test = sub.add_parser("test", help="单元测试")
    p_test.add_argument("-k", "--keyword", default="", help="只跑匹配的用例")
    p_test.set_defaults(func=cmd_test)

    p_check = sub.add_parser("check", help="环境体检")
    p_check.set_defaults(func=cmd_check)

    p_sim = sub.add_parser("sim", help="ROS2 全链路仿真（需 ROS2）")
    p_sim.add_argument("--mode", default="")
    p_sim.set_defaults(func=cmd_sim)

    p_all = sub.add_parser("all", help="test + demo + 复盘报告")
    _demo_args(p_all)
    p_all.add_argument("-k", "--keyword", default="", help="只跑匹配的用例（透传给 pytest）")
    p_all.set_defaults(func=cmd_all)

    p_log = sub.add_parser("log", help="复盘报告：哪一步卡住、为什么、下一步做什么")
    p_log.add_argument("--dir", default=None,
                       help="日志目录（默认 $AG_LOG_DIR，再退到系统临时目录/ag_logs）")
    p_log.add_argument("--session", default=None, help="只看某个会话子目录")
    p_log.add_argument("--all", action="store_true", help="不切局，读目录里全部事件")
    p_log.add_argument("--json", action="store_true", help="输出 JSON（喂给脚本/CI）")
    p_log.add_argument("--strict", action="store_true", help="有严重结论时退出码为 1")
    p_log.add_argument("--out", default=None, help="把 Markdown 报告写到文件")
    p_log.add_argument("--max-gap", type=float, default=None, help="静默告警阈值（秒）")
    p_log.add_argument("--timeline", type=int, default=None, help="时间轴最多几条（0=全部）")
    p_log.add_argument("--list", action="store_true", help="只列出找到的日志目录")
    p_log.set_defaults(func=cmd_log)

    def _car_args(p):
        p.add_argument("host", help="车端 IP（会变，见 docs/实机接口对照.md §9）")
        p.add_argument("--user", default="duzhong")
        p.add_argument("--remote", default="/tmp/ag_logs", help="车端日志根目录")
        p.add_argument("--to", default="logs/car", help="取回到本地的目录")
        p.add_argument("--out", default=None, help="报告另存为 Markdown")
        p.add_argument("--strict", action="store_true", help="有严重结论时退出码为 1")

    p_fetch = sub.add_parser("car-fetch", help="取回车端日志并出复盘报告")
    _car_args(p_fetch)
    p_fetch.set_defaults(func=cmd_car_fetch)

    p_reh = sub.add_parser("car-rehearse", help="车端全链路干跑：编译+跑一圈+取回+报告（不动车）")
    _car_args(p_reh)
    p_reh.add_argument("--build", action="store_true", help="先上传源码并 colcon build")
    p_reh.add_argument("--ws", default="/tmp/ag_ws", help="车端工作空间（默认 /tmp/ag_ws）")
    p_reh.add_argument("--run-s", type=int, default=25, help="干跑时长（秒）")
    p_reh.set_defaults(func=cmd_car_rehearse)

    p_pre = sub.add_parser("preflight", help="上场前体检：配置/环境里最容易忘的项逐条查")
    p_pre.add_argument("--params", default=None, help="指定 params.yaml（默认仓库里的那份）")
    p_pre.add_argument("--run-tests", action="store_true", help="顺便跑一遍单元测试")
    p_pre.set_defaults(func=cmd_preflight)

    p_cam = sub.add_parser("car-camera", help="车端相机健康检查（零动作，视觉联调前先跑）")
    p_cam.add_argument("host", help="车端 IP")
    p_cam.add_argument("--user", default="duzhong")
    p_cam.set_defaults(func=cmd_car_camera)

    return ap


def main(argv: Optional[list] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        # 不给子命令时等价于 `run.py demo`（保持双击 run.bat 的习惯）
        args = ap.parse_args(["demo"] + list(argv or []))
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
