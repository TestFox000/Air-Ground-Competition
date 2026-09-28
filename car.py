#!/usr/bin/env python3
"""车端动作入口 —— **和本机的 `run.py` 是两回事，别混**。

    run.py   在本机跑：测试 / 仿真 / 复盘报告       （不需要车）
    car.py   对车下命令：恢复官方栈 / 让车能动 / 急停 / 取日志   （必须连上车）

为什么单独一个入口：这些命令**会让车真的动起来**，不适合和"本机随便跑"的
入口混在一起 —— 想发车就得明确敲 `car.py move ... --really-move`。

用法（密码走环境变量 AG_CAR_PASS，或按提示输入）：

    python car.py restore 172.23.116.188          # 恢复官方栈（编译 dzactuator，只读源码）
    python car.py deploy  172.23.116.188          # 传源码到车上并编译（**车重启后 /tmp 被清，先跑这个**）
    python car.py move    172.23.116.188          # 一键起：底盘+定位+执行器+决策层 —— dry_run，车不动
    python car.py move    172.23.116.188 --with-vision            # 再加 相机 + det_node(RKNN 检测)
    python car.py move    172.23.116.188 --really-move --run-s 60 # ★真发车（必须有人在场）★
    python car.py stop    172.23.116.188          # 紧急停止：把能动/能开火的节点全停掉
    python car.py logs    172.23.116.188          # 取回黑匣子日志 + 出复盘报告

⚠️ 一键启动**不会**启动瞄准/开火链 `dzrknn_dataprocess`：它一跑就会扫云台并**自主开火**，
   而且不区分蓝红（官方"停车才开火"的 `/move_base/stop_signal` 是死代码，无订阅者）。
   要开火链必须单独、显式地起，且必须有人在场。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent

for stream in (sys.stdout, sys.stderr):
    try:
        stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def _remote(arg_list) -> int:
    """统一走 tools/remote_run.py（它负责 SSH/密码/上传脚本）。"""
    cmd = [sys.executable, str(ROOT / "tools" / "remote_run.py")]
    return subprocess.call(cmd + [str(a) for a in arg_list], cwd=str(ROOT))


def _script(name: str, host: str, user: str, timeout: float,
            env: Optional[list] = None, out: Optional[str] = None) -> int:
    args = ["--host", host, "--user", user, "--timeout", str(timeout),
            "--script", f"tools/{name}"]
    for kv in env or []:
        args += ["--env", kv]
    if out:
        args += ["--out", out]
    return _remote(args)


def cmd_restore(args) -> int:
    print("[1/1] 恢复官方栈（只读官方源码，编译产物放 /tmp，不动 ~/dzrobot）")
    return _script("car_restore_official.sh", args.host, args.user, 900,
                   out="docs/probe/car_restore_official.txt")


def cmd_deploy(args) -> int:
    """上传我们的源码到车端 /tmp 工作区并编译。

    为什么"一键启动"之前必须先有这一步：车上的 `/tmp` **重启就清空**，
    我们自己的 9 个包（`ag_car` / `ag_bridge` …）就都没了，
    此时 `move` 会在前置检查里直接报缺包。以前要敲两条 `tools/remote_run.py`，
    现在一条 `car.py deploy` 即可（官方栈不受影响，它是另一套，用 `restore` 修）。
    """
    print(f"[1/2] 上传源码 -> {args.ws}/src")
    rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "600",
                  "--put", "src", "--dest", f"{args.ws}/src"])
    if rc != 0:
        print("上传失败 —— 车连不上？IP 会变，见 docs/实机接口对照.md §9", file=sys.stderr)
        return rc
    print(f"[2/2] 车端编译（工作区 {args.ws}）")
    rc = _script("car_build.sh", args.host, args.user, 900,
                 env=[f"AG_WS={args.ws}"], out="docs/probe/car_build.txt")
    if rc == 0:
        print()
        print(f"下一步：python car.py move {args.host} --with-vision")
    return rc


def cmd_move(args) -> int:
    if args.really_move:
        print("=" * 66)
        print("★ 真发车模式：车会按路线真的动起来 ★")
        print("  确认：① 有人站在车边、遥控器在手  ② 车周围 2 m 净空或已架起")
        print("  车端还会再等 5 秒才开始；中途要停就 Ctrl-C 或另开一个终端跑 car.py stop")
        print("=" * 66)
    else:
        print("只验链路模式（car_node 用 dry_run=true）：底盘会起来，但**一条命令都不发**，车不会动")

    if args.with_vision:
        print("  视觉链：相机 + det_node(RKNN) 会一起起来；**瞄准/开火链仍然不开**")
        print("  注意：det_node 内部要画框，需要图形后端（车上有 VNC 桌面时脚本会自己借用）")

    env = [f"AG_RUN_S={args.run_s}"]
    if args.really_move:
        env.append("AG_CONFIRM_MOVE=1")
    if args.use_air:
        env.append("AG_USE_AIR=1")
    if args.no_judgment:
        env.append("AG_NO_JUDGMENT=1")
    if args.with_vision:
        env.append("AG_WITH_VISION=1")

    rc = _script("car_move_dryrun.sh", args.host, args.user, args.timeout, env=env,
                 out="docs/probe/car_move.txt")
    if rc == 0:
        print()
        print("想复盘这一趟：python car.py logs %s" % args.host)
    return rc


def cmd_stop(args) -> int:
    print("紧急停止：把车上能动/能开火的节点全部停掉")
    return _script("car_stop_all.sh", args.host, args.user, 180, out="docs/probe/car_stop.txt")


def cmd_logs(args) -> int:
    print(f"[1/2] 取回车端日志 {args.remote} -> {args.to}")
    rc = _remote(["--host", args.host, "--user", args.user, "--timeout", "300",
                  "--get", args.remote, "--to", args.to])
    if rc != 0:
        print("取回失败 —— 车连不上？IP 会变，见 docs/实机接口对照.md §9", file=sys.stderr)
        return rc
    print("[2/2] 复盘报告")
    print()
    argv = [sys.executable, str(ROOT / "tools" / "log_report.py"), "--dir", args.to]
    if args.out:
        argv += ["--out", args.out]
    if args.strict:
        argv.append("--strict")
    return subprocess.call(argv, cwd=str(ROOT))


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="车端动作入口（与 run.py 分开：这个会动车）",
        epilog="提醒：move 默认不动车；真发车要加 --really-move，且必须有人在场。")
    sub = ap.add_subparsers(dest="cmd")

    def _host(p):
        p.add_argument("host", help="车端 IP（会变，见 docs/实机接口对照.md §9）")
        p.add_argument("--user", default="duzhong")

    p_r = sub.add_parser("restore", help="恢复官方栈（编译缺失的 dzactuator，只读源码）")
    _host(p_r)
    p_r.set_defaults(func=cmd_restore)

    p_d = sub.add_parser("deploy", help="上传我们的源码并编译（车重启后 /tmp 被清时先跑这个）")
    _host(p_d)
    p_d.add_argument("--ws", default="/tmp/ag_ws", help="车端工作空间（默认 /tmp/ag_ws）")
    p_d.set_defaults(func=cmd_deploy)

    p_m = sub.add_parser("move", help="一键起：底盘+定位+执行器+决策层（默认 dry_run，不动车）")
    _host(p_m)
    p_m.add_argument("--really-move", action="store_true",
                     help="★真发车★ 车会动，必须有人在场、遥控器在手")
    p_m.add_argument("--run-s", type=int, default=45, help="跑多久（秒）")
    p_m.add_argument("--use-air", action="store_true",
                     help="情报来自机载端（默认用 fake_intel 直连车端，供练车）")
    p_m.add_argument("--no-judgment", action="store_true", help="不启动裁判串口")
    p_m.add_argument("--with-vision", action="store_true",
                     help="连相机 + det_node(RKNN 检测) 一起起；瞄准/开火链仍不开")
    p_m.add_argument("--timeout", type=float, default=600.0)
    p_m.set_defaults(func=cmd_move)

    p_s = sub.add_parser("stop", help="紧急停止：停掉所有能动/能开火的节点")
    _host(p_s)
    p_s.set_defaults(func=cmd_stop)

    p_l = sub.add_parser("logs", help="取回车端日志并出复盘报告")
    _host(p_l)
    p_l.add_argument("--remote", default="/tmp/ag_logs")
    p_l.add_argument("--to", default="logs/car")
    p_l.add_argument("--out", default=None)
    p_l.add_argument("--strict", action="store_true")
    p_l.set_defaults(func=cmd_logs)

    return ap


def main(argv: Optional[list] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 2
    return int(args.func(args) or 0)


if __name__ == "__main__":
    raise SystemExit(main())
