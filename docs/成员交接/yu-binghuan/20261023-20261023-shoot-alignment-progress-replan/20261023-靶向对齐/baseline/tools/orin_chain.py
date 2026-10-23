"""Orin 一键链：部署「除飞控以外的全部无人机侧代码」→ 编译验证 → 全栈拉起 → 体检 → 冒烟。

为什么需要它
------------
仓库里已有 orin_deploy / orin_build_verify / orin_convert_engines / orin_bringup /
preflight / check_stack_health / patch_deploy 等成熟脚本，但**没有任何单一入口**把它们
串起来 —— 部署一次要人工按序敲 4 步，且有三处口径缺口会静默踩坑：

  1) `onboard/dzpx4_patch/`（定位韧性补丁 v2 + localization_watchdog）**不在**
     `orin_deploy.py` 的推送范围（它只推 onboard/dzuav + 模型 + tools/orin_* ），
     必须另跑 `patch_deploy.py`；
  2) `tools/orin_run_dzuav.sh` 默认 `DZUAV_DIR=$HOME/ag_test/dzuav`，而 orin_deploy 把
     二进制落在 `~/dzuav_deploy/air_ground/onboard/dzuav/build/dzuav`，且
     `orin_bringup.sh` 不导出 `DZUAV_DIR` → **dzuav 窗口默认起不来**（本链统一导出修掉）；
  3) 原流程里没有健康检查与冒烟（`check_stack_health.py` / `smoke_test_chain.py`）。

本链**只做编排、不重写任何既有工具**；密码只取一次并导出给子进程（不落盘）。

用法（仓库根执行；密码走 AG_ORIN_PASS 环境变量或交互输入）
----------------------------------------------------------
    set AG_ORIN_PASS=******
    无人机一键部署.bat                        # 或：python tools/orin_chain.py
    python tools/orin_chain.py --dry-run      # 只打印将执行的命令，不连硬件
    python tools/orin_chain.py --from 4       # 从第 4 段续跑（前面已跑过）
    python tools/orin_chain.py --skip-patch --skip-engines
    python tools/orin_chain.py --engines-only # 只做引擎转换（换机/刷机后）

段序（fail-fast，每段独立横幅）
------------------------------
    0 链路探活 → 1 preflight --orin --match → 2 orin_deploy --verify →
    3 patch_deploy → 4 机上引擎转换 → 5 机上编译验证 → 6 bringup down/up →
    7 check_stack_health --kill-dup → 8 smoke_test_chain

回滚
----
不做全局回滚。定位补丁回滚 = `tools/patch_deploy.py --host <IP> --rollback`；
代码回滚 = 重新 `orin_deploy` 上一已知好版本。本链只在第 3 段保留该提示。

范围与硬约束
------------
* 不部署 `reference/onboard/dzuav`（7 类归档版）—— 项目硬约束：13 类为 canonical；
* 不碰飞控固件（官方 dzpx4 的 px4_control / px4_path）。

依赖: pip install paramiko（复用 tools/remote_run.py 的连接/重试/密码守卫）
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import subprocess
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent
sys.path.insert(0, str(TOOLS))

try:
    import remote_run  # noqa: E402 —— 复用连接重试 / read_password 密码守卫
except ImportError:  # pragma: no cover
    print("缺少 paramiko，请先执行: python -m pip install paramiko", file=sys.stderr)
    raise SystemExit(2)

REMOTE_ROOT = "~/dzuav_deploy/air_ground"
# gap 2 修复点：让 orin_run_dzuav.sh 指向真正部署二进制的位置（默认会找 ~/ag_test/dzuav）
DZUAV_DIR_REMOTE = "~/dzuav_deploy/air_ground/onboard/dzuav"

STAGES = (
    (0, "链路探活（SSH + 进程/会话现状）"),
    (1, "机载预检 preflight --orin --match"),
    (2, "部署 代码+模型+工具 orin_deploy --verify"),
    (3, "部署 定位韧性补丁 patch_deploy"),
    (4, "机上 引擎转换 orin_convert_engines.sh"),
    (5, "机上 编译验证 orin_build_verify.sh"),
    (6, "全栈拉起 orin_bringup.sh down/up"),
    (7, "栈体检 check_stack_health --kill-dup"),
    (8, "车端链路冒烟 smoke_test_chain"),
)


class Tee:
    """把输出同时打到控制台与日志文件（fail 现场可回看）。"""

    def __init__(self, log_path: Path):
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(log_path, "w", encoding="utf-8", newline="\n")

    def __call__(self, msg: str = "") -> None:
        print(msg)
        self.f.write(msg + "\n")
        self.f.flush()

    def close(self) -> None:
        self.f.close()


def banner(say, idx: int, name: str) -> None:
    say("\n" + "=" * 70)
    say(f"[段 {idx}/8] {name}")
    say("=" * 70)


def pc(say, argv: list[str], env: dict) -> int:
    """本机执行既有脚本（继承 AG_ORIN_PASS，避免密码进命令行）。"""
    say("  $ " + " ".join(argv))
    p = subprocess.run(argv, cwd=str(REPO), env=env)
    say(f"  -> 退出码 {p.returncode}")
    return p.returncode


def remote(say, host: str, user: str, pw: str, cmd: str, timeout: float, port: int) -> int:
    """机上执行（每次独立连接，复用 remote_run 的重试约定）。"""
    say("  orin$ " + cmd)
    try:
        code, out, err = remote_run.run(host, user, pw, cmd, timeout, port)
    except Exception as e:  # noqa: BLE001 —— 网络层异常一律原样报出
        say(f"  -> 连接/执行失败: {type(e).__name__}: {e}")
        return 1
    for line in (out or "").rstrip().splitlines():
        say("    " + line)
    if err and err.strip():
        for line in err.rstrip().splitlines():
            say("    [stderr] " + line)
    say(f"  -> 退出码 {code}")
    return code


# ---------------------------------------------------------------- 各段

def stage0_link(say, a, pw) -> int:
    """链路探活：先看清机上现状（进程/会话），避免盲部署造成双实例。"""
    probe = (
        "echo HOST=$(hostname); uname -m; "
        "echo '--- 关键进程 (期望各 1) ---'; "
        "for p in livox fastlio px4_odom_convert MicroXRCEAgent dzuav; do "
        "  n=$(pgrep -xc \"$p\" 2>/dev/null || echo 0); echo \"$p=$n\"; done; "
        "echo '--- tmux ---'; tmux ls 2>/dev/null || echo 'no tmux session'; "
        "echo '--- 既有部署 ---'; ls -d ~/dzuav_deploy/air_ground 2>/dev/null || echo none; "
        "ls ~/dzuav/engines/*.engine 2>/dev/null | wc -l"
    )
    return remote(say, a.host, a.user, pw, "bash -lc " + _q(probe), a.timeout, a.port)


def stage1_preflight(say, a, env) -> int:
    return pc(say, [sys.executable, str(TOOLS / "preflight.py"),
                    "--orin", "--match",
                    "--orin-host", a.host, "--user", a.user], env)


def stage2_deploy(say, a, env) -> int:
    return pc(say, [sys.executable, str(TOOLS / "orin_deploy.py"),
                    "--host", a.host, "--user", a.user, "--port", str(a.port),
                    "--verify", "--timeout", str(a.deploy_timeout)], env)


def stage3_patch(say, a, env) -> int:
    say("  （定位韧性补丁：px4_odom_convert v2 + localization_watchdog）")
    say("  回滚提示：tools/patch_deploy.py --host <IP> --user <用户> --rollback")
    return pc(say, [sys.executable, str(TOOLS / "patch_deploy.py"),
                    "--host", a.host, "--user", a.user, "--port", str(a.port)], env)


def stage4_engines(say, a, pw) -> int:
    cmd = (f"bash -lc 'cd ~/dzuav_deploy/air_ground && "
           f"tools/orin_convert_engines.sh -o ~/dzuav/engines'")
    return remote(say, a.host, a.user, pw, cmd, a.build_timeout, a.port)


def stage5_build(say, a, pw) -> int:
    cmd = "bash -lc 'cd ~/dzuav_deploy/air_ground && tools/orin_build_verify.sh'"
    return remote(say, a.host, a.user, pw, cmd, a.build_timeout, a.port)


def stage6_bringup(say, a, pw) -> int:
    teleop = "" if a.teleop else " --no-teleop"
    cmd = (
        "bash -lc '" + f"export DZUAV_DIR={DZUAV_DIR_REMOTE}; "
        "cd ~/dzuav_deploy/air_ground && "
        "tools/orin_bringup.sh down; "
        f"tools/orin_bringup.sh up{teleop}" + "'"
    )
    return remote(say, a.host, a.user, pw, cmd, a.build_timeout, a.port)


def stage7_health(say, a, env) -> int:
    return pc(say, [sys.executable, str(TOOLS / "check_stack_health.py"),
                    "--host", a.host, "--user", a.user, "--port", str(a.port),
                    "--kill-dup"], env)


def stage8_smoke(say, a, env) -> int:
    return pc(say, [sys.executable, str(TOOLS / "smoke_test_chain.py")], env)


def _q(s: str) -> str:
    import shlex
    return shlex.quote(s)


# ---------------------------------------------------------------- main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="orin_chain.py", description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.55.1", help="Orin IP（Type-C NCM 默认 192.168.55.1）")
    ap.add_argument("--user", default=os.environ.get("AG_ORIN_USER") or "duzhong")
    ap.add_argument("--password", default=None, help="密码；推荐 AG_ORIN_PASS 环境变量")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--from", dest="start", type=int, default=0, help="从第 N 段开始（0..8）")
    ap.add_argument("--skip-patch", action="store_true", help="跳过第 3 段（定位韧性补丁）")
    ap.add_argument("--skip-engines", action="store_true", help="跳过第 4 段（引擎转换）")
    ap.add_argument("--engines-only", action="store_true", help="只跑第 0/4 段（换机/刷机后）")
    ap.add_argument("--teleop", action="store_true", help="bringup 带键盘遥控窗口（默认不带）")
    ap.add_argument("--deploy-timeout", type=float, default=600.0, help="部署段超时（模型 43MB）")
    ap.add_argument("--build-timeout", type=float, default=1800.0, help="机上编译/验证段超时")
    ap.add_argument("--timeout", type=float, default=120.0, help="探活段超时")
    ap.add_argument("--dry-run", action="store_true", help="只打印将执行的命令，不连硬件")
    ap.add_argument("--log-dir", default=str(REPO / "logs" / "orin_chain"))
    args = ap.parse_args(argv)

    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    say = Tee(Path(args.log_dir) / f"{ts}.log")

    say("=" * 70)
    say("无人机一键链（除飞控外全量部署 + 拉起 + 体检）")
    say(f"  Orin {args.host}（用户 {args.user}）   仓库 {REPO}")
    say(f"  起始段 {args.start}   跳过补丁={args.skip_patch} 跳过引擎={args.skip_engines} "
        f"仅引擎={args.engines_only}")
    say(f"  日志 {args.log_dir}/{ts}.log")
    say("=" * 70)

    if not (0 <= args.start <= 8):
        say("--from 取值须在 0..8")
        return 2

    if args.dry_run:
        say("\n[dry-run] 将执行的段（不连硬件）：")
        for idx, name in STAGES:
            say(f"  {idx}. {name}")
        say("\n[dry-run] 第 1/2/3/7/8 段命令：")
        say("  python tools/preflight.py --orin --match "
            f"--orin-host {args.host} --user {args.user}")
        say("  python tools/orin_deploy.py "
            f"--host {args.host} --user {args.user} --port {args.port} --verify")
        say("  python tools/patch_deploy.py "
            f"--host {args.host} --user {args.user} --port {args.port}")
        say("  python tools/check_stack_health.py "
            f"--host {args.host} --user {args.user} --port {args.port} --kill-dup")
        say("  python tools/smoke_test_chain.py")
        say("\n[dry-run] 第 4/5/6 段（机上 bash -lc）：")
        say("  cd ~/dzuav_deploy/air_ground && tools/orin_convert_engines.sh -o ~/dzuav/engines")
        say("  cd ~/dzuav_deploy/air_ground && tools/orin_build_verify.sh")
        say(f"  export DZUAV_DIR={DZUAV_DIR_REMOTE}; cd ~/dzuav_deploy/air_ground && "
            "tools/orin_bringup.sh down; tools/orin_bringup.sh up --no-teleop")
        say.close()
        return 0

    # 密码只取一次：--password > AG_ORIN_PASS > 交互提问（与既有工具同一套守卫）
    ns = argparse.Namespace(password=args.password, user=args.user, host=args.host)
    pw = remote_run.read_password(ns, env_name="AG_ORIN_PASS")

    env = dict(os.environ)
    env["AG_ORIN_PASS"] = pw          # 子进程复用；密码不进命令行、不落盘
    env["AG_ORIN_USER"] = args.user

    todo = [idx for idx, _ in STAGES if idx >= args.start]
    if args.engines_only:
        todo = [0, 4]
    else:
        if args.skip_patch and 3 in todo:
            todo.remove(3)
        if args.skip_engines and 4 in todo:
            todo.remove(4)

    for idx, name in STAGES:
        if idx not in todo:
            continue
        banner(say, idx, name)
        if idx == 0:
            rc = stage0_link(say, args, pw)
        elif idx == 1:
            rc = stage1_preflight(say, args, env)
        elif idx == 2:
            rc = stage2_deploy(say, args, env)
        elif idx == 3:
            rc = stage3_patch(say, args, env)
        elif idx == 4:
            rc = stage4_engines(say, args, pw)
        elif idx == 5:
            rc = stage5_build(say, args, pw)
        elif idx == 6:
            rc = stage6_bringup(say, args, pw)
        elif idx == 7:
            rc = stage7_health(say, args, env)
        else:
            rc = stage8_smoke(say, args, env)

        if rc != 0:
            say(f"\n[停止] 第 {idx} 段失败（退出码 {rc}）—— fail-fast，后续段未执行。")
            say("       修复后可用 --from "
                f"{idx} 从这里续跑。")
            say.close()
            return 1

    say("\n" + "=" * 70)
    say("[完成] 全链通过：代码/模型/工具已部署，机上已编译验证并拉起，体检与冒烟均过。")
    say("       机上观察：tmux attach -t agup")
    say("=" * 70)
    say.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
