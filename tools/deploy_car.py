# -*- coding: utf-8 -*-
r"""deploy_car.py —— 新版 dzbrain 的部署工作流（本地唯一真源 → 小车）。

★ 本工具把"铁律"编进代码：**默认只读**，任何会改动车上文件/进程的子命令都要求显式 `--write`。
   （历史教训：助手直接改车上文件，事后自己都不确定改了哪些、无法回滚。）

子命令（不加 --write = 只读，安全）：
    status                     车上节点/话题/门禁快照（只读）
    gate                       在车上跑 `python3 -m dzbrain.main --dry-run`（配置+路线校验，只读）
    pull <远程路径> [本地路径]  从车上拉文件（只读）
    diff <本地相对路径>         对比本地与车上同名文件的 md5（只读）
    plan                       打印"要推哪些文件、会在车上执行什么"（不执行）

子命令（需要 --write，显式授权才动车）：
    push <本地相对路径>...      上传到 ~/dzrobot/<同相对路径>
    build [包名...]             colcon build（默认 dzbrain + dzrknn_dataprocess）
    restart                     ~/dzrobot/start_all.sh restart
    backup                     把车上 config/ 与 start_all.sh 备份到 ~/dzrobot/_backups/<时间戳>/

凭据：走已配置好的 SSH 免密（ssh/scp 命令）。可用环境变量覆盖：
    ROBOT_SSH_HOST（默认 duzhong.local） ROBOT_SSH_USER（默认 duzhong） ROBOT_WS（默认 /home/duzhong/dzrobot）

用法示例：
    python tools\deploy_car.py plan
    python tools\deploy_car.py --write push config/robot.yaml config/map.yaml start_all.sh
    python tools\deploy_car.py --write push src/dzbrain
    python tools\deploy_car.py --write build
    python tools\deploy_car.py gate
    python tools\deploy_car.py --write restart
"""

from __future__ import annotations

import argparse
import hashlib
import os
import pathlib
import subprocess
import sys
import time

sys.stdout.reconfigure(errors="replace")  # Windows 控制台 GBK 兜底

WS = pathlib.Path(__file__).resolve().parents[1]
HOST = os.environ.get("ROBOT_SSH_HOST", "duzhong.local")
USER = os.environ.get("ROBOT_SSH_USER", "duzhong")
REMOTE_WS = os.environ.get("ROBOT_WS", "/home/duzhong/dzrobot")
TARGET = f"{USER}@{HOST}"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-o", "StrictHostKeyChecking=no"]

#: 默认要推的文件/目录（`plan` 打印这个清单）
DEFAULT_PAYLOAD = [
    "src/dzbrain",
    "config/robot.yaml",
    "config/map.yaml",
    "start_all.sh",
    "tools/config_selfcheck.py",
    "docs/云台补丁说明.md",
]


def run(cmd: list[str], *, timeout: float = 600.0) -> tuple[int, str]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                           errors="replace", timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, f"超时（>{timeout:.0f}s）: {' '.join(cmd[:4])}…"
    except FileNotFoundError as exc:
        return 127, f"命令不存在: {exc}"


def ssh(cmd: str, *, timeout: float = 600.0) -> tuple[int, str]:
    return run(["ssh", *SSH_OPTS, TARGET, cmd], timeout=timeout)


def scp_to(local: pathlib.Path, rel: str) -> tuple[int, str]:
    """上传到 ~/dzrobot/<rel>（远程目录先建好；用临时名再 mv，避免半截文件被跑起来）。"""
    remote_dir = f"{REMOTE_WS}/{str(pathlib.PurePosixPath(rel).parent)}"
    remote_tmp = f"{REMOTE_WS}/{rel}.part"
    remote_dst = f"{REMOTE_WS}/{rel}"
    rc, out = ssh(f"mkdir -p {remote_dir}")
    if rc != 0:
        return rc, out
    rc, out = run(["scp", *SSH_OPTS, str(local), f"{TARGET}:{remote_tmp}"], timeout=300)
    if rc != 0:
        return rc, out
    return ssh(f"mv -f {remote_tmp} {remote_dst} && ls -l {remote_dst}")


def md5_local(p: pathlib.Path) -> str:
    h = hashlib.md5()
    h.update(p.read_bytes())
    return h.hexdigest()


def iter_files(rel: str) -> list[tuple[pathlib.Path, str]]:
    """把相对路径展开成 (本地文件, 相对路径) 列表（目录递归，跳过 .git/__pycache__）。"""
    p = WS / rel
    if p.is_file():
        return [(p, str(pathlib.PurePosixPath(rel)))]
    if not p.is_dir():
        return []
    out: list[tuple[pathlib.Path, str]] = []
    for f in sorted(p.rglob("*")):
        if not f.is_file():
            continue
        parts = set(f.parts)
        if "__pycache__" in parts or ".git" in parts or f.name.endswith(".pyc"):
            continue
        out.append((f, f.relative_to(WS).as_posix()))
    return out


# --------------------------------------------------------------------------- #
#  只读子命令
# --------------------------------------------------------------------------- #
def cmd_status(_args) -> int:
    print(f"== 车上状态（只读） {TARGET}:{REMOTE_WS} ==")
    cmds = [
        ("主机/内核", "hostname; uptime | head -1; uname -r"),
        ("工作区", f"ls -d {REMOTE_WS} && du -sh {REMOTE_WS} 2>/dev/null | cut -f1"),
        ("dzbrain 是否已装", "python3 -c 'import dzbrain,sys; print(\"dzbrain\", dzbrain.__version__)' 2>&1 | tail -1"),
        ("配置指纹", f"md5sum {REMOTE_WS}/config/robot.yaml {REMOTE_WS}/config/map.yaml 2>/dev/null"),
        ("标定值", f"python3 -c \"import yaml;d=yaml.safe_load(open('{REMOTE_WS}/config/robot.yaml'))['/dzactuator']['ros__parameters'];print('ticksPerMeter',d['ticksPerMeter'],'ticksPer2PI',d['ticksPer2PI'])\" 2>&1 | tail -1"),
        ("设备节点", "ls -l /dev/dzactuator /dev/dzjudgment 2>&1 | head -2"),
        ("ROS 节点", "source /opt/ros/humble/setup.bash 2>/dev/null; timeout 8 ros2 node list 2>&1 | head -12"),
        ("关键话题", "source /opt/ros/humble/setup.bash 2>/dev/null; timeout 8 ros2 topic list 2>&1 | grep -E 'pursuitAngle|aim/|dron|referee|odom/localization|shoot_count' | head -12"),
    ]
    for label, c in cmds:
        rc, out = ssh(c, timeout=60)
        print(f"\n-- {label} --")
        print(out.strip() or f"(空，rc={rc})")
    return 0


def cmd_gate(args) -> int:
    print("== 车上启动门禁（配置+路线校验，只读）==")
    rc, out = ssh(f"cd {REMOTE_WS} && PYTHONIOENCODING=utf-8 timeout 60 python3 -m dzbrain.main "
                  f"--config {REMOTE_WS}/config/robot.yaml --map {REMOTE_WS}/config/map.yaml --dry-run",
                  timeout=90)
    print(out.strip())
    print(f"\n门禁退出码 = {rc}  →  {'✅ 通过' if rc == 0 else '❌ 未通过（车端禁止起栈）'}")
    return rc


def cmd_diff(args) -> int:
    bad = 0
    for rel in args.paths:
        files = iter_files(rel)
        if not files:
            print(f"?? 本地不存在: {rel}")
            bad += 1
            continue
        for local, r in files:
            _, out = ssh(f"md5sum {REMOTE_WS}/{r} 2>/dev/null | cut -d' ' -f1")
            remote = out.strip().splitlines()[0] if out.strip() else ""
            same = (remote == md5_local(local)) if remote else False
            flag = "SAME" if same else ("缺失" if not remote else "DIFF")
            if not same:
                bad += 1
            print(f"{flag:4} {r}")
    print(f"\n不一致/缺失 {bad} 个文件" + ("（需 push）" if bad else "（无需 push）"))
    return 0


def cmd_pull(args) -> int:
    remote, local = args.remote, args.local or pathlib.Path(args.remote).name
    rc, out = run(["scp", *SSH_OPTS, f"{TARGET}:{remote}", str(local)], timeout=300)
    print(out.strip() or f"已拉到 {local}")
    return rc


def cmd_plan(_args) -> int:
    print("== 部署计划（不执行任何写操作）==")
    print(f"目标: {TARGET}:{REMOTE_WS}\n")
    print("① 会推送的文件：")
    total = 0
    for rel in DEFAULT_PAYLOAD:
        files = iter_files(rel)
        total += len(files)
        print(f"   {rel}  ({len(files)} 个文件)")
    print(f"   合计 {total} 个文件（scp 到 ~/dzrobot/<同路径>，先传 .part 再 mv）")
    print("\n② 车上要执行的命令（需 --write）：")
    print("   cd ~/dzrobot && colcon build --packages-select dzbrain dzrknn_dataprocess")
    print("   source install/setup.bash")
    print("   python3 -m dzbrain.main --config config/robot.yaml --map config/map.yaml --dry-run")
    print("   ./start_all.sh restart        # step7 默认走 dzbrain；LEGACY_PATH=1 回退旧实现")
    print("\n③ 回退：LEGACY_PATH=1 ./start_all.sh restart / SKIP_PRECHECK=1 / git checkout")
    print("\n④ 注意：云台补丁默认关闭（use_external_fire_gate=false），"
          "开火行为与旧版一致；整定步骤见 docs/云台补丁说明.md")
    return 0


# --------------------------------------------------------------------------- #
#  写入子命令（需 --write）
# --------------------------------------------------------------------------- #
def cmd_push(args) -> int:
    rels = args.paths or DEFAULT_PAYLOAD
    ok = fail = 0
    for rel in rels:
        files = iter_files(rel)
        if not files:
            print(f"!! 本地不存在: {rel}")
            fail += 1
            continue
        for local, r in files:
            rc, out = scp_to(local, r)
            if rc == 0:
                ok += 1
                print(f"   ✓ {r}")
            else:
                fail += 1
                print(f"   ✗ {r}  rc={rc}  {out.strip()[:200]}")
    print(f"\n推送完成：成功 {ok} / 失败 {fail}")
    return 1 if fail else 0


def cmd_backup(_args) -> int:
    ts = time.strftime("%Y%m%d_%H%M%S")
    dst = f"{REMOTE_WS}/_backups/deploy_{ts}"
    rc, out = ssh(f"mkdir -p {dst} && cp -a {REMOTE_WS}/config {dst}/ 2>/dev/null; "
                  f"cp -a {REMOTE_WS}/start_all.sh {dst}/ 2>/dev/null; ls -la {dst}")
    print(out.strip() or f"已备份到 {dst}")
    return rc


def cmd_build(args) -> int:
    pkgs = args.packages or ["dzbrain", "dzrknn_dataprocess"]
    sel = "--packages-select " + " ".join(pkgs)
    rc, out = ssh(f"cd {REMOTE_WS} && source /opt/ros/humble/setup.bash && colcon build {sel} 2>&1 | tail -25",
                  timeout=900)
    print(out.strip())
    print(f"\ncolcon 退出码 = {rc}  →  {'✅ 编译通过' if rc == 0 else '❌ 编译失败（把上面输出贴回来）'}")
    return rc


def cmd_restart(_args) -> int:
    rc, out = ssh(f"cd {REMOTE_WS} && ./start_all.sh restart 2>&1 | tail -30", timeout=300)
    print(out.strip())
    return rc


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="deploy_car", description="dzbrain 部署工作流（默认只读）")
    ap.add_argument("--write", action="store_true",
                    help="显式授权执行会改动车上文件/进程的子命令（铁律：默认只读）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status", help="车上状态快照（只读）")
    sub.add_parser("gate", help="车上启动门禁（只读）")
    sub.add_parser("plan", help="打印部署计划（不执行）")
    p_diff = sub.add_parser("diff", help="本地 vs 车上 md5（只读）")
    p_diff.add_argument("paths", nargs="*", default=DEFAULT_PAYLOAD)
    p_pull = sub.add_parser("pull", help="从车上拉文件（只读）")
    p_pull.add_argument("remote")
    p_pull.add_argument("local", nargs="?")

    p_push = sub.add_parser("push", help="上传文件（需 --write）")
    p_push.add_argument("paths", nargs="*")
    sub.add_parser("build", help="colcon build（需 --write）").add_argument("packages", nargs="*")
    sub.add_parser("restart", help="start_all.sh restart（需 --write）")
    sub.add_parser("backup", help="备份车上 config/start_all.sh（需 --write）")

    args = ap.parse_args(argv)
    read_only = {"status", "gate", "plan", "diff", "pull"}
    if args.cmd not in read_only and not args.write:
        print(f"❌ `{args.cmd}` 会改动车上文件/进程 —— 必须显式加 --write（铁律：默认只读）")
        print("   例：python tools\\deploy_car.py --write push config/robot.yaml")
        return 3

    return {
        "status": cmd_status, "gate": cmd_gate, "plan": cmd_plan, "diff": cmd_diff,
        "pull": cmd_pull, "push": cmd_push, "build": cmd_build, "restart": cmd_restart,
        "backup": cmd_backup,
    }[args.cmd](args)


if __name__ == "__main__":
    raise SystemExit(main())
