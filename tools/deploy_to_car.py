# -*- coding: utf-8 -*-
r"""上车部署工具（本地 → 小车，走 SSH/SCP）。

流程（对应"本地唯一真源 → 车端替换 → 编译验证"铁律）：
    probe    解析车端当前 IP 与关键状态（mDNS 会抖，所以先解析出 IP 再用 IP 干活）
    backup   在车上把将被覆盖的文件打包到 ~/dzrobot/_backups/<时间戳>/
    push     把本地文件/目录推到车上（默认含 dzbrain 包、云台补丁源码、config、tools）
    build    车上 colcon build（默认 dzbrain + dzrknn_dataprocess）
    verify   车上验证：启动门禁 dry-run + bench 冒烟（FakeBridge，不动车）+ 包/参数存在性
    all      backup → push → build → verify

安全约定：
- **绝不在验证阶段启动 ROS 栈**（起栈会让车按路线行动；`auto_start` 也必须保持 false）。
- push 前自动 backup；backup 目录名带时间戳，便于回滚。
- config 推送时会把 `auto_start` 强制为 false（车端当前也是 false），并在报告里注明这一处差异。

用法：
    python tools\deploy_to_car.py probe
    python tools\deploy_to_car.py all
    python tools\deploy_to_car.py all --dry-run          # 只打印将执行的命令
"""

from __future__ import annotations

import argparse
import pathlib
import subprocess
import sys
import time

WS = pathlib.Path(__file__).resolve().parents[1]
CAR_HOST = "duzhong.local"
CAR_USER = "duzhong"
CAR_WS = "/home/duzhong/dzrobot"
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=10"]

#: 推送清单：本地相对路径 → 车端绝对路径（目录用 -r）
PUSH_FILES: list[tuple[str, str]] = [
    ("src/dzbrain", f"{CAR_WS}/src/dzbrain"),                                    # 新决策层（整包）
    ("src/dzrknn_dataprocess/include/dzrknn_dataprocess.h",                       # 云台补丁
     f"{CAR_WS}/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h"),
    ("src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp",
     f"{CAR_WS}/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp"),
    ("config/robot.yaml", f"{CAR_WS}/config/robot.yaml"),                        # 统一配置（含新键）
    ("config/map.yaml", f"{CAR_WS}/config/map.yaml"),
    ("tools/config_selfcheck.py", f"{CAR_WS}/tools/config_selfcheck.py"),         # 车上自检
    ("tools/dzbrain.py", f"{CAR_WS}/tools/dzbrain.py"),
]

#: backup 时打包的车端路径
BACKUP_PATHS = [
    f"{CAR_WS}/config/robot.yaml",
    f"{CAR_WS}/config/map.yaml",
    f"{CAR_WS}/start_all.sh",
    f"{CAR_WS}/src/dzbrain",
    f"{CAR_WS}/src/dzrknn_dataprocess",
]

#: ★ 备份必须放在**工作区之外**：放进 ~/dzrobot 里会被 colcon 当成第二个包目录，
#:   直接报 "Duplicate package names not supported" 把整次构建搞失败（实测踩过）。
BACKUP_ROOT = "/home/duzhong/_backups"

BUILD_PACKAGES = ["dzbrain", "dzrknn_dataprocess"]


def log(msg: str) -> None:
    print(msg, flush=True)


def sh(argv: list[str], *, dry: bool = False, input_text: str | None = None) -> int:
    if dry:
        log("  [dry-run] " + " ".join(argv) + (f"   <<< {len(input_text)} 字节 stdin" if input_text else ""))
        return 0
    proc = subprocess.run(argv, input=input_text, text=True, capture_output=True, encoding="utf-8",
                          errors="replace")
    if proc.stdout.strip():
        log(proc.stdout.rstrip())
    if proc.returncode != 0 and proc.stderr.strip():
        log("  stderr: " + proc.stderr.strip()[-1500:])
    return proc.returncode


def resolve_ip(dry: bool = False) -> str:
    """解析车端 IP（mDNS 会抖：最多试 5 次；成功过就用 IP 干活，不再依赖 .local）。"""
    for attempt in range(1, 6):
        argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{CAR_HOST}", "hostname -I"]
        if dry:
            log(f"  [dry-run] {' '.join(argv)}")
            return "0.0.0.0"
        proc = subprocess.run(argv, text=True, capture_output=True, encoding="utf-8", errors="replace")
        if proc.returncode == 0:
            for token in proc.stdout.split():
                if token.count(".") == 3 and token[0].isdigit():
                    log(f"  车端 IP = {token}（第 {attempt} 次解析成功）")
                    return token
        time.sleep(1.0)
    raise SystemExit("❌ 无法解析车端 IP（mDNS 抖动或车不在线）：先确认小车开机并连上同一网络")


def ssh_run(ip: str, cmd: str, *, dry: bool = False, label: str = "") -> int:
    log(f"  $ ssh {ip}: {label or cmd[:90]}")
    argv = ["ssh", *SSH_OPTS, f"{CAR_USER}@{ip}", cmd]
    return sh(argv, dry=dry)


def scp_push(ip: str, local: str, remote: str, *, dry: bool = False) -> int:
    log(f"  $ scp {local} -> {remote}")
    argv = ["scp", "-r", *SSH_OPTS, str(WS / local), f"{CAR_USER}@{ip}:{remote}"]
    return sh(argv, dry=dry)


# --------------------------------------------------------------------------- #
def cmd_probe(ip: str | None, args) -> None:
    ip = ip or resolve_ip(args.dry_run)
    ssh_run(ip, "hostname; date; uptime | head -1", dry=args.dry_run, label="基本信息")
    ssh_run(ip, f"ls {CAR_WS}; echo '--- config ---'; ls -l {CAR_WS}/config; "
                f"echo '--- 是否已有 dzbrain ---'; ls -d {CAR_WS}/src/dzbrain 2>/dev/null || echo '(还没有)'",
            dry=args.dry_run, label="工作区/配置/是否已有 dzbrain")
    ssh_run(ip, "source /opt/ros/humble/setup.bash 2>/dev/null; colcon --version 2>/dev/null | head -1; python3 -V",
            dry=args.dry_run, label="工具链")
    ssh_run(ip, "df -h ~ | tail -1", dry=args.dry_run, label="磁盘")


def cmd_backup(ip: str, args) -> None:
    ts = time.strftime("%Y%m%d_%H%M%S")
    dst = f"{BACKUP_ROOT}/dzbrain_deploy_{ts}"
    files = " ".join(p for p in BACKUP_PATHS)
    ssh_run(ip, f"mkdir -p {dst} && for f in {files}; do [ -e \"$f\" ] && cp -r \"$f\" {dst}/ ; done; "
                f"ls -la {dst} | head -12; echo BACKUP_DIR={dst}",
            dry=args.dry_run, label=f"备份到 {dst}（★工作区之外，避免 colcon 重名）")
    # ★ 清理历史遗留：工作区内如果已经有备份目录（含包目录），colcon 会报重名 —— 给它加 COLCON_IGNORE
    ssh_run(ip, f"for d in {CAR_WS}/_backups {CAR_WS}/backup; do "
                f"[ -d \"$d\" ] && touch \"$d/COLCON_IGNORE\" && echo \"已加 COLCON_IGNORE: $d\"; done; true",
            dry=args.dry_run, label="给工作区内历史备份目录加 COLCON_IGNORE")


def cmd_push(ip: str, args) -> None:
    for local, remote in PUSH_FILES:
        if not (WS / local).exists():
            raise SystemExit(f"❌ 本地文件不存在：{local}")
        scp_push(ip, local, remote, dry=args.dry_run)
    # ★ 推送后把车端 auto_start 强制为 false（验证阶段绝不允许自动发车）
    ssh_run(ip, f"cd {CAR_WS} && python3 - <<'PY'\n"
                "import re, pathlib\n"
                "p = pathlib.Path('config/robot.yaml')\n"
                "s = p.read_text(encoding='utf-8')\n"
                "s2 = re.sub(r'(auto_start:)\\s*true', r'\\1 false', s)\n"
                "s2 = re.sub(r'(auto_start:)\\s*true', r'\\1 false', s2)\n"
                "if s2 != s:\n"
                "    p.write_text(s2, encoding='utf-8')\n"
                "    print('★ auto_start 已强制为 false（验证阶段不发车）')\n"
                "else:\n"
                "    print('auto_start 已是 false')\n"
                "PY", dry=args.dry_run, label="强制 auto_start=false")


def cmd_build(ip: str, args) -> int:
    pkgs = " ".join(args.packages or BUILD_PACKAGES)
    cmd = (f"cd {CAR_WS} && source /opt/ros/humble/setup.bash && "
           f"source ./dzros.env 2>/dev/null; colcon build --packages-select {pkgs} "
           f"--cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -30")
    rc = ssh_run(ip, cmd, dry=args.dry_run, label=f"colcon build {pkgs}")
    if rc != 0 and not args.dry_run:
        log("❌ 编译失败 —— 把上面的输出贴回来，我按行修")
    return rc


def cmd_verify(ip: str, args) -> int:
    rc = 0
    # ① 启动门禁（配置 + 路线校验），不需要 ROS
    rc |= ssh_run(ip, f"cd {CAR_WS} && python3 tools/config_selfcheck.py --quiet 2>&1 | tail -8",
                  dry=args.dry_run, label="车上配置+路线门禁")
    # ② bench 冒烟：真实任务线程 + FakeBridge（不动车、不起 ROS 栈）
    rc |= ssh_run(ip, f"cd {CAR_WS} && python3 tools/dzbrain.py --bench --seconds 4 2>&1 | tail -14",
                  dry=args.dry_run, label="bench 冒烟（四线程，不动车）")
    # ③ 包与入口存在性
    rc |= ssh_run(ip, f"cd {CAR_WS} && source /opt/ros/humble/setup.bash && source install/setup.bash && "
                      f"ros2 pkg executables dzbrain; "
                      f"ros2 pkg prefix dzbrain",
                  dry=args.dry_run, label="dzbrain 包/入口")
    # ④ 云台补丁是否真的编进二进制（防"改了源码没编"）
    rc |= ssh_run(ip, f"grep -c 'use_external_fire_gate' {CAR_WS}/install/dzrknn_dataprocess/lib/"
                      f"dzrknn_dataprocess/dzactuator_node 2>/dev/null || "
                      f"find {CAR_WS}/install/dzrknn_dataprocess -type f -perm -u+x | head -3",
                  dry=args.dry_run, label="云台补丁符号（证明补丁已编入）")
    return rc


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="本地 → 小车 部署工具（SSH/SCP）")
    ap.add_argument("action", choices=["probe", "backup", "push", "build", "verify", "all"])
    ap.add_argument("--ip", default=None, help="直接指定车端 IP（跳过 mDNS 解析）")
    ap.add_argument("--packages", nargs="*", default=None, help="colcon build 的包列表")
    ap.add_argument("--dry-run", action="store_true", help="只打印将执行的命令")
    args = ap.parse_args(argv)

    log("=" * 72)
    log(f"dzrobot 部署工具 · action={args.action} · dry_run={args.dry_run}")
    log("=" * 72)

    ip = args.ip or resolve_ip(args.dry_run)

    if args.action == "probe":
        cmd_probe(ip, args)
        return 0
    if args.action == "backup":
        cmd_backup(ip, args)
        return 0
    if args.action == "push":
        cmd_push(ip, args)
        return 0
    if args.action == "build":
        return cmd_build(ip, args)
    if args.action == "verify":
        return cmd_verify(ip, args)
    # all
    cmd_backup(ip, args)
    cmd_push(ip, args)
    rc = cmd_build(ip, args)
    if rc != 0:
        log("⛔ 编译失败，跳过验证（先修编译）")
        return rc
    return cmd_verify(ip, args)


if __name__ == "__main__":
    sys.exit(main())
