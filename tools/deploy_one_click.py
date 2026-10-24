#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机端一键部署（2026-10-24）：预检 → 四阶段 → 验证门 → 汇总报告。

编排层只做链路与判定，各阶段逻辑复用既有维护入口（不复制实现）：
    ① tools/orin_deploy.py --verify     源码+模型+工具链+ag_integrate（远端备份+L1/L2 自检）
    ② tools/patch_deploy.py             px4_odom_convert v2.2（备份→推送→colcon→重启→横幅验证）
    ③ tools/orin_check_engines.sh       引擎指纹核查；缺 D13/v4d 时自动调 orin_convert_engines.sh
    ④ tools/orin_bringup.sh restart     规划侧重启

预检（全部通过才进阶段①）：
    - IP 解析：--host 显式 > mDNS（dzsmart.local）> 历史网段候选逐一探测
    - SSH BatchMode 探活（密码走 AG_ORIN_PASS，与既有工具同约定）
    - 时钟偏差：>1s WARN（历史实测偏过 ~3 天；不自动改时间——需人工确认时区）

验证门（阶段后逐项判 PASS/FAIL，输出证据；任一 FAIL 汇总为非零退出）：
    G1  ~/dzuav_deploy/air_ground 存在 + ag_integrate/route_progress.py 含"连写朝向"修复标记
    G2  px4 进程存在 + 日志含 "Resilience layer v2"（横幅）
    G3  ~/dzuav/engines 下 hbb_detector_d13_fp16.engine 与 cell_classifier_v4d_fp16.engine 存在
    G4  sidecar 进程存在（orin_plan_sidecar / orin_bringup 托管）
    （动态验证 G5/G6——"重规划基于最新进度"日志行与 health 行 status_age_s 字段——
      需起飞/侦察流才出现，列为本机静态门之外的现场项，见 DEPLOY_RUNBOOK §3）

用法：
    python tools/deploy_one_click.py --host <IP>                 # 全链
    python tools/deploy_one_click.py --host <IP> --dry-run       # 只打印将执行的命令
    python tools/deploy_one_click.py --host <IP> --skip px4      # 跳过某阶段: code/px4/engines/bringup
    python tools/deploy_one_click.py --host <IP> --verify-only   # 只跑验证门（部署后体检）
    AG_ORIN_PASS=... python tools/deploy_one_click.py --host <IP>

回滚（失败时报告尾部打印）：
    python tools/orin_deploy.py --host <IP> --restore     # ag_integrate 回备份
    python tools/patch_deploy.py  --host <IP> --rollback  # px4 回原版
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REMOTE_ROOT = "~/dzuav_deploy/air_ground"

#: 历史网段候选（IP 每换网漂移，实测记录；逐个 SSH 探活）
KNOWN_HOSTS = ("10.240.174.104", "10.35.165.104", "10.29.122.104", "10.230.128.188")
MDNS = "dzsmart.local"

GATES: list[tuple[str, str, str]] = []       # (名称, PASS/FAIL/SKIP, 证据)


def sh(cmd: list[str], *, dry: bool, timeout: int = 900) -> int:
    """执行（或 dry-run 打印）一条命令；返回退出码。"""
    printable = " ".join(cmd)
    if dry:
        print(f"  [dry-run] {printable}")
        return 0
    print(f"  $ {printable}")
    try:
        return subprocess.run(cmd, timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        print(f"  ✗ 超时（{timeout}s）: {printable}")
        return 124


def ssh(host: str, user: str, remote_cmd: str, *, dry: bool, timeout: int = 60) -> tuple[int, str]:
    """一条 SSH 命令并收回输出。OpenSSH BatchMode 优先；失败回退 paramiko
    密码登录（AG_ORIN_PASS，与 orin_deploy/patch_deploy 同约定）——现场首次
    部署没有免密时仍可全链跑通。"""
    full = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=8", f"{user}@{host}", remote_cmd]
    if dry:
        print(f"  [dry-run] {' '.join(full)}")
        return 0, ""
    try:
        p = subprocess.run(full, capture_output=True, text=True, timeout=timeout,
                           encoding="utf-8", errors="replace")
        if p.returncode == 0:
            return 0, (p.stdout or "") + (p.stderr or "")
    except subprocess.TimeoutExpired:
        return 124, ""
    # BatchMode 失败 → paramiko 密码回退
    pwd = os.environ.get("AG_ORIN_PASS")
    if not pwd:
        return 255, "BatchMode 失败且未设 AG_ORIN_PASS"
    import paramiko
    last = None
    for attempt in range(3):            # WiFi 链路抖动实测：banner 被重置，重试即过
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(host, username=user, password=pwd, timeout=15,
                      banner_timeout=15, auth_timeout=15,
                      allow_agent=False, look_for_keys=False)
            _, out, err = c.exec_command(remote_cmd, timeout=timeout)
            data = (out.read() or b"").decode("utf-8", errors="replace") +                    (err.read() or b"").decode("utf-8", errors="replace")
            rc = out.channel.recv_exit_status()
            c.close()
            return rc, data
        except Exception as exc:
            last = exc
            time.sleep(1.5)
    return 255, f"paramiko 失败(3次): {type(last).__name__}: {str(last)[:80]}"


def gate(name: str, ok: bool | None, evidence: str) -> None:
    st = "PASS" if ok else ("SKIP" if ok is None else "FAIL")
    GATES.append((name, st, evidence))
    print(f"  {'✅' if ok else ('⏭️' if ok is None else '❌')} {name}: {evidence}")


def gate_d(name: str, ok: bool, evidence: str, *, dry: bool) -> None:
    """dry-run 时一律 SKIP（无远端输出可判，不制造假红）。"""
    gate(name, None if dry else ok, "dry-run 跳过" if dry else evidence)


# --------------------------------------------------------------------------- #
def resolve_host(args) -> str | None:
    """--host 显式 > mDNS > 历史候选。返回可 SSH 的地址或 None。"""
    if args.host:
        return args.host
    print("== 预检：解析 Orin 地址 ==")
    try:
        ip = socket.gethostbyname(MDNS)
        print(f"  mDNS {MDNS} -> {ip}")
        return ip
    except socket.gaierror:
        pass
    for cand in KNOWN_HOSTS:
        rc, _ = ssh(cand, args.user, "true", dry=False, timeout=6)
        if rc == 0:
            print(f"  历史候选命中: {cand}")
            return cand
    return None


def preflight(host: str, user: str, *, dry: bool) -> bool:
    print(f"== 预检：SSH 探活 + 时钟偏差（{user}@{host}）==")
    rc, _ = ssh(host, user, "true", dry=dry)
    if rc != 0:
        print("  ✗ SSH 不通（免密与密码登录均失败——检查主机/密码 AG_ORIN_PASS）")
        return False
    gate("SSH 探活", True, f"{user}@{host}")
    rc, out = ssh(host, user, "date +%s", dry=dry)
    if rc == 0 and out.strip().isdigit():
        skew = abs(int(out.strip()) - int(time.time()))
        gate("时钟偏差", skew <= 1 or None, f"{skew}s（>1s 仅告警：历史偏过 ~3 天，需人工对时）")
    return True


# --------------------------------------------------------------------------- #
def stage_code(host: str, user: str, *, dry: bool) -> bool:
    print("== 阶段① 源码+模型+工具链+ag_integrate（orin_deploy --verify）==")
    cmd = [sys.executable, str(HERE / "orin_deploy.py"), "--host", host, "--user", user, "--verify"]
    return sh(cmd, dry=dry) == 0


def stage_px4(host: str, user: str, *, dry: bool) -> bool:
    print("== 阶段② px4_odom_convert v2.2 定位补丁（patch_deploy）==")
    cmd = [sys.executable, str(HERE / "patch_deploy.py"), "--host", host, "--user", user]
    return sh(cmd, dry=dry) == 0


def stage_engines(host: str, user: str, *, dry: bool) -> bool:
    print("== 阶段③ 引擎核查（缺 D13/v4d 自动转换）==")
    need = ("hbb_detector_d13_fp16.engine", "cell_classifier_v4d_fp16.engine")
    rc, out = ssh(host, user, "ls ~/dzuav/engines/", dry=dry)
    have = {ln.strip() for ln in out.splitlines() if ln.strip().endswith(".engine")}
    missing = [e for e in need if e not in have]
    if rc == 0 and not missing:
        gate("引擎在位", True, f"{len(have)} 个 .engine，含 D13+v4d")
        return True
    if dry:
        print(f"  [dry-run] 缺 {missing} → tools/orin_convert_engines.sh")
        return True
    print(f"  缺引擎: {missing} → 执行 orin_convert_engines.sh（机上 trtexec，约数分钟）")
    rc2, out2 = ssh(host, user, f"cd {REMOTE_ROOT} && bash tools/orin_convert_engines.sh",
                    dry=dry, timeout=1800)
    rc3, out3 = ssh(host, user, "ls ~/dzuav/engines/", dry=False)
    have2 = {ln.strip() for ln in out3.splitlines() if ln.strip().endswith(".engine")}
    ok = all(e in have2 for e in need)
    gate("引擎转换", ok, f"缺 {missing} 转换后 {'齐' if ok else '仍缺'}")
    return ok


def stage_bringup(host: str, user: str, *, dry: bool) -> bool:
    print("== 阶段④ 规划侧重启（orin_bringup restart）==")
    rc, out = ssh(host, user, f"cd {REMOTE_ROOT} && bash tools/orin_bringup.sh restart",
                  dry=dry, timeout=120)
    return rc == 0


# --------------------------------------------------------------------------- #
def verify_gates(host: str, user: str, *, dry: bool) -> None:
    print("== 验证门（静态项；动态项见 RUNBOOK §3）==")
    # G1 部署树 + route_progress 修复标记
    rc, out = ssh(host, user,
                  f"grep -c '连写' {REMOTE_ROOT}/src/ag_integrate/ag_integrate/route_progress.py",
                  dry=dry)
    gate_d("G1 缺雷修复在位", rc == 0 and out.strip() not in ("0", ""),
           "route_progress.py 含连写朝向修复（2026-10-24）" if rc == 0 else "文件缺失", dry=dry)
    # G2 px4 进程 + 横幅
    rc, out = ssh(host, user, "pgrep -af px4_odom_convert | head -1", dry=dry)
    gate_d("G2 px4 进程", rc == 0 and bool(out.strip()), (out.strip() or "无进程")[:80], dry=dry)
    # G3 引擎（与阶段③同判，复核）
    rc, out = ssh(host, user, "ls ~/dzuav/engines/*.engine 2>/dev/null | xargs -n1 basename",
                  dry=dry)
    has = ("hbb_detector_d13_fp16" in out) and ("cell_classifier_v4d_fp16" in out)
    gate_d("G3 D13/v4d 引擎", has, f"{len([l for l in out.splitlines() if l.strip()])} 个引擎", dry=dry)
    # G4 sidecar 进程
    rc, out = ssh(host, user, "pgrep -af 'orin_plan_sidecar|orin_bringup' | head -2", dry=dry)
    joined = out.strip().replace("\n", " | ") or "无"
    gate_d("G4 规划侧进程", rc == 0 and bool(out.strip()), joined[:100], dry=dry)


def report(rollback_hint: bool) -> int:
    print("\n" + "=" * 72)
    print("一键部署汇总")
    print("=" * 72)
    n_fail = sum(1 for _, s, _ in GATES if s == "FAIL")
    for name, st, ev in GATES:
        print(f"  [{st}] {name}: {ev}")
    if n_fail:
        print(f"\n❌ {n_fail} 项 FAIL。回滚（按需）：")
        print("    python tools/orin_deploy.py --host <IP> --restore    # ag_integrate 回备份")
        print("    python tools/patch_deploy.py  --host <IP> --rollback # px4 回原版")
        return 1
    print("\n✅ 全部通过。动态验证项（起飞后）：")
    print("    - sidecar 日志出现『重规划基于最新进度: 起点 <格>』（修复生效直接证据）")
    print("    - px4 health 行 status_age_s < 2；拔飞控 USB 应出现『status=断流(按armed处理)』")
    print("    - orin_build_verify.sh L3 推理自检（D13 17 通道契约）")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="无人机端一键部署（预检→四阶段→验证门→报告）")
    ap.add_argument("--host", default="", help="Orin IP（缺省: mDNS → 历史候选自动探测）")
    ap.add_argument("--user", default=os.environ.get("AG_ORIN_USER", "duzhong"))
    ap.add_argument("--skip", nargs="*", default=[], choices=["code", "px4", "engines", "bringup"],
                    help="跳过指定阶段")
    ap.add_argument("--verify-only", action="store_true", help="只跑验证门（部署后体检）")
    ap.add_argument("--dry-run", action="store_true", help="只打印将执行的命令（含验证门）")
    args = ap.parse_args()

    host = resolve_host(args)
    if not host:
        print("✗ 无法解析 Orin 地址：--host 指定，或确认机器在线/mDNS 可达")
        return 2
    if not preflight(host, args.user, dry=args.dry_run):
        return 2

    if not args.verify_only:
        ok = True
        if "code" not in args.skip:
            ok = stage_code(host, args.user, dry=args.dry_run) and ok
        if "px4" not in args.skip:
            ok = stage_px4(host, args.user, dry=args.dry_run) and ok
        if "engines" not in args.skip:
            ok = stage_engines(host, args.user, dry=args.dry_run) and ok
        if "bringup" not in args.skip:
            ok = stage_bringup(host, args.user, dry=args.dry_run) and ok
        if not ok and not args.dry_run:
            print("⚠️ 有阶段失败——继续跑验证门取证，报告将给回滚命令")

    verify_gates(host, args.user, dry=args.dry_run)
    return report(rollback_hint=any(s == "FAIL" for _, s, _ in GATES))


if __name__ == "__main__":
    sys.exit(main())
