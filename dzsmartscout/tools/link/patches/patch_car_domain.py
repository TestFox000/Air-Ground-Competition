#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 A：把 ROS 域号钉进【小车】的 ~/dzrobot/start_all.sh（保持 118）。

背景（2026-10-07 无人机↔小车联调事故）：
    小车从交互式终端起栈 → 读 ~/.bashrc → ROS_DOMAIN_ID=118
    从 systemd / cron / ssh 一行命令 / 别的脚本起栈 → 不读 ~/.bashrc → 默认 0
    两种起法域号不同，症状完全一样且一个错都不报：
      ping 通、SSH 通、网页台 :8080 通，但 ROS 死活看不见对方。

本补丁做两处插入：
    ① ROS 环境段之前 → DZ_ROS_DOMAIN=118 + export ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY
    ② 启动横幅之前  → 打印实际生效域号，偏离标准值就报警

特性：幂等（已打过就只报告）、先备份到 ~/dzrobot/_backups/、锚点唯一性断言、
      插入位置早于首次 spawn 的断言、末尾跑 bash -n 语法检查。

用法：python3 patch_car_domain.py [--dry-run]
"""
from __future__ import annotations

import hashlib
import pathlib
import shutil
import subprocess
import sys
import time

TARGET = pathlib.Path.home() / "dzrobot" / "start_all.sh"
BAK_DIR = pathlib.Path.home() / "dzrobot" / "_backups"

SENTINEL = "DZ_ROS_DOMAIN"                     # 幂等标记
ANCHOR_A = "# ---- stop / restart 需要在 stop_all 之前"   # 唯一的注释行
ANCHOR_B = '        echo "1/8 底盘串口'        # 实际是行首，这里只取不重复的子串
ANCHOR_B_SUB = '1/8 底盘串口'

ENV_BLOCK = """\
# ==============================================================================
#  ★ ROS 域号：钉在这里，不再依赖 ~/.bashrc（2026-10-07 无人机↔小车联调事故）
#    · 交互式终端起栈              → 读 ~/.bashrc → 118
#    · systemd / cron / ssh 一行命令 → 不读 ~/.bashrc → 默认 0
#    两种起法域号不同，症状一模一样且【一个错都不报】：
#      ping 通、SSH 通、网页台 :8080 通，但 ROS 死活看不见对方。
#    ★ 改域号必须【两端一起改】：无人机侧对应 ~/dzlink_tools/dzlink.env
#    外部已显式 export 的 ROS_DOMAIN_ID 仍优先（临时 A/B 用），但启动横幅会报警。
# ==============================================================================
DZ_ROS_DOMAIN=118
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

"""

BANNER_BLOCK = """\
# ---- 链路自检：域号不一致是最阴的静默故障（对端不同域时 ROS 一个错都不报）----
echo "ROS 链路域号: ROS_DOMAIN_ID=$ROS_DOMAIN_ID  ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY  (车标准域 $DZ_ROS_DOMAIN)"
if [ "$ROS_DOMAIN_ID" != "$DZ_ROS_DOMAIN" ]; then
  echo "        ⚠️  域号偏离标准值 —— 无人机侧必须跟着改 ~/dzlink_tools/dzlink.env，否则互相看不见"
fi
"""


def die(msg: str) -> "None":
    print(f"❌ {msg}")
    raise SystemExit(1)


def main() -> int:
    dry = "--dry-run" in sys.argv
    if not TARGET.is_file():
        die(f"找不到 {TARGET}")

    raw = TARGET.read_bytes()
    if b"\r\n" in raw:
        die("目标文件是 CRLF 换行，拒绝改动（先查清楚为什么）")
    text = raw.decode("utf-8")
    before_md5 = hashlib.md5(raw).hexdigest()
    lines = text.split("\n")

    print(f"目标        : {TARGET}")
    print(f"原 md5      : {before_md5}  行数 {len(lines)}")

    if SENTINEL in text:
        print("✅ 已打过本补丁（检测到 DZ_ROS_DOMAIN），无需重复。")
        for i, ln in enumerate(lines, 1):
            if SENTINEL in ln or "ROS_DOMAIN_ID" in ln:
                print(f"   {i:>4}: {ln}")
        return 0

    # ---- 锚点唯一性 ----
    idx_a = [i for i, ln in enumerate(lines) if ANCHOR_A in ln]
    idx_b = [i for i, ln in enumerate(lines) if ANCHOR_B_SUB in ln]
    if len(idx_a) != 1:
        die(f"锚点A 命中 {len(idx_a)} 次（要 1 次）：{ANCHOR_A}")
    if len(idx_b) != 1:
        die(f"锚点B 命中 {len(idx_b)} 次（要 1 次）：{ANCHOR_B_SUB}")
    a, b = idx_a[0], idx_b[0]
    print(f"锚点A       : 第 {a + 1} 行  {lines[a][:60]}…")
    print(f"锚点B       : 第 {b + 1} 行  {lines[b].strip()[:60]}")

    # ---- 插入（先插后面的，避免行号位移）----
    new = list(lines)
    new[b:b] = BANNER_BLOCK.split("\n")[:-1]
    new[a:a] = ENV_BLOCK.split("\n")[:-1]

    # ---- 断言：export 必须早于首次 spawn，也早于首个 ros2 调用 ----
    export_ln = next(i for i, ln in enumerate(new, 1)
                     if ln.startswith("export ROS_DOMAIN_ID="))
    first_spawn = next((i for i, ln in enumerate(new, 1)
                        if ln.startswith("spawn ") or ln.startswith("spawn\t")), None)
    if first_spawn is None:
        die("找不到任何 `spawn ` 调用，锚点假设失效")
    if not export_ln < first_spawn:
        die(f"export 在第 {export_ln} 行，晚于首次 spawn 第 {first_spawn} 行")
    print(f"断言 ✅     : export 第 {export_ln} 行 < 首次 spawn 第 {first_spawn} 行")

    out = "\n".join(new)
    if dry:
        print("--- dry-run，未写入 ---")
        for i in range(a, a + len(ENV_BLOCK.split("\n")) + 2):
            print(f"   {i + 1:>4}: {new[i]}")
        return 0

    # ---- 备份 ----
    BAK_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bak = BAK_DIR / f"start_all.sh.bak_{stamp}"
    shutil.copy2(TARGET, bak)
    print(f"备份        : {bak}")

    TARGET.write_bytes(out.encode("utf-8"))
    after_md5 = hashlib.md5(TARGET.read_bytes()).hexdigest()
    print(f"新 md5      : {after_md5}  行数 {len(new)}")

    # ---- 语法检查 ----
    r = subprocess.run(["bash", "-n", str(TARGET)], capture_output=True, text=True)
    print(f"bash -n     : exit={r.returncode} {r.stderr.strip()}")
    if r.returncode != 0:
        shutil.copy2(bak, TARGET)
        die("语法检查失败，已回滚")
    print("✅ 补丁完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
