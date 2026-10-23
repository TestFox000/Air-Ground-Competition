#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 A 的【无人机侧】补丁：
    ① 写 ~/dzlink_tools/dzlink.env（域号唯一真相源，与小车 118 对齐）
    ② 给 ~/dzlink_tools/dz_send_waypoint.py 加"自动读 env + 打印实际域号"

为什么必须在 import 阶段读 env：rclpy 在 rclpy.init() 时读 ROS_DOMAIN_ID，
晚了就无效 —— 这正是 2026-10-07 那次"发出去没人收"的根因。

三处插入（全部基于唯一锚点，幂等，先备份）：
    A. import 段之后 → os/pathlib + _read_link_env() + domain_report()
    B. main() 解析完参数 → 每个子命令都打印实际生效域号
    C. send() 的"没有车侧订阅者"分支 → 直接报出域号，别让人再去猜

用法：python3 patch_drone_sender.py [--dry-run]
"""
from __future__ import annotations

import hashlib
import pathlib
import shutil
import subprocess
import sys
import time

HOME = pathlib.Path.home()
ENV_FILE = HOME / "dzlink_tools" / "dzlink.env"
SENDER = HOME / "dzlink_tools" / "dz_send_waypoint.py"
BAK_DIR = HOME / "dzlink_tools" / "_backups"

SENTINEL = "domain_report"
ANCHOR_IMPORT = "import time"
ANCHOR_ARGS = "    args = ap.parse_args(argv)"
ANCHOR_NOSUB = '        print("  ⚠️ 没有车侧订阅者 —— 两种可能：")'

ENV_CONTENT = """\
# ==============================================================================
#  无人机侧链路环境（★ 域号唯一真相源；小车侧对应 ~/dzrobot/start_all.sh 的 DZ_ROS_DOMAIN）
#  2026-10-07 事故：车在域 118（读 ~/.bashrc），无人机默认 0 →
#    ping 通、SSH 通、网页台通，但 ROS 互相看不见，且一个错都不报。
#  dz_send_waypoint.py 在 import 阶段自动读本文件（晚于 rclpy.init 就无效）。
# ==============================================================================
export ROS_DOMAIN_ID=118
export ROS_LOCALHOST_ONLY=0
"""

IMPORT_BLOCK = '''
import os
import pathlib

# --------------------------------------------------------------------------- #
#  ★ 链路环境：ROS 域号必须与小车一致（2026-10-07 联调事故，见 dzlink.env）
#    必须在 import 阶段灌进 os.environ —— rclpy.init() 之后设就晚了。
#    外部显式设的 ROS_DOMAIN_ID 仍优先，但 domain_report() 会报警，不会静默走错域。
# --------------------------------------------------------------------------- #
DZLINK_ENV = pathlib.Path.home() / "dzlink_tools" / "dzlink.env"


def _read_link_env(path=DZLINK_ENV) -> dict:
    """读 dzlink.env（只认 export KEY=VALUE / KEY=VALUE，不做 shell 求值）。"""
    out = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return out
    for raw in text.splitlines():
        s = raw.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("export "):
            s = s[7:].strip()
        if "=" not in s:
            continue
        k, v = s.split("=", 1)
        k = k.strip()
        if k:
            out[k] = v.strip().strip('"').strip("'")
    return out


LINK_ENV = _read_link_env()
for _k, _v in LINK_ENV.items():
    os.environ.setdefault(_k, _v)


def domain_report() -> str:
    """一行域号摘要，不一致时直接报警（所有子命令都会打印）。"""
    eff = os.environ.get("ROS_DOMAIN_ID") or "0"
    want = LINK_ENV.get("ROS_DOMAIN_ID")
    if not DZLINK_ENV.exists():
        return (f"ROS 域: {eff}（默认）  ⚠️ 没找到 {DZLINK_ENV} —— "
                f"车侧在域 118 时必须建它，否则发出去没人收")
    if want is not None and eff != want:
        return (f"ROS 域: {eff}  ⚠️ 与 {DZLINK_ENV.name}({want}) 不一致 —— "
                f"车侧也在 {eff} 才看得见")
    return f"ROS 域: {eff}（来自 {DZLINK_ENV.name}）"

'''

ARGS_BLOCK = '''
    print(domain_report())          # ★ 每个子命令都先把域号亮出来

'''

# ★ 缩进必须是 12 空格 —— 这一行要插在 `if matched == 0:` 块内第一条之后，
#   写成 8 空格会让后面的 12 空格语句变成 IndentationError。
NOSUB_BLOCK = '''            print(f"  ⚠️ 当前 {domain_report()}  ← 域号不一致是本故障的头号原因")
'''


def die(msg: str) -> "None":
    print(f"❌ {msg}")
    raise SystemExit(1)


def write_env(dry: bool) -> None:
    print("=" * 70)
    print(f"[1/2] 链路环境文件 {ENV_FILE}")
    ENV_FILE.parent.mkdir(parents=True, exist_ok=True)
    # ★ 优先用与本脚本同目录的 dzlink.env 模板（= 工作区 tools/link/dzlink.env 的原样副本），
    #   这样"装上去的文件"与"版本管理里的文件"逐字节一致，不会出现两份文本各自演化。
    tpl = pathlib.Path(__file__).with_name("dzlink.env")
    if tpl.is_file():
        content = tpl.read_text(encoding="utf-8")
        print(f"   模板来源: {tpl}（{len(content.encode('utf-8'))} bytes）")
    else:
        content = ENV_CONTENT
        print("   模板来源: 脚本内嵌兜底文本")
    old = ENV_FILE.read_text(encoding="utf-8") if ENV_FILE.is_file() else None
    if old == content:
        print("✅ 已存在且内容一致，跳过")
        return
    if old is not None:
        BAK_DIR.mkdir(parents=True, exist_ok=True)
        bak = BAK_DIR / f"dzlink.env.bak_{time.strftime('%Y%m%d_%H%M%S')}"
        if not dry:
            shutil.copy2(ENV_FILE, bak)
        print(f"   旧文件已备份 → {bak}")
    if dry:
        print("   (dry-run 未写入)")
        return
    ENV_FILE.write_bytes(content.encode("utf-8"))
    print(f"✅ 写入完成 ({len(content.encode('utf-8'))} bytes, LF)")
    for ln in content.splitlines():
        if ln.startswith("export "):
            print(f"   {ln}")


def patch_sender(dry: bool) -> None:
    print("=" * 70)
    print(f"[2/2] 发送器 {SENDER}")
    if not SENDER.is_file():
        die(f"找不到 {SENDER}")
    raw = SENDER.read_bytes()
    if b"\r\n" in raw:
        die("发送器是 CRLF 换行，拒绝改动")
    text = raw.decode("utf-8")
    print(f"   原 md5 {hashlib.md5(raw).hexdigest()}  行数 {len(text.splitlines())}")

    if SENTINEL in text:
        print("✅ 已打过本补丁（检测到 domain_report），无需重复")
        return

    lines = text.split("\n")
    for name, anchor in (("import", ANCHOR_IMPORT), ("parse_args", ANCHOR_ARGS),
                         ("no-subscriber", ANCHOR_NOSUB)):
        hit = [i for i, ln in enumerate(lines) if ln == anchor or ln.strip() == anchor.strip()]
        if len(hit) != 1:
            die(f"锚点 {name} 命中 {len(hit)} 次（要 1 次）：{anchor!r}")
        print(f"   锚点 {name:<14}: 第 {hit[0] + 1} 行")

    # 从后往前插，避免行号位移。
    # ★ 用 strip 后的比较：锚点行带缩进，写死缩进数容易错（本轮就踩过一次）。
    def find(anchor: str) -> int:
        return next(i for i, ln in enumerate(lines)
                    if ln.strip() == anchor.strip())

    for anchor, block in ((ANCHOR_NOSUB, NOSUB_BLOCK),
                          (ANCHOR_ARGS, ARGS_BLOCK),
                          (ANCHOR_IMPORT, IMPORT_BLOCK)):
        i = find(anchor)
        ins = block.split("\n")
        if ins and ins[0] == "":
            ins = ins[1:]
        if ins and ins[-1] == "":
            ins = ins[:-1]
        lines[i + 1:i + 1] = ins

    out = "\n".join(lines)
    if "def domain_report" not in out or out.count("domain_report()") < 2:
        die("插入结果自检失败（domain_report 未生效）")
    if not out.lstrip().startswith("#!"):
        die("首行 shebang 被破坏")

    if dry:
        print("   (dry-run 未写入)")
        return

    BAK_DIR.mkdir(parents=True, exist_ok=True)
    bak = BAK_DIR / f"dz_send_waypoint.py.bak_{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(SENDER, bak)
    print(f"   备份 → {bak}")
    SENDER.write_bytes(out.encode("utf-8"))
    print(f"   新 md5 {hashlib.md5(SENDER.read_bytes()).hexdigest()}  行数 {len(lines)}")

    r = subprocess.run([sys.executable, "-m", "py_compile", str(SENDER)],
                       capture_output=True, text=True)
    print(f"   py_compile: exit={r.returncode} {r.stderr.strip()}")
    if r.returncode != 0:
        shutil.copy2(bak, SENDER)
        die("编译检查失败，已回滚")


def main() -> int:
    dry = "--dry-run" in sys.argv
    write_env(dry)
    patch_sender(dry)
    print("=" * 70)
    print("✅ 无人机侧完成" if not dry else "(dry-run 结束)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
