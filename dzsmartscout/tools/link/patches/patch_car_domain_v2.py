#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""方案 A 完整版（v2）：把车侧 ROS 域号收敂到【唯一真相源】~/dzrobot/dzros.env。

v1 只改了 start_all.sh（交互式 vs 非交互式两种起法不一致的根因）。
v2 补齐另外 4 个会起 ROS / 跑 ros2 命令的入口 —— 它们同样"不设域号"：
    ~/dzrobot/monitor_aim.sh          监控自瞄数值
    ~/dzrobot/tools/healthcheck.sh    出车体检（域号不对会给出假警报）
    ~/dzrobot/tools/restart_node.sh   ★ 单独重启节点：域号不对 = 该节点与全车失联
    ~/dzrobot/tools/test_voice.sh     语音自检

做完之后：改域号只改 dzros.env 一个文件，全车 5 个入口自动跟随。
（车侧无 systemd 自启单元、无 crontab —— 已实测确认，所以入口只有这 5 个。）

幂等、先备份、锚点唯一性断言、逐文件 bash -n。

用法：python3 patch_car_domain_v2.py [--dry-run]
"""
from __future__ import annotations

import hashlib
import pathlib
import shutil
import subprocess
import sys
import time

HOME = pathlib.Path.home()
DZ = HOME / "dzrobot"
ENV_FILE = DZ / "dzros.env"
START_ALL = DZ / "start_all.sh"
BAK_DIR = DZ / "_backups"
AUX = ["monitor_aim.sh", "tools/healthcheck.sh",
       "tools/restart_node.sh", "tools/test_voice.sh"]

ANCHOR_SETUP = "/opt/ros/humble/setup.bash"

# ---- start_all.sh 内的两处精确替换（v1 留下的文本 → v2 文本）----
OLD_BLOCK = '''\
DZ_ROS_DOMAIN=118
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
'''

NEW_BLOCK = '''\
if [ -f "$WS/dzros.env" ]; then . "$WS/dzros.env"; fi    # ★ 全车唯一真相源
DZ_ROS_DOMAIN="${DZ_ROS_DOMAIN:-118}"                    # 兜底：上面文件缺失时才用它
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DZ_ROS_DOMAIN}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
'''

OLD_COMMENT = "#    ★ 改域号必须【两端一起改】：无人机侧对应 ~/dzlink_tools/dzlink.env"
NEW_COMMENT = ("#    ★ 改域号只改一个文件：$WS/dzros.env"
               "（无人机侧对应 ~/dzlink_tools/dzlink.env）")

MARK = "dzros.env"


def die(msg: str) -> "None":
    print(f"❌ {msg}")
    raise SystemExit(1)


def bak(path: pathlib.Path, dry: bool) -> None:
    BAK_DIR.mkdir(parents=True, exist_ok=True)
    b = BAK_DIR / f"{path.name}.bak_{time.strftime('%Y%m%d_%H%M%S')}"
    if not dry:
        shutil.copy2(path, b)
    print(f"   备份 → {b}")


def syntax_ok(path: pathlib.Path) -> bool:
    r = subprocess.run(["bash", "-n", str(path)], capture_output=True, text=True)
    if r.returncode != 0:
        print(f"   ❌ bash -n 失败: {r.stderr.strip()}")
        return False
    return True


def step_env_file(dry: bool) -> None:
    print("=" * 74)
    print(f"[1/3] 唯一真相源 {ENV_FILE}")
    tpl = pathlib.Path(__file__).with_name("dzros.env")
    if tpl.is_file():
        content = tpl.read_text(encoding="utf-8")
        print(f"   模板来源: {tpl}（{len(content.encode('utf-8'))} bytes）")
    else:
        die("找不到同目录的 dzros.env 模板（就是工作区 tools/link/dzros.env）")
    old = ENV_FILE.read_text(encoding="utf-8") if ENV_FILE.is_file() else None
    if old == content:
        print("✅ 已存在且逐字节一致，跳过")
        return
    if old is not None:
        bak(ENV_FILE, dry)
    if dry:
        print("   (dry-run 未写入)")
        return
    ENV_FILE.write_bytes(content.encode("utf-8"))
    print(f"✅ 写入 {len(content.encode('utf-8'))} bytes  "
          f"md5={hashlib.md5(ENV_FILE.read_bytes()).hexdigest()}")


def step_start_all(dry: bool) -> None:
    print("=" * 74)
    print(f"[2/3] {START_ALL}")
    raw = START_ALL.read_bytes()
    text = raw.decode("utf-8")
    if "全车唯一真相源" in text:
        print("✅ 已是 v2（已 source dzros.env），跳过")
        return
    for name, needle in (("OLD_BLOCK", OLD_BLOCK), ("OLD_COMMENT", OLD_COMMENT)):
        if text.count(needle) != 1:
            die(f"{name} 命中 {text.count(needle)} 次（要 1 次）")
    new = text.replace(OLD_BLOCK, NEW_BLOCK).replace(OLD_COMMENT, NEW_COMMENT)
    if new.count(MARK) != 3:
        die(f"替换后 dzros.env 出现 {new.count(MARK)} 次（期望 3：注释1+注释2+逻辑1）")
    if dry:
        print("   (dry-run 未写入)  将替换：")
        for ln in NEW_BLOCK.splitlines():
            print(f"      {ln}")
        return
    bak(START_ALL, dry)
    START_ALL.write_bytes(new.encode("utf-8"))
    print(f"✅ 写入 {len(new.splitlines())} 行  "
          f"md5={hashlib.md5(START_ALL.read_bytes()).hexdigest()}")
    if not syntax_ok(START_ALL):
        die("语法失败（备份在 _backups/，请手工回滚）")


def step_aux(dry: bool) -> None:
    print("=" * 74)
    print("[3/3] 另外 4 个入口")
    for rel in AUX:
        p = DZ / rel
        print(f"\n--- {rel} ---")
        if not p.is_file():
            print("   ⚠️ 文件不存在，跳过")
            continue
        raw = p.read_bytes()
        if b"\r\n" in raw:
            print("   ⚠️ CRLF，跳过（不碰）")
            continue
        lines = raw.decode("utf-8").split("\n")
        if MARK in raw.decode("utf-8"):
            print("   ✅ 已 patched，跳过")
            continue
        hits = [i for i, ln in enumerate(lines) if ANCHOR_SETUP in ln]
        if not hits:
            print(f"   ⚠️ 找不到 {ANCHOR_SETUP}，跳过")
            continue
        i = hits[0]
        indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
        ins = [f"{indent}# ★ ROS 域号：统一从 dzros.env 取（别再依赖 ~/.bashrc —— 换种起法会静默变成域 0）",
               f'{indent}if [ -f "$HOME/dzrobot/dzros.env" ]; then . "$HOME/dzrobot/dzros.env"; fi']
        print(f"   在 {rel}:{i + 1} 之前插入 2 行（缩进 {len(indent)} 空格）")
        if dry:
            for ln in ins:
                print(f"      {ln}")
            continue
        lines[i:i] = ins
        bak(p, dry)
        p.write_bytes("\n".join(lines).encode("utf-8"))
        print(f"   ✅ 写入 {len(lines)} 行  "
              f"md5={hashlib.md5(p.read_bytes()).hexdigest()}")
        if not syntax_ok(p):
            die(f"{rel} 语法失败（备份在 _backups/）")


def main() -> int:
    dry = "--dry-run" in sys.argv
    step_env_file(dry)
    step_start_all(dry)
    step_aux(dry)
    print("=" * 74)
    if dry:
        print("(dry-run 结束，未写入任何文件)")
        return 0
    print("✅ 车侧 v2 完成。下一步跑 verify_car_domain.sh 复核。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
