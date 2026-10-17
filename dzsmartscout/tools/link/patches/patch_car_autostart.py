#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""临时开关小车的 auto_start（原地待命 / 一键起栈后自动出发）。

背景：`config/robot.yaml` 的 `auto_start: true` 意味着 start_all.sh 起栈后
      约 10 秒，车就用【内置 34 点路线】自己出发了。调试/台架验证时必须先关掉。
      注意 `auto_start_delay` 是另一个键，本脚本绝不动它。

用法（在小车上）：
  python3 patch_car_autostart.py            # auto_start -> false（默认，先备份）
  python3 patch_car_autostart.py --on       # auto_start -> true
  python3 patch_car_autostart.py --status   # 只看当前值
  python3 patch_car_autostart.py --restore  # 从最近一次备份整体还原 robot.yaml

每次改动前都会备份到 ~/dzrobot/_backups/，并用 PyYAML 复核文件仍可解析；
复核失败立刻回滚。幂等：值已经对了就什么都不做。
"""
from __future__ import annotations

import pathlib
import re
import shutil
import sys
import time

CFG = pathlib.Path.home() / "dzrobot" / "config" / "robot.yaml"
BAK_DIR = pathlib.Path.home() / "dzrobot" / "_backups"
PAT = re.compile(r"^(\s*)auto_start:(\s*)(true|false)(\s*)(#.*)?$", re.M)


def die(msg: str) -> "None":
    print(f"❌ {msg}")
    raise SystemExit(1)


def yaml_ok(path: pathlib.Path) -> bool:
    try:
        import yaml
        yaml.safe_load(path.read_text(encoding="utf-8"))
        return True
    except Exception as exc:                                  # noqa: BLE001
        print(f"   YAML 解析失败: {exc}")
        return False


def backups() -> list:
    return sorted(BAK_DIR.glob("robot.yaml.bak_*"))


def main() -> int:
    args = set(sys.argv[1:])
    if not CFG.is_file():
        die(f"找不到 {CFG}")
    text = CFG.read_text(encoding="utf-8")
    hits = PAT.findall(text)
    if len(hits) != 1:
        die(f"auto_start 命中 {len(hits)} 次（必须恰好 1 次）")

    m = PAT.search(text)
    cur = m.group(3)

    if "--status" in args:
        print(f"当前 auto_start = {cur}")
        print(f"  行: {m.group(0).strip()}")
        bs = backups()
        print(f"  备份: {bs[-1].name if bs else '（无）'}")
        return 0

    if "--restore" in args:
        bs = backups()
        if not bs:
            die("没有备份可还原")
        src = bs[-1]
        BAK_DIR.mkdir(parents=True, exist_ok=True)
        # ★ 还原前的现场存成 prerestore_ 前缀：它【不】匹配 robot.yaml.bak_* 这个 glob，
        #   否则下一次 --restore 会挑中它、把刚才那个状态又装回去（自己给自己下套）。
        shutil.copy2(CFG, BAK_DIR / f"robot.yaml.prerestore_{time.strftime('%Y%m%d_%H%M%S')}")
        shutil.copy2(src, CFG)
        if not yaml_ok(CFG):
            die("还原后 YAML 解析失败")
        m2 = PAT.search(CFG.read_text(encoding="utf-8"))
        print(f"✅ 已从 {src.name} 还原，auto_start = {m2.group(3)}")
        return 0

    want = "true" if "--on" in args else "false"
    if cur == want:
        print(f"✅ 已经是 auto_start = {want}，无需改动")
        return 0

    BAK_DIR.mkdir(parents=True, exist_ok=True)
    bak = BAK_DIR / f"robot.yaml.bak_{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.copy2(CFG, bak)
    print(f"备份 → {bak}")

    new = text[:m.start(3)] + want + text[m.end(3):]
    CFG.write_text(new, encoding="utf-8")
    if not yaml_ok(CFG):
        shutil.copy2(bak, CFG)
        die("YAML 复核失败，已回滚")

    m2 = PAT.search(CFG.read_text(encoding="utf-8"))
    print(f"✅ auto_start: {cur} → {m2.group(3)}")
    print(f"   {m2.group(0).strip()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
