#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工作流 v2 · 收口门禁

用途：每张卡关闭前跑一次，回答三个问题：
  1. 冻结区有没有「未登记」的脏文件？
  2. 已登记的 in_flight 有没有过期还没收口的？
  3. 三本真相文件（台账/复用/工作流）在不在？

用法：
    cd dzrobot_v2 && python tools/wf/check.py
    python tools/wf/check.py --json
退出码：0 通过；1 有阻断项。
"""
import io
import json
import re
import subprocess
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:  # noqa: BLE001
    pass
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CFG = Path(__file__).resolve().parent / "frozen.json"
REQUIRED_DOCS = [
    "docs/决策台账.md",
    "docs/复用清单.md",
    "docs/工作流v2.md",
]


def glob_to_re(pat):
    out, i = [], 0
    while i < len(pat):
        c = pat[i]
        if c == "*":
            if pat[i:i + 2] == "**":
                out.append(".*")
                i += 2
                continue
            out.append("[^/]*")
        elif c == "?":
            out.append("[^/]")
        else:
            out.append(re.escape(c))
        i += 1
    return re.compile("^" + "".join(out) + "$")


def git_status():
    try:
        # ★2026-10-18 修：Windows 默认 GBK 解码 git 输出，遇到中文文件名（如 docs/审查_xxx.md）
        #   直接 UnicodeDecodeError，且 r.stdout 变 None 让后面 splitlines() 崩。
        #   必须显式 utf-8 + errors=replace，与文件名的真实编码（UTF-8）对齐。
        r = subprocess.run(
            ["git", "-C", str(ROOT), "-c", "core.quotepath=false", "status", "--porcelain"],
            capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=60,
        )
    except Exception as e:  # noqa: BLE001
        return None, "git 调用失败: %s" % e
    if r.returncode != 0:
        return None, "git status 返回 %d: %s" % (r.returncode, r.stderr.strip())
    files = []
    for line in (r.stdout or "").splitlines():
        if len(line) < 4:
            continue
        path = line[3:]
        if " -> " in path:  # rename/copy
            path = path.split(" -> ")[-1]
        files.append(path.strip().strip('"'))
    return files, None


def main():
    as_json = "--json" in sys.argv
    fail, warn = [], []

    if not CFG.exists():
        fail.append("缺少 tools/wf/frozen.json（冻结区配置）")
        cfg = {"frozen": [], "in_flight": []}
    else:
        cfg = json.loads(CFG.read_text(encoding="utf-8"))

    for d in REQUIRED_DOCS:
        if not (ROOT / d).exists():
            fail.append("缺少真相文件: %s" % d)

    # 文档链接健康：docs 与历史索引断链 = 文档与真源漂移，直接判红（2026-10-18 并入）
    link_status = ""
    lc = Path(__file__).resolve().parent / "check_links.py"
    if lc.exists():
        try:
            r = subprocess.run([sys.executable, str(lc)], capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=120)
            out = (r.stdout or "").strip()
            if r.returncode != 0:
                fail.append("文档断链:\n%s" % out)
            else:
                link_status = out.splitlines()[0] if out else ""
        except Exception as e:  # noqa: BLE001
            fail.append("文档链接检查执行失败: %s" % e)
    else:
        warn.append("缺少 tools/wf/check_links.py（文档链接检查跳过）")

    # 复用门禁：新增 .py 未在复用清单/能力索引登记 → 红（防"又造一个轮子"）
    reuse_status = ""
    rc = Path(__file__).resolve().parent / "check_reuse.py"
    if rc.exists():
        try:
            r = subprocess.run([sys.executable, str(rc)], capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=120)
            out = (r.stdout or "").strip()
            if r.returncode != 0:
                fail.append("复用门禁未通过（新增代码未登记）:\n%s" % out)
            else:
                reuse_status = out.splitlines()[0] if out else ""
        except Exception as e:  # noqa: BLE001
            fail.append("复用门禁执行失败: %s" % e)
    else:
        warn.append("缺少 tools/wf/check_reuse.py（复用门禁跳过）")

    files, err = git_status()
    if err:
        fail.append(err)
        files = []

    pats = [(p, glob_to_re(p)) for p in cfg.get("frozen", [])]
    dirty_frozen = []
    for f in files:
        posix = f.replace("\\", "/")
        if any(rx.match(posix) for _, rx in pats):
            dirty_frozen.append(posix)

    reg = {e["path"].replace("\\", "/"): e for e in cfg.get("in_flight", [])}
    today = date.today()

    unregistered = [f for f in dirty_frozen if f not in reg]
    for f in unregistered:
        fail.append("冻结区未登记改动: %s" % f)

    expired, unassigned = [], []
    for path, e in reg.items():
        try:
            exp = date.fromisoformat(e.get("expires", "1970-01-01"))
        except ValueError:
            fail.append("登记日期格式错误: %s" % path)
            continue
        if exp < today:
            expired.append(path)
            fail.append("登记已过期未收口: %s（卡 %s，到期 %s）" % (path, e.get("card", "?"), exp))
        if e.get("owner", "unassigned") in ("", "unassigned"):
            unassigned.append(path)
            # ★2026-10-18 升级为阻断：无 owner 的在途改动 = 没人收口的悬空改动（项目“反复开倒车”的物理来源）。
            #   曾经只 WARN，结果 4 条 v4.0 收口改动悬了多轮无人认领。
            fail.append("登记未指定 owner（无人收口）: %s（卡 %s）" % (path, e.get("card", "?")))
        if path not in dirty_frozen:
            warn.append("登记已失效（文件已不脏，可移除）: %s" % path)

    result = {
        "ok": not fail,
        "frozen_dirty": dirty_frozen,
        "registered": sorted(reg),
        "unregistered": unregistered,
        "expired": expired,
        "warnings": warn,
        "errors": fail,
    }
    if as_json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print("== 工作流 v2 收口门禁 ==")
        print("冻结区脏文件: %d" % len(dirty_frozen))
        for f in dirty_frozen:
            mark = "OK " if f in reg else "RED"
            print("  [%s] %s" % (mark, f))
        if link_status:
            print("  [OK ] 文档链接 %s" % link_status)
        if reuse_status:
            print("  [OK ] 复用门禁 %s" % reuse_status)
        if unregistered:
            print("未登记（阻断）: %d" % len(unregistered))
        for w in warn:
            print("  [WARN] %s" % w)
        if fail:
            print("阻断项:")
            for e in fail:
                print("  - %s" % e)
            print("结果: RED（%d 项）" % len(fail))
        else:
            print("结果: GREEN")
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
