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


#: ★★ WB-20：台账全量视图。**过期检查与失效检查都从这里取**，
#:   不允许两处各遍历一遍（那会变成第二份实现）。
#:
#: 为什么需要它（本卡根因）：原写法 `reg = {e["path"]: e for e in in_flight}`
#: 是 dict，同 path 只留**最后一条** ⇒ 实测 36 条只映射出 19 个 path，
#: **17 条被静默覆盖**，其中 3 条过期未收口 ⇒ 门禁报 GREEN。
#:   “后来的登记代表当前状态”这个假设在本仓是错的：多条登记的语义
#:   是“**多个卡同时在改同一文件**”（铁证：`config/robot.yaml` 那条把 5 个卡号
#:   塞进一个 `card` 字符串——使用者已在手动绕开 dict）。
#:
#: 返回顺序保留原文件顺序（可逐条对账）；`is_last` 标记同 path 的最后一条
#: （仅供“被覆盖”诊断与兼容旧输出之用，**不参与判定是否有主**）。
def _ledger_view(in_flight: list) -> list[dict]:
    """台账全量视图。每条登记都出现一次，不合并、不覆盖。"""
    last_idx: dict[str, int] = {}
    for i, e in enumerate(in_flight):
        last_idx[e.get("path", "").replace("\\", "/")] = i
    out = []
    for i, e in enumerate(in_flight):
        path = e.get("path", "").replace("\\", "/")
        out.append({
            "index": i,
            "path": path,
            "entry": e,
            "is_last": last_idx.get(path) == i,
        })
    return out


#: ★★ WB-20：卡库卡号索引。登记里写的卡号必须**真实存在**。
#:   为什么需要它：实测 13 条登记指向不存在的卡号
#:   （4 幻灵 + 7 截断碎片），其中含 `main.py` / `config/map.yaml` 这种核心文件
#:   ⇒ 等于“**用不存在的卡洗白冻结区改动**”，而门禁完全不知道。
def _card_index(card_root: str | None = None) -> set[str]:
    """卡库里所有卡号（**在办 + 归档**都算，只看文件名，不看 status）。"""
    import pathlib as _pl
    root = _pl.Path(card_root) if card_root else (ROOT / ".." / "docs" / "30_任务卡库")
    try:
        return {p.stem for p in root.rglob("*.md")}
    except OSError:
        return set()


def _split_cards(card_field: object) -> list[str]:
    """`card` 字段可能是 `A + B + C` 拼接形态（实测有人这么写），拆开逐个验。"""
    return [c.strip() for c in str(card_field or "").split("+") if c.strip()]


def _superseded_problem(e: dict, path: str, cards: set[str]) -> str:
    """校 `superseded_by`。返回空字符串 = 合法；否则返回不合法理由。"""
    sup = str(e.get("superseded_by") or "").strip()
    if not sup:
        return ""
    my_cards = _split_cards(e.get("card", ""))
    if sup in my_cards:
        return "`superseded_by` 指向自己（%s）—— 自指 = 无操作" % sup
    if sup not in cards:
        return ("`superseded_by` 指向不存在的卡号 %s "
                "—— 不许用虚构接手者洗白过期条" % sup)
    return ""


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

    # ★★ WB-20：全量视图。下面两处遍历（过期检查 + 失效检查）**都从它取**。
    ledger = _ledger_view(cfg.get("in_flight", []))
    # ★★ WB-20 第③步：卡库索引（一次建好，后面校 `card` 与 `superseded_by` 都用它）。
    cards = _card_index()
    today = date.today()

    # “该文件有没有人管”= **任一登记存在**（不要求未过期——过期会单独报 RED。
    #   这里只判“有没有登记”，避免把两件事（有主 / 过期）挤在一个判据里。
    registered_paths = {x["path"] for x in ledger}
    unregistered = [f for f in dirty_frozen if f not in registered_paths]
    for f in unregistered:
        fail.append("冻结区未登记改动: %s" % f)

    expired, unassigned = [], []
    for x in ledger:
        path, e = x["path"], x["entry"]
        try:
            exp = date.fromisoformat(e.get("expires", "1970-01-01"))
        except ValueError:
            fail.append("登记日期格式错误: %s（卡 %s）" % (path, e.get("card", "?")))
            continue
        if exp < today:
            # ★ WB-20 第③步：过期条分两级——
            #   写了**合法** `superseded_by` ⇒ WARN（已被取代，可从台账移除）；
            #   没写 / 写了但不合法        ⇒ RED（过期且无人有效声明接手）。
            expired.append(path)
            sup_bad = _superseded_problem(e, path, cards)
            sup = str(e.get("superseded_by") or "").strip()
            if sup and not sup_bad:
                warn.append("登记已过期但已声明取代: %s（卡 %s，到期 %s，由 %s 接手）"
                            % (path, e.get("card", "?"), exp, sup))
            else:
                fail.append("登记已过期未收口: %s（卡 %s，到期 %s）%s"
                            % (path, e.get("card", "?"), exp,
                               ("；" + sup_bad) if sup_bad else ""))
        # ★ WB-20 第③步：`card` 里每个卡号必须真实存在（截断碎片与幻灵都由此判出）。
        bad_cards = [c for c in _split_cards(e.get("card", "")) if c not in cards]
        if bad_cards:
            fail.append("登记指向不存在的卡号: %s（查不到 %s）"
                        % (path, ", ".join(bad_cards)))
        if e.get("owner", "unassigned") in ("", "unassigned"):
            unassigned.append(path)
            fail.append("登记未指定 owner（无人收口）: %s（卡 %s）" % (path, e.get("card", "?")))
        # ★ WB-20：失效检查也走**全量**——旧写法只对 dict 的 19 条跑，
        #   被覆盖的 17 条即使“文件已不脏、登记该删了”也永远不提示。
        if path not in dirty_frozen:
            warn.append("登记已失效（文件已不脏，可移除）: %s（卡 %s）" % (path, e.get("card", "?")))

    result = {
        "ok": not fail,
        "frozen_dirty": dirty_frozen,
        "registered": sorted(registered_paths),
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
            mark = "OK " if f in registered_paths else "RED"
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
