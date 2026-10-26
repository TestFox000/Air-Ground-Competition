#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数字速查真源校验器 —— 把 docs/20_资料索引库/数字速查.yaml 里的每个数字重算一遍。

它解决的问题：文档里的数字（单元格 0.35 m、前视 clamp 区间、线程 50 Hz、心跳 3 s …）
散在 README 与各分册 00_索引.md 里，抄一次就漂一次，而且没人对账过。本脚本把
「值」与「代码真源」绑成可执行校验：能重算的重算，重算不了的进「需人核」清单。

零第三方依赖（只用标准库；不 import yaml —— 本文件的扁平 YAML 子集由 parse_yaml_flat 解析）。

用法（在 v1 仓库根跑）：
    python docs/_工具/check_numbers.py                # 退出码 1 = 有 mismatch 或新增漂移
    python docs/_工具/check_numbers.py -v             # 每行带表达式
    python docs/_工具/check_numbers.py --strict       # 连存量漂移一起判红（收口用）
    python docs/_工具/check_numbers.py --fix-sources  # 自愈：把能唯一定位的 出处/代码锚点 行号改对
    python docs/_工具/check_numbers.py --write-baseline  # 把现存漂移收进基线（只减不增）
    python docs/_工具/check_numbers.py --yaml <路径> --repo <仓库根>

判红口径（2026-10-26 改）：
    旧的「文档侧漂移默认不判红、只有 --strict 才判」等于让 46 条行号漂移永远沉默：
    robot.yaml 从 894 行长到 900 行后 heartbeat_timeout_s:894 一直指错地方，
    没有一次收口会停下来处理它。现在按 (id :: 类别) 记账，与 check_anchors 的
    anchor_baseline.json 同一范式：
      * 异文仍存（文档值与代码真源冲突）—— 直接判红。除非该条在 yaml 里显式声明
        「异文说明」（= 有意并列的口径冲突，如用户尺量与代码常量），那才放行。
      * 其余漂移（代码锚点行号搬了 / 出处行号或字面量漂了 / 异文已消失）——
        存量写进 numbers_baseline.json，不在表里的新增漂移判红。
    写基线有守卫：本次会新增条目时拒绝写入，必须显式 --allow-grow 并写理由。
    改基线等于抹证据，和 anchor_baseline 的老坑同源。

退出码：
    0  = 全部重算通过，且无新增漂移、无异文冲突
    1  = 有 mismatch（重算 diff 超容差 / 值缺失 / 锚点或出处文件不存在 / 表达式非法）
         或 新增漂移 / 未声明异文 / （--strict 时）任何漂移
    2  = YAML 本身读不懂（格式错）

校验表达式可用函数（在 yaml 的 校验 字段里写，只能是算术表达式）：
    const(路径, 符号名)        读 python 文件里模块级 NAME = <数字>
    const_str(路径, 符号名)    读模块级 NAME = "<字符串>"
    spec(路径, 点号键)         读 core/config.py 里 "键": Spec(<默认值>, ...) 的首个参数
    ros(路径, 键, 段)          读 ROS yaml 里「段」内最后一次出现的键（段写 '/dzbrain' 这种）
    init_rate(路径)            读 tasks/*.py 里 super().__init__("<名字>", <频率>, ...) 的频率
    str_index(字符串, 子串)     子串在字符串里的下标
    tan/sin/cos/atan/sqrt/radians/degrees/pi/abs/round/min/max/len

为什么 segment（段）是必填参数：robot.yaml 的 lost_frames_to_rescan 在
/dzrknn_dataprocess 与 /dzbrain 两段各有一份、值分别是 180 与 60，只给键名会读错。
"""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
from pathlib import Path
from typing import Any

DEFAULT_YAML = "docs/20_资料索引库/数字速查.yaml"
BASELINE_NAME = "numbers_baseline.json"
REQUIRED_FIELDS = ("id", "名称", "值", "单位", "出处", "代码锚点", "校验", "状态")
STATUS_VALUES = ("实测", "文档", "待核")
DEFAULT_TOL = 1e-3        # 相对容差
ABS_TOL = 1e-9            # 绝对容差（用于 值 = 0 的项）

# --------------------------------------------------------------------------- #
#  读文件
# --------------------------------------------------------------------------- #
def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        raise AnchorError(f"文件不存在：{path}")


class AnchorError(Exception):
    """代码/文档锚点找不到（= mismatch，退出码 1）。"""


def line_count(path: Path) -> int:
    return len(read_text(path).splitlines())


# --------------------------------------------------------------------------- #
#  极简 YAML（只支持本文件用到的扁平子集：顶层 '- key: v' 列表 + 2 空格缩进的 key: value）
# --------------------------------------------------------------------------- #
def strip_comment(line: str) -> str:
    """去掉行尾注释，但不碰引号里的 #（值里写 '#' 也不会被截断）。"""
    out = []
    quote = None
    for ch in line:
        if quote:
            out.append(ch)
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
            out.append(ch)
        elif ch == "#":
            break
        else:
            out.append(ch)
    return "".join(out)


def split_kv(text: str, lineno: int) -> tuple[str, str]:
    m = re.match(r"^([^:]+):\s*(.*)$", text)
    if not m:
        raise AnchorError(f"第 {lineno} 行不是 key: value 形态：{text!r}")
    return m.group(1).strip(), m.group(2)


def scalar(text: str) -> Any:
    t = text.strip()
    if not t:
        return None
    if len(t) >= 2 and t[0] == t[-1] and t[0] in "\"'":
        return t[1:-1]
    low = t.lower()
    if low in ("null", "~", "none"):
        return None
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    for cast in (int, float):
        try:
            return cast(t)
        except ValueError:
            continue
    return t


def parse_yaml_flat(text: str) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    cur: dict[str, Any] | None = None
    for lineno, raw in enumerate(text.splitlines(), 1):
        line = strip_comment(raw).rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip())
        body = line.strip()
        if body.startswith("- "):
            if cur is not None:
                items.append(cur)
            cur = {}
            key, val = split_kv(body[2:].strip(), lineno)
            cur[key] = scalar(val)
            cur["_line"] = lineno
            continue
        if cur is None:
            raise AnchorError(f"第 {lineno} 行出现在第一个 '- id:' 之前：{body!r}")
        if indent < 2:
            raise AnchorError(f"第 {lineno} 行缩进异常（应 >= 2 空格）：{raw!r}")
        key, val = split_kv(body, lineno)
        cur[key] = scalar(val)
        cur["_" + key + "_line"] = lineno
    if cur is not None:
        items.append(cur)
    return items


# --------------------------------------------------------------------------- #
#  表达式求值（AST 白名单 + 空 builtins，禁属性/下标，只能调白名单函数）
# --------------------------------------------------------------------------- #
ALLOWED_NODES = (
    ast.Expression, ast.Constant, ast.Name, ast.Load,
    ast.BinOp, ast.UnaryOp, ast.Call,
    ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow,
    ast.USub, ast.UAdd,
)


class Ctx:
    """表达式可见的只读上下文：仓库根 + 读锚点的小工具。"""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.cache: dict[Path, str] = {}

    def text(self, rel: str) -> str:
        p = (self.root / rel).resolve()
        if p not in self.cache:
            self.cache[p] = read_text(p)
        return self.cache[p]

    # ---- 代码锚点读取 ----
    def const(self, rel: str, name: str) -> float:
        pat = re.compile(rf"^{re.escape(name)}\s*=\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)", re.M)
        m = pat.search(self.text(rel))
        if not m:
            raise AnchorError(f"{rel} 里找不到模块级常量 {name}")
        return float(m.group(1))

    def const_str(self, rel: str, name: str) -> str:
        pat = re.compile(rf"^{re.escape(name)}\s*=\s*['\"]([^'\"]*)['\"]", re.M)
        m = pat.search(self.text(rel))
        if not m:
            raise AnchorError(f"{rel} 里找不到字符串常量 {name}")
        return m.group(1)

    def spec(self, rel: str, dotted: str) -> float:
        pat = re.compile(
            rf'"{re.escape(dotted)}"\s*:\s*Spec\(\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)')
        m = pat.search(self.text(rel))
        if not m:
            raise AnchorError(f"{rel} 的 SCHEMA 里找不到 {dotted} 的 Spec 默认值")
        return float(m.group(1))

    def ros(self, rel: str, key: str, section: str | None = None) -> float:
        lines = self.text(rel).splitlines()
        lo, hi = 0, len(lines)
        if section:
            want = section.strip().rstrip(":") + ":"
            start = None
            for i, ln in enumerate(lines):
                if ln.rstrip() == want or (ln.strip() == want and not ln[:1].isspace()):
                    start = i
                    break
            if start is None:
                raise AnchorError(f"{rel} 里找不到段 {section}")
            end = len(lines)
            for j in range(start + 1, len(lines)):
                ln = lines[j]
                if ln and not ln[0].isspace() and ln.strip() and not ln.lstrip().startswith("#"):
                    end = j
                    break
            lo, hi = start, end
        pat = re.compile(rf"^\s*(?:-\s*)?{re.escape(key)}\s*:\s*(.+?)\s*$")
        hits = []
        for i in range(lo, hi):
            if lines[i].lstrip().startswith("#"):
                continue
            m = pat.match(lines[i])
            if m:
                hits.append((i + 1, strip_comment(m.group(1)).strip()))
        if not hits:
            raise AnchorError(f"{rel} 的段 {section or '(全文)'} 里找不到键 {key}")
        lineno, raw = hits[-1]
        try:
            return float(raw)
        except ValueError:
            raise AnchorError(f"{rel}:{lineno} 的 {key} 不是数字：{raw!r}")

    def init_rate(self, rel: str) -> float:
        pat = re.compile(r"super\(\)\.__init__\(\s*\"[^\"]*\"\s*,\s*([\d.]+)")
        m = pat.search(self.text(rel))
        if not m:
            raise AnchorError(f"{rel} 里找不到 super().__init__(\"名字\", 频率, ...)")
        return float(m.group(1))


def make_env(ctx: Ctx) -> dict[str, Any]:
    return {
        "const": ctx.const,
        "const_str": ctx.const_str,
        "spec": ctx.spec,
        "ros": ctx.ros,
        "init_rate": ctx.init_rate,
        "str_index": lambda s, sub: s.index(sub),
        "tan": math.tan, "sin": math.sin, "cos": math.cos, "atan": math.atan,
        "sqrt": math.sqrt, "radians": math.radians, "degrees": math.degrees,
        "pi": math.pi, "abs": abs, "round": round, "min": min, "max": max, "len": len,
    }


def eval_expr(expr: str, env: dict[str, Any]) -> float:
    try:
        tree = ast.parse(expr, mode="eval")
    except SyntaxError as exc:
        raise AnchorError(f"表达式语法错：{exc}")
    for node in ast.walk(tree):
        if not isinstance(node, ALLOWED_NODES):
            raise AnchorError(f"表达式里不允许的语法节点：{type(node).__name__}")
    try:
        out = eval(compile(tree, "<校验>", "eval"), {"__builtins__": {}}, env)  # noqa: S307
    except AnchorError:
        raise
    except Exception as exc:  # noqa: BLE001
        raise AnchorError(f"表达式求值失败：{type(exc).__name__}: {exc}")
    if not isinstance(out, (int, float)) or isinstance(out, bool):
        raise AnchorError(f"表达式结果不是数字：{out!r}")
    return float(out)


# --------------------------------------------------------------------------- #
#  锚点检查
# --------------------------------------------------------------------------- #
LOC_RE = re.compile(r"^(?P<path>.+?):(?P<line>\d+)\s*(?P<rest>.*)$")


def parse_loc(text: str) -> tuple[str, int, str] | None:
    """'path:123 token' -> (path, 123, token)；'无' 之类返回 None。"""
    s = str(text or "").strip()
    if not s or s == "无":
        return None
    m = LOC_RE.match(s)
    if not m:
        return None
    return m.group("path").strip(), int(m.group("line")), m.group("rest").strip()


def check_code_anchor(ctx: Ctx, raw: str) -> tuple[str, str]:
    """返回 (level, 说明)。level: ok / drift / fail"""
    loc = parse_loc(raw)
    if loc is None:
        return "skip", "无锚点（待核项）"
    rel, line, rest = loc
    path = (ctx.root / rel).resolve()
    if not path.exists():
        return "fail", f"锚点文件不存在：{rel}"
    lines = ctx.text(rel).splitlines()
    if not rest:
        return ("ok" if 1 <= line <= len(lines) else "fail"), "仅行号"
    token = rest.split()[0]
    at_line = lines[line - 1] if 1 <= line <= len(lines) else ""
    if token in at_line:
        return "ok", f"{rel}:{line} 含 {token}"
    hits = [i + 1 for i, ln in enumerate(lines) if token in ln]
    if not hits:
        return "fail", f"{rel} 全文找不到 {token}"
    shown = ",".join(str(h) for h in hits[:6]) + ("…" if len(hits) > 6 else "")
    return "drift", f"{token} 已不在 :{line}，实际在 :{shown}"


def check_doc_source(ctx: Ctx, raw: str, must_contain: str) -> tuple[str, str]:
    loc = parse_loc(raw)
    if loc is None:
        return "skip", "无出处（待核项）"
    rel, line, _ = loc
    path = (ctx.root / rel).resolve()
    if not path.exists():
        return "fail", f"出处文件不存在：{rel}"
    lines = ctx.text(rel).splitlines()
    if not (1 <= line <= len(lines)):
        return "drift", f"出处行号越界：{rel}:{line}（文件仅 {len(lines)} 行）"
    if not must_contain:
        return "ok", f"{rel}:{line}"
    tokens = [t.strip() for t in str(must_contain).split(";") if t.strip()]
    missing = [t for t in tokens if t not in lines[line - 1]]
    if missing:
        return "drift", f"{rel}:{line} 已不含 {missing}"
    return "ok", f"{rel}:{line} 含 {len(tokens)} 个字面量"


def fmt_num(v: float) -> str:
    if v == int(v) and abs(v) < 1e15:
        return str(int(v))
    return f"{v:.6g}"


# --------------------------------------------------------------------------- #
#  自愈：把行号改回真代码所在行（只改行号，不改路径、不改值）
# --------------------------------------------------------------------------- #
HEAL_ANCHOR_RE = re.compile(r"^(?P<path>.+?):(?P<line>\d+)\s+(?P<token>\S*)")
HEAL_SOURCE_RE = re.compile(r"^(?P<path>.+?):(?P<line>\d+)")


def self_heal(items: list[dict[str, Any]], drift: dict[str, Any],
              yaml_path: Path, root: Path, ctx: Ctx) -> int:
    """把「代码锚点行号漂移」里能唯一定位的条目改回实际行号。

    只处理 token 在目标文件里**唯一出现**的情形：唯一 = 没有二义，改是安全的。
    多处出现一律不猜（宁留红，不许猜错）。出处（文档侧）不动——那是别人正文。
    """
    out = read_text(yaml_path)
    lines = out.splitlines(keepends=True)
    fixed = 0
    for key, (kind, eid, heal) in drift.items():
        if kind != "anchor_drift" or not heal:
            continue
        raw = str(next((it for it in items if str(it.get("id")) == eid), {}).get(heal["field"], ""))
        m = HEAL_ANCHOR_RE.match(raw.strip())
        if not m:
            continue
        rel, old_line, token = m.group("path"), int(m.group("line")), m.group("token")
        tpath = (root / rel).resolve()
        if not tpath.exists():
            continue
        body = ctx.text(rel).splitlines()
        hits = [i + 1 for i, ln in enumerate(body) if token in ln]
        if not hits:
            continue
        if len(hits) == 1:
            new_line = hits[0]
        else:
            # 多处出现：取离旧行号最近的一个，且不许有并列（并列 = 猜，宁留红）
            dist = sorted((abs(h - old_line), h) for h in hits)
            if len(dist) > 1 and dist[0][0] == dist[1][0]:
                continue
            new_line = dist[0][1]
        if new_line == old_line:
            continue
        lineno_key = heal["line_key"]
        idx = next((it for it in items if str(it.get("id")) == eid), {})
        yline = idx.get(lineno_key)
        if not isinstance(yline, int) or not (1 <= yline <= len(lines)):
            continue
        old_text = lines[yline - 1]
        new_text = old_text.replace(f"{rel}:{old_line}", f"{rel}:{new_line}", 1)
        if new_text != old_text:
            lines[yline - 1] = new_text
            fixed += 1
            continue
        continue

    # ---- 出处（文档侧）行号：按 出处应含 字面量唯一定位 ----
    for key, (kind, eid, heal) in drift.items():
        if kind != "source_drift" or not heal:
            continue
        idx = next((it for it in items if str(it.get("id")) == eid), {})
        raw = str(idx.get(heal["field"], "") or "").strip()
        m = HEAL_SOURCE_RE.match(raw)
        if not m:
            continue
        rel, old_line = m.group("path"), int(m.group("line"))
        tokens = [t.strip() for t in str(idx.get("出处应含") or "").split(";") if t.strip()]
        if not tokens:
            continue
        dpath = (root / rel).resolve()
        if not dpath.exists():
            continue
        body = read_text(dpath).splitlines()
        hits = [i + 1 for i, ln in enumerate(body) if all(t in ln for t in tokens)]
        if not hits:
            continue
        if len(hits) == 1:
            new_line = hits[0]
        else:
            dist = sorted((abs(h - old_line), h) for h in hits)
            if len(dist) > 1 and dist[0][0] == dist[1][0]:
                continue          # 并列最近 = 猜，宁留红
            new_line = dist[0][1]
        if new_line == old_line:
            continue
        yline = idx.get(heal["line_key"])
        if not isinstance(yline, int) or not (1 <= yline <= len(lines)):
            continue
        old_text = lines[yline - 1]
        new_text = old_text.replace(f"{rel}:{old_line}", f"{rel}:{new_line}", 1)
        if new_text != old_text:
            lines[yline - 1] = new_text
            fixed += 1
    if fixed:
        yaml_path.write_text("".join(lines), encoding="utf-8")
    return fixed


# --------------------------------------------------------------------------- #
#  主流程
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    try:  # 控制台可能是 GBK（Windows），别让中文打印抛 UnicodeEncodeError
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass

    ap = argparse.ArgumentParser(description="校验 docs/20_资料索引库/数字速查.yaml")
    ap.add_argument("--yaml", default=None, help=f"真源文件（默认 {DEFAULT_YAML}）")
    ap.add_argument("--repo", default=None, help="仓库根（默认为本脚本上两级）")
    ap.add_argument("-v", "--verbose", action="store_true", help="打印表达式与锚点")
    ap.add_argument("--strict", action="store_true", help="连存量漂移一起判红（收口用）")
    ap.add_argument("--fix-sources", action="store_true",
                    help="自愈：把能唯一定位的 代码锚点/出处 行号改回真代码所在行")
    ap.add_argument("--write-baseline", action="store_true",
                    help="把现存漂移收进 numbers_baseline.json（只减不增）")
    ap.add_argument("--allow-grow", action="store_true",
                    help="配合 --write-baseline：本次确实要新增条目时显式允许（须写明理由）")
    args = ap.parse_args(argv)

    root = Path(args.repo).resolve() if args.repo else Path(__file__).resolve().parents[2]
    yaml_path = Path(args.yaml) if args.yaml else root / DEFAULT_YAML
    if not yaml_path.is_absolute():
        yaml_path = root / yaml_path
    if not yaml_path.exists():
        print(f"[FATAL] 真源文件不存在：{yaml_path}")
        return 2

    try:
        items = parse_yaml_flat(read_text(yaml_path))
    except AnchorError as exc:
        print(f"[FATAL] YAML 解析失败：{exc}")
        return 2

    print("=" * 78)
    print("数字速查真源校验")
    print(f"  真源: {yaml_path}")
    print(f"  仓库: {root}")
    print("=" * 78)

    ctx = Ctx(root)
    env = make_env(ctx)

    fails: list[str] = []
    # 漂移警告： (类别, id, 人读说明, 自愈信息)
    #   类别 = anchor_drift / source_drift / doc_gone / doc_conflict / doc_unsourced
    #   doc_conflict 直接判红（除非 yaml 里写了 异文说明）；其余按基线判红
    warns: list[tuple[str, str, str, dict[str, Any] | None]] = []
    manual: list[tuple[str, str, str]] = []
    passed = 0
    auto_total = 0
    anchor_fail_entries = 0

    for idx, item in enumerate(items, 1):
        missing = [f for f in REQUIRED_FIELDS if f not in item]
        eid = str(item.get("id") or f"<第{idx}条>")
        if missing:
            fails.append(f"{eid}: 缺必填字段 {missing}")
            print(f"[{idx:>2}] FAIL  {eid:<28} 缺字段 {missing}")
            continue

        status = str(item["状态"])
        if status not in STATUS_VALUES:
            fails.append(f"{eid}: 状态 {status!r} 不在 {STATUS_VALUES}")
        value = item["值"]
        unit = str(item["单位"])
        check = str(item["校验"]).strip()

        n_fails_before = len(fails)

        # ---- 锚点/出处检查（与重算独立） ----
        lvl, why = check_code_anchor(ctx, item["代码锚点"])
        if lvl == "fail":
            fails.append(f"{eid}: 代码锚点失效 —— {why}")
        elif lvl == "drift":
            warns.append(("anchor_drift", eid, f"代码锚点行号漂移 —— {why}",
                          {"field": "代码锚点", "line_key": "_代码锚点_line", "why": why}))

        dlvl, dwhy = check_doc_source(ctx, item["出处"], str(item.get("出处应含") or ""))
        if dlvl == "fail":
            fails.append(f"{eid}: 出处失效 —— {dwhy}")
        elif dlvl == "drift":
            warns.append(("source_drift", eid, f"出处内容/行号变化 —— {dwhy}",
                          {"field": "出处", "line_key": "_出处_line", "why": dwhy}))

        # ---- 文档异文 ----
        doc_value = item.get("文档值")
        if doc_value is not None:
            loc = parse_loc(item.get("文档值出处") or "")
            token = fmt_num(float(doc_value))
            declared = str(item.get("异文说明") or "").strip()  # 有意并列的口径冲突
            if loc is None:
                warns.append(("doc_unsourced", eid, f"填了 文档值={token} 但没填 文档值出处", None))
            else:
                dp = (root / loc[0]).resolve()
                if not dp.exists():
                    fails.append(f"{eid}: 文档值出处文件不存在 —— {loc[0]}")
                else:
                    dlines = read_text(dp).splitlines()
                    still = 1 <= loc[1] <= len(dlines) and token in dlines[loc[1] - 1]
                    now = fmt_num(float(value)) if value is not None else "待核"
                    if still:
                        note = f"异文仍存 —— 文档 {loc[0]}:{loc[1]} 记 {token}，代码现值 {now}"
                        if declared:
                            note += f"（已声明并列：{declared}）"
                        warns.append(("doc_conflict", eid, note, None))
                    else:
                        warns.append(("doc_gone", eid,
                                      f"异文已消失（可清理文档值）—— {loc[0]}:{loc[1]} 已无 {token}", None))

        # ---- 重算 ----
        if check == "manual":
            manual.append((eid, unit, str(item.get("备注") or "")))
            print(f"[{idx:>2}] MAN   {eid:<28} 需人核（{unit}）")
            if args.verbose:
                print("        " + "备注".ljust(4) + ": " + str(item.get("备注") or ""))
            continue

        auto_total += 1
        if value is None:
            fails.append(f"{eid}: 状态 {status} 但 值 为空，无法重算")
            print(f"[{idx:>2}] FAIL  {eid:<28} 值 为空")
            continue
        if isinstance(value, str):
            fails.append(f"{eid}: 值 {value!r} 不是数字")
            print(f"[{idx:>2}] FAIL  {eid:<28} 值非数字：{value!r}")
            continue
        try:
            got = eval_expr(check, env)
        except AnchorError as exc:
            fails.append(f"{eid}: {exc}")
            print(f"[{idx:>2}] FAIL  {eid:<28} {exc}")
            continue

        want = float(value)
        diff = abs(got - want)
        rel = diff / abs(want) if abs(want) > ABS_TOL else 0.0
        tol = float(item.get("容差") or DEFAULT_TOL)
        ok = diff <= ABS_TOL or rel <= tol
        if ok:
            passed += 1
            hard = len(fails) > n_fails_before
            tag = "FAIL" if hard else "PASS"
            tail = "   ← 重算一致，但锚点/出处失效（见汇总）" if hard else ""
            if hard:
                anchor_fail_entries += 1
            print(f"[{idx:>2}] {tag}  {eid:<28} {fmt_num(want):>12} {unit:<7} = {fmt_num(got)}{tail}")
        else:
            fails.append(
                f"{eid}: 重算 {fmt_num(got)} != 值 {fmt_num(want)}（差 {fmt_num(diff)}，"
                f"相对 {rel:.3%} > 容差 {tol:.1%}）")
            print(f"[{idx:>2}] FAIL  {eid:<28} 值 {fmt_num(want)} {unit} "
                  f"!= 重算 {fmt_num(got)}  (差 {fmt_num(diff)})")
        if args.verbose:
            print("        " + "校验".ljust(4) + ": " + check)

    # ---------------- 汇总 ----------------
    print("=" * 78)
    print(f"条目 {len(items)}：可重算 {auto_total} / manual {len(manual)}")
    print(f"可重算：重算通过 {passed} / 重算不符 {auto_total - passed}")
    if anchor_fail_entries:
        print(f"        另有 {anchor_fail_entries} 条重算一致但锚点/出处失效（见下方 mismatch）")
    if manual:
        print(f"\n需人核 {len(manual)} 条（算不了，别当已核）：")
        for eid, unit, note in manual:
            tail = f" —— {note}" if note else ""
            print(f"  - {eid} ({unit}){tail}")
    # ---------------- 漂移记账 / 自愈 / 判红 ----------------
    base_path = Path(args.repo).resolve() / "docs/_工具" / BASELINE_NAME if args.repo \
        else Path(__file__).resolve().parent / BASELINE_NAME
    base_keys: set[str] = set()
    if base_path.exists():
        try:
            import json as _json
            base_keys = set(_json.loads(read_text(base_path)).get("keys") or [])
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] 基线读不懂，按空处理：{exc}")

    drift_now: dict[str, tuple[str, str, dict[str, Any] | None]] = {}
    conflicts: list[tuple[str, str]] = []
    for kind, eid, why, heal in warns:
        if kind == "doc_conflict":
            if "（已声明并列：" in why:
                continue                          # 作者已在 yaml 里声明并列：不是债，也不入基线
            conflicts.append((eid, why))          # 未声明的异文：直接判红
            continue
        drift_now[f"{kind} :: {eid}"] = (kind, eid, heal)

    fixed = 0
    if args.fix_sources:
        fixed = self_heal(items, drift_now, yaml_path, root, ctx)

    new_drift = sorted(k for k in drift_now if k not in base_keys)
    gone_drift = sorted(base_keys - set(drift_now))

    if args.write_baseline:
        if new_drift and not args.allow_grow:
            print("\n[拒绝写基线] 本次会新增 %d 条漂移条目（基线只许收缩）：" % len(new_drift))
            for k in new_drift[:10]:
                print("   + " + k)
            print("   修文档/yaml，别改基线；确属历史遗留须一次性收编，加 --allow-grow 并写明理由。")
            return 1
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import json as _json
        payload = {
            "generated": __import__("datetime").datetime.now().strftime("%Y-%m-%d %H:%M"),
            "count": len(drift_now),
            "说明": ("数字侧漂移的 (类别 :: id) 集合，语义只减不增。不在本表里的新增漂移直接判红；"
                     "write-baseline 拒绝新增（须 --allow-grow）。异文冲突不受本表豁免，"
                     "要放行只能在 数字速查.yaml 里写 异文说明。"),
            "keys": sorted(drift_now),
        }
        base_path.write_text(_json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
        print("\n[基线已写入] %s  存量漂移 %d 条" % (base_path.name, len(drift_now)))

    if warns:
        print(f"\n文档侧漂移 {len(warns)} 条（分类见下；新增/未声明异文判红，存量进基线）：")
        for kind, eid, why, _heal in warns:
            mark = "RED" if (f"{kind} :: {eid}" in new_drift or kind == "doc_conflict"
                             and "（已声明并列：" not in why) else "base"
            print(f"  {mark} {eid}: {why}")
    if new_drift:
        print(f"\n[新增漂移] {len(new_drift)} 条不在基线里（修 yaml 或跑 --fix-sources）：")
        for k in new_drift[:20]:
            _kind, eid, heal = drift_now[k]
            tail = " | 自愈: " + heal["why"] if heal else ""
            print(f"   + {k}{tail}")
    if conflicts:
        print(f"\n[异文冲突] {len(conflicts)} 条：文档值与代码真源冲突且未声明并列")
        for eid, why in conflicts[:20]:
            print(f"   x {eid}: {why}")
        print("   要么改文档，要么在 数字速查.yaml 里给该条写 异文说明（有意并列才许写）。")
    if gone_drift:
        print(f"\n[可精简] 基线里 {len(gone_drift)} 条漂移已消失，跑 --write-baseline 收缩基线")
    if fixed:
        print(f"\n[自愈] 已就地修正 {fixed} 条 yaml 行号引用（重跑确认）")

    if fails:
        print(f"\n[FAIL] mismatch {len(fails)} 条：")
        for f in fails:
            print(f"  x {f}")
    print("=" * 78)
    bad = bool(fails) or bool(conflicts) or bool(new_drift) or (args.strict and bool(drift_now))
    print(f"退出码 {1 if bad else 0}"
          + ("（有 mismatch）" if fails else "")
          + (f"（新增漂移 {len(new_drift)}）" if new_drift else "")
          + (f"（异文冲突 {len(conflicts)}）" if conflicts else "")
          + ("（strict：存量漂移也判红）" if args.strict and drift_now else ""))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
