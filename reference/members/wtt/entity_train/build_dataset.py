# -*- coding: utf-8 -*-
"""从项目代码自动抽取"事物"标注样本 —— 纯标准库, 无需安装任何依赖.

三种粒度:
  1. 函数级: 每个函数/方法整段源码(截断到 FUNC_MAX_LINES 行)
  2. 行级  : 命中特征的行 + 上下文各 2 行, 组成 5 行窗口
  3. 窗口级: 不含任何特征的 8 行滑窗 -> 自动作为负样本 "other"
输出 JSONL: {"id","label","source","kind","lineno","text"}

用法:
    python build_dataset.py                          # 抽取本仓库两个主文件
    python build_dataset.py --files mission_planner.py --out demo.jsonl
    python build_dataset.py --stats dataset.jsonl    # 只看统计
"""
import argparse
import ast
import json
import sys
from pathlib import Path

if sys.stdout and hasattr(sys.stdout, "reconfigure"):   # Windows 控制台 UTF-8
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from entity_labels import CLASSES, label_text, hits_of

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FILES = [ROOT / "mission_planner.py", ROOT / "car_client_demo.py"]

FUNC_MAX_LINES = 60     # 函数级样本最长行数(超长函数截断)
CTX_HALF = 2            # 行级样本上下文半宽
WINDOW = 8              # 负样本滑窗行数
CAP_PER_CLASS = 40      # 每个文件每类窗口级样本上限(防正负失衡)


def read_lines(path):
    with open(path, encoding="utf-8") as f:
        return f.read().splitlines()


def iter_functions(lines):
    """用 AST 拿到每个函数/方法的行号区间(语法错误时降级为规则切分)."""
    try:
        tree = ast.parse("\n".join(lines))
    except SyntaxError:
        # 降级: 按顶格 def 行切分
        starts = [i for i, l in enumerate(lines) if l.startswith("def ")]
        for k, s in enumerate(starts):
            e = (starts[k + 1] if k + 1 < len(starts) else len(lines)) - 1
            yield s + 1, e + 1, None
        return
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield node.lineno, node.end_lineno, node.name


def build(paths, cap_per_class, func_max, ctx, window):
    samples, seen = [], set()
    nid = 0
    for p in paths:
        lines = read_lines(p)
        name = Path(p).name
        # ---- 1) 函数级 ----
        for lo, hi, fname in iter_functions(lines):
            body = "\n".join(lines[lo - 1:min(hi, lo - 1 + func_max)])
            lab, _ = label_text(body)
            key = (name, lo, "func")
            if key not in seen:
                seen.add(key)
                samples.append({"id": nid, "label": lab, "source": name,
                                "kind": "func", "lineno": lo,
                                "fname": fname, "text": body})
                nid += 1
        # ---- 2) 行级(命中行 + 上下文窗口) ----
        for i, ln in enumerate(lines):
            _, h = label_text(ln)
            if sum(h.values()) == 0:
                continue
            lo = max(0, i - ctx)
            hi = min(len(lines), i + ctx + 1)
            text = "\n".join(lines[lo:hi])
            lab, _ = label_text(text)
            key = (name, i, "line")
            if key not in seen:
                seen.add(key)
                samples.append({"id": nid, "label": lab, "source": name,
                                "kind": "line", "lineno": i + 1,
                                "fname": None, "text": text})
                nid += 1
        # ---- 3) 窗口级负样本(other) ----
        cnt = 0
        for i in range(0, max(1, len(lines) - window + 1), window):
            text = "\n".join(lines[i:i + window])
            if sum(hits_of(text).values()) > 0:
                continue          # 只留"纯负"窗口
            key = (name, i, "win")
            if key not in seen and cnt < cap_per_class:
                seen.add(key)
                samples.append({"id": nid, "label": "other", "source": name,
                                "kind": "win", "lineno": i + 1,
                                "fname": None, "text": text})
                nid += 1
                cnt += 1
    return samples


def stats(samples):
    from collections import Counter
    c = Counter(s["label"] for s in samples)
    print("\n样本统计:")
    total = len(samples)
    for lab in CLASSES:
        n = c.get(lab, 0)
        bar = "#" * int(40 * n / max(total, 1))
        print(f"  {lab:14s} {n:4d}  {bar}")
    print(f"  {'合计':14s} {total:4d}")
    print(f"  粒度: {dict(Counter(s['kind'] for s in samples))}")
    print(f"  来源: {dict(Counter(s['source'] for s in samples))}")
    return c


def main():
    ap = argparse.ArgumentParser(description="抽取代码事物标注样本")
    ap.add_argument("--files", nargs="*", default=[str(p) for p in DEFAULT_FILES],
                    help="要抽取的代码文件")
    ap.add_argument("--out", default=str(ROOT / "entity_train" / "dataset.jsonl"))
    ap.add_argument("--stats-only", help="只统计已有 JSONL, 不重新抽取")
    ap.add_argument("--cap", type=int, default=CAP_PER_CLASS)
    ap.add_argument("--func-max", type=int, default=FUNC_MAX_LINES)
    ap.add_argument("--show", type=int, default=3, help="每类打印几条样例")
    args = ap.parse_args()

    if args.stats_only:
        samples = [json.loads(l) for l in open(args.stats_only, encoding="utf-8")]
    else:
        samples = build(args.files, args.cap, args.func_max, CTX_HALF, WINDOW)
        with open(args.out, "w", encoding="utf-8") as f:
            for s in samples:
                f.write(json.dumps(s, ensure_ascii=False) + "\n")
        print(f"已写出 {len(samples)} 条 -> {args.out}")
    stats(samples)

    if args.show > 0:
        shown = {}
        for s in samples:
            if s["label"] not in shown:
                shown[s["label"]] = s
            if all(len(v) >= args.show for v in shown.values()):
                break
        for lab in CLASSES:
            s = shown.get(lab)
            if not s:
                continue
            print(f"\n--- 样例 [{lab}] {s['source']}:{s['lineno']} "
                  f"({s['kind']}, {s.get('fname') or '-'}) ---")
            print(s["text"][:300])


if __name__ == "__main__":
    main()
