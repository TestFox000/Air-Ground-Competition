# -*- coding: utf-8 -*-
"""推理: 给一段代码判断它属于哪个"事物"(水桶/爆炸物/敌打击区/...).

依赖: 同 train_classifier.py (torch + transformers/modelscope + sklearn)。

用法:
    python predict.py --model-dir output --text "BUCKETS_N = 6  # 散落障碍物"
    python predict.py --model-dir output --file mission_planner.py --func strike_cell
    python predict.py --model-dir output --file mission_planner.py --line 380
    python predict.py --model-dir output --file mission_planner.py            # 逐函数摘要
"""
import argparse
import ast
import json
import os
import sys
from pathlib import Path

if sys.stdout and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parent
DEFAULT_DIR = ROOT / "output"


def predict(texts, model_dir):
    """加载训练产物并判断(复用 entity_model.EntityClassifier)."""
    from entity_model import EntityClassifier
    cls = EntityClassifier(model_dir)
    return cls.classify_batch(texts, top=3)


def file_segments(path, func=None, line=None):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    if line is not None:
        lo, hi = max(0, line - 3), min(len(lines), line + 2)
        return [(f"L{line}", "\n".join(lines[lo:hi]))]
    if func is None:
        try:
            tree = ast.parse("\n".join(lines))
        except SyntaxError:
            return [("?", "\n".join(lines))]
        segs = []
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                segs.append((node.name,
                             "\n".join(lines[node.lineno - 1:node.end_lineno])))
        return segs
    for node in ast.walk(ast.parse("\n".join(lines))):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                and node.name == func:
            return [(func, "\n".join(lines[node.lineno - 1:node.end_lineno]))]
    raise SystemExit(f"未找到函数 {func}")


def main():
    ap = argparse.ArgumentParser(description="代码事物分类推理")
    ap.add_argument("--model-dir", default=str(DEFAULT_DIR))
    ap.add_argument("--text", help="直接给一段代码")
    ap.add_argument("--file", help="代码文件(配 --func / --line / 或整文件摘要)")
    ap.add_argument("--func", help="只判断某个函数")
    ap.add_argument("--line", type=int, help="只判断某行附近窗口")
    args = ap.parse_args()

    if args.text:
        segs = [("<text>", args.text)]
    elif args.file:
        segs = file_segments(args.file, args.func, args.line)
    else:
        raise SystemExit("请提供 --text 或 --file")

    for name, code in segs:
        top3 = predict([code], args.model_dir)[0]
        print(f"\n[{name}]")
        print("  " + "  ".join(f"{lab}: {p:.3f}" for lab, p in top3))
        print("  " + code.strip().splitlines()[0][:100])


if __name__ == "__main__":
    main()
