# -*- coding: utf-8 -*-
"""ft_v1 官方爆炸物集漏检根因诊断（2026-10-26，[MODEL-DIAG-20261026]）。

上一轮图级召回 108/157=68.8%（conf0.25、cls0 paper）。本脚本回答：
49 张漏检图是怎么漏的——
  A. 阈值卡掉：有 paper 框但 conf<0.25（能力在，门限过严，可调 per-class conf）
  B. 类别被抢：无 paper 框（<0.10），但其它类有高分框（域偏导致类别漂移）
  C. 真检不出：全类最高分都 <0.10（特征域差，需增广重训）
输出：漏检分型计数、conf 扫描表（两模型）、6 张代表漏检图带框渲染
（_h2h_20261026/diag/*.jpg 供人工复核）。

用法：C:/Users/29212/miniconda/envs/xany/python.exe tools/ml/diag_official_ft_v1.py
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "h2h", os.path.join(HERE, "h2h_models.py"))
h2h = importlib.util.module_from_spec(spec)
sys.argv = ["x"]
spec.loader.exec_module(h2h)

OUT = os.path.join(h2h.REPO, "_h2h_20261026", "diag")
os.makedirs(OUT, exist_ok=True)
LOW_CONF, GATE = 0.05, 0.25
SWEEPS = (0.05, 0.10, 0.15, 0.20, 0.25)


def render(img, dets, path):
    vis = img.copy()
    for pid, s, b in dets[:14]:
        c = (0, 0, 255) if pid == 0 else (255, 160, 0)
        p1, p2 = (int(b[0]), int(b[1])), (int(b[2]), int(b[3]))
        cv2.rectangle(vis, p1, p2, c, 2)
        cv2.putText(vis, f"{h2h.NAMES14[pid] if pid < 14 else pid}:{s:.2f}",
                    (p1[0], max(12, p1[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, c, 1)
    cv2.imwrite(path, vis)


def main():
    files = sorted(f for f in os.listdir(h2h.OFFICIAL_DIR)
                   if f.lower().endswith(".jpg"))
    report = {}
    for name in ("ft_v1", "d13"):
        sess, _ = h2h.load_session(h2h.MODELS[name])
        saved = h2h.CONF
        h2h.CONF = LOW_CONF
        rows = []
        for fn in files:
            img = cv2.imdecode(
                np.fromfile(os.path.join(h2h.OFFICIAL_DIR, fn), np.uint8),
                cv2.IMREAD_COLOR)
            d = h2h.detect(sess, img)
            best_paper = max((s for c, s, _ in d if c == 0), default=0.0)
            others = sorted(
                ((h2h.NAMES14[c] if c < 14 else str(c), round(s, 2))
                 for c, s, _ in d if c != 0), key=lambda x: -x[1])[:3]
            rows.append(dict(fn=fn, paper_conf=round(best_paper, 4),
                             others=others, detected=best_paper >= GATE))
        h2h.CONF = saved
        sweep = {str(t): sum(1 for r in rows if r["paper_conf"] >= t)
                 for t in SWEEPS}
        miss_a = [r for r in rows if not r["detected"]
                  and r["paper_conf"] >= 0.10]
        miss_b = [r for r in rows if not r["detected"]
                  and r["paper_conf"] < 0.10 and r["others"]]
        miss_c = [r for r in rows if not r["detected"]
                  and r["paper_conf"] < 0.10 and not r["others"]]
        report[name] = dict(
            n=len(files), detected=sum(r["detected"] for r in rows),
            miss_A_threshold=len(miss_a), miss_B_class_taken=len(miss_b),
            miss_C_nothing=len(miss_c), sweep=sweep,
            miss_A=[(r["fn"], r["paper_conf"]) for r in miss_a][:10],
            miss_B=[(r["fn"], r["others"][:1]) for r in miss_b][:10],
            miss_C=[r["fn"] for r in miss_c][:10],
        )
        # 代表漏检图渲染（A/B/C 各两张），框画低阈值(0.05)全类
        h2h.CONF = LOW_CONF
        for tag, group in (("A", miss_a[:2]), ("B", miss_b[:2]),
                           ("C", miss_c[:2])):
            for r in group:
                img = cv2.imdecode(np.fromfile(
                    os.path.join(h2h.OFFICIAL_DIR, r["fn"]), np.uint8),
                    cv2.IMREAD_COLOR)
                d = h2h.detect(sess, img)
                render(img, d, os.path.join(OUT, f"{name}_miss{tag}_{r['fn']}"))
        h2h.CONF = saved
    json.dump(report, open(os.path.join(OUT, "official_diag.json"), "w",
                           encoding="utf-8"), ensure_ascii=False, indent=1)
    for name, r in report.items():
        print(f"==== {name} ====")
        print(f"检出 {r['detected']}/{r['n']}  漏检分型："
              f"A阈值卡掉(paper≥0.10) {r['miss_A_threshold']} | "
              f"B类别被抢 {r['miss_B_class_taken']} | "
              f"C真检不出(全类<0.10) {r['miss_C_nothing']}")
        print("conf扫描:", r["sweep"])
        print("A样例:", r["miss_A"][:5])
        print("B样例:", r["miss_B"][:5])
        print("C样例:", r["miss_C"][:5])


if __name__ == "__main__":
    main()
