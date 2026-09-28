# -*- coding: utf-8 -*-
"""批量抽取比赛 PDF 文本，并统计每页有效字符数以判断是否需要 OCR/视觉兜底。"""
import sys
import time
from pathlib import Path

import pdfplumber

BASE = Path(r"C:\Users\29212\Desktop\人工智能算法精英大赛-空地协同侦排挑战赛")
WX = Path(r"C:\Users\29212\Documents\xwechat_files\wxid_esyua6tojf0x22_b065\msg\file\2026-09")
OUT = Path(r"c:\Users\29212\CodeBuddy\20260927104539\docs_txt")
OUT.mkdir(parents=True, exist_ok=True)

SOURCES = [
    ("01_比赛规则解读_直播培训", WX / "算法精英赛-渡众空地协同侦排挑战赛-直播培训-比赛规则解读.pdf"),
    ("02_15空地协同侦排挑战赛", BASE / "文件" / "15空地协同侦排挑战赛.pdf"),
    ("03_场地布置说明书V2", BASE / "文件" / "场地布置说明书V2-空地协同侦排挑战赛.pdf"),
    ("04_遥控器控制", BASE / "文件" / "遥控器控制.pdf"),
    ("05_VNC远程桌面使用方法", BASE / "文件" / "VNC远程桌面使用方法.pdf"),
    ("06_SmartMaster硬件操作指南", BASE / "文件" / "SmartMaster硬件操作指南-V1.0.pdf"),
    ("07_SmartMaster裁判系统操作指南", BASE / "车载裁判系统" / "SmartMaster裁判系统操作指南-V1.0.pdf"),
    ("08_AR250快速使用与安全指南", BASE / "1-无人机操作指南" / "SmartScout-AR250_快速使用与安全指南v3(1).pdf"),
    ("09_AR5高级功能用户手册", BASE / "1-无人机操作指南" / "SmartScout-AR5_高级功能用户手册202609.pdf"),
]

report = []

for name, path in SOURCES:
    if not path.exists():
        report.append((name, "MISSING", 0, 0))
        print(f"[缺失] {name} -> {path}")
        continue

    t0 = time.time()
    pages_text = []
    try:
        with pdfplumber.open(str(path)) as pdf:
            n = len(pdf.pages)
            for i, page in enumerate(pdf.pages, 1):
                try:
                    txt = page.extract_text() or ""
                except Exception as e:
                    txt = f"<<页面抽取失败: {e}>>"
                pages_text.append(txt)
    except Exception as e:
        report.append((name, f"ERROR: {e}", 0, 0))
        print(f"[错误] {name}: {e}")
        continue

    joined = "\n\n".join(
        f"===== 第 {i} 页 =====\n{t}" for i, t in enumerate(pages_text, 1)
    )
    total_chars = sum(len(t.strip()) for t in pages_text)
    empty_pages = sum(1 for t in pages_text if len(t.strip()) < 10)

    (OUT / f"{name}.txt").write_text(joined, encoding="utf-8")
    cost = time.time() - t0
    report.append((name, f"页数{n}", total_chars, empty_pages))
    print(f"[完成] {name}: {n}页, {total_chars}字, 低文本页{empty_pages}, {cost:.1f}s")

print("\n================ 汇总 ================")
print(f"{'文档':<34}{'页数':<12}{'总字数':>10}{'低文本页':>10}")
for name, pages, chars, empty in report:
    print(f"{name:<34}{pages:<12}{chars:>10}{empty:>10}")
print(f"\n输出目录: {OUT}")
