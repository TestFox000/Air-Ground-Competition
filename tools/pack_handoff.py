#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打包“agent 交接包”（L0 速览 / L1 接管 / L2 代码）。

用法：
    python tools/pack_handoff.py --level all
    python tools/pack_handoff.py --level L1
    python tools/pack_handoff.py --level L2 --no-redact   # 内网自用，保留原文

产出：docs/agent_handoff/packs/*.zip 与 MANIFEST.md
原则：只装“读得完”的文件；排除 .git / 模型 / 录像 / _tmp_session；文本自动脱敏。
"""
import argparse
import datetime as dt
import hashlib
import math
import os
import re
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTDIR = ROOT / "docs" / "agent_handoff" / "packs"
STAMP = dt.datetime.now().strftime("%Y-%m-%d %H:%M")

L0 = [
    ("docs/agent_handoff/00_一页纸.md", "00_先读我.md"),
]
L1_EXTRA = [
    ("docs/项目总览_AGENT_20261020.md", "docs/项目总览_AGENT_20261020.md"),
    ("AGENTS.md", "AGENTS.md"),
    ("dzrobot_v2/docs/README.md", "dzrobot_v2/docs/README.md"),
    ("dzrobot_v2/docs/项目现状_20261018.md", "dzrobot_v2/docs/项目现状_20261018.md"),
    ("dzrobot_v2/docs/决策台账.md", "dzrobot_v2/docs/决策台账.md"),
    ("dzrobot_v2/docs/TEAM_STATUS.md", "dzrobot_v2/docs/TEAM_STATUS.md"),
]
L2_TREES = [
    "docs",
    "dzrobot_v2/config",
    "dzrobot_v2/tools",
    "dzrobot_v2/docs",
    "dzrobot_v2/src/dzbrain",
    "dzrobot_v2/src/dzactuator",
    "dzrobot_v2/src/dzrknn_dataprocess",
    "dzrobot_v2/src/dzjudgment",
    "dzrobot_v2/src/rknn_pt",
    "dzrobot_v2/src/car_localization",
    "dzrobot_v2/src/naviPointPath",
    "dzrobot_v2/src/rviz",
    "dzrobot_v2/start_all.sh",
    "dzrobot_v2/dzros.env",
    "dzsmartscout/docs",
    "dzsmartscout/tools",
    "dzsmartscout/src/px4_control",
    "dzsmartscout/src/px4_imu_bridge",
    "dzsmartscout/src/px4_odom_convert",
    "dzsmartscout/src/px4_path",
    "dzsmartscout/src/uav_camera_code",
]
EXCLUDE_DIRS = {
    ".git", "build", "install", "log", "logs", "__pycache__", "node_modules",
    ".pdfdeps", "_pdf_text", "fc-audit", "cam_probe", "models", "artifacts",
    "backup", "_backups", "_tmp_session", "_scratch_aim", "_scratch_auto_plan",
    "_scratch_displayfix", "_scratch_scout", "_tmp_full_regression", ".agent-teams",
    "history", "附加资源", "drone_recordings", "air_ground_models", "packs",
    "vendor", "3rdparty", "doc", "Log", "maps", "archive", "_tmp",
}
EXCLUDE_SUFFIX = {
    ".rknn", ".onnx", ".pt", ".pth", ".engine", ".bag", ".db3", ".mcap", ".zip",
    ".7z", ".rar", ".png", ".jpg", ".jpeg", ".bmp", ".webp", ".mp4", ".avi",
    ".pdf", ".bundle", ".so", ".a", ".o", ".bin", ".hex", ".npy", ".npz",
    ".pkl", ".log", ".csv", ".ini", ".pem", ".key", ".crt", ".pyc", ".pyo",
    ".svg", ".rst", ".lock",
}
MAX_BYTES = 2 * 1024 * 1024
TEXT_SUFFIX = {".md", ".txt", ".py", ".sh", ".ps1", ".cpp", ".cc", ".hpp", ".h",
               ".yaml", ".yml", ".json", ".xml", ".cfg", ".ini", ".toml", ".env"}
CODE_SUFFIX = {".py", ".sh", ".ps1", ".cpp", ".cc", ".hpp", ".h", ".yaml", ".yml",
               ".json", ".xml", ".cfg", ".ini", ".toml", ".env"}

REDACT_RULES = [
    (re.compile(r"(sshpass\s+-p\s+)(['\"]?)([^'\"\s]{1,128})(\2)"), r"\1\2***\4"),
    (re.compile(r"(echo\s+)(['\"])([^'\"]{1,128})(\2)(\s*\|\s*sudo\s+-S)"), r"\1\2***\4\5"),
    (re.compile(r"((?:password|passwd|pwd|PASS|PASSWORD|PASSWD)\s*[:=]\s*)(['\"])([^'\"]{1,128})(\2)", re.I),
     r"\1\2***\4"),
    (re.compile(r"((?:ssh|scp|rsync|sshpass)[^\n]{0,120}?[\w.-]+:)([^@\s'\"]{1,128})(@)"), r"\1***\3"),
]
TEXT_SECRET_HINT = re.compile(r"(sshpass|sudo\s+-S|password|passwd|PASS=|pwd)", re.I)


def is_excluded(rel: Path) -> bool:
    parts = set(rel.parts)
    if parts & EXCLUDE_DIRS:
        return True
    if any(p.startswith("_scratch_") for p in rel.parts):
        return True
    if rel.suffix.lower() in EXCLUDE_SUFFIX:
        return True
    return False


def sanitize(text: str):
    n = 0
    for pat, repl in REDACT_RULES:
        def _sub(m):
            nonlocal n
            n += 1
            return m.expand(repl)
        text = pat.sub(_sub, text)
    return text, n


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()[:16]


def estimate_tokens(chars: int, suffix: str) -> int:
    return math.ceil(chars / (3.2 if suffix.lower() in CODE_SUFFIX else 1.7))


def collect_level(level: str):
    items = []
    if level in ("L0", "L1", "L2"):
        for src, arc in L0:
            items.append((ROOT / src, arc))
    if level in ("L1", "L2"):
        for src, arc in L1_EXTRA:
            items.append((ROOT / src, arc))
    if level == "L2":
        for tree in L2_TREES:
            base = ROOT / tree
            if not base.exists():
                continue
            if base.is_file():
                items.append((base, base.relative_to(ROOT).as_posix()))
                continue
            for dirpath, dirnames, filenames in os.walk(base):
                dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS and not d.startswith("_scratch_")]
                for fn in filenames:
                    p = Path(dirpath) / fn
                    rel = p.relative_to(ROOT)
                    if is_excluded(rel):
                        continue
                    if p.stat().st_size > MAX_BYTES:
                        continue
                    items.append((p, rel.as_posix()))
    seen, uniq = set(), []
    for p, a in items:
        if a not in seen and p.exists():
            seen.add(a)
            uniq.append((p, a))
    return uniq


def build(level: str, redact: bool):
    items = collect_level(level)
    records, total_raw, total_chars, total_tokens, total_red = [], 0, 0, 0, 0
    for p, arc in items:
        raw = p.read_bytes()
        total_raw += len(raw)
        text, nred, chars, toks = None, 0, 0, 0
        if p.suffix.lower() in TEXT_SUFFIX:
            try:
                if b"\x00" not in raw[:8192]:
                    t = raw.decode("utf-8", errors="replace")
                    chars = len(t)
                    toks = estimate_tokens(chars, p.suffix)
                    if redact and TEXT_SECRET_HINT.search(t):
                        text, nred = sanitize(t)
            except Exception:
                pass
        data = text.encode("utf-8") if text is not None else raw
        total_chars += chars
        total_tokens += toks
        total_red += nred
        records.append({"arc": arc, "bytes": len(data), "chars": chars,
                        "tokens": toks, "sha": sha(data), "red": nred, "data": data})
    return records, total_raw, total_chars, total_tokens, total_red


def manifest(level, records, total_raw, total_chars, total_tokens, total_red):
    desc = {
        "L0": "30 秒速览：只含一页纸，可直接贴进对话。",
        "L1": "5 分钟接管：一页纸 + 总览 + 规则 + 现状 + 台账 + 状态板。推荐默认发包。",
        "L2": "深挖代码（内部）：L1 + 车/机自研源码与现行文档（已排除 .git/第三方/模型/录像/_tmp_session）。注意：这是文件系统包，按目录检索，不要整包入上下文；仅自动脱敏明显凭据模式，外发前必须人工复核。",
    }[level]
    lines = [
        f"# {level} 交接包 · MANIFEST",
        "",
        f"- 生成时间：{STAMP}",
        f"- 用途：{desc}",
        f"- 文件数：**{len(records)}**",
        f"- 解压后体积：**{total_raw/1024/1024:.2f} MB**；其中文本 {total_chars:,} 字符",
        f"- 估算上下文：**≈ {total_tokens:,} tokens**（中文按 1.7 字符/token；代码按 3.2）",
        f"- 已脱敏处：**{total_red}** 处（sshpass / sudo -S / password 赋值 / user:pass@）",
        "",
        "## 文件清单",
        "",
        "| # | 路径 | KB | 字符 | ≈tokens | 脱敏 | sha256 |",
        "|---:|---|---:|---:|---:|---:|---|",
    ]
    for i, r in enumerate(records, 1):
        lines.append(f"| {i} | `{r['arc']}` | {r['bytes']/1024:.1f} | {r['chars']:,} | {r['tokens']:,} | {r['red']} | `{r['sha']}` |")
    lines += [
        "",
        "## 阅读顺序",
        "1. `00_先读我.md`（一页纸）",
        "2. `docs/项目总览_AGENT_20261020.md`（全量）",
        "3. `AGENTS.md` → `dzrobot_v2/docs/项目现状_20261018.md` → `决策台账.md` → `TEAM_STATUS.md`",
        "",
        "## 未包含（及原因）",
        "- `.git/`：根仓约 **400 MB**，且历史含 15 个明文凭据脚本——**任何情况下不要外发**。",
        "- `models/`、`*.rknn`：二进制权重（约 51 MB），交接只需模型名与指标。",
        "- `drone_recordings/`、`src/history/`：录像与 2.1 GB 历史附件，读不完也用不上。",
        "- `_tmp_session/`：约 600 MB 过程草稿，含临时脚本与口令，默认排除。",
        "- `artifacts/`、`backup/`：可再生产物与备份。",
        "",
        "> 需要可执行的原样代码（含真实口令）时，请走**同机路径**或内网拷贝，不要用本包。",
        "",
    ]
    if level == "L2":
        lines += [
            "> ⚠️ **L2 含项目脚本与内网地址，可能残留账号信息**：默认只用于同机/内网；",
            "> 对外部 agent 优先发 L1，确需代码时只挑具体文件，不要整包外发。",
            "",
        ]
    return "\n".join(lines)


def write_zip(level, records, man):
    OUTDIR.mkdir(parents=True, exist_ok=True)
    name = {"L0": "L0_速览包", "L1": "L1_接管包", "L2": "L2_代码包"}[level]
    zpath = OUTDIR / f"{name}.zip"
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for r in records:
            z.writestr(r["arc"], r["data"])
        z.writestr("MANIFEST.md", man)
    return zpath


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", default="all", choices=["all", "L0", "L1", "L2"])
    ap.add_argument("--no-redact", action="store_true", help="不脱敏（仅限内网自用）")
    args = ap.parse_args()

    levels = ["L0", "L1", "L2"] if args.level == "all" else [args.level]
    for lv in levels:
        records, raw, chars, toks, red = build(lv, not args.no_redact)
        man = manifest(lv, records, raw, chars, toks, red)
        zp = write_zip(lv, records, man)
        (OUTDIR / f"{lv}_MANIFEST.md").write_text(man, encoding="utf-8")
        print(f"{lv}: {len(records):3d} files | zip {zp.stat().st_size/1024/1024:6.2f} MB | "
              f"text {chars:7,d} chars | ~{toks:6,d} tokens | redacted {red}")
    print(f"\n输出目录: {OUTDIR}")


if __name__ == "__main__":
    main()
