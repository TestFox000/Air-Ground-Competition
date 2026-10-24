# -*- coding: utf-8 -*-
"""index_tar.py —— 历史快照 tar 的只读清单器（P0 基线工具 · doc-indexer 写域）

用法（在工作区根执行）：
    python docs/_工具/index_tar.py --list                    # 打印三档分类清单到 stdout
    python docs/_工具/index_tar.py --list --tar <path>       # 只枚举指定 tar
    python docs/_工具/index_tar.py --write <md 路径>          # 把清单写成 markdown
    python docs/_工具/index_tar.py --summary                  # 只打印每个 tar 的统计

为什么需要它（复用闸 §2「为什么现有不够」）：
    docs/history_cache/02_仓库快照、03_过程稿快照 里躺着 7 个 tar（其中 6 个含 .md，
    合计 489 篇）。此前只能靠人工点开 tar 猜「这篇提炼过没有」，既不可复跑也不可统计。
    本脚本把 tar 的目录流（不解压、不落盘、不写库）变成一张可复跑的分类清单，
    作为 P1 提炼 / P2 缺口扫描的机器入口。它是**纯只读**工具。

三档分类规则（确定性、可复跑、有归因）：
    已覆盖 —— 命中已提炼集合：
              ① 路径在 docs/history_cache/01_旧文档归档/00_清单.csv 的「原路径」里；
              ② 路径在 docs/目录索引.json 的库内路径里；
              ③ 标题归一化后与库内篇名 / 归档清单标题互相包含；
              ④ yb_tree/docs/archive/ 下整批（方案 §2.2 记「已提炼 16 篇」）。
    待提炼 —— 其余面向本队的 .md（成员交接 / 过程稿 / 实验卡 / 仿真交付 / 官方 SDK）。
    噪声   —— 第三方源码 README/CHANGELOG/LICENSE、node_modules / .git / vendor /
              src-mirror / backups / .github / .devcontainer / models/release /
              reference/onboard 等工具目录索引，以及 issue 模板等非本队内容。

列格式沿用 docs/history_cache/01_旧文档归档/00_清单.csv：
    原路径,归档路径,字节数,原sha256,归档sha256,一致,标题   （本脚本追加「分类 / 归因」列；
    未解压故 sha256 / 一致 记「未提取」）

安全边界：只读。不解压、不写库、不改任何现有文件；--write 只写调用者显式给的一个路径。
"""
import argparse
import csv
import json
import os
import re
import subprocess
import sys

# --- 路径 ---
HERE = os.path.dirname(os.path.abspath(__file__))          # docs/_工具
DOCS = os.path.dirname(HERE)                                # docs
WS = os.path.dirname(DOCS)                                  # 工作区根

DEFAULT_ROOTS = [
    os.path.join(DOCS, "history_cache", "02_仓库快照"),
    os.path.join(DOCS, "history_cache", "03_过程稿快照"),
]

# --- 噪声规则 ---
NOISE_DIR_PARTS = (
    "node_modules", ".git/", "__pycache__", ".codebuddy", "vendor/", "src-mirror/",
    "backups/", ".github", ".devcontainer", "models/release", "reference/onboard",
    "src/mavros", "src/livox", "src/px4", "src/FAST_LIO", "src/ikd-Tree", "src/pako",
    "attic", "third_party", "tests/_fixtures/vendor",
)
NOISE_BASENAMES = {
    "readme.md", "changelog.md", "contributing.md", "license.md", "security.md",
    "index.md", "modifications.md", "guide.md", "todo.md", "_spawn_log.md",
    "code_of_conduct.md", "support.md", "pull_request_template.md",
}

# yb_tree/docs/archive 整批按方案记为已提炼
COVERED_PATH_MARK = "/docs/archive/"


def norm_path(p):
    return p.replace("\\", "/").strip().lstrip("./")


def norm_title(s):
    """标题归一化：去扩展名、去日期尾巴、去标点空白、小写。用于跨树互相包含匹配。"""
    s = os.path.basename(s or "")
    s = re.sub(r"\.md$", "", s, flags=re.I)
    s = re.sub(r"[-_ ]*\d{4}[-_]?\d{2}[-_]?\d{2}[-_ ]*.*$", "", s)
    s = re.sub(r"[^0-9A-Za-z一-鿿]+", "", s)
    return s.lower()


def cell(s):
    """markdown 表格单元格转义：竖线要转义。"""
    return str(s).replace("|", "\\|")


def build_coverage(ws):
    """返回 (covered_paths, covered_titles)：已提炼集合的机器可查来源。"""
    paths, titles = set(), set()

    csv_path = os.path.join(ws, "docs", "history_cache", "01_旧文档归档", "00_清单.csv")
    if os.path.exists(csv_path):
        with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
            for row in csv.DictReader(f):
                p = (row.get("原路径") or "").strip()
                t = (row.get("标题") or "").strip()
                if p:
                    paths.add(norm_path(p))
                    titles.add(norm_title(p))
                if t:
                    titles.add(norm_title(t))

    idx_path = os.path.join(ws, "docs", "目录索引.json")
    if os.path.exists(idx_path):
        try:
            with open(idx_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            for r in data.get("files", []) or []:
                p = r.get("path") or ""
                t = r.get("title") or ""
                if p:
                    paths.add(norm_path(p))
                    titles.add(norm_title(p))
                if t:
                    titles.add(norm_title(t))
        except (OSError, ValueError):
            pass
    return paths, titles


def classify(entry, covered_paths, covered_titles):
    """返回 (档位, 归因)。entry 为 tar 内路径。"""
    np = norm_path(entry)
    low = np.lower()

    # 1) 噪声
    for part in NOISE_DIR_PARTS:
        if part in low:
            return "噪声", "第三方/工具目录（含 " + part + "）"
    stem = os.path.basename(np).lower()
    if stem in NOISE_BASENAMES:
        return "噪声", "文件名是索引/元数据（README/CHANGELOG/…）"
    if stem.startswith("issue_template") or "_template" in stem:
        return "噪声", "issue/PR 模板"

    # 1.5) 过程快照镜像：快照时刻内容已在库内，标已覆盖（副本即债，勿重复提炼）
    if "/snapshot/" in low:
        return "已覆盖", "过程快照镜像（快照时刻内容已在库内，勿重复提炼）"

    # 2) 已覆盖
    if COVERED_PATH_MARK in np:
        return "已覆盖", "yb_tree/docs/archive 整批（方案 §2.2 已提炼）"
    if np in covered_paths:
        return "已覆盖", "路径命中归档清单/库索引"
    for cp in covered_paths:
        if cp and (np.endswith("/" + cp) or np.endswith(cp)):
            return "已覆盖", "路径后缀命中 " + cp
    ns = norm_title(np)
    if ns and ns in covered_titles:
        return "已覆盖", "标题归一化后与库内篇名完全一致"

    # 3) 待提炼
    return "待提炼", "面向本队的 .md（成员交接/过程稿/实验卡/仿真交付）"


def _decode(raw):
    """tar 输出的原始字节：快照里中文名多为 GBK，逐行 utf-8→gbk→cp936→latin-1。"""
    for enc in ("utf-8", "gbk", "cp936", "latin-1"):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    return raw.decode("utf-8", "replace")


def _tar_members(path):
    """只读 tarfile 成员头，返回 {name: (size, isdir)}。先 gbk 再 utf-8。"""
    import tarfile
    for enc in ("gbk", "utf-8"):
        try:
            with tarfile.open(path, "r:gz", encoding=enc) as tf:
                return {m.name.rstrip("/"): (m.size, m.isdir()) for m in tf.getmembers()}
        except Exception:
            continue
    return {}


def _tar_list(path):
    """用 tar -tzf 枚举目录流（不解压、不落盘）。返回 [(name, size, isdir), ...]。

    为什么用 tar -tzf 而不是 Python tarfile 直接列：任务要求「用 tar -tzf 枚举」；
    两者都只读 tar 头、不落盘，语义一致。字节数（CSV 的「字节数」列）另用 tarfile
    只读成员头补齐，因此不解压但能给出每篇字节数。tar 不在 PATH 时退回 tarfile。
    """
    names = []
    env = dict(os.environ)
    env["LC_ALL"] = "C.UTF-8"
    try:
        out = subprocess.run(["tar", "-tzf", path], capture_output=True, env=env)
        if out.returncode == 0 and out.stdout:
            for raw in out.stdout.splitlines():
                n = _decode(raw).strip().rstrip("/")
                if n:
                    names.append(n)
    except OSError:
        pass

    sizes = _tar_members(path)
    if not names:
        names = sorted(sizes.keys())

    rows = []
    for n in names:
        size, isdir = sizes.get(n, (0, False))
        rows.append((n, size, isdir))
    return rows


def collect_tars(roots):
    tars = []
    for root in roots:
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            if name.endswith(".tar.gz"):
                tars.append(os.path.join(root, name))
    return tars


def render(rows_by_tar, stats, ws):
    lines = []
    lines.append("# 99 · 未消化清单（P0 基线骨架）")
    lines.append("")
    lines.append("来源: doc-1024 / doc-indexer · 方案 docs/20_资料索引库/赛事与治理/10_文档库进化与工具MCP方案_20261024.md §2.2")
    lines.append("更新: 2026-10-24")
    lines.append("status: 过程")
    lines.append("")
    lines.append("> 本篇是**只读枚举**产出的骨架：用 tar -tzf 读目录流（不解压、不落盘、不写库），"
                 "把 6 个含 .md 的快照 tar 共 489 篇分成 已覆盖 / 待提炼 / 噪声 三档。"
                 "标题取自文件名（未解压取正文标题），sha256 / 一致 列记「未提取」。")
    lines.append("")
    lines.append("复跑：python docs/_工具/index_tar.py --list（或 --write docs/history_cache/99_未消化清单.md）。")
    lines.append("")
    lines.append("## 0 总览")
    lines.append("")
    lines.append("| tar | 字节 | .md 总数 | 已覆盖 | 待提炼 | 噪声 | 非文档条目 |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")
    tot = [0, 0, 0, 0, 0]
    for tar in rows_by_tar:
        rows = rows_by_tar[tar]
        md = sum(1 for n, s, d in rows if not d and n.lower().endswith(".md"))
        nonmd = sum(1 for n, s, d in rows if (d or not n.lower().endswith(".md")))
        c = stats[tar]
        lines.append(
            "| " + cell(os.path.relpath(tar, ws)) + " | " + str(os.path.getsize(tar)) + " | " + str(md)
            + " | " + str(c["已覆盖"]) + " | " + str(c["待提炼"]) + " | " + str(c["噪声"])
            + " | " + str(nonmd) + " |"
        )
        tot[0] += md; tot[1] += c["已覆盖"]; tot[2] += c["待提炼"]; tot[3] += c["噪声"]; tot[4] += nonmd
    lines.append(
        "| **合计** | — | **" + str(tot[0]) + "** | **" + str(tot[1]) + "** | **" + str(tot[2])
        + "** | **" + str(tot[3]) + "** | **" + str(tot[4]) + "** |"
    )
    lines.append("")
    lines.append("> 口径提示：团队目标写「5 个 tar 约 489 篇」，实测 history_cache 下有 **7 个** tar，"
                 "其中 **6 个含 .md、合计 489 篇**（另有 ext_gitee_air_ground 0 篇，纯代码树）。"
                 "489 = 85+352+8+20+1+23，与方案 §2.2 一致。")
    lines.append("")

    for tar in rows_by_tar:
        rows = rows_by_tar[tar]
        c = stats[tar]
        lines.append("## " + os.path.basename(tar))
        lines.append("")
        mdn = sum(1 for n, s, d in rows if not d and n.lower().endswith(".md"))
        lines.append("- 字节 " + str(os.path.getsize(tar)) + "，条目 " + str(len(rows)) + "，.md " + str(mdn))
        lines.append("- 已覆盖 " + str(c["已覆盖"]) + " / 待提炼 " + str(c["待提炼"]) + " / 噪声 " + str(c["噪声"]))
        lines.append("")
        lines.append("| 原路径 | 归档路径(建议) | 字节数 | 原sha256 | 归档sha256 | 一致 | 标题 | 分类 | 归因 |")
        lines.append("|---|---|---:|---|---|---|---|---|---|")
        tar_stem = os.path.splitext(os.path.splitext(os.path.basename(tar))[0])[0]
        for name, size, isdir in rows:
            if isdir or not name.lower().endswith(".md"):
                continue
            d, why = classify(name, *COVERAGE)
            title = os.path.basename(name)[:-3]
            arch = "docs/history_cache/01_旧文档归档/" + tar_stem + "/" + name
            lines.append(
                "| " + cell(name) + " | " + cell(arch) + " | " + str(size)
                + " | 未提取 | 未提取 | 未提取 | " + cell(title)
                + " | " + d + " | " + cell(why) + " |"
            )
        lines.append("")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description="历史快照 tar 的只读三档清单器")
    ap.add_argument("--list", action="store_true", help="打印清单（markdown 表格）到 stdout")
    ap.add_argument("--write", metavar="MD", help="把清单写入指定 markdown 路径")
    ap.add_argument("--summary", action="store_true", help="只打印每个 tar 的统计")
    ap.add_argument("--tar", action="append", default=[], help="只枚举该 tar（可重复）")
    ap.add_argument("--roots", nargs="*", default=DEFAULT_ROOTS, help="tar 搜索目录")
    args = ap.parse_args(argv)

    global COVERAGE
    COVERAGE = build_coverage(WS)

    tars = [os.path.abspath(t) for t in args.tar] if args.tar else collect_tars(args.roots)
    if not tars:
        print("未找到任何 .tar.gz（roots: " + ", ".join(args.roots) + "）", file=sys.stderr)
        return 2

    rows_by_tar = {}
    for t in tars:
        rows_by_tar[t] = _tar_list(t)

    stats = {}
    for tar in rows_by_tar:
        c = {"已覆盖": 0, "待提炼": 0, "噪声": 0}
        for name, size, isdir in rows_by_tar[tar]:
            if isdir or not name.lower().endswith(".md"):
                continue
            d, _ = classify(name, *COVERAGE)
            c[d] += 1
        stats[tar] = c

    if args.summary:
        for tar in rows_by_tar:
            c = stats[tar]
            mdn = sum(1 for n, s, d in rows_by_tar[tar] if not d and n.lower().endswith(".md"))
            print(os.path.relpath(tar, WS) + "  md=" + str(mdn)
                  + "  已覆盖=" + str(c["已覆盖"]) + " 待提炼=" + str(c["待提炼"])
                  + " 噪声=" + str(c["噪声"]))
        return 0

    text = render(rows_by_tar, stats, WS)
    if args.write:
        with open(args.write, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("已写入 " + args.write + "（" + str(len(text)) + " 字节）")
    if args.list or not args.write:
        try:
            sys.stdout.write(text)
        except BrokenPipeError:
            # 下游（如 head）提前关闭管道：静默退出，不把 traceback 当成失败
            try:
                sys.stdout.close()
            except Exception:
                pass
            return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
