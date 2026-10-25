# -*- coding: utf-8 -*-
"""资料库本地检索器 —— 按相关度返回 top-N 命中文档，解决「知道现象、不知道在哪篇」。

用法（在任意目录执行均可，路径按脚本自身位置解析，不依赖 cwd）：
    python docs/_工具/doc_search.py "激光不亮"
    python docs/_工具/doc_search.py "pose_stale" -k 10
    python docs/_工具/doc_search.py "arrival_m" --in 20_资料索引库 --json
    python docs/_工具/doc_search.py "桥洞" --live-only
    python docs/_工具/doc_search.py "跑飞" -k 0          # 0 = 全部命中

为什么有它（2026-10-24 立项）：
    docs/00_项目总览 下 6 篇技术全景合计 12432 行，占该册 93%。agent 知道现象（"激光不亮"）
    但不知道在哪篇，只能全文 grep；grep 只给行、不给排序，40 条命中仍不知道先读哪篇。
    本工具给「篇级排序 + 篇内命中行 + 命中片段」，一次查询即可定位。

设计（零第三方依赖 / 不联网 / 不落盘缓存 / 一次运行内建内存索引 / 输出可复现）：
  1. 分词：中文按 2-gram 滑窗（"激光不亮" -> 激光 光不 不亮），单个汉字保留 1-gram；
     英文/数字/下划线按 [a-z0-9_]+ 整词，小数整体保留（"0.3"/"1.41"/"v1.2"）；
     长度 <2 的 ASCII 词（m / s / a 这类单位与冠词）丢弃，避免噪声。去重保序，最多 16 词。
  2. 排序键 = (命中词种数 降序, 加权分 降序, 归档层级 升序, 相对路径 升序)。
     末两键保证同分时结果完全确定 —— 同一语料、同一查询，永远同一输出，可直接进测试。
  3. 加权分：每词正文词频贡献 min(tf,3)；词出现在标题 +5；出现在 front matter +3；
     出现在（非标题的）小标题 +2。即"标题与 front matter 命中加权"，但词种数仍优先。
  4. 超长行（技术全景的表格行常见 300+ 字符）以首个命中原点取 200 字窗口，两侧加 "…"，
     连续空白折叠为单空格 —— 保证一行一条、可直接读。
  5. 英文整词匹配失败时自动退化为子串匹配：若该词在全语料边界匹配为 0 命中，
     则改用子串（例 "arrival" 能命中 arrival_m）。退化的词在 --json 里标 mode="sub"。
  6. 归档降权：history_cache/、20_资料索引库/历史/ 与 _work/ 在"同词种数同分"时排在
     活动库之后（默认开），避免旧文压掉现行结论；--no-demote 关闭，--live-only 直接排除。

退出码：0 有命中；1 无命中（grep 习惯，便于链式判断）；2 用法错误。
"""
import argparse
import json
import os
import re
import sys
import time

DOCS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # .../v1/docs
ROOT = os.path.dirname(DOCS)                                        # .../v1
SKIP_DIRS = {"_工具", "_archive", ".git"}                          # 脚本目录与快照不进索引

CJK_RE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff]+")
# 英文/数字整词；允许小数（0.3 / 1.41 / v1.2），否则「0.3 m/s」会被切成 0 与 3 两个单字符而丢弃
WORD_RE = re.compile(r"[a-z0-9_]+(?:\.[0-9]+)?")
MAX_TERMS = 16
SNIPPET_WIDTH = 198          # + 至多 2 个省略号 = 200 字符上限

TIER_NAME = {0: "库", 1: "临时", 2: "归档"}


def setup_streams():
    """Windows 控制台默认 cp936，直接 print 中文会炸/乱码；统一改 UTF-8 并容错。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass


def tokenize(query):
    """中文 2-gram + 英文整词；返回 [(term, kind, pos)]，kind: 'w' 英文词 / 'c' 中文。"""
    low = query.lower()
    cand = []
    for m in WORD_RE.finditer(low):
        w = m.group(0)
        if len(w) >= 2:
            cand.append((m.start(), w, "w"))
    for m in CJK_RE.finditer(low):
        run = m.group(0)
        if len(run) == 1:
            cand.append((m.start(), run, "c"))
        else:
            for i in range(len(run) - 1):
                cand.append((m.start() + i, run[i:i + 2], "c"))
    cand.sort()
    out, seen = [], set()
    for _, t, kind in cand:
        if t in seen:
            continue
        seen.add(t)
        out.append((t, kind))
        if len(out) >= MAX_TERMS:
            break
    if not out:                                   # 单字符/纯符号查询：整体当一个词
        t = query.strip().lower()
        if t:
            out.append((t, "w" if WORD_RE.fullmatch(t) else "c"))
    return out


def boundary_re(term):
    return re.compile(r"(?<![a-z0-9_])" + re.escape(term) + r"(?![a-z0-9_])")


def collect_files(in_prefixes):
    """枚举 docs/**/*.md；--in 前缀过滤（支持 'docs/20_...' / './20_...' / '20_...'）。"""
    prefixes = []
    for p in in_prefixes or []:
        p = p.replace("\\", "/").strip().strip("/")
        if p.startswith("./"):
            p = p[2:]
        if p.startswith("docs/"):
            p = p[len("docs/"):]
        prefixes.append(p)
    all_files = []
    for dirpath, dirnames, filenames in os.walk(DOCS):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        for fn in sorted(filenames):
            if not fn.lower().endswith(".md"):
                continue
            full = os.path.join(dirpath, fn)
            rel = os.path.relpath(full, DOCS).replace("\\", "/")
            all_files.append((full, rel))
    all_files.sort(key=lambda x: x[1])
    if not prefixes:
        return all_files

    def strict(rel):
        return any(rel == p or rel.startswith(p.rstrip("/") + "/") for p in prefixes)

    def loose(rel):
        """--in 只写子目录名（如 视觉与云台）时，按完整路径段匹配。"""
        seg = "/" + rel
        return any("/" + p.rstrip("/") + "/" in seg for p in prefixes)

    hit = [f for f in all_files if strict(f[1])]
    if not hit:
        hit = [f for f in all_files if loose(f[1])]
    return hit


def read_lines(full_path):
    try:
        with open(full_path, "r", encoding="utf-8-sig", errors="replace") as fh:
            return fh.read().splitlines()
    except OSError as exc:
        print("[warn] 读不了 %s: %s" % (full_path, exc), file=sys.stderr)
        return []


def split_front_matter(lines):
    """返回 (front_matter_lines, body_start_index)。"""
    if not lines or lines[0].strip() != "---":
        return [], 0
    for idx in range(1, min(len(lines), 40)):
        if lines[idx].strip() == "---":
            return lines[1:idx], idx + 1
    return [], 0


def doc_title(lines, rel, front):
    """标题来源优先级：front matter 的 title: -> 首个 H1 -> 文件名。"""
    for raw in front:
        m = re.match(r"\s*title\s*:\s*(.+)", raw, re.I)
        if m:
            return m.group(1).strip().strip('"').strip("'")
    for raw in lines:
        if raw.startswith("# "):
            return raw[2:].strip()
    stem = os.path.basename(rel)
    return stem[:-3] if stem.lower().endswith(".md") else stem


def make_snippet(raw, low, matched):
    """行内片段：去缩进，超长以首个命中原点取 198 字窗口，空白折叠。"""
    s = raw.rstrip()
    stripped = s.strip()
    lead = len(s) - len(s.lstrip())
    pos = -1
    for t in matched:
        p = low.find(t)
        if p >= 0 and (pos < 0 or p < pos):
            pos = p
    pos = max(0, pos - lead)
    if len(stripped) <= SNIPPET_WIDTH:
        text = stripped
    else:
        half = SNIPPET_WIDTH // 2
        if pos <= half:
            start = 0
        else:
            start = pos - half
            if start + SNIPPET_WIDTH > len(stripped):
                start = max(0, len(stripped) - SNIPPET_WIDTH)
        text = stripped[start:start + SNIPPET_WIDTH]
        if start > 0:
            text = "\u2026" + text
        if start + SNIPPET_WIDTH < len(stripped):
            text = text + "\u2026"
    return re.sub(r"\s{2,}", " ", text).strip()


def tier_of(rel):
    """归档层级：2 = 归档（排序末位 + 被 --live-only 排除），1 = 临时，0 = 现行。

    20_资料索引库/历史/ 是沿革库（该分册 9 篇全为 status: 历史，按 §8.7
    "只作沿革、不作判据"），不归入归档层的话 10-03 旧核手册会压掉现行结论。
    """
    if rel.startswith("history_cache/") or rel.startswith("20_资料索引库/历史/"):
        return 2
    if rel.startswith("_work/"):
        return 1
    return 0


def search(terms, corpus, sub_mode, bre_map, show_lines, demote):
    """corpus: [(rel, lines)]；返回按排序键排好的结果列表。"""
    term_list = [(t, kind) for t, kind in terms]
    results = []

    for rel, lines in corpus:
        front, body_start = split_front_matter(lines)
        front_low = "\n".join(front).lower()
        title = doc_title(lines, rel, front)
        stem = os.path.basename(rel)[:-3]
        title_low = (stem + " " + title).lower()

        def match_in_low(low):
            found = []
            for t, kind in term_list:
                if t not in low:
                    continue
                if kind == "w" and not sub_mode.get(t):
                    if not bre_map[t].search(low):
                        continue
                found.append(t)
            return found

        title_hits = set(match_in_low(title_low))
        front_hits = set(match_in_low(front_low))

        tf = {}
        head_hits = set()
        line_rows = []
        for idx in range(body_start, len(lines)):
            raw = lines[idx]
            if not raw.strip():
                continue
            low = raw.lower()
            if not any(t in low for t, _ in term_list):      # 快速剪枝
                continue
            matched = match_in_low(low)
            if not matched:
                continue
            occ = 0
            for t in matched:
                kind = "w"
                for tt, kk in term_list:
                    if tt == t:
                        kind = kk
                        break
                if kind == "w" and not sub_mode.get(t):
                    c = len(bre_map[t].findall(low))
                else:
                    c = low.count(t)
                c = max(c, 1)
                tf[t] = tf.get(t, 0) + c
                occ += c
            if raw.lstrip().startswith("#"):
                head_hits.update(matched)
            line_rows.append((len(matched), occ, idx + 1, raw, matched, low))

        all_matched = set(tf) | title_hits | front_hits
        if not all_matched:
            continue

        score = 0
        for t in all_matched:
            score += min(tf.get(t, 0), 3)
            if t in title_hits:
                score += 5
            if t in front_hits:
                score += 3
            if t in head_hits:
                score += 2

        line_rows.sort(key=lambda r: (-r[0], -r[1], r[2]))
        hits = []
        for mcount, _occ, lineno, raw, matched, low in line_rows[:show_lines]:
            hits.append({
                "line": lineno,
                "terms": sorted(matched),
                "text": make_snippet(raw, low, matched),
            })
        hits.sort(key=lambda h: h["line"])

        results.append({
            "path": "docs/" + rel,
            "rel": rel,
            "title": title,
            "score": score,
            "terms_matched": len(all_matched),
            "tier": tier_of(rel),
            "_matched": sorted(all_matched),
            "hits": hits,
        })

    results.sort(key=lambda r: (
        -r["terms_matched"],
        -r["score"],
        r["tier"] if demote else 0,
        r["path"],
    ))
    return results


def human(n):
    return "{:,}".format(n)


def main(argv=None):
    setup_streams()
    ap = argparse.ArgumentParser(
        prog="doc_search.py",
        description="docs/ 本地检索：中文 2-gram + 英文整词，按命中词种数与加权分排序。",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("query", help="查询串，例：\"激光不亮\" / \"pose_stale\"")
    ap.add_argument("-k", "--top", type=int, default=5, help="返回篇数，默认 5；0 = 全部")
    ap.add_argument("--json", action="store_true", help="结构化输出（机器读）")
    ap.add_argument("--in", dest="in_prefix", action="append", default=None,
                    help="限定 docs 下子路径，可重复；写路径前缀（20_资料索引库）或子目录名"
                         "（视觉与云台）都行，例：--in 视觉与云台 --in 无人机")
    ap.add_argument("--lines", type=int, default=3, help="每篇展示命中行数，默认 3")
    ap.add_argument("--live-only", action="store_true",
                    help="排除归档层（history_cache/、20_资料索引库/历史/）与 _work/")
    ap.add_argument("--no-demote", action="store_true", help="归档层级不参与排序（纯路径定序）")
    ap.add_argument("--no-timing", action="store_true",
                    help="不输出耗时（仅用于逐字节复现测试；排序本身与时序无关）")
    args = ap.parse_args(argv)

    query = (args.query or "").strip()
    if not query:
        print("用法错误：查询串为空。", file=sys.stderr)
        return 2
    terms = tokenize(query)
    if not terms:
        print("用法错误：查询串没有可用词。", file=sys.stderr)
        return 2

    t0 = time.perf_counter()
    files = collect_files(args.in_prefix)
    corpus = []
    total_lines = 0
    total_bytes = 0
    for full, rel in files:
        if args.live_only and tier_of(rel) > 0:
            continue
        lines = read_lines(full)
        try:
            total_bytes += os.path.getsize(full)
        except OSError:
            pass
        total_lines += len(lines)
        corpus.append((rel, lines))

    # 英文整词边界在全语料一次判定；0 命中则退化为子串（"arrival" -> arrival_m）
    corpus_low = "\n".join("\n".join(ls) for _, ls in corpus).lower()
    bre_map = {}
    sub_mode = {}
    for t, kind in terms:
        if kind != "w":
            continue
        rx = boundary_re(t)
        bre_map[t] = rx
        if not rx.search(corpus_low):
            sub_mode[t] = True

    results = search(terms, corpus, sub_mode, bre_map, max(1, args.lines), not args.no_demote)
    elapsed = time.perf_counter() - t0

    ks = results if args.top == 0 else results[:max(0, args.top)]
    term_desc = [{"term": t, "kind": k, "mode": "sub" if sub_mode.get(t) else "exact"}
                 for t, k in terms]

    if args.json:
        payload = {
            "query": query,
            "terms": term_desc,
            "elapsed_ms": None if args.no_timing else round(elapsed * 1000, 1),
            "corpus": {"files": len(corpus), "lines": total_lines, "bytes": total_bytes},
            "total_documents": len(results),
            "returned": len(ks),
            "results": [{k: v for k, v in r.items() if not k.startswith("_")} for r in ks],
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if results else 1

    print("查询: %s" % query)
    print("词: %s   (%d 词%s)" % (
        " ".join(("%s" % t if not sub_mode.get(t) else t + "*") for t, _ in terms),
        len(terms),
        "，* = 整词 0 命中已退化为子串" if sub_mode else ""))
    print("语料: %d 篇 / %s 行 / %.1f KB    %s归档降权: %s" % (
        len(corpus), human(total_lines), total_bytes / 1024.0,
        "" if args.no_timing else "耗时 %.3f s    " % elapsed,
        "关" if args.no_demote else "开"))
    print("")

    if not results:
        print("无命中。换更短的词（中文 2 字、英文整词）或去掉 --in/--live-only 再试。")
        return 1

    for i, rec in enumerate(ks, 1):
        print("%d) %s" % (i, rec["path"]))
        print("   命中 %d/%d 词 · 分 %d · %s" % (
            rec["terms_matched"], len(terms), rec["score"], TIER_NAME.get(rec["tier"], "?")))
        for h in rec["hits"]:
            print("   L%-6d | %s" % (h["line"], h["text"]))
        print("")

    if len(ks) < len(results):
        print("其余 %d 篇命中未显示（-k 调大，-k 0 = 全部，--json 取结构化）。" % (len(results) - len(ks)))
    archived = sum(1 for r in ks if r["tier"] == 2)
    if archived:
        print("注: 显示结果含 %d 篇 history_cache 旧文（只读、不作判据，AGENTS §8.1）；"
              "--live-only 只看现行。" % archived)
    print("命中篇数 %d（显示 %d）%s" % (
        len(results), len(ks), "" if args.no_timing else "· 耗时 %.3f s" % elapsed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
