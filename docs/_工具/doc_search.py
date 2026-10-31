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
     表格行的贡献另封顶为 min(tf,1)（LIB-03）：表格行天然堆词，不封顶就等于奖励长表。
     路由页额外 -4（LIB-03）：README/00_索引/目录索引 这类页是"去哪找"的路由，
     不承载结论；它们的表格行词密度极高，不降权就必抢所有症状查询的 top-1。
     判据是「文件名 ∈ {README.md, 00_索引.md, 目录索引.md} 或 front matter role: 路由」，
     **不用「路径含'索引'」**——那会误伤 01_症状索引_52条.md（它是权威篇本身）。
  4. 超长行（技术全景的表格行常见 300+ 字符）以首个命中原点取 200 字窗口，两侧加 "…"，
     连续空白折叠为单空格 —— 保证一行一条、可直接读。
  5. 英文整词匹配失败时自动退化为子串匹配：若该词在全语料边界匹配为 0 命中，
     则改用子串（例 "arrival" 能命中 arrival_m）。退化的词在 --json 里标 mode="sub"。
  6. 归档降权：history_cache/、20_资料索引库/历史/ 与 _work/ 在"同词种数同分"时排在
     活动库之后（默认开），避免旧文压掉现行结论；--no-demote 关闭，--live-only 直接排除。
      LIB-03 起 30_任务卡库/（过程区，186 篇）也降到 tier 1：任务卡是"这活干到哪了"的流水，
      不是"这现象由什么造成"的判据，与百科同权会把症状查询带进卡堆里。

  7. 巨册（front matter role: 巨册）命中时，结果附带 sections 字段：命中所属的 ## 章节标题。
     巨册动辄上千行，只给篇名等于"告诉你在那本书里"，还得自己翻。非巨册文档不带该字段。

  回滚开关（LIB-03）：第 3 条的表格行封顶 / 路由页降权 / 30_任务卡库降级 / 巨册章节
  整体挂在 --legacy-rank 下可一次退回旧行为（默认关 = 走新行为）。

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
# 脚本目录、快照、过程稿区不进索引。
# LIB-04（2026-10-31）：_work/ 入库。它与 check_doc_policy 的 skip_dirs 口径对齐——
# 门禁把它当「豁免」不判，检索就不该把它当正常语料参与排序：豁免 ≠ 不存在，
# 但「豁免了却还能抢 top-5」会让 agent 拿过程稿当结论（15 篇里含
# plan_库外清理.md / index_audit.md / rebuild_8_lost.md 等纯草稿）。
SKIP_DIRS = {"_工具", "_archive", ".git", "_work"}                 # 脚本目录、快照、过程稿不进索引

CJK_RE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff]+")
# 英文/数字整词；允许小数（0.3 / 1.41 / v1.2），否则「0.3 m/s」会被切成 0 与 3 两个单字符而丢弃
WORD_RE = re.compile(r"[a-z0-9_]+(?:\.[0-9]+)?")
MAX_TERMS = 16
SNIPPET_WIDTH = 198          # + 至多 2 个省略号 = 200 字符上限

TIER_NAME = {0: "库", 1: "临时", 2: "归档"}

# 路由页在排序第一键上的等价降权：命中词种数减这么多，等于"少命中这么多词"。
# 取 2 而不是 1：路由页（README/00_索引/目录索引）的表格行天然把词种数堆到最高，
# 只减 1 仍能与正文并列，实测减 1 时 top1 仍只有个位数；减 2 才让它稳定退到
# "多读一篇"的位置，且不把它彻底埋掉（用户仍可能想看路由页）。
ROUTE_TERM_PENALTY = 2

# 表格证据占比达到这个值，就按"汇总表/索引表"再降一档（与路由页同量级）。
# 为什么取 0.8：命中词有 8/10 以上只出现在表格行里，说明这篇文档对查询的"回应"
# 几乎全是表格复述，而不是机制叙述；真实库里 01_症状索引_52条.md 是 1.00、
# 目录索引.md 是 1.00，而散文权威篇通常 0.0~0.3，阈值放在 0.8 不会误伤正文。
TABLE_BULK_RATIO = 0.8


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

    LIB-03：30_任务卡库/ 也归 tier 1（过程区）。186 篇任务卡、在办 141 张，
    它们是"这活干到哪了"的流水，不是"这现象由什么造成"的判据；与百科同权
    （tier 0）时，症状查询会被卡面里的 symptom 复述吸走。
    注意：--live-only 的口径随之变成"只留 tier 0"，即同时排除任务卡。
    """
    if rel.startswith("history_cache/") or rel.startswith("20_资料索引库/历史/"):
        return 2
    if rel.startswith("_work/"):
        return 1
    if rel.startswith("30_任务卡库/"):
        return 1
    return 0


def legacy_tier_of(rel):
    """--legacy-rank 用：LIB-03 之前的层级（30_任务卡库/ 仍是 tier 0）。"""
    if rel.startswith("history_cache/") or rel.startswith("20_资料索引库/历史/"):
        return 2
    if rel.startswith("_work/"):
        return 1
    return 0


ROUTE_FILENAMES = {"README.md", "00_索引.md", "目录索引.md"}


def is_route_page(rel, front):
    """路由页判据（LIB-03）：文件名命中 或 front matter 写 role: 路由。

    为什么不用「路径含'索引'」：那会误伤 01_症状索引_52条.md —— 它是权威篇本身，
    压住它等于把答案排到下面。判据必须"窄且准"。
    """
    name = os.path.basename(rel)
    if name in ROUTE_FILENAMES:
        return True
    for raw in front:
        m = re.match(r"\s*role\s*:\s*(.+)", raw, re.I)
        if m and m.group(1).strip().strip('"').strip("'") == "路由":
            return True
    return False


def role_of(front):
    """front matter 的 role: 值（路由 / 巨册 / …）；没有则空串。"""
    for raw in front:
        m = re.match(r"\s*role\s*:\s*(.+)", raw, re.I)
        if m:
            return m.group(1).strip().strip('"').strip("'")
    return ""


def capsule_ratio(table_tf, tf_total, matched):
    """命中词里"证据**只**来自表格行"的比例，取值 0..1。

    为什么需要它（LIB-03 同类小改动⑤）：
    表格行封顶只能压 score，压不住排序第一键（命中词种数）。而"症状索引"
    这类 52 条汇总表既不是路由三件套、也不写 role: 路由，靠表格堆词仍能抢到
    top-1 —— 实测真实库 64.7%。本函数把"这篇文档的证据有多少来自表格"量化出来，
    喂给排序键。

    ★判据为什么是「仅表格」而不是「表格里出现过」（LIB-03R 修正，2026-10-31）：
    初版写成 `table_tf.get(t, 0) > 0` —— 只要该词在表格里出现过就计入。
    这在**变长文档**上会制造假阳性：一个词在表格出现 3 次、在散文出现 1 次，
    仍被算作「表格堆词」。

    实测证据（症-12「机上检测 det=0」，目标篇 无人机/03_感知与视觉侦察.md）：
        检测  tf=4  table=3     ← 散文里有 1 次命中
        det   tf=4  table=2     ← 散文里有 2 次命中
        机上  tf=2  table=2
        上检  tf=1  table=1
    旧判据 4/4=1.00 ⇒ 触发 −2 惩罚 ⇒ 该篇掉到第 7 名 ⇒ M4 判失败（权威篇连 top-5 都进不去）。
    而这篇是真·机制叙述篇，根本不是汇总表 —— 纯属误伤。
    新判据（`tb >= total`）下 检测 3/4、det 2/4 都不满足 ⇒ ratio=0.50 ⇒ 不再被罚 ⇒ 回到第 2 名。

    两个替代方案都实测无效，不要往回改：
      · `tb/total >= 0.9`：症状索引在「激光不亮」类查询上是 5/9=0.56，同样跌破阈值；
      · 加「最高词数兜底」：多一条特例，且对症状索引反而更松。
    根因是结构性的：**TABLE_BULK_RATIO 是定值 0.8，而命中词数随文档规模变化**
    （症状索引一次能命中 9 个词，无人机/03 只有 4 个），定值阈值必然顾此失彼。
    要根治应让阈值随文档规模浮动 —— 那是另一张卡，不在 LIB-03R 范围。

    判据是**每篇自己算**：没有命中的文档返回 0，散文权威篇拿到接近 0 的值，不会被误伤。
    """
    if not matched:
        return 0.0
    table_only = 0
    for t in matched:
        tb = table_tf.get(t, 0)
        total = tf_total.get(t, 0)
        # 只有"该词的命中全部落在表格行"才算表格堆词；散文里只要有命中就不算。
        if tb > 0 and tb >= total:
            table_only += 1
    return float(table_only) / len(matched)


def section_of(lines, idx, body_start):
    """返回第 idx 行所属的最近 ## 章节标题（不含 ## 本身）；找不到则空串。

    巨册（role: 巨册）动辄上千行，只回篇名等于"告诉你在那本书里"，还得自己翻。
    """
    for i in range(idx, max(body_start - 1, -1), -1):
        raw = lines[i]
        if raw.startswith("## ") and not raw.startswith("### "):
            return raw[3:].strip()
    return ""



def search(terms, corpus, sub_mode, bre_map, show_lines, demote, new_rank=True,
           features=None, tiebreak=None):
    """corpus: [(rel, lines)]；返回按排序键排好的结果列表。

    new_rank=False 即 --legacy-rank：退回 LIB-03 之前的排序行为
    （表格行不封顶、路由页不降权、30_任务卡库/ 仍是 tier 0、无 sections 字段）。

    features / tiebreak 仅供 test_doc_search.py 与 LIB-03 的分项测量用：
    默认 None 时按 new_rank 取全开/全关；显式传入可只开其中几项，
    以证明"四处改动各自有效"而不是一把梭。
    """
    if features is None:
        features = {"table": new_rank, "route": new_rank,
                    "tasktier": new_rank, "sections": new_rank}
    if tiebreak is None:
        tiebreak = new_rank
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
        table_tf = {}
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
            is_table = raw.lstrip().startswith("|")
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
                if is_table:
                    table_tf[t] = table_tf.get(t, 0) + c
                occ += c
            if raw.lstrip().startswith("#"):
                head_hits.update(matched)
            line_rows.append((len(matched), occ, idx + 1, raw, matched, low))

        all_matched = set(tf) | title_hits | front_hits
        if not all_matched:
            continue

        # LIB-03 改动③：表格行 tf 封顶。
        # 为什么不能只对"每行 tf"封顶：单词总分本来就有 min(tf,3) 封顶，对 tf 逐行
        # 封顶对最终 score 毫无影响（已实测 0 变化）。真正奖励长表的是"表格行喂了多少分"，
        # 因此把表格行超过 min(tf,1) 的那部分从该词贡献里扣掉：
        #   贡献 = min(tf,3) - max(0, min(table_tf,3) - 1)
        # 语义：同样 3 次命中，出现在正文散文里算满，靠 N 行表格堆出来只算 1。
        def tf_score(t):
            total = min(tf.get(t, 0), 3)
            if not features["table"]:
                return total
            excess = max(0, min(table_tf.get(t, 0), 3) - 1)
            return max(0, total - excess)

        score = 0
        for t in all_matched:
            score += tf_score(t)
            if t in title_hits:
                score += 5
            if t in front_hits:
                score += 3
            if t in head_hits:
                score += 2

        # LIB-03 改动②：路由页降权。判据窄（文件名精确 ∈ 三件套 或 role: 路由），
        # 绝不按"路径含'索引'"——01_症状索引_52条.md 是权威篇本身，不许误伤。
        is_route = is_route_page(rel, front) if features["route"] else False
        if is_route:
            score -= 4

        line_rows.sort(key=lambda r: (-r[0], -r[1], r[2]))
        hits = []
        for mcount, _occ, lineno, raw, matched, low in line_rows[:show_lines]:
            hits.append({
                "line": lineno,
                "terms": sorted(matched),
                "text": make_snippet(raw, low, matched),
            })
        hits.sort(key=lambda h: h["line"])

        rec = {
            "path": "docs/" + rel,
            "rel": rel,
            "title": title,
            "score": score,
            "terms_matched": len(all_matched),
            "tier": (tier_of(rel) if features["tasktier"] else legacy_tier_of(rel)),
            "_matched": sorted(all_matched),
            "_route": is_route,
            "_table_ratio": capsule_ratio(table_tf, tf, all_matched),
            "hits": hits,
        }
        # LIB-03 改动④：巨册附带章节。只在巨册上加字段，其它文档输出不变。
        if features["sections"] and role_of(front) == "巨册":
            secs = []
            for _mc, _occ, lineno, _raw, _m, _low in line_rows[:show_lines]:
                s = section_of(lines, lineno - 1, body_start)
                if s and s not in secs:
                    secs.append(s)
            rec["sections"] = secs
        results.append(rec)

    # 排序（2026-10-31 LIB-03 修正，LIB-03R 再修）：命中词种数原本是**第一键**且不带
    # 任何降权，任何"大而全"的文档靠词种数就锁死第一，score 上的降权（tier/表格/路由）
    # 全都排不上用场——实测：只做 score 降权时 hub 从 56.9% 降到 2.0%，但
    # 01_症状索引_52条.md 立刻接盘 62.7%（它是 52 条症状的汇总表，词种数天然最高）。
    #
    # 因此 tiebreak 开启时把降权折进第一键：
    #   eff_terms = 命中词种数 − 路由页惩罚
    #
    # ★LIB-03R（2026-10-31）撤掉了原来的第二项「− 2×表格证据占比」。
    # 为什么撤（实测三候选，逐项单独跑）：
    #   A 保留二次惩罚 : M1=8(15.7%) M2=25(49.0%) M4=41(80.4%)  症状索引top1=24(47.1%)
    #   B 全回退      : M1=5( 9.8%) M2=32(62.7%) M4=42(82.4%)  症状索引top1=?（见 --legacy-rank）
    #   C 撤二次惩罚  : M1=5( 9.8%) M2=30(58.8%) M4=42(82.4%)  症状索引top1=29(56.9%)
    # 判据来源：AGENTS §1 明确规定「只知道现象 → 先查症状索引」，症状索引坐 top-1
    # **就是正确答案**（它的命中行里带着机制 + 位置:行 + 处置命令）。t7 实测确认：
    # M2 的分数几乎 100% 来自症状索引坐 top-1（26 条命中 26），其它文档坐 top-1 时
    # 只有 4%（1/25）。压症状索引 = 自残 M2，且 M4 也会净降。
    # A 方案虽然把 M1 抬到 15.7%，但 M2 掉到 49.0% 且 M4 只有 41 —— 用可用性换 M1，不接受。
    #
    # ★一句话理由（LIB-03R）：**表格二次惩罚已撤** —— 它压 M2 抬 M1，而 M1/M2 在排序层
    # 结构性互斥（同一份 top-1 只能属于一方）；实测撤后 M4 80.4→82.4、M2 49.0→58.8。
    # 不要把这一项加回来：它换来的 M1 提升（9.8→15.7）不足以抵消 M2 的塌方（58.8→49.0）。
    #
    # 保留的两条（都对 M1 有益且不伤症状索引）：
    #   ① 路由页降权 —— 只压 hub/README/00_索引/目录索引 这类**纯转述型**路由；
    #      is_route_page() 按设计不认症状索引（认了就成宽判据，会把权威篇一起误伤）。
    #   ② 表格行 tf 封顶（tf_score）—— 压的是 score 而非第一键，不影响症状索引置顶。
    # 排序键其余不变：tier → eff_terms → score → path，保持完全确定。
    def sort_key(r):
        eff = r["terms_matched"]
        if tiebreak and r["_route"]:
            eff -= ROUTE_TERM_PENALTY
        head = (r["tier"],) if demote else ()
        return head + (-eff, -r["score"], r["path"])

    results.sort(key=sort_key)
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
                    help="排除归档层（history_cache/、20_资料索引库/历史/）、_work/ 与 30_任务卡库/")
    ap.add_argument("--no-demote", action="store_true", help="归档层级不参与排序（纯路径定序）")
    ap.add_argument("--legacy-rank", action="store_true",
                    help="回退到 LIB-03 之前的排序（表格行不封顶 / 路由页不降权 / "
                         "30_任务卡库 仍与百科同权 / 无 sections 字段）。"
                         "默认走新行为（自适应排序）；拿不准时用本开关整体退回")
    ap.add_argument("--no-timing", action="store_true",
                    help="不输出耗时（仅用于逐字节复现测试；排序本身与时序无关）")
    args = ap.parse_args(argv)

    new_rank = not args.legacy_rank
    tier_fn = tier_of if new_rank else legacy_tier_of

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
        if args.live_only and tier_fn(rel) > 0:
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

    results = search(terms, corpus, sub_mode, bre_map, max(1, args.lines),
                     not args.no_demote, new_rank)
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
