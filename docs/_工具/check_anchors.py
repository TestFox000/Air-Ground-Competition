# -*- coding: utf-8 -*-
"""锚点守卫（anchor gate）——防止文档里的「路径:行号」锚点漂移。

为什么需要它：
  check_links.py 只验 markdown 链接的「文件是否存在」；
  gen_index.py 只做索引；正文里成千上万条 "src/x/y.py:123" 式的代码锚点
  （指向哪个文件、第几行）没有任何机器在管。文件被搬走/改名、行号被删减后，
  文档照样"看起来正常"，直到有人照着指过去才发现是空的。

定级：
  RED    路径在**全盘（含归档区）**都找不到 → 引用彻底悬空。
         默认只对「新增红」判失败（退出码 1）；存量红进基线，只统计不判红。
  YELLOW 路径只在非现行区有（git_save/ _tmp_session/ _archive/ docs/history_cache/
         backup/ docs/_work/）= 引用已搬走，应改指现行位置或标〔档〕。
  WARN   文件在、但行号 > 该文件实际行数（尾部被删/重写后行号漂移）。
  AMBIG  直接路径都不成立，且 basename 在现行树里 ≥2 份同名 → 无法判定，**不计红**。

解析顺序（路径感知，避免朴素 basename 误判）：
  1 相对该文档所在目录  2 相对 v1 根  3 相对 docs/  4 相对代码仓根（dzrobot_v2/ 等）
  5 现行树内唯一/后缀唯一同名文件  6 上文最近一次成功解析文件所在目录 + basename
  7 同样的 1~4 在非现行区命中 → 黄   8 非现行区唯一同名 → 黄   9 其余 → 红

词法过滤（防误判，实测必需，见 _tmp_session/lead-1024/B/REPORT.md）：
  - 裸文本只认「带行号」的锚点，且路径部分必须含 / 或带已知扩展名；
    否则会把时间 21:20、比分 1:1、端口 UDP:50000、MAC 32e4:0234 当成锚点。
  - 占位符（尖括号、星号、花括号、美元符、省略号）不判。
  - 反引号里只写 ":245" 的「承前省略」写法，按同文档上文最近的文件补齐。

用法（在 v1 根跑）：
  python docs/_工具/check_anchors.py                    # 体检：只对新增红判失败
  python docs/_工具/check_anchors.py --all              # 连存量红一起列出
  python docs/_工具/check_anchors.py --json             # 机器读
  python docs/_工具/check_anchors.py --ns lead-1023     # 只扫路径含该串的文档
  python docs/_工具/check_anchors.py --write-baseline   # 收口时重建基线（只减不增）
  python docs/_工具/check_anchors.py --include-archive  # 连 history_cache/_work 一起扫
  python docs/_工具/check_anchors.py --root <树根>      # 换工作区根（负向测试/沙盒，不动真文档）
退出码：0 无新增红；1 有新增红。
"""
import json
import os
import re
import sys
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
WS = os.path.dirname(DOCS)
POLICY = os.path.join(HERE, "doc_policy.json")
BASELINE = os.path.join(HERE, "anchor_baseline.json")

BT = chr(96)

# 外部根：工作区之外的兄弟仓（DSH 插件工作区 D:/User/Desktop/ZhiYuan/dsh_plugin 等）。
# 命中的引用算「外部」——文件确实存在，只是不在本仓，不计红、不判失败（--extra-root 可增补）。
DEFAULT_EXTRA_ROOTS = [os.path.normpath(os.path.join(WS, "..", "..", "ZhiYuan"))]

# ---- 扫描面（与 doc_policy.json 对齐：只看「现行文档」，归档/过程稿默认不扫）----
DEFAULT_ROOTS = ["docs/00_项目总览", "docs/10_agent提示词", "docs/20_资料索引库", "docs/30_任务卡库"]
DEFAULT_ROOT_FILES = ["docs/README.md", "docs/目录索引.md"]
DEFAULT_SKIP = {".git", "node_modules", "__pycache__", ".agent-teams", ".venv", ".idea",
                ".vscode", "dist", "build", ".pytest_cache"}

# ---- 非现行区（命中即黄）----
AREAS = [
    ("git_save/", "git_save"),
    ("_tmp_session/", "_tmp_session"),
    ("_archive/", "_archive"),
    ("docs/history_cache/", "history_cache"),
    ("backup/", "backup"),
    ("dzrobot_v2/backup/", "backup"),
    ("dzrobot_v2/_backups/", "backup"),
    ("dzrobot_v2/_scratch_scout/", "_scratch"),
    ("docs/_work/", "_work"),
]

# 数据区：算「存在」（直接命中有效），但不参与 basename 兜底——否则一份录像/仿真快照
# 的 basename 唯一性会把代码锚点误吸进数据目录（实测：dzrobot_v2/artifacts/sim/**/
# map_override/robot.yaml 有 1001 份，不排除则 robot.yaml 永远 ambiguous）。
# vendor/third_party 同理：dzsmartscout/tools/planner/vendor/dzbrain_ws/** 是同一份
# 代码的第三方快照，留着会让 planner.py/config.py/physics.py 永远 ambiguous。
DATA_SEGS = {"artifacts", "drone_recordings", "vendor", "third_party"}
DATA_PREFIXES = ("memory/",)


def is_data(rel):
    """True = 不参与 basename 兜底（数据区/第三方快照），但仍算「文件存在」。"""
    if rel.startswith(DATA_PREFIXES):
        return True
    return any(seg in DATA_SEGS for seg in rel.split("/")[:-1])

EXT = (r"py|md|json|ya?ml|sh|ps1|txt|csv|toml|cfg|ini|xml|html|css|js|mjs|cjs|ts|"
       r"cpp|cc|cxx|hpp|hh|h|c|cs|java|go|rs|rb|php|lua|sql|db|log|bag|png|jpg|jpeg|"
       r"svg|zip|tar|gz|pdf|launch|msg|srv|urdf|xacro|world|rviz")
# 注意：EXT 是 | 拼接的，必须用非捕获组包住，否则 $ 只作用于最后一个分支
# （实测踩过：^...\.py|md|json|...$ 会把 Config.get、logging.rate_limit_s 当成文件名）
PATH_PART = re.compile(r"^(?:.*/)?[A-Za-z0-9_.\-]+\.(?:" + EXT + r")$", re.I)
LINE_TAIL = re.compile(r"[:：]L?(\d+)(?:[-–~](\d+))?$|#L(\d+)(?:[-–~]L?(\d+))?$")


def load(p):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def norm(p):
    return p.replace("\\", "/")


def read_text(p):
    with open(p, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


# ---------------------------------------------------------------- 索引
def area_of(rel):
    for pre, name in AREAS:
        if rel.startswith(pre):
            return name
    return "live"


class Tree(object):
    """现行树 + 非现行区的文件索引（同名反查、行数缓存）。"""

    def __init__(self, ws, skip, extra_roots=()):
        self.ws = ws
        self.ext = {}
        for root in extra_roots:
            if not os.path.isdir(root):
                continue
            for dp, dns, fns in os.walk(root):
                dns[:] = [d for d in dns if d not in skip and not d.startswith(".")]
                for n in fns:
                    self.ext.setdefault(n, []).append(norm(os.path.join(dp, n)))
        self.live = set()
        self.byname_live = {}
        self.byname_any = {}
        self.lines = {}
        for dp, dns, fns in os.walk(ws):
            dns[:] = [d for d in dns if d not in skip and not d.startswith(".")]
            for n in fns:
                abs_p = os.path.join(dp, n)
                try:
                    rel = norm(os.path.relpath(abs_p, ws))
                except ValueError:
                    continue          # Windows 保留设备名（如根目录的 nul）不是真路径
                area = area_of(rel)
                self.byname_any.setdefault(n, []).append((rel, area))
                if area == "live":
                    self.live.add(rel)
                    if not is_data(rel):
                        self.byname_live.setdefault(n, []).append(rel)

    def is_file(self, rel):
        return rel in self.live

    def nlines(self, rel):
        if rel not in self.lines:
            p = os.path.join(self.ws, rel)
            try:
                with open(p, "rb") as f:
                    data = f.read()
                self.lines[rel] = -1 if b"\x00" in data[:4096] else data.count(b"\n") + 1
            except OSError:
                self.lines[rel] = -1
        return self.lines[rel]


# ---------------------------------------------------------------- 抽取
PLACEHOLDER = ("<", ">", "*", "{", "}", "$")


def split_line(tok):
    """a/b.py:12-34 -> (a/b.py, 12, 34)"""
    m = LINE_TAIL.search(tok)
    if not m:
        return tok, None, None
    path = tok[:m.start()]
    a = m.group(1) or m.group(3)
    b = m.group(2) or m.group(4)
    return path, int(a) if a else None, int(b) if b else None


def clean_path(path):
    """规整成「相对 v1 根的路径」；返回空串表示不是仓库内锚点（工作区外的绝对路径）。"""
    p = norm(path.strip())
    p = p.strip("'" + chr(34) + "()[]<>|,")
    p = p.lstrip("./")
    p = p.rstrip(".,;:，。；")
    if not p:
        return ""
    if p.startswith("~/"):
        return ""                                  # 家目录运行期路径
    if re.match(r"^[A-Za-z]:/", p) or p.startswith("/"):
        try:
            rel = norm(os.path.relpath(p, WS))
        except ValueError:
            return ""
        if rel.startswith("../"):
            return ""                              # 工作区外（车端 /opt/dz/... 之类）
        return rel
    return p


def path_ok(p):
    """路径部分像不像文件引用（挡住时间/比分/端口/MAC/占比）。"""
    if not p or len(p) < 3 or len(p) > 240:
        return False
    if any(b in p for b in PLACEHOLDER):
        return False
    if " " in p or "\t" in p or "…" in p or "..." in p:
        return False
    if "=" in p:
        return False
    # CamelCase.小写 更像成员访问（JsonlLogger.log / Config.get），不是文件；
    # README.md、MISSION.md 这类全大写文件名不受影响
    if "/" not in p and re.match(r"^[A-Z][a-z]+[A-Za-z0-9]*.(log|txt|csv|json|md|py|sh)$", p):
        return False
    if p.startswith(("http", "www.", "mailto:")):
        return False
    if re.fullmatch(r"[\d.\-:/]+", p):
        return False
    base = p.rsplit("/", 1)[-1]
    if not base or base.startswith(".") or "." not in base:
        return False
    return bool(PATH_PART.match(p))


BT_RE = re.compile(BT + "([^" + BT + r"\n]{1,300})" + BT)
MD_RE = re.compile(r"\]\(([^)\n]{1,300})\)")
# 裸文本锚点：必须带行号，否则不认（时间/比分/端口会灌进来）
BARE_RE = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.\-/\\]*(?:[:：#]L?\d+(?:[-–~]\d+)?)")


def extract(text):
    """-> [(pos, kind, path, l1, l2)]  kind in {bt, md, bare, carry}"""
    ev = []
    spans = [(m.start(), m.end()) for m in BT_RE.finditer(text)]
    for m in BT_RE.finditer(text):
        raw_p, l1, l2 = split_line(m.group(1).strip())
        p = clean_path(raw_p) if raw_p else ""
        if p and path_ok(p):
            ev.append((m.start(), "bt", p, l1, l2))
        elif l1 is not None and not raw_p.strip():
            ev.append((m.start(), "carry", None, l1, l2))   # 承前省略 :245
    for m in MD_RE.finditer(text):
        raw = m.group(1).strip()
        if " " in raw:
            raw = raw.split(" ", 1)[0]
        raw = raw.strip("<>")
        if "://" in raw:
            continue
        p, l1, l2 = split_line(raw)
        if l1 is None:
            continue                      # 无行号的 md 链接归 check_links.py 管，不重复
        if "#" in p:
            p = p.split("#", 1)[0]
        p = clean_path(p)
        if p and path_ok(p):
            ev.append((m.start(), "md", p, l1, l2))
    for m in BARE_RE.finditer(text):
        st = m.start()
        if any(a <= st < b for a, b in spans):
            continue
        if st and text[st - 1] in "/\\":
            continue
        if "://" in text[max(0, st - 30):st]:
            continue
        p, l1, l2 = split_line(m.group(0))
        p = clean_path(p)
        if l1 is None or not p or not path_ok(p):
            continue
        ev.append((st, "bare", p, l1, l2))
    ev.sort(key=lambda x: x[0])
    return ev


# ---------------------------------------------------------------- 解析
CODE_ROOTS = ["dzrobot_v2", "dzsmartscout"]


class Resolver(object):
    def __init__(self, tree, ws):
        self.t = tree
        self.ws = ws

    def direct(self, p, docdir):
        cands = [os.path.join(docdir, p)]
        for base in [self.ws, DOCS]:
            cands.append(os.path.join(base, p))
        for cr in CODE_ROOTS:
            cands.append(os.path.join(self.ws, cr, p))
        out = []
        for c in cands:
            try:
                rel = norm(os.path.relpath(os.path.abspath(c), self.ws))
            except ValueError:
                continue
            if rel.startswith("../"):
                continue
            out.append((rel, os.path.exists(c)))
        return out

    def fits(self, rel, hi):
        if not hi:
            return True
        n = self.t.nlines(rel)
        return n <= 0 or hi <= n

    def resolve(self, p, docdir, hint, votes=None, hi=None):
        """-> (status, rel, how)  status in {live, archived, red, ambiguous}

        兜底猜测（hint-dir / doc-dir）只会把「判不了」变「判到了」，不会制造红；
        代价是极少数情况下会指到同名的另一份，所以 --all/--json 里带 how 供人工核。
        """
        base = p.rsplit("/", 1)[-1]
        for rel, ok in self.direct(p, docdir):
            if ok and self.t.is_file(rel):
                return "live", rel, "direct"
        hits = self.t.byname_live.get(base, [])
        guess = None
        uniq = [h for h in hits if h == p or h.endswith("/" + p)]
        if len(uniq) == 1:
            guess = ("live", uniq[0], "suffix-unique")
        elif len(hits) == 1:
            guess = ("live", hits[0], "basename-unique")
        elif hint:
            hd = hint.rsplit("/", 1)[0] if "/" in hint else ""
            cand = (hd + "/" + base).lstrip("/")
            if self.t.is_file(cand):
                guess = ("live", cand, "hint-dir")
        if guess is None and len(hits) > 1 and votes:
            for d, _c in sorted(votes.items(), key=lambda x: -x[1])[:4]:
                cand = ((d + "/" + base) if d else base).lstrip("/")
                if self.t.is_file(cand):
                    guess = ("live", cand, "doc-dir")
                    break
        if guess is not None:
            # 行号落在候选之外 = 猜错了（同名不同文件），换一份能落进去的；换不出来就不猜
            if self.fits(guess[1], hi):
                return guess
            alt = [c for c in hits if self.fits(c, hi)]
            alt += [r for r, a in self.t.byname_any.get(base, []) if a != "live" and self.fits(r, hi)]
            if len(alt) == 1:
                return ("live" if self.t.is_file(alt[0]) else "archived"), alt[0], "line-fit"
            return "ambiguous", None, "行号落不进任何同名候选"
        if len(hits) > 1:
            return "ambiguous", None, "%d 份同名" % len(hits)
        for rel, ok in self.direct(p, docdir):
            if ok:
                return "archived", rel, area_of(rel)
        any_hits = [r for r, a in self.t.byname_any.get(base, []) if a != "live"]
        if len(any_hits) == 1:
            return "archived", any_hits[0], area_of(any_hits[0])
        ext_hits = self.t.ext.get(base, [])
        if len(ext_hits) == 1:
            return "extern", ext_hits[0], "外部根"
        return "red", None, ""


# ---------------------------------------------------------------- 主流程
def live_docs(cfg, ns, include_archive):
    roots = cfg.get("doc_roots") or DEFAULT_ROOTS
    files = []
    for r in roots:
        d = os.path.join(WS, r)
        if not os.path.isdir(d):
            continue
        for dp, dns, fns in os.walk(d):
            dns[:] = [x for x in dns if not x.startswith(".")]
            for n in fns:
                if n.lower().endswith(".md"):
                    files.append(norm(os.path.relpath(os.path.join(dp, n), WS)))
    for f in cfg.get("doc_root_files") or DEFAULT_ROOT_FILES:
        if os.path.exists(os.path.join(WS, f)):
            files.append(f)
    if include_archive:
        for r in ("docs/history_cache", "docs/_work"):
            d = os.path.join(WS, r)
            if not os.path.isdir(d):
                continue
            for dp, _dns, fns in os.walk(d):
                for n in fns:
                    if n.lower().endswith(".md"):
                        files.append(norm(os.path.relpath(os.path.join(dp, n), WS)))
    files = sorted(set(files))
    if ns:
        files = [f for f in files if ns in f]
    return files


def apply_root(path):
    """--root：把工作区根换到别处（负向测试/沙盒用，不写任何共享文件）。"""
    global WS, DOCS, POLICY, BASELINE
    WS = os.path.abspath(path)
    DOCS = os.path.join(WS, "docs")
    POLICY = os.path.join(DOCS, "_工具", "doc_policy.json")
    BASELINE = os.path.join(DOCS, "_工具", "anchor_baseline.json")


def main():
    if "--root" in sys.argv:
        i = sys.argv.index("--root")
        if i + 1 < len(sys.argv):
            apply_root(sys.argv[i + 1])
    cfg = {}
    if os.path.exists(POLICY):
        try:
            cfg = load(POLICY)
        except ValueError:
            cfg = {}
    extra = [r for r in DEFAULT_EXTRA_ROOTS if os.path.isdir(r)]
    for i, a in enumerate(sys.argv):
        if a == "--extra-root" and i + 1 < len(sys.argv):
            extra.append(os.path.normpath(sys.argv[i + 1]))
    tree = Tree(WS, set(cfg.get("skip_dirs") or DEFAULT_SKIP), extra)
    res = Resolver(tree, WS)

    baseline = {}
    if os.path.exists(BASELINE):
        try:
            baseline = load(BASELINE)
        except ValueError:
            baseline = {}
    base_keys = set(baseline.get("keys", []))

    ns = ""
    if "--ns" in sys.argv:
        i = sys.argv.index("--ns")
        if i + 1 < len(sys.argv):
            ns = sys.argv[i + 1]
    docs = live_docs(cfg, ns, "--include-archive" in sys.argv)

    reds, yellows, warns, ambs, externs, unfit = [], [], [], [], [], []
    n_ref = n_anchor = 0
    for rel in docs:
        text = read_text(os.path.join(WS, rel))
        docdir = os.path.dirname(os.path.join(WS, rel))
        hints = []            # 最近成功解析的文件（新在前），给「承前省略 :245」用
        votes = {}            # 本文档已解析文件的目录票数，给同名 basename 兜底用
        for _pos, kind, p, l1, l2 in extract(text):
            hi = max(x for x in (l1, l2) if x) if l1 is not None else None
            if kind == "carry":
                if not hints:
                    continue
                pick = None
                for h in hints:        # 行号能落进去的最近一份，优先
                    if res.fits(h, hi):
                        pick = h
                        break
                if pick is None:
                    unfit.append((rel, ":".join([str(x) for x in (l1,) if x]), hints[0]))
                    n_ref += 1
                    continue
                status, hit, how = "live", pick, "carry"
                key_p = p if p else hit
            else:
                status, hit, how = res.resolve(p, docdir, hints[0] if hints else None, votes, hi)
                key_p = p
            n_ref += 1
            if l1 is not None:
                n_anchor += 1
            key = rel + " :: " + key_p
            if status == "live":
                if hit not in hints:
                    hints.insert(0, hit)
                    del hints[4:]
                d = hit.rsplit("/", 1)[0] if "/" in hit else ""
                votes[d] = votes.get(d, 0) + 1
                if l1 is not None:
                    n = tree.nlines(hit)
                    hi = max(x for x in (l1, l2) if x)
                    if n > 0 and hi > n:
                        warns.append((rel, key, hit, hi, n))
            elif status == "ambiguous":
                ambs.append((rel, key, how))
            elif status == "archived":
                yellows.append((rel, key, hit, how))
            elif status == "extern":
                externs.append((rel, key, hit))
            else:
                reds.append((rel, key, p))

    new_reds = [r for r in reds if r[1] not in base_keys]
    old_reds = [r for r in reds if r[1] in base_keys]
    gone = sorted(base_keys - set(r[1] for r in reds))

    if "--write-baseline" in sys.argv:
        keys = sorted(set(r[1] for r in reds))
        payload = {
            "generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
            "count": len(keys),
            "说明": ("存量红的 (文档 :: 路径) 集合，语义只减不增。默认只对不在此表里的新增红判失败；"
                     "修好一条就该变小一条（重跑 --write-baseline 精简）。行号不进 key：同一文件的多个行号引用只算一条。"),
            "keys": keys,
        }
        with open(BASELINE, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=1)
        print("[基线已写入] docs/_工具/anchor_baseline.json  存量红 %d 条" % len(keys))

    areas = {}
    for _rel, _k, _hit, how in yellows:
        areas[how] = areas.get(how, 0) + 1
    summary = ("扫描 %d 篇 / 引用 %d 条（带行号锚点 %d）｜新增红 %d（存量红 %d）｜黄 %d｜行号越界 %d"
               "｜同名待判 %d｜行号无主 %d｜外部 %d"
               % (len(docs), n_ref, n_anchor, len(new_reds), len(old_reds), len(yellows),
                  len(warns), len(ambs), len(unfit), len(externs)))

    if "--json" in sys.argv:
        out = {
            "summary": summary,
            "counts": {"docs": len(docs), "refs": n_ref, "anchors": n_anchor,
                       "red_new": len(new_reds), "red_baseline": len(old_reds),
                       "yellow": len(yellows), "warn": len(warns), "ambiguous": len(ambs),
                       "extern": len(externs), "unfit": len(unfit), "yellow_areas": areas},
            "red_new": [{"doc": r[0], "key": r[1], "target": r[2]} for r in new_reds],
            "red_baseline": [{"doc": r[0], "key": r[1], "target": r[2]} for r in old_reds],
            "yellow": [{"doc": y[0], "key": y[1], "found": y[2], "area": y[3]} for y in yellows],
            "warn": [{"doc": w[0], "key": w[1], "file": w[2], "line": w[3], "lines": w[4]} for w in warns],
            "ambiguous": [{"doc": a[0], "key": a[1], "note": a[2]} for a in ambs],
            "extern": [{"doc": e[0], "key": e[1], "found": e[2]} for e in externs],
            "unfit": [{"doc": u[0], "line": u[1], "nearest": u[2]} for u in unfit],
            "baseline_stale": gone,
        }
        print(json.dumps(out, ensure_ascii=False, indent=1))
        return 1 if new_reds else 0

    print("=" * 72)
    print("锚点守卫  " + summary)
    print("=" * 72)
    if new_reds:
        print("\n[新增红] 路径全盘不存在，必须处置（改路径 / 标〔失效〕 / 删引用）：")
        for rel, _key, p in new_reds[:40]:
            print("   RED  %s\n        %s" % (rel, p))
        if len(new_reds) > 40:
            print("   ... 另有 %d 条（--all 看全集）" % (len(new_reds) - 40))
    if old_reds:
        print("\n[存量红] %d 条已在基线里，不判失败（修一条少一条）：" % len(old_reds))
        if "--all" in sys.argv:
            for rel, _key, p in old_reds[:80]:
                print("   base %s  ->  %s" % (rel, p))
            if len(old_reds) > 80:
                print("   ... 另有 %d 条" % (len(old_reds) - 80))
        else:
            per = {}
            for rel, _k, _p in old_reds:
                per[rel] = per.get(rel, 0) + 1
            for rel, c in sorted(per.items(), key=lambda x: -x[1])[:12]:
                print("   base %-58s %d 条" % (rel, c))
            print("   （加 --all 看逐条）")
    if yellows:
        print("\n[黄] 引用已搬走，只在非现行区有 %d 条：" % len(yellows))
        print("   分区：" + "、".join("%s %d" % (k, v) for k, v in sorted(areas.items(), key=lambda x: -x[1])))
        for rel, key, hit, how in yellows[:30]:
            print("   YEL  %s  %s  ->  %s（%s）" % (rel, key.split(" :: ")[1], hit, how))
        if len(yellows) > 30:
            print("   ... 另有 %d 条（--json 看全集）" % (len(yellows) - 30))
    if warns:
        print("\n[行号越界] %d 条（文件在，但行号超过实际行数 = 尾部被删/重写后漂移）：" % len(warns))
        for rel, _key, hit, hi, n in warns[:20]:
            print("   WRN  %s:%s  实际 %d 行  (%s)" % (hit, hi, n, rel))
        if len(warns) > 20:
            print("   ... 另有 %d 条" % (len(warns) - 20))
    if ambs:
        print("\n[同名待判] %d 条（同一 basename 现行树里多份，不判红；建议锚点写全路径）：" % len(ambs))
        for _rel, key, note in ambs[:10]:
            print("   AMB  %s  %s" % (key.split(" :: ")[1], note))
        if len(ambs) > 10:
            print("   ... 另有 %d 条" % (len(ambs) - 10))
    if unfit:
        print("\n[行号无主] %d 条「承前省略」行号落不进上文任何文件（不判红，需人工确认指谁）：" % len(unfit))
        for rel, ln, near in unfit[:8]:
            print("   UNF  %s  :%s  最近上文 %s" % (rel, ln, near))
        if len(unfit) > 8:
            print("   ... 另有 %d 条" % (len(unfit) - 8))
    if externs:
        print("\n[外部] %d 条指向工作区外兄弟仓（文件存在，不计红）：" % len(externs))
        for rel, key, hit in externs[:8]:
            print("   EXT  %s  ->  %s" % (key.split(" :: ")[1], hit))
        if len(externs) > 8:
            print("   ... 另有 %d 条" % (len(externs) - 8))
    if gone:
        print("\n[可精简] 基线里 %d 条已不再是红（已修好），重跑 --write-baseline 收缩基线" % len(gone))

    if new_reds:
        print("\n结论：RED —— %d 条新增红。修法：路径改到现行位置，或标〔档〕〔失效〕。" % len(new_reds))
    else:
        print("\n结论：GREEN —— 无新增悬空锚点（存量 %d 条在基线内，黄 %d 条待清理）。" % (len(old_reds), len(yellows)))
    return 1 if new_reds else 0


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:  # noqa: BLE001
        pass
    sys.exit(main())
