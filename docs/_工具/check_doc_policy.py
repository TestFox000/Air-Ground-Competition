# -*- coding: utf-8 -*-
"""文档守卫（doc policy gate）——防止长期文档再次散落。

判红（退出码 1）：
  R1 库外新增 .md     —— 不在 docs/ 白名单、也不在基线里（=有人又在别处拉屎）
  R2 库内位置越权     —— docs/ 下但不在允许的库目录/根文件里（=又开了个新库）
  R3 库内未登记       —— docs/ 下的文档没进 目录索引.json（=写了没登记）
  R4 任务卡 status 越界 —— 在办卡必须 pending/in_progress；归档卡必须 已归档（按日合并摘要除外）
  R6 20 库篇数口径漂移  —— README 写"应为 N 篇"与实际 .md 数不符（手写数字必腐）
  R7 命名空间不合法     —— 在办/<ns>/ 缺 00_树.md、卡面 ns 与目录不符、ns 未登记、缺 agent
  R8 卡未挂树           —— 缺 树节点 / 树里没有本卡 / 树指向不存在的卡
  R9 归档归属错         —— 归档未按 <ns> 分层、归档卡 ns 与目录不符、缺「文档变更」
提醒（不判红）：
  Y1 库内文档缺 front matter status
  Y2 status=现行 但 updated 超过 stale_days 天（过期候选）
  Y3 库内标题重复（副本即债）
  Y4 基线里已消失的路径（可精简基线）
  Y5 根目录临时件（_tmp*/_work_tmp*/_recover_tmp*，应进 _tmp_session/ 或 _archive/）
  Y9 文档引用 _tmp_session 失效路径数上升（过程区已清空，应改指归档包或标〔档〕）

用法：
  python docs/_工具/check_doc_policy.py                 # 体检
  python docs/_工具/check_doc_policy.py --write-baseline # 收口时重建基线（只减不增）
  python docs/_工具/check_doc_policy.py --accept <path> --reason "为什么它可以留在库外"
  python docs/_工具/check_doc_policy.py --json          # 机器读
"""
import json, os, sys, re
from datetime import datetime, date

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
WS = os.path.dirname(DOCS)
POLICY = os.path.join(HERE, "doc_policy.json")

def load(p):
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)

def norm(rel):
    return rel.replace("\\", "/")

def walk(ws, cfg):
    skip = set(cfg.get("skip_dirs", []))
    out = []
    for dp, dns, fns in os.walk(ws):
        rel_dir = norm(os.path.relpath(dp, ws))
        if rel_dir == ".":
            rel_dir = ""
        dns[:] = [d for d in dns if d not in skip and not d.startswith(".")]
        for n in fns:
            if n.lower().endswith(".md"):
                out.append(norm(os.path.join(rel_dir, n)) if rel_dir else n)
    return sorted(out)

def classify(rel, cfg):
    """-> (class, reason)  class in {doc, readonly, exempt, vendor, managed}"""
    for r in cfg["readonly_roots"]:
        if rel.startswith(r + "/"):
            return "readonly", "历史留存区只读"
    for p in cfg["exempt_prefixes"]:
        if rel.startswith(p):
            return "exempt", "过程/备份/档案区"
    for m in cfg["vendor_markers"]:
        if m in "/" + rel:
            return "vendor", "第三方源码自带"
    if rel in cfg["doc_root_files"]:
        return "doc", "库入口"
    for r in cfg["doc_roots"]:
        if rel.startswith(r + "/"):
            return "doc", "库内"
    if rel.startswith("docs/"):
        return "out_of_policy", "docs/ 下但不在白名单"
    return "managed", "库外受管"

def front_matter(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read(4000)
    except OSError:
        return {}
    if not text.startswith("---"):
        return {}
    end = text.find("\n---", 3)
    if end < 0:
        return {}
    fm = {}
    for line in text[3:end].splitlines():
        if ":" in line:
            k, v = line.split(":", 1)
            fm[k.strip()] = v.strip()
    return fm

def main():
    cfg = load(POLICY)
    bl_path = os.path.join(WS, cfg["baseline"])
    baseline = set()
    if os.path.exists(bl_path):
        baseline = set(load(bl_path).get("managed", []))

    if "--accept" in sys.argv:
        i = sys.argv.index("--accept")
        reason = ""
        if "--reason" in sys.argv:
            reason = sys.argv[sys.argv.index("--reason") + 1]
        targets = [sys.argv[i + 1]]
        if not os.path.exists(bl_path):
            json.dump({"generated": "", "count": 0, "managed": []}, open(bl_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        data = load(bl_path)
        data.setdefault("accept_log", [])
        for t in targets:
            t = norm(t)
            if not os.path.exists(os.path.join(WS, t)):
                print("[错误] 不存在：" + t); return 2
            data["managed"] = sorted(set(data.get("managed", [])) | {t})
            data["accept_log"].append({"path": t, "reason": reason, "at": datetime.now().strftime("%Y-%m-%d %H:%M")})
        data["count"] = len(data["managed"])
        json.dump(data, open(bl_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("[已登记基线] " + t + "  理由：" + (reason or "（未填）"))
        return 0

    index_json = os.path.join(DOCS, "目录索引.json")
    registered = set()
    if os.path.exists(index_json):
        d = load(index_json)
        registered = {"docs/" + norm(x["path"]) for x in d.get("files", [])}

    files = walk(WS, cfg)
    buckets = {"doc": [], "readonly": [], "exempt": [], "vendor": [], "managed": [], "out_of_policy": []}
    for rel in files:
        cls, why = classify(rel, cfg)
        buckets[cls].append(rel)

    red, yellow = [], []
    # R1 库外新增
    managed_now = set(buckets["managed"])
    for rel in sorted(managed_now - baseline):
        red.append(("R1", rel, "库外出现未登记的 .md（新增文档只能进 docs/，确属代码仓自带请 --accept 并写理由）"))
    # 基线消失（提醒精简）
    for rel in sorted(baseline - managed_now):
        yellow.append(("Y4", rel, "基线里的路径已不存在，可重跑 --write-baseline 精简"))
    # R2/R3 + Y1/Y2 库内
    life = cfg.get("lifecycle", {})
    stale_days = int(life.get("stale_days", 180))
    today = date.today()
    titles = {}
    for rel in buckets["doc"]:
        if rel in ("docs/目录索引.md", "docs/README.md"):
            continue
        if rel not in registered:
            red.append(("R3", rel, "库内文档未登记进 目录索引.json（跑 gen_index.py）"))
        fm = front_matter(os.path.join(WS, rel))
        in_scope = any(rel.startswith(s + "/") for s in life.get("status_scope", []))
        st = fm.get("status", "")
        if in_scope and not st and life.get("require_status", False):
            red.append(("R3", rel, "缺 front matter status"))
        elif in_scope and not st:
            yellow.append(("Y1", rel, "缺 front matter status（现行/计划/历史/过程）"))
        elif in_scope and st not in life.get("statuses", []) and life.get("require_status", False):
            red.append(("R3", rel, "status 取值非法：" + st))
        if in_scope and st == "现行" and fm.get("更新"):
            try:
                d = datetime.strptime(fm["更新"][:10], "%Y-%m-%d").date()
                if (today - d).days > stale_days:
                    yellow.append(("Y2", rel, "status=现行 但已 %d 天未更新（>%d 天）" % ((today - d).days, stale_days)))
            except ValueError:
                pass
        base = rel.rsplit("/", 1)[-1][:-3]
        if base in ("README", "00_索引") or rel.startswith("docs/30_任务卡库/"):
            continue
        titles.setdefault(base, []).append(rel)
    for rel in buckets["out_of_policy"]:
        red.append(("R2", rel, "docs/ 下但不在允许的库目录：新增库必须先改 doc_policy.json"))
    for t, paths in titles.items():
        if len(paths) > 1:
            yellow.append(("Y3", paths[0], "标题/文件名重复：" + " | ".join(paths)))

    # R4 任务卡 status 值域（卡库用自己的三值，见 AGENTS §8.5；细节写正文，不进 frontline）
    cs = cfg.get("card_status", {})
    if cs.get("enabled", True):
        live_vals = cs.get("in_progress_values", ["pending", "in_progress"])
        done_val = cs.get("archived_value", "已归档")
        for rel in buckets["doc"]:
            if not rel.startswith("docs/30_任务卡库/"):
                continue
            name = rel.rsplit("/", 1)[-1]
            if name in ("README.md", "00_索引.md", "00_树.md") or "/模板/" in rel or "/_设计/" in rel:
                continue
            st = front_matter(os.path.join(WS, rel)).get("status", "")
            if "/在办/" in rel:
                if st not in live_vals:
                    red.append(("R4", rel, "在办卡 status 只能是 %s，现在是「%s」；细节挪正文（§8.5）"
                                % (" / ".join(live_vals), st or "空")))
            elif "/归档/" in rel:
                if re.match(r"^\d{4}-\d{2}-\d{2}\.md$", name):
                    continue  # 按日合并摘要不是卡面
                if st != done_val:
                    red.append(("R4", rel, "归档卡 status 必须是「%s」，现在是「%s」" % (done_val, st or "空")))

    # R6 20 库 README 篇数口径 vs 实际（手写计数必腐，用扫描结果对账）
    lib_readme = os.path.join(WS, "docs/20_资料索引库/README.md")
    if os.path.exists(lib_readme):
        with open(lib_readme, "r", encoding="utf-8", errors="replace") as f:
            rt = f.read()
        m = re.search(r"应为\s*(\d+)\s*篇", rt)
        actual = sum(1 for r in files if r.startswith("docs/20_资料索引库/"))
        if m and int(m.group(1)) != actual:
            red.append(("R6", "docs/20_资料索引库/README.md",
                        "README 写「应为 %s 篇」，实际 %d 篇（口径：本目录全部 .md，含 README 与册级索引）"
                        % (m.group(1), actual)))

    # Y9 文档里 _tmp_session 引用失效（10-21 过程目录清空后的烂指针），只准降不准升
    tr = cfg.get("tmprefs", {})
    if tr.get("enabled", True):
        base_dead = int(tr.get("dangling_baseline", 0))
        cited = set()
        for rel in buckets["doc"]:
            try:
                with open(os.path.join(WS, rel), "r", encoding="utf-8", errors="replace") as f:
                    txt = f.read()
            except OSError:
                continue
            for m in re.finditer(r"_tmp_session/[^\s，,；;）)】\]|]+", txt):
                raw = m.group(0).rstrip(".。,，;；:：").rstrip(chr(96) + chr(34) + chr(39))
                for suf in ("〔档〕", "〔证失效〕", chr(96)):
                    if raw.endswith(suf):
                        raw = raw[:-len(suf)]
                if "<" in raw or "*" in raw or raw == "_tmp_session/" or raw.endswith("..."):
                    continue
                cited.add(raw)
        dead = [r for r in cited if not os.path.exists(os.path.join(WS, r))]
        if len(dead) > base_dead:
            yellow.append(("Y9", "docs/**",
                           "引用 _tmp_session 失效路径 %d 个（基线 %d）：docs 不该指过程区，改指归档包或标〔档〕"
                           % (len(dead), base_dead)))

    # Y5 根目录临时件（应进 _tmp_session/，不参与版本管理）
    for n in sorted(os.listdir(WS)):
        if n == "_tmp_session":
            continue
        if n.startswith(("_tmp", "_work_tmp", "_recover_tmp")):
            yellow.append(("Y5", n, "根目录临时文件/目录（收口前移入 _tmp_session/ 或 _archive/）"))

    # R7/R8/R9 命名空间与任务树（2026-10-23 由 Y6~Y8 转红线）
    # 规则真源：docs/30_任务卡库/_设计/00_命名空间与任务树.md
    tree_cache = {}

    def _tree_text(dir_rel):
        if dir_rel not in tree_cache:
            p = os.path.join(WS, dir_rel, "00_树.md")
            tree_cache[dir_rel] = (open(p, "r", encoding="utf-8", errors="replace").read()
                                   if os.path.exists(p) else None)
        return tree_cache[dir_rel]

    CARD_LIVE = "docs/30_任务卡库/在办/"
    reg_path = os.path.join(WS, "docs/30_任务卡库/_设计/01_ns登记.md")
    registered_ns = set()
    if os.path.exists(reg_path):
        with open(reg_path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = re.match(r"^\|\s*([a-z][a-z0-9-]*\d{4})\s*\|", line)
                if m:
                    registered_ns.add(m.group(1))
    for rel in buckets["doc"]:
        if not rel.startswith(CARD_LIVE):
            continue
        parts = rel.split("/")
        if len(parts) < 5:
            continue  # 旧卡（无 ns 层）：试点期不纳入
        ns = parts[3]
        if parts[-1] == "00_树.md":
            continue
        dir_rel = "/".join(parts[:4])
        tree = _tree_text(dir_rel)
        fm = front_matter(os.path.join(WS, rel))
        if tree is None:
            red.append(("R7", rel, "所在 ns 缺 00_树.md（ns=%s）" % ns))
            continue
        if fm.get("ns", "") != ns:
            red.append(("R7", rel, "ns: 与目录名不一致（目录 %s / 卡面 %s）" % (ns, fm.get("ns", "空"))))
        if registered_ns and ns not in registered_ns:
            red.append(("R7", rel, "ns 未在 _设计/01_ns登记.md 登记（ns=%s）" % ns))
        if not fm.get("agent"):
            red.append(("R7", rel, "缺 agent:（一张卡必须绑定一个智能体）"))
        stem = parts[-1][:-3]
        if not fm.get("树节点"):
            red.append(("R8", rel, "缺 树节点:（卡必须挂在树上）"))
        elif stem not in tree:
            red.append(("R8", rel, "树里没有指向本卡的节点（卡号 %s 未出现）" % stem))
    for dir_rel, tree in tree_cache.items():
        if not tree:
            continue
        for cam in re.findall(r"\[卡\]\s*([^\s·|（(）)]+)", tree):
            if not os.path.exists(os.path.join(WS, dir_rel, cam + ".md")):
                red.append(("R8", dir_rel + "/00_树.md", "树节点指向的卡不存在：" + cam))

    # R9 归档归属：必须落在 归档/<ns>/ 下 + ns 与目录一致 + 写清「文档变更」
    ARCH = "docs/30_任务卡库/归档/"
    for rel in buckets["doc"]:
        if not rel.startswith(ARCH):
            continue
        parts = rel.split("/")
        name = parts[-1]
        if name == "00_索引.md":
            continue
        if len(parts) == 4:
            red.append(("R9", rel, "归档未按 ns 分层（移到 归档/<ns>/%s）" % name))
            continue
        if len(parts) < 5:
            continue
        ns = parts[3]
        if ns.startswith("_"):
            continue  # _按日摘要 等非卡目录
        fm = front_matter(os.path.join(WS, rel))
        if fm.get("ns", "") != ns:
            red.append(("R9", rel, "归档卡 ns: 与目录不一致（目录 %s / 卡面 %s）" % (ns, fm.get("ns", "空"))))
        try:
            with open(os.path.join(WS, rel), "r", encoding="utf-8", errors="replace") as fh:
                body = fh.read()
        except OSError:
            continue
        m = re.search(r"文档变更", body)
        seg = body[m.end():].split("\n## ", 1)[0] if m else ""
        if not seg.strip().strip("-—:： ").strip():
            red.append(("R9", rel, "已归档卡缺「文档变更」落地记录（§8.8 / 设计 §5）"))

    # 写基线模式
    if "--write-baseline" in sys.argv:
        payload = {"generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
                   "count": len(managed_now), "managed": sorted(managed_now)}
        prev = load(bl_path) if os.path.exists(bl_path) else {}
        payload["accept_log"] = prev.get("accept_log", [])
        json.dump(payload, open(bl_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("[基线已写入] %s  受管 %d 篇" % (cfg["baseline"], len(managed_now)))

    summary = ("受管 %d（库内 %d / 库外受管 %d / 只读 %d / 豁免 %d / 第三方 %d）｜红线 %d ｜提醒 %d"
               % (len(files), len(buckets["doc"]), len(buckets["managed"]), len(buckets["readonly"]),
                  len(buckets["exempt"]), len(buckets["vendor"]), len(red), len(yellow)))
    if "--json" in sys.argv:
        print(json.dumps({"summary": summary, "red": red, "yellow": yellow,
                          "counts": {k: len(v) for k, v in buckets.items()}}, ensure_ascii=False, indent=1))
    else:
        print("=" * 68)
        print("文档守卫  " + summary)
        print("=" * 68)
        if red:
            print("\n[红线] 必须处置：")
            for code, rel, why in red[:60]:
                print("  %s  %s\n       %s" % (code, rel, why))
            if len(red) > 60:
                print("  ... 另有 %d 条" % (len(red) - 60))
        if yellow:
            print("\n[提醒] 不判红：")
            for code, rel, why in yellow[:20]:
                print("  %s  %s  %s" % (code, rel, why))
            if len(yellow) > 20:
                print("  ... 另有 %d 条" % (len(yellow) - 20))
        if not red:
            print("\n结论：GREEN —— 库外无新增文档，库内全部登记。")
        else:
            print("\n结论：RED —— %d 条红线。" % len(red))
    return 1 if red else 0

if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")
    sys.exit(main())
