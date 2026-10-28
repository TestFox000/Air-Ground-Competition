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
  Y9 新增 _tmp_session 死链 —— 按 (文档 :: 路径) 记账：存量进 tmprefs_baseline.json 只减不增，
                          不在表里的新死链判红；该行标〔档〕或〔证失效〕= 声明原物已不在，豁免
提醒（不判红）：
  Y1 库内文档缺 front matter status
  Y2 status=现行 但 updated 超过 stale_days 天（过期候选）
  Y3 库内标题重复（副本即债）
  Y4 基线里已消失的路径（可精简基线）
  Y5 根目录临时件（_tmp*/_work_tmp*/_recover_tmp*，应进 _tmp_session/ 或 _archive/）
  Y10 归档卡缺「经验总结」段  —— 归档卡正文必须有六要素经验总结（现象→机制→位置→处置→证据→落点）

用法：
  python docs/_工具/check_doc_policy.py                 # 体检
  python docs/_工具/check_doc_policy.py --write-baseline # 收口时重建基线（只减不增）
  python docs/_工具/check_doc_policy.py --accept <path> --reason "为什么它可以留在库外"
  python docs/_工具/check_doc_policy.py --write-tmprefs-baseline  # 修完死链后精简存量清单（只减不增）
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

def looks_like_path(raw):
    r"""Y9 专用：判断一段 `_tmp_session/...` 捕获是否真像路径（而不是散文里顺带提到过程区）。

    起因（2026-10-26）：原正则 `_tmp_session/[^s，,；;）)】]|]+` 会把中文列举、花括号占位、
    省略号截断一并吞进来，实测 836 条捕获里 384 条根本不是路径（如
    `_tmp_session/aimtune、aim-lat`、`_tmp_session/{t3`、`_tmp_session/DRONE-S…`），
    把 Y9 的数字撑大了约一倍，导致"真实死链"被噪声淹没、无法据数决策。
    判据：带占位符/枚举符/省略号/全角括号 → 非路径；结尾是 / → 是目录不是文件；
    只有一段且无扩展名 → 多半是散文；含中文却无扩展名 → 散文。
    """
    if any(ch in raw for ch in "{}、…（"):
        return False
    if raw.endswith("/"):
        return False
    body = raw[len("_tmp_session/"):]
    if not body:
        return False
    segs = body.split("/")
    has_ext = bool(re.search(r"\.[A-Za-z0-9]{1,6}$", body))
    if len(segs) < 2 and not has_ext:
        return False
    if re.search(r"[\u4e00-\u9fff]", body) and not has_ext:
        return False
    return True


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

def read_text_top(path):
    """整篇读入（册级索引要按文件名做包含判断，不能只读前 4000 字节）。缺文件返回 None。"""
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None

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
        # R11（2026-10-27 增）：只要写了 status，就必须是合法取值——不受 require_status 开关、也不受 status_scope 限制。
        # 起因实测：树模板把整句说明写进 status（"现行  # 唯一正本……"），30 库不在 status_scope 内、
        # require_status 又是 false，两道闸都不管，于是守卫全绿，而 目录索引.md 的「状态」列被渲染成一整句话。
        # 另：更新 值必须以 YYYY-MM-DD 开头（允许尾随括注），否则 目录索引 的日期排序与「最近更新」失真。
        _cs = cfg.get("card_status", {})
        _valid_st = (set(life.get("statuses", [])) | set(_cs.get("in_progress_values", []))
                     | {_cs.get("archived_value", "已归档")})
        if st and st not in _valid_st:
            red.append(("R11", rel, "status 取值非法（只能一个词，说明写正文）：" + st))
        _upd = fm.get("更新", "")
        if _upd and not re.match(r"^\d{4}-\d{2}-\d{2}", _upd):
            yellow.append(("Y11", rel, "更新 不以 YYYY-MM-DD 开头，索引排序会失真：" + _upd))
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

    # R10 册级索引漏登（2026-10-26 增）——细则 11.4「索引义务三层，缺一层就白写」的第一层，
    # 此前只有文字承诺、没有任何机器判据：gen_index 会把这 232 篇全收进 目录索引（第二层），
    # 于是"写进全库索引"必然成立，第一层漂移被第二层完全掩盖。
    # 实证：编年 6 卷 + 10 库 2 个自报数 + 20 库 2 个分册篇数，全是手写、全腐。
    # 口径：20 库按各分册 00_索引.md 逐个对文件名；00/10 按库根 00_索引.md 对（含子目录）。
    _INDEX_CHECK_LIBS = {
        "docs/20_资料索引库": "vol",
        "docs/00_项目总览": "root",
        "docs/10_agent提示词": "root",
    }
    for _lib, _mode in _INDEX_CHECK_LIBS.items():
        _libp = os.path.join(WS, _lib)
        if not os.path.isdir(_libp):
            continue
        if _mode == "vol":
            for _vol in sorted(os.listdir(_libp)):
                _vp = os.path.join(_libp, _vol)
                if not os.path.isdir(_vp):
                    continue
                _txt = read_text_top(os.path.join(_vp, "00_索引.md"))
                if _txt is None:
                    red.append(("R10", _lib + "/" + _vol,
                                "分册缺 00_索引.md（细则 11.4 第一层：册级索引）"))
                    continue
                # 分册内可能还有子目录（当前无，防将来），按 walk 遍历而不是只 listdir
                for _dp, _dn, _fn in os.walk(_vp):
                    _dn[:] = [d for d in _dn if not d.startswith(".")]
                    for _f in _fn:
                        if not _f.endswith(".md") or _f == "00_索引.md":
                            continue
                        _relsub2 = norm(os.path.relpath(os.path.join(_dp, _f), _vp))
                        if _relsub2 not in _txt and _f not in _txt:
                            red.append(("R10", _lib + "/" + _vol + "/" + _relsub2,
                                        "未登记进本分册 00_索引.md（全库索引会收它，但读者从册级索引找不到）"))
        else:
            _txt = read_text_top(os.path.join(_libp, "00_索引.md"))
            if _txt is None:
                red.append(("R10", _lib, "库缺 00_索引.md（细则 11.4 第一层：册级索引）"))
                continue
            for _dp, _dn, _fn in os.walk(_libp):
                _dn[:] = [d for d in _dn if not d.startswith(".")]
                for _f in _fn:
                    if not _f.endswith(".md") or _f == "00_索引.md":
                        continue
                    _relsub = norm(os.path.relpath(os.path.join(_dp, _f), _libp))
                    if _f not in _txt and _relsub not in _txt:
                        red.append(("R10", _lib + "/" + _relsub,
                                    "未登记进库根 00_索引.md（细则 11.4 第一层：册级索引）"))

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
        # R6b 逐分册篇数（2026-10-26 增）：原来只校验总数，分册表里 8 个数字全靠手写。
        # 实测 2 处已腐且总数仍对得上（赛事与治理写 12 实 16；运维与排障写 6 实 7）——
        # 总数校验掩盖了分册漂移，读者按表找篇数会找错册。
        _vol_actual = {}
        for _vol in os.listdir(os.path.join(WS, "docs/20_资料索引库")):
            _vp = os.path.join(WS, "docs/20_资料索引库", _vol)
            if os.path.isdir(_vp):
                _vol_actual[_vol] = sum(1 for r in files
                                        if r.startswith("docs/20_资料索引库/" + _vol + "/"))
        for _mm in re.finditer(r"\| \[([^/\]]+)/\]\([^)]*\) \| (\d+) \|", rt):
            _vol, _claim = _mm.group(1), int(_mm.group(2))
            _act = _vol_actual.get(_vol)
            if _act is not None and _claim != _act:
                red.append(("R6", "docs/20_资料索引库/README.md",
                            "分册表「%s」写 %d 篇，实际 %d 篇（含该册 00_索引.md；总数对不代表分册对）"
                            % (_vol, _claim, _act)))

    # Y9 文档里 _tmp_session 引用失效（10-21 过程目录清空后的烂指针），只准降不准升
    tr = cfg.get("tmprefs", {})
    if tr.get("enabled", True):
        tb_path = os.path.join(HERE, tr.get("baseline_file", "tmprefs_baseline.json"))
        cited, acked = {}, set()   # cited: 文档 → 该文档引用的过程区路径集合（按 key 记账，不再用总量阈值）
        for rel in buckets["doc"]:
            try:
                with open(os.path.join(WS, rel), "r", encoding="utf-8", errors="replace") as f:
                    txt = f.read()
            except OSError:
                continue
            for line in txt.splitlines():
                # 〔档〕= 作者已声明"此处只有文档记载、原物已进归档包"。规则文字一直承诺这条
                # 豁免，但旧代码只是把标记后缀从字符串尾部剥掉、仍照旧计入死链 —— 等于标记无效。
                # 2026-10-26 起：整行带〔档〕的引用计入"已声明"，不参与死链判定。
                # 〔证失效〕与〔档〕同为"宣告该引用已无害"的标记（细则 §11.3.1 表），
                # 旧代码只把它从字符串尾部剥掉、没有豁免，等于标记无效——与〔档〕同一个坑。
                declared = "〔档〕" in line or "〔证失效〕" in line
                for m in re.finditer(r"_tmp_session/[^\s，,；;）)】\]|]+", line):
                    raw = m.group(0).rstrip(".。,，;；:：").rstrip(chr(96) + chr(34) + chr(39))
                    for suf in ("〔档〕", "〔证失效〕", chr(96)):
                        if raw.endswith(suf):
                            raw = raw[:-len(suf)]
                    if "<" in raw or "*" in raw or raw == "_tmp_session/" or raw.endswith("..."):
                        continue
                    if not looks_like_path(raw):
                        continue
                    if declared:
                        acked.add(raw)
                    cited.setdefault(rel, set()).add(raw)
        # 2026-10-26：旧的「死链总数 > 250」阈值撑不住只减不增——实测存量 136、余量 114，
        # 等于允许静默新增 114 条烂指针而不判红。改成按 (文档 :: 路径) 记账：
        # 存量进 tmprefs_baseline.json（只减不增），不在表里的 = 新债 = 判红。
        dead_pairs = set()
        for rel, raws in cited.items():
            for r in raws:
                if r in acked:
                    continue
                if not os.path.exists(os.path.join(WS, r)):
                    dead_pairs.add(rel + " :: " + r)
        if "--write-tmprefs-baseline" in sys.argv:
            # 防「用重建基线偷偷洗白新债」：基线只许收缩。要放大必须显式 --allow-grow 并写理由，
            # 否则改基线就从"修文档的动作"变成了"抹掉证据的动作"（同 --write-baseline 的老坑）。
            _old = load(tb_path) if os.path.exists(tb_path) else {}
            _old_keys = set(_old.get("keys", []))
            _new_pairs = dead_pairs - _old_keys
            if _new_pairs and "--allow-grow" not in sys.argv:
                print("[拒绝写基线] 本次会新增 %d 条存量条目（基线只许收缩）：" % len(_new_pairs))
                for k in sorted(_new_pairs)[:10]:
                    print("   + " + k)
                print("   修文档，别改基线；确属历史遗留须一次性收编，加 --allow-grow 并写明理由。")
                return 1
            payload = {"generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
                       "count": len(dead_pairs),
                       "说明": "存量过程区死链的 (文档 :: 路径) 集合，语义只减不增。修好一条就该变小一条"
                              "（修完重跑本命令精简；新增死链不在本表 = Y9 判红）。行号不进 key。",
                       "keys": sorted(dead_pairs)}
            json.dump(payload, open(tb_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
            print("[过程区死链基线已写入] %s  存量 %d 条" % (cfg["tmprefs"]["baseline_file"], len(dead_pairs)))
        if tr.get("enforce_keys", False):
            prev_tb = load(tb_path) if os.path.exists(tb_path) else {}
            base_keys = set(prev_tb.get("keys", []))
            for key in sorted(dead_pairs - base_keys):
                doc_rel, raw_ref = key.split(" :: ", 1)
                red.append(("Y9", doc_rel,
                            "新增过程区死链 `%s`：docs 不该指过程区，改指归档包，或在该行标〔档〕/〔证失效〕；"
                            "确属存量则跑 --write-tmprefs-baseline（只许收已存在的老债）" % raw_ref))
            resolved = len(base_keys - dead_pairs)
            if resolved:
                yellow.append(("Y9", "docs/**",
                               "存量死链已修复 %d 条，跑 --write-tmprefs-baseline 精简基线（不降不算修完）" % resolved))
        else:
            yellow.append(("Y9", "docs/**",
                           "过程区死链 %d 条（key 基线尚未启用：跑 --write-tmprefs-baseline 固化后新增才判红）"
                           % len(dead_pairs)))

    # R12 front matter「原文路径」可达性（2026-10-27 增）
    # 起因：10-21 文档收口把 dzrobot_v2/docs、dzsmartscout/docs、根 docs/ 的旧文整体搬进
    # docs/history_cache/，但 60 篇库内文档的 来源: / 取代: 仍写着**迁移前**的路径。
    # 后果：读者想追溯原始材料，照 front matter 抄路径一律找不到 —— 实测 151 条里 101 条可解析
    # （同一 basename 就在 history_cache 里）、8 条全库无副本、其余是通配/伪匹配。
    # 判据：库内文档 front matter 的 来源/取代 里出现的路径必须指向**现存**文件；
    # 行尾标〔档〕/〔证失效〕= 已声明无副本，豁免。存量进 fm_paths_baseline.json，只减不增。
    fp = cfg.get("fm_paths", {})
    if fp.get("enabled", True):
        fp_path = os.path.join(HERE, fp.get("baseline_file", "fm_paths_baseline.json"))
        _fm_re = re.compile(r"((?:dzsmartscout|dzrobot_v2|docs)/[^\s、，,；;（）()|" + chr(96) + r"]+?\.(?:md|yaml|json))")
        dead_fm = set()
        for rel in buckets["doc"]:
            try:
                with open(os.path.join(WS, rel), "r", encoding="utf-8", errors="replace") as f:
                    txt = f.read()
            except OSError:
                continue
            if not txt.startswith("---"):
                continue
            raw_head = txt.split("---")[1]
            for line in raw_head.splitlines():
                if not line.startswith(("来源:", "取代:")):
                    continue
                if "〔档〕" in line or "〔证失效〕" in line:
                    continue
                for tok in _fm_re.findall(line):
                    if "*" in tok or "<" in tok or "..." in tok:
                        continue
                    if not os.path.exists(os.path.join(WS, tok)):
                        dead_fm.add(rel + " :: " + tok)
        if "--write-fm-baseline" in sys.argv:
            _first = not os.path.exists(fp_path)   # 首次固化：全部都是"存量"，不算洗白
            _old = load(fp_path) if not _first else {}
            _oldk = set(_old.get("keys", []))
            _grow = dead_fm - _oldk
            if _first:
                _grow = set()
            if _grow and "--allow-grow" not in sys.argv:
                print("[拒绝写基线] 会新增 %d 条 front matter 死路径（基线只许收缩）：" % len(_grow))
                for k in sorted(_grow)[:10]:
                    print("   + " + k)
                print("   改文档（指到 history_cache 现存位置）或在该行标〔档〕，不要改基线。")
                return 1
            with open(fp_path, "w", encoding="utf-8", newline="\n") as f:
                json.dump({"generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
                           "count": len(dead_fm),
                           "说明": "front matter 来源/取代 指向不存在的路径之 (文档 :: 路径) 集合，语义只减不增",
                           "keys": sorted(dead_fm)}, f, ensure_ascii=False, indent=1)
            print("[front matter 路径基线已写入] %s  存量 %d 条" % (fp.get("baseline_file"), len(dead_fm)))
        else:
            _base = set((load(fp_path) if os.path.exists(fp_path) else {}).get("keys", []))
            for key in sorted(dead_fm - _base):
                doc_rel, tok = key.split(" :: ", 1)
                red.append(("R12", doc_rel,
                            "front matter 的来源/取代 指向不存在的路径 %s：改指 docs/history_cache 下的现存位置，"
                            "或在该行标〔档〕（无副本）" % tok))
            _res = len(_base - dead_fm)
            if _res:
                yellow.append(("R12", "docs/**",
                               "front matter 死路径已修 %d 条，跑 --write-fm-baseline 精简（不降不算修完）" % _res))
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

    # Y10 归档卡经验总结（2026-10-25 增 · 六要素规范 §8.1）
    # 规范真源：docs/10_agent提示词/06_工作完结经验总结规范_20261024.md
    # 归档卡正文必须有「经验总结」段，至少含「现象」和「处置」两个要素
    for rel_y in buckets["doc"]:
        if not rel_y.startswith(ARCH):
            continue
        parts_y = rel_y.split("/")
        name_y = parts_y[-1]
        if name_y == "00_索引.md" or name_y.endswith("_按日摘要.md"):
            continue
        if len(parts_y) < 5:
            continue
        ns_y = parts_y[3]
        if ns_y.startswith("_"):
            continue
        try:
            with open(os.path.join(WS, rel_y), "r", encoding="utf-8", errors="replace") as fh_y:
                body_y = fh_y.read()
        except OSError:
            continue
        # 找「经验总结」段：从标题到下一个 ## 标题之间
        # 2026-10-25 修复：原正则 `^## .*?经验总结.*?$...` 因 DOTALL 让 `.*?` 跨行，
        # 会从文件里第一个 `## ` 标题一直吃到「经验总结」标题之前，抓到的是该段**之前**的正文，
        # 导致写得对的卡也被判「缺现象」。改为只匹配标题行 + 捕获其后的段体。
        m_y = re.search(r"^##[^\n]*经验总结[^\n]*$(?:\n(?!## )[\s\S]*?)*?(?=^## |\Z)", body_y, re.MULTILINE)
        if not m_y:
            # 存量豁免（2026-10-26）：归档索引（archive/00_索引.md）一开头就写明
            # 「存量 11 张无该段的卡暂不追溯」，但守卫此前不认这条豁免，
            # 每次体检都报同一批历史卡 → 执行者误当违规，进而可能替历史卡编造「经验总结」
            # （伪造证据，比缺段更糟）。改为显式白名单：写进 doc_policy.json 的
            # y10_legacy_exempt，只对列明的那几张免催办；新归档卡一律照判，不许搪塞。
            if rel_y in set(cfg.get("y10_legacy_exempt", [])):
                continue
            yellow.append(("Y10", rel_y, "归档卡缺「经验总结」段（六要素：现象→机制→位置→处置→证据→落点）"))
            continue
        seg_y = m_y.group(0)
        # 至少含「现象」和「处置」两个要素
        has_phenom = bool(re.search(r"现象|症状|故障|错误|异常|失败|挂了|炸了|不亮|不动|无响应", seg_y))
        has_action = bool(re.search(r"处置|修复|改|加|删|调|重启|回滚|临时|绕过", seg_y))
        if not has_phenom:
            yellow.append(("Y10", rel_y, "经验总结段缺「现象」描述（说了怎么修，没说坏在哪）"))
        if not has_action:
            yellow.append(("Y10", rel_y, "经验总结段缺「处置」描述（说了坏在哪，没说怎么修）"))
        if not has_phenom and not has_action:
            yellow.append(("Y10", rel_y, "经验总结段缺「现象」和「处置」（六要素至少写两要素）"))
    if "--write-baseline" in sys.argv:
        payload = {"generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
                   "count": len(managed_now), "managed": sorted(managed_now)}
        prev = load(bl_path) if os.path.exists(bl_path) else {}
        payload["accept_log"] = prev.get("accept_log", [])
        json.dump(payload, open(bl_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        print("[基线已写入] %s  受管 %d 篇" % (cfg["baseline"], len(managed_now)))

    y9_mode = "按key判红" if cfg.get("tmprefs", {}).get("enforce_keys") else "仅提示(未启用key基线)"
    summary = ("受管 %d（库内 %d / 库外受管 %d / 只读 %d / 豁免 %d / 第三方 %d）｜红线 %d ｜提醒 %d ｜Y9 %s"
               % (len(files), len(buckets["doc"]), len(buckets["managed"]), len(buckets["readonly"]),
                  len(buckets["exempt"]), len(buckets["vendor"]), len(red), len(yellow), y9_mode))
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
            print("\n结论：GREEN —— 库外无新增文档，库内全部登记，无新增过程区死链。")
        else:
            print("\n结论：RED —— %d 条红线。" % len(red))
    return 1 if red else 0

if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")
    sys.exit(main())
