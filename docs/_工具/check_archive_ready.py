# -*- coding: utf-8 -*-
"""归档前置门禁（archive-readiness gate）—— 给「经验总结」制度补上执行点。

为什么存在
==========
规则早就齐了（`docs/10_agent提示词/06_工作完结经验总结规范_20261024.md` 六要素），
但**归档动作没有任何机器拦截**：卡从 `在办/` 搬到 `归档/` 全靠人手，
漏写经验总结只有 `check_doc_policy.py` 的 Y10 **事后**报黄线。
后果是「归档时漏写 → 事后补写 → 补写就是编造」，而规范自己写着「没有证据的经验 = 过程记录，不进库」。
本工具把判据**前移到搬卡之前**：跑不过就说明这张卡还不该归档。

配套的 Y12 黄线（`check_doc_policy.py`）
=====================================
本工具的「默认面」是 `在办/` 下 `status: 已归档` 的卡；同一批卡在 `check_doc_policy.py` 里
由 **Y12** 报黄线（`在办目录残留已归档卡`），并同时被 **R4** 判红（R4 说「status 值域越界」，
Y12 说「位置放错、下一步该做什么」）。
> 编号备忘：这条黄线原拟叫 Y11，但 **Y11 早被「更新字段格式」占用**（`check_doc_policy.py:208`）。
> 加新编号前先 `grep -n '"Y[0-9]*"' docs/_工具/check_doc_policy.py` 查已用号，别撞号。

判什么（四项，逐条独立判）
==========================
  A 经验总结段  有 `## ...经验总结` 段，且段内含六要素里的**至少「现象」+「处置」**
                —— 与 Y10 **同一套正则**（见下方 SYNC 段），不是另写一套
  B 落点+反向指针  段内有落点（三选一：坑库 / 百科 / 台账 D-xxx），且给出可利用的反向指针
                （具体文件路径，或 D 编号，或分册 §节号）
  C status      卡面 status 已是归档态（默认「已归档」）
  D 文档变更     卡面有「文档变更」段且内容非空（R9 只在**归档后**判，这里提前判）

用法
====
  python docs/_工具/check_archive_ready.py                 # 默认：扫 在办/ 下 status 已是归档态的卡
  python docs/_工具/check_archive_ready.py <卡路径> ...     # 指定卡（在办/归档/临时夹具都行）
  python docs/_工具/check_archive_ready.py --all-live      # 预览：对 在办/ 全部卡跑一遍「够不够格归档」
  python docs/_工具/check_archive_ready.py --json

退出码：0 = 全部 PASS；1 = 有 FAIL；2 = 用法/路径错误。

设计纪律
========
* **不读** `doc_policy.json` 的 `y10_legacy_exempt`。那是给历史卡的一次性免催办，
  本工具判的是「**即将**归档的卡」，新卡一律照判，不许拿历史豁免搪塞。
* **不给存量卡补写经验总结**。判据前移的目的恰恰是**不要事后补写**：
  没执行完的卡没有「可迁移结论」，硬写就是伪造证据。本工具只报事实，不生成内容。
* 落点判据刻意**不要求**命中某个白名单路径：落点合法形态有四种（分册 §节、
  坑库「来源: 卡号」、台账 D 编号、百科篇链接），白名单会把合法写法判死。
  这里只判「有没有一个可追的反向指针」，宁可漏判也不误杀。

SYNC（与 Y10 同源，靠自测守）
============================
`test_check_archive_ready.py` 会把本文件的 `EXP_SECTION_RE / PHENOM_RE / ACTION_RE`
与 `check_doc_policy.py` 里 Y10 段的**字面量逐字节比对**，不一致即测试失败。
这样既满足「复用同一正则」，又不必去改 Y10 的代码（Y10 段一行都不许动）。
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
WS = os.path.dirname(DOCS)

LIVE = "docs/30_任务卡库/在办/"
ARCH = "docs/30_任务卡库/归档/"

# ---- SYNC 区：以下三项必须与 check_doc_policy.py 的 Y10 段逐字节一致（自测守）----
# 出处：check_doc_policy.py:576（段标题）、:589（现象词表）、:590（处置词表）
# Y10 段 2026-10-25 的修复注释说明为什么标题正则不能写成 `^## .*?经验总结.*?$` + DOTALL：
# `.*?` 跨行会从文件里第一个 `## ` 一直吃到「经验总结」标题之前（抓到的是该段之前的正文），
# 于是写得对的卡也被判缺现象。这里沿用修正后的写法。
EXP_SECTION_RE = re.compile(r"^##[^\n]*经验总结[^\n]*$(?:\n(?!## )[\s\S]*?)*?(?=^## |\Z)", re.MULTILINE)
PHENOM_RE = re.compile(r"现象|症状|故障|错误|异常|失败|挂了|炸了|不亮|不动|无响应")
ACTION_RE = re.compile(r"处置|修复|改|加|删|调|重启|回滚|临时|绕过")

# 归档态取值：主值 + 英文与卡库旧写法（都算「已声明归档」）
ARCHIVED_STATES = ("已归档", "archived", "completed")
DEFAULT_ARCHIVED = "已归档"

# ---- D 项（文档变更）最低信息量：2026-10-31 LIB-06R ----
# 旧版只判「非空」，写一个「有」字就过（对抗夹具 C4 实测 PASS）。
DOC_CHANGE_MIN_CHARS = 8      # 去标点/空白后的最少字数
DOC_CHANGE_FILLER_RE = re.compile(r"无|略|待补|TODO|待定|同前|见上|—|-|（空）")
# 裸文件名（`check_archive_ready.py` 这种没反引号、没目录前缀的写法）也算可核对对象
BARE_FILENAME_RE = re.compile(r"[A-Za-z0-9_./\-]+\.(?:py|md|ya?ml|json|sh|ps1|txt|cpp|cc|h)"
                              r"|\d+-[A-Z][A-Z0-9-]+")   # 或卡号形态 2026-10-31_XXX

# ---- 跨卡逐字重复检测（D-A1）：规范化正文 ≥ 此长度才纳入比对 ----
DUP_MIN_CHARS = 40

# 落点三选一的关键词（仅用于报「写了哪种落点」，**本身不作通过判据**）
LANDING_KINDS = (
    ("台账", re.compile(r"\bD-\d{3}\b|决策台账")),
    ("坑库", re.compile(r"坑库|症状索引|十坑|坑库汇总")),
    ("百科", re.compile(r"百科|20_资料索引库|资料索引库")),
)

# ---- B 项（落点）判定：2026-10-31 LIB-06R 重写 ----
# 旧版把「某行含 `路径:行号` 或 D 编号」等同于落点 —— 这是错的。
# 证据行（`证据:` / `位置:` / `详见:` / `来源:`）天然带 `路径:行号`，它不是落点；
# 实测 4 张 B 通过的卡里 2 张是这个原因假通过的（RECT-AIM 命中「场景…D-033」口径注释、
# RECT-REVIEW 命中「位置：…config.py:695-754」证据行）。
#
# 新判据 = 「显式落点标签」+「同一行（或紧邻下一行）有可追的反向指针」：
#   ① 行内出现落点标签：`落点` / `去向` / `落库` / `沉淀到` / `归档到`
#      —— 真实库 15 张卡在用这个写法（RECT-LINK 用行内 `；落点 …`、DBG-LINK 用 `**落点**：…`、
#         六要素表用 `| 6 落点 | …`），所以它是有据可依的既有约定，不是我发明的新格式。
#   ② 该行（或紧邻下一行）要给出反向指针：文件路径 / D 编号 / §节号。
#      「紧邻下一行」是为了不误杀 C2 那种「落点：坑库」换行再写 `见 …` 的合法写法。
#   ③ 标签行若以证据类前缀开头（证据/位置/详见/来源/证据等级），一律不算落点。
LANDING_LABEL_RE = re.compile(r"落点|去向|落库|沉淀到|归档到")
EVIDENCE_PREFIX_RE = re.compile(r"^\s*(?:[-*|>#]+\s*)?(?:\*\*)?(?:证据|位置|详见|来源|证据等级|依据)"
                                r"(?:\*\*)?\s*[:：]")
# 落点标签前若紧贴「机制」「现象」「处置」等要素词，说明这个标签是在解释**别的东西**的落点
# （例：RECT-AIM 的 `…D-033` 出现在场景口径注释里，压根没有「落点」二字，这里只作双保险）。
LANDING_PLACEHOLDER_RE = re.compile(r"D-xxx|待定|待 Lead|待补|TODO|（空）|\(\s*\)")

# 反向指针：具体文件路径（含反引号内的相对/绝对路径）、或 D 编号、或「§N」节号
POINTER_PATH_RE = re.compile(r"`[^`]*?(?:\.md|\.py|\.ya?ml|\.json)[^`]*?`|(?:docs|dzrobot_v2|_tmp_session|artifacts)/[^\s，。；)|]+")
POINTER_D_RE = re.compile(r"\bD-\d{3}\b")
POINTER_SECTION_RE = re.compile(r"§\s*[0-9]+(?:\.[0-9]+)?")


def read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def front_matter(path):
    """与 check_doc_policy.py:112 front_matter 同口径（只读前 4000 字节，不解析 YAML）。"""
    text = read_text(path)
    if text is None:
        return {}
    text = text[:4000]
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


def section_body(body, keyword_re):
    """取标题含 keyword_re 的 `## ` 段（到下一个 `## ` 为止）。找不到返回 None。

    与 Y10 的段提取同构，但用于「文档变更」这类标题不含固定词的段时需要额外判据，
    故这里按关键字构造同一形态的正则。
    """
    pat = re.compile(r"^##[^\n]*(?:" + keyword_re + r")[^\n]*$(?:\n(?!## )[\s\S]*?)*?(?=^## |\Z)",
                     re.MULTILINE)
    m = pat.search(body)
    return m.group(0) if m else None


def landing_lines_of(seg):
    r"""从经验总结段里挑出**真正的落点行**，返回 [(行号, 行文本)]。

    2026-10-31 LIB-06R 重写：旧版「含路径:行号即落点」把证据行也认成落点（实测假阳性率 50%）。
    现在只认**带显式落点标签**的行，且排除证据类前缀。三种真实写法都覆盖：
        `- 落点：坑库（`文件` §12.5）`        ← 行首标签
        `…；落点 \`docs/…\`（待 Lead 提炼）`  ← 行内标签（RECT-LINK 那种）
        `| 6 落点 | 坑库 / 百科 / 台账 D-xxx |` ← 六要素表行
    """
    out = []
    lines = seg.splitlines()
    for i, ln in enumerate(lines):
        if ln.startswith("##"):
            continue                  # 段标题本身可能就叫「…六要素 + 落点」，那不是落点行
        if not LANDING_LABEL_RE.search(ln):
            continue
        if EVIDENCE_PREFIX_RE.search(ln):
            continue                      # 「证据：…」「位置：…」不是落点
        out.append((i, ln))
    return out, lines


def has_pointer(text):
    """该文本里是否有可追的反向指针（文件路径 / D 编号 / §节号）。"""
    return bool(POINTER_PATH_RE.search(text) or POINTER_D_RE.search(text)
                or POINTER_SECTION_RE.search(text))


def check_card(path):
    """对一张卡跑四项判据。返回 [(项名, ok, 说明)]，顺序固定。"""
    body = read_text(path)
    if body is None:
        return [("读取", False, "文件读不出（不存在或无权限）：" + path)]

    results = []

    # ---- A 经验总结段（与 Y10 同口径）----
    m = EXP_SECTION_RE.search(body)
    if not m:
        results.append(("A 经验总结段", False,
                        "缺「## …经验总结」段（六要素：现象→机制→位置→处置→证据→落点）"))
    else:
        seg = m.group(0)
        has_phenom = bool(PHENOM_RE.search(seg))
        has_action = bool(ACTION_RE.search(seg))
        if has_phenom and has_action:
            results.append(("A 经验总结段", True, "有段，且含「现象」+「处置」"))
        else:
            missing = []
            if not has_phenom:
                missing.append("现象")
            if not has_action:
                missing.append("处置")
            results.append(("A 经验总结段", False,
                            "有段，但缺「%s」（Y10 同口径：至少两要素）" % "+".join(missing)))

    # ---- B 落点 + 反向指针（只在 A 有段时才判，否则 A 已经说明原因）----
    if not m:
        results.append(("B 落点+反向指针", False, "无经验总结段 ⇒ 落点无从谈起"))
    else:
        seg = m.group(0)
        land, lines = landing_lines_of(seg)
        if not land:
            results.append(("B 落点+反向指针", False,
                            "段内无**显式落点标签**（需写「落点」/「去向」/「落库」；"
                            "证据行、位置行不算落点——它们天然带 `路径:行号`）"))
        else:
            # 指针可在标签同一行，或**紧邻下一行**（C2 那种「落点：坑库」换行再「见 …」）
            ok_any, bad = False, []
            for idx, ln in land:
                window = ln + ("\n" + lines[idx + 1] if idx + 1 < len(lines) else "")
                # 占位符只在**没有真指针**时才判负。「落点 `docs/…`（待 Lead 提炼）」
                # 是 RECT-LINK 的真实写法：路径已经给了，读者追得到，「待提炼」只是后续动作。
                if has_pointer(window) and not (
                        LANDING_PLACEHOLDER_RE.search(ln)
                        and not POINTER_PATH_RE.search(ln) and not POINTER_D_RE.search(ln)):
                    ok_any = True
                    break
                bad.append(ln.strip()[:70])
            if ok_any:
                kinds = [name for name, rx in LANDING_KINDS
                         if any(rx.search(l) for _i, l in land)]
                results.append(("B 落点+反向指针", True,
                                "有显式落点标签（%s）+ 同行/下一行有反向指针" % ("/".join(kinds) or "未分类")))
            else:
                results.append(("B 落点+反向指针", False,
                                "落点行缺反向指针或写了占位符（`D-xxx`/待定/待 Lead）——读者追不到："
                                + "、".join("「%s」" % x for x in bad[:2])))

    # ---- C status 已是归档态 ----
    st = front_matter(path).get("status", "")
    if st in ARCHIVED_STATES:
        results.append(("C status", True, "status=「%s」已是归档态" % st))
    else:
        results.append(("C status", False,
                        "status=「%s」，归档态应为「%s」（先改状态再搬卡）"
                        % (st or "空", DEFAULT_ARCHIVED)))

    # ---- D 文档变更段：非空 **且** 有信息量 ----
    # 2026-10-31 LIB-06R：旧版只判「非空」，写一个「有」字就过（对抗夹具 C4 实测 PASS）。
    # 现在要求：去掉标点/空白后 ≥ DOC_CHANGE_MIN_CHARS，且不含敷衍词。
    seg_dc = section_body(body, "文档变更")
    if seg_dc is None:
        results.append(("D 文档变更", False, "缺「文档变更」段（AGENTS §8.8 / R9 归档后会判）"))
    else:
        dc_body = seg_dc.split("\n", 1)[-1]
        core = re.sub(r"[\s\-—:：|*`>\[\]()（）【】，。；、]+", "", dc_body)
        filler = DOC_CHANGE_FILLER_RE.search(dc_body)
        if not core:
            results.append(("D 文档变更", False, "有「文档变更」段但正文为空"))
        elif filler and len(core) < DOC_CHANGE_MIN_CHARS:
            results.append(("D 文档变更", False,
                            "「文档变更」正文只有敷衍词「%s」（去标点 %d 字 < %d）——要写清改了什么"
                            % (filler.group(0), len(core), DOC_CHANGE_MIN_CHARS)))
        elif len(core) < DOC_CHANGE_MIN_CHARS:
            results.append(("D 文档变更", False,
                            "「文档变更」正文信息量不足（去标点 %d 字 < %d）：至少写清改动对象"
                            "（路径/卡号/文件名）" % (len(core), DOC_CHANGE_MIN_CHARS)))
        elif not (has_pointer(dc_body)
                  or BARE_FILENAME_RE.search(dc_body)          # 裸文件名也算可核对对象
                  or re.search(r"[「『\"][^」』\"]{2,}[」』\"]", dc_body)
                  or re.search(r"\d+\s*行|\+\d+|-{1,2}\d+", dc_body)):
            results.append(("D 文档变更", False,
                            "「文档变更」写了 %d 字但没有任何可核对对象（无路径/D 编号/文件名/行数）"
                            "——无法据此判断改了什么" % len(core)))
        else:
            results.append(("D 文档变更", True, "有段、%d 字、含可核对对象" % len(core)))

    return results


def exp_body_norm(seg):
    """经验总结段正文的规范化形式（去标题行、去空白/标点/表格符号），用于跨卡重复检测。"""
    txt = "\n".join(ln for ln in seg.splitlines() if not ln.startswith("##"))
    return re.sub(r"[\s|`*\-—:：()\[\]（）【】，。；、]+", "", txt)


def find_duplicate_groups(paths):
    """跨卡逐字重复检测（D-A1）。

    为什么必须单独做：四项判据都是**逐卡独立**判的，看不见「多张卡抄同一段」。
    而规范 §1 写着「没有证据的经验 = 过程记录，不进库」——抄一段通用话正是编造，
    而且它在真实库已经发生了（2026-10-31 实测在办 26 张有经验总结段 ⇒ 只有 16 种正文）。

    口径：规范化后长度 ≥ DUP_MIN_CHARS 的正文，若在 ≥2 张卡里逐字相同 ⇒ 整组判重。
    阈值取 40 字：低于此的多半是六要素表头/空壳，不属于「抄一段经验」。
    返回 {规范化文本: [rel, ...]}，只含 ≥2 张的组。
    """
    groups = {}
    for rel in paths:
        body = read_text(os.path.join(WS, rel)) or ""
        m = EXP_SECTION_RE.search(body)
        if not m:
            continue
        norm = exp_body_norm(m.group(0))
        if len(norm) < DUP_MIN_CHARS:
            continue
        groups.setdefault(norm, []).append(rel)
    return {k: v for k, v in groups.items() if len(v) >= 2}


def default_targets():
    """默认面：`在办/` 下 status 已是归档态的卡 —— 即「已声明归档却还赖在在办」的那批。"""
    out = []
    for dp, dns, fns in os.walk(os.path.join(WS, LIVE)):
        dns[:] = [d for d in dns if not d.startswith(".")]
        for n in sorted(fns):
            if not n.lower().endswith(".md"):
                continue
            rel = os.path.relpath(os.path.join(dp, n), WS).replace("\\", "/")
            if n in ("00_树.md", "README.md", "00_索引.md"):
                continue
            if front_matter(os.path.join(WS, rel)).get("status", "") in ARCHIVED_STATES:
                out.append(rel)
    return out


def all_live_targets():
    """`--all-live`：`在办/` 下全部卡 —— 用于「这批里谁已经够格归档」的预览。"""
    out = []
    for dp, dns, fns in os.walk(os.path.join(WS, LIVE)):
        dns[:] = [d for d in dns if not d.startswith(".")]
        for n in sorted(fns):
            if not n.lower().endswith(".md"):
                continue
            if n in ("00_树.md", "README.md", "00_索引.md"):
                continue
            out.append(os.path.relpath(os.path.join(dp, n), WS).replace("\\", "/"))
    return sorted(out)


def census(targets):
    """对 `在办/` 全量做一次**只统计不判**的普查。

    为什么只统计不判：存量 141 张在办卡大多**还在执行中**，
    规范定义「写经验总结」的时机是「卡执行完、准备归档」。给未完成的卡补写 = 伪造证据，
    所以这批**不该**让本工具判失败 —— 判失败只会逼人去补写。
    普查的价值是给出「制度缺口有多大」的数，而不是催办。
    """
    n = {"cards": len(targets), "exp": 0, "phenom_action": 0, "landing_ptr": 0,
         "archived_status": 0, "doc_change": 0, "all_four": 0}
    status_hist = {}
    for rel in targets:
        abs_p = os.path.join(WS, rel)
        st = front_matter(abs_p).get("status", "") or "（空）"
        status_hist[st] = status_hist.get(st, 0) + 1
        res = {name: ok for name, ok, _w in check_card(abs_p)}
        if res.get("A 经验总结段"):
            n["exp"] += 1
        # 段内「现象+处置」两要素是否齐（A 的细粒度版本）
        body = read_text(abs_p) or ""
        m = EXP_SECTION_RE.search(body)
        if m and PHENOM_RE.search(m.group(0)) and ACTION_RE.search(m.group(0)):
            n["phenom_action"] += 1
        if res.get("B 落点+反向指针"):
            n["landing_ptr"] += 1
        if res.get("C status"):
            n["archived_status"] += 1
        if res.get("D 文档变更"):
            n["doc_change"] += 1
        if all(res.values()):
            n["all_four"] += 1
    return n, status_hist


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    as_json = "--json" in argv
    all_live = "--all-live" in argv

    known = {"--json", "--all-live"}
    for a in argv[1:]:
        if a.startswith("--") and a not in known:
            print("[用法错误] 未知参数 %s；支持 --json / --all-live" % a)
            return 2

    if args:
        targets = []
        for a in args:
            rel = a.replace("\\", "/")
            abs_p = rel if os.path.isabs(rel) else os.path.join(WS, rel)
            if not os.path.exists(abs_p):
                print("[用法错误] 卡不存在：%s" % a)
                return 2
            targets.append(rel)
        mode = "指定卡（%d 张）" % len(targets)
    elif all_live:
        targets = all_live_targets()
        mode = "在办全量预览（%d 张）" % len(targets)
    else:
        targets = default_targets()
        mode = "在办里已声明归档的卡（%d 张）" % len(targets)

    rows = []
    for rel in targets:
        abs_p = rel if os.path.isabs(rel) else os.path.join(WS, rel)
        rows.append((rel, check_card(abs_p)))

    # ---- 跨卡逐字重复（D-A1）：跨卡判据，所以在逐卡结果之外单独算一遍 ----
    # 只在「待判集里 ≥2 张同文」时命中。指定单卡跑时也会去全库找同文，
    # 否则「拿一张抄袭卡单独跑」会永远 PASS —— 那正是对抗夹具 D5 的形态。
    # 只给 1 张时也要能发现抄袭 ⇒ 与「全库」比。全库 = 待判集 ∪ 在办 ∪ 归档，
    # 这样临时夹具（_tmp_session 下的假卡）也能和自己同文的那张对上，
    # 不至于「逐张单独跑就一路绿灯」（夹具 D5 正是这么骗过旧门禁的）。
    if len(targets) >= 2:
        dup_source = targets
    else:
        extra = [r for r in all_live_targets() if r not in targets]
        for dp, dns, fns in os.walk(os.path.join(WS, ARCH)):
            dns[:] = [d for d in dns if not d.startswith(".")]
            for n in sorted(fns):
                if n.lower().endswith(".md"):
                    extra.append(os.path.relpath(os.path.join(dp, n), WS).replace("\\", "/"))
        dup_source = targets + extra
    # 临时夹具可能放在工作区外（测试用 tempfile），那种路径塞进来会读不到；
    # 读不到的卡在 find_duplicate_groups 里被跳过，不影响判定。

    dup_groups = find_duplicate_groups(dup_source)
    dup_of_card = {}
    for norm, rels in dup_groups.items():
        for r in rels:
            dup_of_card.setdefault(r, []).append((len(norm), rels))

    def final_rows():
        for rel, res in rows:
            out = list(res)
            if rel in dup_of_card:
                lens, rels = dup_of_card[rel][0]
                out.append(("E 跨卡不重复", False,
                            "经验总结正文与另外 %d 张卡**逐字相同**（规范化 %d 字）：%s"
                            " —— 规范 §1「没有证据的经验 = 过程记录」，抄一段通用话就是编造，"
                            "必须逐卡写自己的真结论"
                            % (len(rels) - 1, lens,
                               "、".join(os.path.basename(x) for x in rels if x != rel)) ))
            else:
                out.append(("E 跨卡不重复", True,
                            "正文在全库 %d 张里唯一" % len(dup_source)))
            yield rel, out

    rows = list(final_rows())
    failed = [r for r in rows if not all(ok for _n, ok, _w in r[1])]
    per_item_fail = {}
    for _rel, res in rows:
        for name, ok, _w in res:
            if not ok:
                per_item_fail[name] = per_item_fail.get(name, 0) + 1

    if as_json:
        print(json.dumps({
            "mode": mode,
            "scanned": len(rows),
            "pass": len(rows) - len(failed),
            "fail": len(failed),
            "by_item": per_item_fail,
            "dup_groups": [{"norm_chars": len(k), "cards": v} for k, v in sorted(
                dup_groups.items(), key=lambda kv: -len(kv[1]))],
            "cards": [{"path": rel, "pass": all(ok for _n, ok, _w in res),
                       "items": [{"item": n, "ok": ok, "why": w} for n, ok, w in res]}
                      for rel, res in rows],
        }, ensure_ascii=False, indent=1))
        return 0 if not failed else 1

    print("=" * 74)
    print("归档前置门禁  %s" % mode)
    print("=" * 74)
    if not rows:
        print("\n本次没有需要判的卡。")
        print("  · 默认面 = 在办/ 下 status 已是归档态的卡（当前 0 张，说明没人把'已归档'的卡留在在办）")
        print("  · 归档前请显式指定卡路径：python docs/_工具/check_archive_ready.py <卡路径>")
        print("  · 想预览'在办里谁已经够格归档'：加 --all-live")
        print("\n—— 在办全量普查（只统计不判红；未完成的卡不该被催写经验总结）——")
        census_targets = all_live_targets()
        cn, hist = census(census_targets)
        print("  在办卡 %d 张；status 分布：%s"
              % (cn["cards"], "、".join("%s %d" % kv for kv in sorted(hist.items(), key=lambda x: -x[1]))))
        print("  有「经验总结」段        %3d / %3d  (%5.1f%%)"
              % (cn["exp"], cn["cards"], 100.0 * cn["exp"] / max(cn["cards"], 1)))
        print("  段内含「现象」+「处置」  %3d / %3d  (%5.1f%%)"
              % (cn["phenom_action"], cn["cards"], 100.0 * cn["phenom_action"] / max(cn["cards"], 1)))
        print("  有落点+反向指针        %3d / %3d  (%5.1f%%)"
              % (cn["landing_ptr"], cn["cards"], 100.0 * cn["landing_ptr"] / max(cn["cards"], 1)))
        print("  有「文档变更」段        %3d / %3d  (%5.1f%%)"
              % (cn["doc_change"], cn["cards"], 100.0 * cn["doc_change"] / max(cn["cards"], 1)))
        print("  四项全过（够格归档）    %3d / %3d  (%5.1f%%)"
              % (cn["all_four"], cn["cards"], 100.0 * cn["all_four"] / max(cn["cards"], 1)))
        dg = find_duplicate_groups(census_targets)
        print("  跨卡逐字重复            %3d 组 / %3d 张  (≥%d 字同文)"
              % (len(dg), sum(len(v) for v in dg.values()), DUP_MIN_CHARS))
        for _k, _v in sorted(dg.items(), key=lambda kv: -len(kv[1])):
            print("      ×%d  %s  (%d 字)" % (len(_v), "、".join(os.path.basename(x)[:34] for x in _v), len(_k)))
        print("\n结论：PASS —— 无待判卡（0 张）；普查仅为给出缺口规模，不催办。")
        return 0

    for rel, res in rows:
        ok_all = all(ok for _n, ok, _w in res)
        print("\n%s  %s" % ("PASS" if ok_all else "FAIL", rel))
        for name, ok, why in res:
            print("   [%s] %-16s %s" % ("PASS" if ok else "FAIL", name, why))

    print("\n" + "-" * 74)
    print("统计：扫描 %d 张｜PASS %d｜FAIL %d" % (len(rows), len(rows) - len(failed), len(failed)))
    if per_item_fail:
        print("未通过项分布：" + "、".join("%s %d 张" % (k, v) for k, v in sorted(per_item_fail.items())))
    if failed:
        print("\n结论：FAIL —— %d 张卡还不该归档。**不要靠补写经验总结过关**：" % len(failed))
        print("  没执行完的卡没有可迁移结论，硬写就是伪造证据（规范 §1「没有证据的经验 = 过程记录」）。")
        print("  正确处置：把卡做完 → 有了真结论再写 → 重跑本工具 → exit 0 才搬卡。")
        return 1
    print("\n结论：PASS —— 可以搬卡到 归档/<ns>/，并按规范 §2.1 三步收尾。")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(errors="replace")
    sys.exit(main(sys.argv))
