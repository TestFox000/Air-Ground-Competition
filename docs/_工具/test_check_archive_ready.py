# -*- coding: utf-8 -*-
"""check_archive_ready.py 的自测（不依赖任何真实卡）。

跑法：python -B -X utf8 docs/_工具/test_check_archive_ready.py

覆盖：
  L1 齐全通过（四项全过 ⇒ exit 0）
  L2 缺经验总结段
  L3 有段但缺「处置」要素（只有现象）
  L4 有落点但没有反向指针（落点写了、读者追不到）
  L5 缺文档变更段
  L6 status 还是 in_progress（没改归档态）
  L7 文档变更段存在但正文为空
  S1 与 Y10 同源：本工具三个正则必须与 check_doc_policy.py 的 Y10 段**逐字节一致**
  S2 不读 y10_legacy_exempt：即使卡在豁免名单里，本工具照判（用假卡验证工具源码里没有该键）

每个用例都用临时目录造假卡，跑完即删。
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.dirname(HERE)
WS = os.path.dirname(DOCS)
TOOL = os.path.join(HERE, "check_archive_ready.py")
POLICY = os.path.join(HERE, "check_doc_policy.py")

FULL = """---
卡号: LIB-TEST-20261031
owner: lib-guard
ns: lib-1031
agent: lib-guard
树节点: LIB-1
status: 已归档
写域: _tmp_session/LIB-TEST/
---

# LIB-TEST-20261031 · 夹具卡

## 1 执行记录

做了点事。

## 7.1 经验总结（归档必写，六要素 + 落点）
| # | 要素 | 填写 |
|---|---|---|
| 1 现象 | 门禁判据事后才跑，卡归档时漏写经验总结，只能事后补写 |
| 2 机制 | 归档动作靠人手搬文件，没有任何机器拦截；Y10 只在归档**后**报黄线 |
| 3 位置 | `docs/_工具/check_doc_policy.py:552` |
| 4 处置 | 搬卡前先跑 `python docs/_工具/check_archive_ready.py <卡>`，exit 0 才动 |
| 5 证据 | 〔实〕本工具自测 L2 用例复现 |
| 6 落点 | 坑库（`docs/20_资料索引库/赛事与治理/12_卡库命名空间与任务树细则_20261025.md` §12.5） |

## 8 文档变更
| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/_工具/check_archive_ready.py` | 归档前置门禁 |
"""


def variant(base, old, new):
    assert old in base, "夹具模板里找不到要替换的片段：%r" % old[:40]
    return base.replace(old, new)


def run_tool(args, cwd=None):
    """跑工具，返回 (exit_code, stdout)。用 -B 防写 pyc，与验收命令同形态。"""
    cmd = [sys.executable, "-B", "-X", "utf8", TOOL] + args
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    p = subprocess.run(cmd, cwd=cwd or WS, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=env)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


class ArchiveReadyTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lib-ar-test-")
        self.cards = []

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def make(self, name, text):
        p = os.path.join(self.tmp, name)
        with open(p, "w", encoding="utf-8") as f:
            f.write(text)
        self.cards.append(p)
        return p

    # ---------------------------------------------------------------- L1
    def test_L1_complete_card_passes(self):
        """L1 齐全卡 ⇒ 四项全 PASS、exit 0。"""
        p = self.make("complete.md", FULL)
        rc, out = run_tool([p])
        self.assertEqual(rc, 0, "齐全卡应 exit 0，实际 %d\n%s" % (rc, out))
        for item in ("A 经验总结段", "B 落点+反向指针", "C status", "D 文档变更"):
            self.assertIn("[PASS] " + item, out)
        self.assertIn("结论：PASS", out)
        self.assertNotIn("[FAIL]", out)

    # ---------------------------------------------------------------- L2
    def test_L2_missing_experience_section_fails(self):
        """L2 整段经验总结缺失 ⇒ A / B 双双 FAIL，exit 1。"""
        body = FULL.split("## 7.1 经验总结")[0] + FULL.split("## 8 文档变更")[1].join(
            ["## 8 文档变更", ""])
        p = self.make("no_exp.md", body)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "缺经验总结应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] A 经验总结段", out)
        self.assertIn("缺「## …经验总结」段", out)
        self.assertIn("[FAIL] B 落点+反向指针", out)

    # ---------------------------------------------------------------- L3
    def test_L3_section_without_action_fails(self):
        """L3 有段、有现象、但缺「处置」⇒ A FAIL、exit 1（与 Y10 同口径）。"""
        text = variant(
            FULL,
            "| 2 机制 | 归档动作靠人手搬文件，没有任何机器拦截；Y10 只在归档**后**报黄线 |",
            "| 2 机制 | 归档动作靠人手搬文件，没有任何机器拦截 |")
        text = variant(
            text,
            "| 4 处置 | 搬卡前先跑 `python docs/_工具/check_archive_ready.py <卡>`，exit 0 才动 |",
            "| 4 处置 | （留空） |")
        # 把「处置」这个词也去掉，只剩「现象」一侧的词表命中
        text = text.replace("| 4 处置 |", "| 4 动作 |")
        text = text.replace("| 5 证据 |", "| 5 凭据 |")
        text = text.replace("| 6 落点 |", "| 6 归宿 |")
        text = text.replace("Y10 只在归档", "Y10 只在归档")
        p = self.make("no_action.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "缺处置应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] A 经验总结段", out)
        self.assertIn("缺「处置」", out)

    # ---------------------------------------------------------------- L4
    def test_L4_landing_without_pointer_fails(self):
        """L4 写了落点但无反向指针（没有路径/D 编号/§节号）⇒ B FAIL、exit 1。"""
        text = variant(
            FULL,
            "| 6 落点 | 坑库（`docs/20_资料索引库/赛事与治理/12_卡库命名空间与任务树细则_20261025.md` §12.5） |",
            "| 6 落点 | 坑库 |")
        p = self.make("no_pointer.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "落点无指针应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] B 落点+反向指针", out)
        self.assertIn("缺反向指针", out)

    # ---------------------------------------------------------------- L5
    def test_L5_missing_doc_change_fails(self):
        """L5 缺「文档变更」段 ⇒ D FAIL、exit 1。"""
        text = FULL.split("## 8 文档变更")[0].rstrip() + "\n"
        p = self.make("no_dc.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "缺文档变更应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] D 文档变更", out)
        self.assertIn("缺「文档变更」段", out)

    # ---------------------------------------------------------------- L6
    def test_L6_live_status_fails(self):
        """L6 status 仍 in_progress ⇒ C FAIL、exit 1。"""
        p = self.make("live_status.md", variant(FULL, "status: 已归档", "status: in_progress"))
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "status 未改应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] C status", out)

    # ---------------------------------------------------------------- L7
    def test_L7_empty_doc_change_body_fails(self):
        """L7 「文档变更」段存在但正文空 ⇒ D FAIL（有段≠有内容）。"""
        text = FULL.split("## 8 文档变更")[0] + "## 8 文档变更\n"
        p = self.make("empty_dc.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "空文档变更应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("有「文档变更」段但正文为空", out)

    # ---------------------------------------------------------------- S1
    def test_S1_regexes_identical_to_Y10(self):
        """S1 同源守卫：本工具三个正则必须与 check_doc_policy.py 的 Y10 段逐字节一致。

        为什么要有这条：卡面要求「与 Y10 同口径，复用同一正则，不要另写一套」，
        但 Y10 的代码**一行都不许改**（历史卡豁免体系依赖它）。所以两边只能是两份字面量，
        靠这条测试把它们钉住：谁改了其中一份而没改另一份，测试立刻红。
        """
        with open(TOOL, encoding="utf-8") as fh:
            tool_src = fh.read()
        with open(POLICY, encoding="utf-8") as fh:
            pol_src = fh.read()

        def grab(src, pattern, label):
            m = re.search(pattern, src)
            self.assertIsNotNone(m, "%s 里找不到 %s" % (label, pattern))
            return m.group(1)

        tool_exp = grab(tool_src, r'EXP_SECTION_RE = re\.compile\((r"[\s\S]*?"), re\.MULTILINE\)', "check_archive_ready.py")
        pol_exp = grab(pol_src, r'm_y = re\.search\((r"[\s\S]*?"), body_y, re\.MULTILINE\)', "check_doc_policy.py")

        tool_ph = grab(tool_src, r'PHENOM_RE = re\.compile\((r"[\s\S]*?")\)', "check_archive_ready.py")
        pol_ph = grab(pol_src, r'has_phenom = bool\(re\.search\((r"[\s\S]*?"), seg_y\)\)', "check_doc_policy.py")

        tool_ac = grab(tool_src, r'ACTION_RE = re\.compile\((r"[\s\S]*?")\)', "check_archive_ready.py")
        pol_ac = grab(pol_src, r'has_action = bool\(re\.search\((r"[\s\S]*?"), seg_y\)\)', "check_doc_policy.py")

        self.assertEqual(tool_exp, pol_exp, "段标题正则与 Y10 不一致")
        self.assertEqual(tool_ph, pol_ph, "「现象」词表与 Y10 不一致")
        self.assertEqual(tool_ac, pol_ac, "「处置」词表与 Y10 不一致")

    # ---------------------------------------------------------------- S2
    def test_S2_ignores_y10_legacy_exempt(self):
        """S2 本工具不读 y10_legacy_exempt —— 源码里连这个键名都不出现。

        理由：那是给历史卡的一次性免催办；本工具判的是「即将归档的卡」，新卡一律照判。
        如果哪天有人图省事把豁免读进来，这条测试立刻红。
        """
        with open(TOOL, encoding="utf-8") as fh:
            src = fh.read()
        # docstring 里会提到这个键名（说明「不读它」），只查**代码**里有没有引用
        code = re.sub(r'r?""".*?"""', "", src, flags=re.S)
        code = re.sub(r"#.*", "", code)
        self.assertNotIn("y10_legacy_exempt", code,
                         "check_archive_ready.py 的代码里出现了 y10_legacy_exempt —— 不许读历史豁免")

    # ---------------------------------------------------------------- S3
    def test_S3_default_mode_and_exitcodes(self):
        """S3 默认模式无待判卡 ⇒ exit 0；不存在路径 ⇒ exit 2（用法错误不算判失败）。"""
        rc, out = run_tool([])
        self.assertEqual(rc, 0, "默认模式应 exit 0，实际 %d\n%s" % (rc, out))
        rc2, out2 = run_tool([os.path.join(self.tmp, "不存在的卡.md")])
        self.assertEqual(rc2, 2, "不存在的卡应 exit 2，实际 %d\n%s" % (rc2, out2))
        self.assertIn("[用法错误]", out2)

    # ================================================================
    # B 项重写（LIB-06R）：证据行 ≠ 落点。正反两向都要钉。
    # ================================================================
    def test_B1_evidence_line_is_not_landing(self):
        """B1 负：只写「证据：<路径>:行号」、没有落点标签 ⇒ B 必须 FAIL。

        这是 captain 对抗夹具 C1 的形态，也是真实库 RECT-AIM/RECT-REVIEW 的形态。
        旧判据把「含 路径:行号」当落点，假阳性率实测 50%。
        """
        text = FULL.split("## 7.1 经验总结")[0] + """## 7.1 经验总结

- 现象：检索页抢 top-1，症状查询全落到 README。
- 处置：加路由页降权，score -4。

证据：`docs/20_资料索引库/车端/05_配置与部署.md:128`

## 8 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 修改 | `docs/_工具/check_archive_ready.py` | 加落点判据 |
"""
        p = self.make("evidence_line.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "证据行冒充落点应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] B 落点+反向指针", out)
        self.assertIn("无**显式落点标签**", out)

    def test_B2_landing_and_pointer_on_separate_lines_passes(self):
        """B2 正：落点单独一行、指针在**紧邻下一行** ⇒ B 必须 PASS（不得误杀）。

        这是 captain 夹具 C2，真实卡常见写法。它是防止我把判据做成一刀切的反向验证。
        """
        text = FULL.split("## 7.1 经验总结")[0] + """## 7.1 经验总结

- 现象：归档卡漏写经验总结，事后补写等于编造。
- 处置：把判据前移到搬卡之前。

落点：坑库
见 `docs/20_资料索引库/运维与排障/01_症状索引_52条.md`

## 8 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/_工具/check_archive_ready.py` | 归档前置门禁 |
"""
        p = self.make("landing_split.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 0, "落点与指针分行是合法写法，应 exit 0，实际 %d\n%s" % (rc, out))
        self.assertIn("[PASS] B 落点+反向指针", out)

    def test_B3_inline_landing_label_passes(self):
        """B3 正：行内标签 `…；落点 <路径>`（RECT-LINK 的真实写法）⇒ PASS。"""
        text = FULL.split("## 7.1 经验总结")[0] + """## 7.1 经验总结

- **现象**：旧 checker 无条件 grep，判红误报。
- **处置**：用真实 DroneTask 双状态对照复现；落点 `docs/20_资料索引库/空地链路/`（待 Lead 提炼）。

## 8 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 修改 | `docs/_工具/check_archive_ready.py` | 落点判据 |
"""
        p = self.make("inline_landing.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 0, "行内落点标签应 exit 0，实际 %d\n%s" % (rc, out))
        self.assertIn("[PASS] B 落点+反向指针", out)

    def test_B4_placeholder_pointer_fails(self):
        """B4 负：落点只写占位 `D-xxx`/「待 Lead」⇒ 仍 FAIL（读者追不到）。"""
        text = FULL.split("## 7.1 经验总结")[0] + """## 7.1 经验总结

- 现象：在办里残留已归档卡。
- 处置：加 Y12 黄线。

落点：台账 D-xxx（待 Lead 定）

## 8 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 修改 | `docs/_工具/check_doc_policy.py` | 加 Y12 |
"""
        p = self.make("placeholder.md", text)
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "占位指针应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("占位符", out)

    # ================================================================
    # D 项：文档变更要「有信息量」，不是「非空」
    # ================================================================
    def test_D1_single_word_doc_change_fails(self):
        """D1 负：文档变更只写一个「有」字 ⇒ FAIL（captain 夹具 C4）。"""
        p = self.make("dc_one_word.md",
                      variant(FULL, """## 8 文档变更
| 动作 | 文档 | 说明 |
|---|---|---|
| 新增 | `docs/_工具/check_archive_ready.py` | 归档前置门禁 |""",
                              "## 8 文档变更\n\n- 有"))
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "一字文档变更应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] D 文档变更", out)
        self.assertIn("敷衍词", out)

    def test_D2_detailed_doc_change_passes(self):
        """D2 正：写清动作 + 对象 + 说明 ⇒ PASS。"""
        rc, out = run_tool([self.make("dc_full.md", FULL)])
        self.assertEqual(rc, 0, "完整文档变更应 exit 0，实际 %d\n%s" % (rc, out))
        self.assertIn("[PASS] D 文档变更", out)

    def test_D3_padding_without_object_fails(self):
        """D3 负：凑够字数但没有可核对对象（无路径/文件名/D 编号/行数）⇒ FAIL。"""
        p = self.make("dc_padding.md",
                      variant(FULL, """| 新增 | `docs/_工具/check_archive_ready.py` | 归档前置门禁 |""",
                              "| 无 | 本次改动已完成，一切正常，没有其他需要说明的情况 |"))
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "无对象的凑字应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("可核对对象", out)

    # ================================================================
    # E 项（跨卡逐字重复）：真实库已中招，必须能检出
    # ================================================================
    PLAGIARIZED = """## 经验总结

- 现象：改一个变量时同时改了三个，最后说不清是哪个起作用，归因完全失效。
- 处置：一次只改一个变量，改完立刻跑判据留证据，连续两轮无提升就停下来写清停因。
"""

    def test_E1_cross_card_identical_fails(self):
        """E1 负：两张卡经验总结正文**逐字相同** ⇒ 两张都 FAIL（captain 夹具 D5）。

        旧门禁逐卡独立判，对抄袭完全无感——这是规范 §1 要防的编造，
        而且真实库已有 4 组（LS 线 4 张 / P2 线 4 张 / TRAJ-FRAME 3 张 / TRAJ-GEOM 3 张）。
        """
        a = self.make("dup_a.md", FULL.split("## 7.1 经验总结")[0] + self.PLAGIARIZED +
                      "\n\n## 8 文档变更\n\n| 修改 | `docs/a.py` | 甲 |\n")
        b = self.make("dup_b.md", FULL.split("## 7.1 经验总结")[0] + self.PLAGIARIZED +
                      "\n\n## 8 文档变更\n\n| 修改 | `docs/b.py` | 乙 |\n")
        rc, out = run_tool([a, b])
        self.assertEqual(rc, 1, "逐字相同应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] E 跨卡不重复", out)
        self.assertIn("逐字相同", out)

    def test_E2_distinct_summaries_pass(self):
        """E2 正：两张卡正文不同 ⇒ E 项都 PASS（不得误杀）。

        注意 FULL 本身自带唯一的正文，所以再加一张不同内容的卡即可。
        """
        a = self.make("uniq_a.md", FULL)
        b = self.make("uniq_b.md",
                      FULL.split("## 7.1 经验总结")[0] + """## 7.1 经验总结

| # | 要素 | 填写 |
|---|---|---|
| 1 现象 | 数字守卫自愈 33 条后复查又冒出 7 条，因为别人同时给源文档加了段 |
| 2 机制 | 行号是位置依赖的；任何人插一段，其后全部下移，基线不是稳定参照 |
| 3 位置 | `docs/_工具/check_numbers.py` |
| 4 处置 | 自愈后必须人工复读目标行；一对多歧义会选错（rail_span_m 选了注释行） |
| 5 证据 | 〔实〕t3 实测 33→7→0 |
| 6 落点 | 百科 `docs/20_资料索引库/车端/12_可调参数速查_按场景.md` §0.3 |

## 8 文档变更

| 动作 | 文档 | 说明 |
|---|---|---|
| 修改 | `docs/_工具/check_numbers.py` | 无 |
""".replace("| 修改 | `docs/_工具/check_numbers.py` | 无 |",
                "| 修改 | `docs/_工具/check_numbers.py` | 自愈后加人工复读纪律 |"))
        rc, out = run_tool([a, b])
        self.assertEqual(rc, 0, "两卡正文不同应 exit 0，实际 %d\n%s" % (rc, out))
        self.assertIn("[PASS] E 跨卡不重复", out)

    def test_E3_duplicate_detected_from_single_real_card(self):
        """E3 负：**只拿一张真实抄袭卡单独跑**也必须 FAIL（靠全库比对命中）。

        为什么钉这条：若只在「本次待判集」里比，单卡永远 PASS，
        抄袭者逐张跑就能一路绿灯 —— 这正是夹具 D5 骗过旧门禁的形态。
        这里用**真实库里已确认逐字重复**的 LS 线卡（×4），只传一张。
        """
        hits = []
        for dp, _d, fns in os.walk(os.path.join(WS, "docs/30_任务卡库")):
            for n in fns:
                if n == "LS-GUARD.md":
                    hits.append(os.path.relpath(os.path.join(dp, n), WS))
        if not hits:
            self.skipTest("真实库里已无 LS-GUARD.md，改用别的方式验全库比对")
        rc, out = run_tool([hits[0].replace("\\", "/")])
        self.assertIn("[FAIL] E 跨卡不重复", out,
                      "单张真实抄袭卡应被全库比对命中\n%s" % out)
        self.assertIn("逐字相同", out)

    def test_E3b_real_library_duplicate_groups_found(self):
        """E3b 正（普查）：`--all-live --json` 必须报出真实库的逐字重复组。

        这条把「判据在真实库上确实生效」钉成机器判据，而不是靠人读输出。
        """
        with open(TOOL, encoding="utf-8") as fh:
            pass  # 仅确保文件可读
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        p = subprocess.run([sys.executable, "-B", "-X", "utf8", TOOL, "--all-live", "--json"],
                           cwd=WS, capture_output=True, text=True, encoding="utf-8",
                           errors="replace", env=env)
        import json as _json
        data = _json.loads(p.stdout)
        groups = data["dup_groups"]
        self.assertGreaterEqual(len(groups), 4,
                                "真实库应报出 ≥4 组逐字重复，实际 %d" % len(groups))
        total = sum(len(g["cards"]) for g in groups)
        self.assertGreaterEqual(total, 14, "重复组应覆盖 ≥14 张卡，实际 %d" % total)

    def test_E4_short_body_not_flagged(self):
        """E4 正：正文 < DUP_MIN_CHARS 的短空壳不参与跨卡比对（避免把表头/骨架判成抄袭）。

        空壳该由 A/B 项判（缺现象/缺处置/缺落点），不该用「重复」这个名头——
        否则报错信息会把人引向「改几个字就不重复了」这种错误处置。
        """
        short = "## 经验总结\n\n- 现象：无\n- 处置：无\n"
        a = self.make("short_a.md", FULL.split("## 7.1 经验总结")[0] + short +
                      "\n## 8 文档变更\n\n| 修改 | `docs/x.py` | 甲 |\n")
        b = self.make("short_b.md", FULL.split("## 7.1 经验总结")[0] + short +
                      "\n## 8 文档变更\n\n| 修改 | `docs/y.py` | 乙 |\n")
        rc, out = run_tool([a, b])
        self.assertNotIn("[FAIL] E 跨卡不重复", out, "短正文不该被判跨卡重复\n%s" % out)

    # ================================================================
    # 未改动的旧判据：C 项（status）
    # ================================================================
    def test_C1_live_status_still_fails(self):
        """C1 负（回归）：status 仍 in_progress ⇒ C FAIL。旧判据不许在重写中被弄丢。"""
        p = self.make("live2.md", variant(FULL, "status: 已归档", "status: in_progress"))
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "status 未改应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] C status", out)

    def test_A1_missing_section_still_fails(self):
        """A1 负（回归）：缺经验总结段 ⇒ A/B FAIL。"""
        p = self.make("noexp.md", FULL.split("## 7.1 经验总结")[0] +
                      "## 8 文档变更\n\n| 修改 | `docs/z.py` | 说明 |\n")
        rc, out = run_tool([p])
        self.assertEqual(rc, 1, "缺经验总结应 exit 1，实际 %d\n%s" % (rc, out))
        self.assertIn("[FAIL] A 经验总结段", out)


if __name__ == "__main__":
    unittest.main(verbosity=2)
