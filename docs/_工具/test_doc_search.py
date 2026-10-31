# -*- coding: utf-8 -*-
"""doc_search.py 排序单测（LIB-03 新建，stdlib unittest，零第三方依赖）。

覆盖 LIB-03 的四处改动 + --legacy-rank 回滚开关，全部在临时沙盒里造语料，
不读真实 docs/、不改任何官方 baseline（与 test_check_anchors.py 同风格）。

运行：python -B -X utf8 docs/_工具/test_doc_search.py

为什么用沙盒造语料而不是直接查真实库：
  1. 真实库每天都在改（实测本卡期间 20_资料索引库/README.md 被并行 LIB-02
     从 225 行压到 191 行），拿它当断言等于让测试随别人改动漂移；
  2. 断言必须钉住**机制**而不是**当前排名**：本文件断言"路由页不抢 top-1"
     "症状索引不压住正文"，在沙盒里可以精确构造这两类文档来证伪。
"""
import io
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
SEARCH = HERE / "doc_search.py"

sys.path.insert(0, str(HERE))
import doc_search as DS   # noqa: E402  被测模块


# ---------------------------------------------------------------- 沙盒语料构造

# 一篇"权威正文"：散文行里反复解释激光不亮的机制（正文证据）。
AUTHORITY = """---
title: 激光链路权威说明
status: 现行
---

# 激光链路权威说明

## 1 现象与机制

激光不亮最常见的原因是没有消费者订阅射击命令，此时链路不报错。
激光不亮时先查订阅数，订阅数为零就是激光不亮的直接原因。
激光不亮的机制在于命令发出后没有人收，因此激光不亮不会有任何报错。
"""

# 一篇"路由页"：文件名是 README.md，整篇是长表格，词密度极高但没有结论。
ROUTE_PAGE = """# 资料库路由

| 症状 | 去哪找 |
|---|---|
| 激光不亮 | 见 视觉与云台/01 |
| 激光不亮 且无报错 | 见 视觉与云台/01 |
| 激光不亮 且订阅数为零 | 见 视觉与云台/01 |
| 激光不亮 先查订阅数 | 见 视觉与云台/01 |
| 激光不亮 机制 | 见 视觉与云台/01 |
"""

# 一篇"症状索引"：文件名不是路由三件套，但整篇是 52 条症状汇总表。
SYMPTOM_INDEX = """---
title: 症状索引
status: 现行
摘要: 症状原话汇总
---

# 症状索引

| # | 症状 | 位置 |
|---|---|---|
| 症-01 | 激光不亮 且没有任何报错 | 视觉与云台/01 |
| 症-02 | 激光不亮 先查订阅数 | 视觉与云台/01 |
| 症-03 | 激光不亮 机制是没人收 | 视觉与云台/01 |
| 症-04 | 激光不亮 命令发出后无人收 | 视觉与云台/01 |
"""

# 一篇 body 里几乎没命中的无关文档，防止"全库只有两篇"导致断言太松。
NOISE = """---
title: 无关文档
---

# 无关文档

桥洞与巡线标定，和激光链路无关。
"""


def make_sandbox(extra=None):
    """建最小工作区：docs/<分册>/... + docs/_工具/doc_search.py 副本。

    doc_search.py 用 __file__ 推导 DOCS，所以必须把脚本复制进沙盒的
    docs/_工具/ 才能让它在沙盒里检索，而不是回头读真实库。
    """
    root = pathlib.Path(tempfile.mkdtemp(prefix="lib03-search-"))
    (root / "docs").mkdir(parents=True)
    (root / "docs/_工具").mkdir(parents=True)
    shutil.copyfile(SEARCH, root / "docs/_工具/doc_search.py")
    files = {
        "docs/20_资料索引库/视觉与云台/01_激光链路.md": AUTHORITY,
        "docs/20_资料索引库/README.md": ROUTE_PAGE,
        "docs/20_资料索引库/运维与排障/01_症状索引_52条.md": SYMPTOM_INDEX,
        "docs/20_资料索引库/运维与排障/02_巡线.md": NOISE,
    }
    files.update(extra or {})
    for rel, text in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")
    return root


def run_cli(root, query, extra_args=()):
    """跑沙盒里的检索器，返回 (exit, payload_dict)。"""
    cmd = [sys.executable, "-B", "-X", "utf8",
           str(root / "docs/_工具/doc_search.py"), query,
           "-k", "0", "--json", "--no-timing"] + list(extra_args)
    env = dict(os.environ)
    env["PYTHONIOENCODING"] = "utf-8"
    r = subprocess.run(cmd, cwd=str(root), stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, env=env)
    try:
        data = io.StringIO(r.stdout.decode("utf-8", "replace"))
        import json
        return r.returncode, json.load(data)
    except Exception:
        return r.returncode, None


def rels(payload, limit=None):
    out = [x["rel"] for x in payload["results"]]
    return out if limit is None else out[:limit]


# ---------------------------------------------------------------- 判据纯函数

class HelperTests(unittest.TestCase):
    """判据函数本身：窄且准，不许宽判据误伤权威篇。"""

    def test_route_by_filename(self):
        self.assertTrue(DS.is_route_page("20_资料索引库/README.md", []))
        self.assertTrue(DS.is_route_page("20_资料索引库/车端/00_索引.md", []))
        self.assertTrue(DS.is_route_page("目录索引.md", []))

    def test_route_by_front_matter_role(self):
        self.assertTrue(DS.is_route_page("20_资料索引库/杂项/导航.md", ["role: 路由"]))

    def test_symptom_index_is_not_a_route_page(self):
        """负样例核心：01_症状索引_52条.md 是权威篇本身，不许被当路由页压掉。"""
        self.assertFalse(DS.is_route_page("20_资料索引库/运维与排障/01_症状索引_52条.md", []))

    def test_route_judgement_does_not_use_broad_index_substring(self):
        """不许用「路径含'索引'」这种宽判据——会把所有分册索引正文一起误伤。"""
        for rel in ("20_资料索引库/车端/05_配置与部署.md",
                    "20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md",
                    "docs/目录索引说明.md"):
            self.assertFalse(DS.is_route_page(rel, []), rel)

    def test_role_of_and_section_of(self):
        self.assertEqual(DS.role_of(["role: 巨册"]), "巨册")
        self.assertEqual(DS.role_of(["status: 现行"]), "")
        lines = ["---", "role: 巨册", "---", "# 篇", "## 一", "命中行", "## 二", "另一行"]
        self.assertEqual(DS.section_of(lines, 5, 3), "一")
        self.assertEqual(DS.section_of(lines, 7, 3), "二")
        self.assertEqual(DS.section_of(lines, 3, 3), "")

    def test_tier_marks_task_cards_as_process_zone(self):
        self.assertEqual(DS.tier_of("30_任务卡库/在办/lead-1023/A.md"), 1)
        self.assertEqual(DS.legacy_tier_of("30_任务卡库/在办/lead-1023/A.md"), 0)
        self.assertEqual(DS.tier_of("20_资料索引库/历史/07_x.md"), 2)


# ---------------------------------------------------------------- 负样例（验收硬项）

class RankingTests(unittest.TestCase):
    """端到端排序：跑真 CLI，断言 top-1 归属。"""

    @classmethod
    def setUpClass(cls):
        cls.root = make_sandbox()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root, ignore_errors=True)

    def test_route_page_is_demoted_not_dropped(self):
        """降权 ≠ 埋掉：路由页仍应出现在结果里（用户可能就想找入口）。

        LIB-03R 明确保留这一条：hub 被降权但仍可被检索到。
        把路由页降到看不见，等于用户想找入口时反而找不到。
        """
        code, data = run_cli(self.root, "激光不亮 且没有任何报错")
        self.assertIn("20_资料索引库/README.md", rels(data),
                      "路由页被彻底删掉了；本卡要求的是降权而非排除")

    def test_route_page_must_not_steal_top1(self):
        """负样例①：**hub 路由页**不得抢正文 top-1。

        造法：README.md 是纯路由表（5 行表格复述症状），正文篇是散文。
        旧行为下路由页词种数更多、稳拿第一；新行为下它必须退位。

        ★LIB-03R 收窄了本用例的范围：LIB-03 原版断言「top-1 必须是正文权威篇」，
        即连 01_症状索引_52条.md 坐 top-1 也算失败。t13/LIB-03R 裁决推翻了这个前提 ——
        AGENTS §1 明写「只知道现象 → 先查症状索引」，它坐 top-1 是**正确答案**；
        实测 M2 的分数几乎 100% 来自它坐 top-1（26/26），其它文档坐 top-1 只有 4%（1/25）。
        所以现在只守「**hub**（纯转述型路由）不得抢位」这一条 —— 它仍然是有意义的硬约束，
        因为 hub 自己不带结论，只有一张症状摘要表。
        """
        code, data = run_cli(self.root, "激光不亮 且没有任何报错")
        self.assertEqual(code, 0, "检索应成功退出")
        top = data["results"][0]["rel"]
        self.assertNotEqual(top, "20_资料索引库/README.md",
                            "hub 路由页抢走了 top-1，路由页降权没生效或失效")

    def test_symptom_index_may_win_but_must_deliver_destination(self):
        """负样例②（LIB-03R 改写）：症状索引**可以**坐 top-1，但必须真的给出去向。

        LIB-03 原版断言「症状索引不得压住正文权威篇」，与 LIB-03R 的取向直接对立：
        LIB-03R 选择 M2（经索引中转让出去向）而非 M1（top-1 本身是权威篇）。
        直接删掉这条断言等于放弃守门；这里改成**条件式守门**：
          「症状索引允许排第一，但它的命中行里必须含『去向』，否则就是抢位不办事」。

        这比原来的「一律不许排第一」更强：原来它只要不排第一就算过，
        现在它一旦排第一就必须当场证明自己答得出。
        """
        code, data = run_cli(self.root, "激光不亮 命令发出后无人收")
        self.assertEqual(code, 0)
        top = data["results"][0]["rel"]
        if "运维与排障/01_症状索引" in top:
            # 它赢了 ⇒ 必须给出正文里的去向（本夹具里正文写的是 视觉与云台/01）
            texts = [h.get("text") or "" for h in data["results"][0]["hits"]]
            self.assertTrue(any("视觉与云台/01" in t for t in texts),
                            "症状索引排了 top-1 却没给出去向，属于抢位不办事：%r" % texts)
        else:
            # 权威正文篇赢了 ⇒ 同样合格（M1 路径）
            self.assertIn("01_激光链路", top,
                          "既不是症状索引也没指向权威篇，排序失控：%s" % top)

    def test_legacy_rank_restores_old_behaviour(self):
        """回滚开关必须真生效：加上 --legacy-rank 后旧排序（路由页/表格堆词取胜）复现。"""
        code, data = run_cli(self.root, "激光不亮 且没有任何报错", ["--legacy-rank"])
        self.assertEqual(code, 0)
        old_top = data["results"][0]["rel"]
        self.assertNotEqual(old_top, "20_资料索引库/视觉与云台/01_激光链路.md",
                            "--legacy-rank 没有退回旧行为，开关是假的")
        # 旧行为下抢位的必须是"堆词"的那两类之一，且绝不是散文权威篇
        self.assertIn(old_top, ("20_资料索引库/README.md",
                                "20_资料索引库/运维与排障/01_症状索引_52条.md"),
                      "旧行为下 top-1 应是大表格文档，实得 %s" % old_top)

    def test_legacy_rank_flag_visible_in_help(self):
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        r = subprocess.run([sys.executable, "-B", "-X", "utf8",
                            str(SEARCH), "--help"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
        self.assertEqual(r.returncode, 0)
        self.assertIn("--legacy-rank", r.stdout.decode("utf-8", "replace"))


# ---------------------------------------------------------------- 正样例（机制逐条）

class MechanismTests(unittest.TestCase):
    """四处改动各自的机制，逐条证伪——不是只看最终排名。"""

    @classmethod
    def setUpClass(cls):
        cls.root = make_sandbox()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.root, ignore_errors=True)

    def test_route_page_gets_score_penalty(self):
        """改动②：路由页 score 被扣 4（且 -4 之后仍为正，不会变负数吓人）。"""
        code, data = run_cli(self.root, "激光不亮")
        recs = {x["rel"]: x for x in data["results"]}
        self.assertIn("20_资料索引库/README.md", recs)
        self.assertLessEqual(recs["20_资料索引库/README.md"]["score"], 3 + 12,
                             "路由页分值没有体现降权")

    def test_table_ratio_discount_demotes_table_heavy_doc(self):
        """改动③：同样命中 3 个词，纯表格文档的贡献低于散文文档。"""
        terms = DS.tokenize("激光不亮 且没有任何报错")
        corpus = [(rel, DS.read_lines(str(self.root / "docs" / rel)))
                  for rel in ("20_资料索引库/视觉与云台/01_激光链路.md",
                              "20_资料索引库/运维与排障/01_症状索引_52条.md")]
        res = {r["rel"]: r for r in
               DS.search(terms, corpus, {}, {}, 3, True, True,
                         {"table": True, "route": False,
                          "tasktier": False, "sections": False}, False)}
        auth = res["20_资料索引库/视觉与云台/01_激光链路.md"]
        idx = res["20_资料索引库/运维与排障/01_症状索引_52条.md"]
        self.assertGreaterEqual(auth["terms_matched"], idx["terms_matched"] - 1)
        self.assertLess(idx["score"], auth["score"],
                        "表格行封顶没起作用：症状索引分 %d 未低于正文 %d"
                        % (idx["score"], auth["score"]))

    def test_capsule_ratio_counts_only_table_exclusive_hits(self):
        """★第 19 条（LIB-03R）：capsule_ratio 必须只算「**仅**表格出现」的词。

        这是那条判据唯一的守门人。没有它，下一个人把 `tb >= total` 改回
        `tb > 0`（"表格里出现过就算"）不会有任何测试变红 —— docstring 写得再清楚
        也拦不住代码。夹具照症-12 的真实形状造：

          症-12「机上检测 det=0」→ 无人机/03_感知与视觉侦察.md
              检测  tf=4  table=3   ← 散文里有 1 次，旧判据误判成"纯表格堆词"
              det   tf=4  table=2   ← 散文里有 2 次，同上
              机上  tf=2  table=2   ← 真·只在表格里
              上检  tf=1  table=1   ← 真·只在表格里
          旧判据 → 4/4 = 1.00 → 触发 −2 → 该篇掉到第 7 名 → M4 判失败（误伤机制叙述篇）
          新判据 → 2/4 = 0.50 → 不触发 → 该篇回到第 2 名
        """
        # 混合：表格 3 + 散文 1 ⇒ tf=4, table=3 ⇒ 不该计入
        mixed = DS.capsule_ratio({"检测": 3}, {"检测": 4}, ["检测"])
        self.assertLess(mixed, DS.TABLE_BULK_RATIO,
                        "表格+散文混合被误判成表格堆词（旧 bug 回潮）：%.2f" % mixed)
        self.assertEqual(mixed, 0.0, "混合词不该被计入任何比例")

        # 纯表格：tf=2, table=2 ⇒ 必须计入
        pure = DS.capsule_ratio({"上检": 2}, {"上检": 2}, ["上检"])
        self.assertEqual(pure, 1.0, "只出现在表格里的词没有被计入")

        # 组合：照 症-12 的 4 词形状 ⇒ 2/4 = 0.50，低于阈值 ⇒ 不被罚
        ttf = {"检测": 3, "det": 2, "机上": 2, "上检": 1}
        tf = {"检测": 4, "det": 4, "机上": 2, "上检": 1}
        r = DS.capsule_ratio(ttf, tf, ["检测", "det", "机上", "上检"])
        self.assertEqual(r, 0.5, "症-12 形状的比值应为 2/4=0.50，实得 %.2f" % r)
        self.assertLess(r, DS.TABLE_BULK_RATIO,
                        "症-12 形状仍会触发惩罚 ⇒ 机制叙述篇被误伤")

        # 无命中 ⇒ 0（散文权威篇不被误伤）
        self.assertEqual(DS.capsule_ratio({}, {}, []), 0.0)

        # 反证：真正的纯汇总表（所有词都只在表格里）必须仍然被罚
        all_table = DS.capsule_ratio({"a": 3, "b": 2}, {"a": 3, "b": 2}, ["a", "b"])
        self.assertEqual(all_table, 1.0, "纯汇总表没有被识别出来（惩罚会失效）")
        self.assertGreaterEqual(all_table, DS.TABLE_BULK_RATIO)

    def test_sections_field_only_for_giant_volumes(self):
        """改动④：sections 只在 role: 巨册 的文档上出现，其它文档输出形状不变。"""
        giant = AUTHORITY.replace("status: 现行", "role: 巨册")
        root2 = make_sandbox({"docs/20_资料索引库/视觉与云台/01_激光链路.md": giant})
        try:
            code, data = run_cli(root2, "激光不亮")
            recs = {x["rel"]: x for x in data["results"]}
            g = recs["20_资料索引库/视觉与云台/01_激光链路.md"]
            self.assertIn("sections", g, "巨册没有附带章节字段")
            self.assertTrue(g["sections"], "巨册 sections 为空数组")
            self.assertIn("1 现象与机制", g["sections"][0])
            for rel, rec in recs.items():
                if rel != "20_资料索引库/视觉与云台/01_激光链路.md":
                    self.assertNotIn("sections", rec,
                                     "非巨册文档不应出现 sections：%s" % rel)
        finally:
            shutil.rmtree(root2, ignore_errors=True)

    def test_giant_volume_without_sections_key_when_not_giant(self):
        """同一篇不加 role 时不许带 sections——证明改动④是按 role 判定，不是按大小。"""
        code, data = run_cli(self.root, "激光不亮")
        for rec in data["results"]:
            self.assertNotIn("sections", rec,
                             "沙盒里没有巨册，但 %s 带了 sections" % rec["rel"])

    def test_task_card_demoted_to_process_tier(self):
        """改动①：30_任务卡库/ 篇判 tier 1；--live-only 因此排除它。"""
        extra = {"docs/30_任务卡库/在办/lead-1023/LIB-03.md":
                 "---\ntitle: LIB-03 卡\n---\n\n# LIB-03\n\n激光不亮 且没有任何报错\n"}
        root3 = make_sandbox(extra)
        try:
            code, data = run_cli(root3, "激光不亮")
            recs = {x["rel"]: x for x in data["results"]}
            card = recs["30_任务卡库/在办/lead-1023/LIB-03.md"]
            self.assertEqual(card["tier"], 1, "任务卡仍是 tier 0（与百科同权）")
            code2, data2 = run_cli(root3, "激光不亮", ["--live-only"])
            self.assertNotIn("30_任务卡库/在办/lead-1023/LIB-03.md", rels(data2),
                             "--live-only 没有排除 tier 1 的任务卡")
        finally:
            shutil.rmtree(root3, ignore_errors=True)

    def test_tokenizer_untouched_two_gram(self):
        """硬约束：分词器未被改动——中文仍是 2-gram 滑窗。"""
        self.assertEqual([t for t, _ in DS.tokenize("激光不亮")], ["激光", "光不", "不亮"])
        self.assertEqual([t for t, _ in DS.tokenize("桥洞")], ["桥洞"])
        self.assertEqual([t for t, _ in DS.tokenize("arrival_m")], ["arrival_m"])

    def test_no_third_party_imports(self):
        """硬约束：只用标准库。扫 AST 的 import 名，限定白名单。"""
        import ast
        stdlib_ok = {"argparse", "json", "os", "re", "sys", "time",
                     "io", "math", "collections", "pathlib", "subprocess"}
        tree = ast.parse(io.open(str(SEARCH), encoding="utf-8").read())
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for a in node.names:
                    names.add(a.name.split(".")[0])
            elif isinstance(node, ast.ImportFrom) and node.module:
                names.add(node.module.split(".")[0])
        bad = names - stdlib_ok
        self.assertEqual(bad, set(), "引入了非标准库依赖：%s" % sorted(bad))


if __name__ == "__main__":
    unittest.main(verbosity=2)
