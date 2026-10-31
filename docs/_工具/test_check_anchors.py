# -*- coding: utf-8 -*-
"""锚点守卫单测（G1 新建，stdlib unittest，零第三方依赖）。

覆盖 D-060 第 3 条的两套机制共 14 条正负样例（对应 G0 §4.3 的 A1-A7 / M1-M5 / B1-B2）：
  机制甲 = 同文档唯一显式 Markdown 迁移别名，只在直接解析失败（red）之后尝试；
  机制乙 = 同行完整规范词〔证失效〕+ 同行配对闭合才豁免（不新增未闭合前缀）。

运行：python -B -X utf8 docs/_工具/test_check_anchors.py
说明：全部在 --root 沙盒里跑，不触碰真实工作区，不改任何官方 baseline。
"""
import hashlib
import json
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
GUARD = HERE / "check_anchors.py"
POLICY = HERE / "doc_policy.json"
BASELINE = HERE / "anchor_baseline.json"
OFFICIAL = ["anchor_baseline.json", "doc_policy_baseline.json", "numbers_baseline.json",
            "tmprefs_baseline.json", "fm_paths_baseline.json", "doc_policy.json"]


def make_sandbox(docs):
    "建最小工作区：docs/00_项目总览 + docs/_工具(策略/基线) + artifacts 真文件。"
    root = pathlib.Path(tempfile.mkdtemp(prefix="g1-anchors-"))
    (root / "docs/00_项目总览").mkdir(parents=True)
    (root / "docs/_工具").mkdir(parents=True)
    shutil.copyfile(POLICY, root / "docs/_工具/doc_policy.json")
    shutil.copyfile(BASELINE, root / "docs/_工具/anchor_baseline.json")
    (root / "artifacts/aim-mv-20261024").mkdir(parents=True)
    (root / "artifacts/aim-mv-20261024/mvlog.csv").write_text("a,b\n1,2\n", encoding="utf-8")
    (root / "artifacts/aim-mv-20261024/mvrec.py").write_text("# ok\n", encoding="utf-8")
    for name, text in docs.items():
        f = root / "docs/00_项目总览" / name
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(text, encoding="utf-8")
    return root


def run_guard(root):
    "在沙盒根跑守卫，返回 (exit, parsed_json_or_None)。"
    cmd = [sys.executable, "-B", "-X", "utf8", str(GUARD), "--json", "--root", str(root)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        d = json.loads(r.stdout.decode("utf-8", "replace"))
    except Exception:
        d = None
    return r.returncode, d


def red_targets(d):
    return sorted(x["target"] for x in (d or {}).get("red_new", []))


def cleanup(root):
    shutil.rmtree(root, ignore_errors=True)


class MechanismA(unittest.TestCase):
    "机制甲：同文档唯一显式 Markdown 迁移别名（仅在直接解析失败后）。"

    def test_A1_direct_hit_means_alias_not_consulted(self):
        "A1 正：直连解析成功时，同行的另一条链接不改变 live 结果。"
        root = make_sandbox({"a.md": (
            "见 `artifacts/aim-mv-20261024/mvlog.csv:1` 与 "
            "[dead](artifacts/aim-mv-20261024/nope.csv)。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["alias_doc"], 0)
        finally:
            cleanup(root)

    def test_A2_unique_alias_resolves(self):
        "A2 正：唯一别名 + 目标存在 ⇒ 不再判红，计入 alias_doc。"
        root = make_sandbox({"a.md": (
            "回放 [mvlog.csv](../../artifacts/aim-mv-20261024/mvlog.csv) 与 `mvlog.csv`：参考。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["alias_doc"], 1, d)
            self.assertEqual(d["alias_doc"][0]["found"], "artifacts/aim-mv-20261024/mvlog.csv")
        finally:
            cleanup(root)

    def test_A3_alias_target_missing_still_red(self):
        "A3 负：别名目标不存在 ⇒ 仍红。"
        root = make_sandbox({"a.md": (
            "回放 [mvrec.py](artifacts/aim-mv-20261024/nope.py) 与 `mvrec.py`。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvrec.py"], d)
            self.assertEqual(rc, 1)
        finally:
            cleanup(root)

    def test_A4_two_targets_is_red_not_arbitrated(self):
        "A4 负：同一旧引用两条不同目标 ⇒ 歧义、仍红（不择优）。"
        root = make_sandbox({"a.md": (
            "[mvlog.csv](artifacts/aim-mv-20261024/mvlog.csv) "
            "[mvlog.csv](artifacts/aim-mv-20261024/mvrec.py) 与 `mvlog.csv`。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvlog.csv"], d)
            self.assertEqual(d["counts"]["alias_doc"], 0)
        finally:
            cleanup(root)

    def test_A5_target_escapes_workspace_rejected(self):
        "A5 负：目标越过工作区（../）⇒ 拒、仍红。"
        root = make_sandbox({"a.md": (
            "[mvlog.csv](../../../outside/mvlog.csv) 与 `mvlog.csv`。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvlog.csv"], d)
        finally:
            cleanup(root)

    def test_A6_alias_in_other_doc_does_not_help(self):
        "A6 负：别名写在另一篇 ⇒ 对本篇无效（无全局注册表）。"
        root = make_sandbox({
            "a.md": "[mvlog.csv](../../artifacts/aim-mv-20261024/mvlog.csv) 链接在此篇。\n",
            "b.md": "回放 `mvlog.csv`：参考。\n"})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvlog.csv"], d)
        finally:
            cleanup(root)

    def test_A7_label_must_match_exactly(self):
        "A7 负：旧引用与裸名不完全相等（大小写差）⇒ 不解析。"
        root = make_sandbox({"a.md": (
            "[MVLOG.csv](artifacts/aim-mv-20261024/mvlog.csv) 与 `mvlog.csv`。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvlog.csv"], d)
        finally:
            cleanup(root)


class MechanismB(unittest.TestCase):
    "机制乙：同行完整规范词〔证失效〕且同行配对闭合才豁免。"

    def test_M1_exact_token_without_reason_exempts(self):
        """M1 正（已按 D-060 更正）：同行精确规范词 + 理由写在括号外 ⇒ 豁免。

        原期待写的是 〔证失效：原因〕（理由在括号内）应豁免 —— 那与 D-060
        「只精确〔证失效〕、原因在括号外」相反，是测试期待本身错了。
        原错误期待的字节证据见 Lead 保存的旧测试快照；本文件按规范更正，不删不 skip。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 理由写在括号外：工作区 0 命中、两仓 git log 0 命中。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(rc, 0)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_M1a_fullwidth_colon_reason_is_not_exempt(self):
        "M1a 负：〔证失效：理由〕 全角冒号 + 理由在括号内 ⇒ **非规范，仍红**（不得扩族）。"
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效：工作区 0 命中、不可验证〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
            self.assertEqual(rc, 1)
        finally:
            cleanup(root)

    def test_M1b_halfwidth_colon_reason_is_not_exempt(self):
        "M1b 负：〔证失效:理由〕 半角冒号 ⇒ 同样非规范，仍红。"
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效:工作区 0 命中、不可验证〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_M1c_empty_colon_reason_is_not_exempt(self):
        "M1c 负：〔证失效：〕 冒号后空理由 ⇒ 非规范，仍红。"
        root = make_sandbox({"a.md": "原物 `gone.py` 〔证失效：〕 理由在外面\n"})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_M1d_ordinary_example_not_treated_as_marker(self):
        """M1d 负：正文里把标记字形当**普通示例**举出来（非豁免声明）⇒ 不得豁免。

        原版断言 `assertIn("gone.py", red_targets(d) + ["gone.py"])` 是**恒真**：期望值被
        预先塞进被测列表，任何结果都通过（trim-doc 实测四组输入全 True，连空列表也过）。
        现改为**可证伪**断言：直接比较实际 red 集合与 dead_marked 计数。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py`；示例写法：在锚点行写〔证失效〕即可，理由写在括号外。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
            self.assertEqual(rc, 1)
        finally:
            cleanup(root)

    def test_M2_unclosed_marker_does_not_exempt(self):
        "M2 负：〔证失效 无闭合 〕 ⇒ 不豁免。"
        root = make_sandbox({"a.md": "原物 `gone.py` 〔证失效：没说清\n"})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
        finally:
            cleanup(root)

    def test_M3_marker_on_next_line_does_not_exempt(self):
        "M3 负：标记写在下一行 ⇒ 不豁免（不跨行）。"
        root = make_sandbox({"a.md": (
            "原物 `gone.py`\n〔证失效：原因在下一行〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
        finally:
            cleanup(root)

    def test_M4_marker_does_not_change_live(self):
        "M4 负：目标真实存在时，行内标记不改变 live 判定。"
        root = make_sandbox({"a.md": (
            "`artifacts/aim-mv-20261024/mvlog.csv:1` 〔证失效：故意误标〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 0)
        finally:
            cleanup(root)

    def test_M5_dang_marker_does_not_cover_next_line(self):
        "M5 负：〔档〕 不得跨行生效。"
        root = make_sandbox({"a.md": (
            "`gone.py` 〔档〕\n\n另一段 `gone2.py` 无标记。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py", "gone2.py"], d)
        finally:
            cleanup(root)


class SideEffects(unittest.TestCase):
    "B1/B2：官方 baseline 跑前跑后不得被写，且工具不接受 --accept。"

    def test_B1_official_baselines_byte_identical(self):
        "B1 正：官方 baseline 文件跑前跑后逐字节相同。"
        before = {n: hashlib.sha256((HERE / n).read_bytes()).hexdigest()
                  for n in OFFICIAL if (HERE / n).exists()}
        root = make_sandbox({"a.md": "回放 `mvlog.csv`：参考。\n"})
        try:
            run_guard(root)
        finally:
            cleanup(root)
        after = {n: hashlib.sha256((HERE / n).read_bytes()).hexdigest()
                 for n in OFFICIAL if (HERE / n).exists()}
        self.assertEqual(before, after)
        self.assertIn("anchor_baseline.json", before)

    def test_B2_no_write_baseline_or_accept(self):
        "B2 负：--write-baseline / --accept 未被调用，沙盒基线也不得被写。"
        src = GUARD.read_text(encoding="utf-8")
        self.assertNotIn("--accept", src)
        root = make_sandbox({"a.md": "回放 `mvlog.csv`。\n"})
        try:
            before = (root / "docs/_工具/anchor_baseline.json").read_bytes()
            run_guard(root)
            after = (root / "docs/_工具/anchor_baseline.json").read_bytes()
            self.assertEqual(before, after)
        finally:
            cleanup(root)


class SelfRepair(unittest.TestCase):
    """自纠负例（P1/P1b/P2/P3）：对应 Lead 冻结的 ca38b 坏实现里的三个真实缺陷。
    每条都必须在新实现下被拒绝，不得为了「局部门 0」放宽。"""

    def test_P1_unclosed_qualified_plus_other_bracket_stays_red(self):
        "P1 负：〔证失效：没闭合 + 同行另有 〔别的〕 ⇒ 不得豁免（闭合符必须紧跟规范词）。"
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效：没闭合 另有 〔别的〕 在同行\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_P1b_fake_glyph_not_exempt(self):
        "P1b 负：伪字形 〔证失效XX〕 不是完整规范词 ⇒ 不得豁免。"
        root = make_sandbox({"a.md": (
            "原物 `gone2.py` 〔证失效XX〕 理由写在外面\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone2.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_M1b_canonical_without_reason_exempts(self):
        "M1b 正：〔证失效〕 完整规范词 + 紧随闭合，理由写在括号外 ⇒ 豁免（D-060 第3条口径）。"
        root = make_sandbox({"a.md": (
            "原物 `gone3.py` 〔证失效〕 理由写在括号外\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(rc, 0)
        finally:
            cleanup(root)

    def test_S1_colon_reason_halfwidth_is_negative(self):
        """S1 负（原 M1c 期待已更正）：〔证失效：理由〕 半角冒号 ⇒ 非规范，仍红。

        我上一版把这条写成「应豁免」，理由是「G0 §4.2 也这么写」——但 G0 的冒号写法
        是已被否决/替代的新族提案，不是已部署旧行为，不能以向后兼容把它带回来。
        按 D-060 更正为负例；原错误期待的字节证据由 Lead 保存，不删不 skip。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone4.py` 〔证失效：工作区 0 命中、不可验证〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone4.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_S2_colon_reason_fullwidth_is_negative(self):
        "S2 负（原 M1d 期待已更正）：〔证失效：理由〕 全角冒号 ⇒ 同样非规范，仍红。"
        root = make_sandbox({"a.md": (
            "原物 `gone5.py` 〔证失效：原因在括号内，全角冒号〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone5.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_P2_symlink_escape_rejected(self):
        "P2 负：别名目标是指向工作区外的符号链接 ⇒ 必须拒绝（realpath containment）。"
        import os
        root = make_sandbox({"a.md": ""})
        outside = pathlib.Path(tempfile.mkdtemp(prefix="g1-outside-"))
        try:
            (outside / "secret.txt").write_text("SECRET\n", encoding="utf-8")
            (root / "artifacts").mkdir(parents=True, exist_ok=True)
            link = root / "artifacts" / "escape.txt"
            os.symlink(str(outside / "secret.txt"), str(link))
            (root / "docs/00_项目总览/a.md").write_text(
                "回放 [raise.txt](../../artifacts/escape.txt) 与 `raise.txt`\n", encoding="utf-8")
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["raise.txt"], d)
            self.assertEqual(d["counts"]["alias_doc"], 0, d)
            self.assertEqual(rc, 1)
        finally:
            cleanup(root)
            shutil.rmtree(outside, ignore_errors=True)

    def test_P3_alias_out_of_range_line_still_red(self):
        "P3 负：别名命中但行号 9999 落不进目标（3 行）⇒ 别名不成立、仍红（不把 exists 当免检）。"
        root = make_sandbox({"a.md": (
            "回放 [mvlog.csv](../../artifacts/aim-mv-20261024/mvlog.csv) 与 `mvlog.csv:9999`\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["mvlog.csv"], d)
            self.assertEqual(d["counts"]["alias_doc"], 0, d)
        finally:
            cleanup(root)

    def test_P3b_alias_in_range_line_resolves(self):
        "P3b 正：别名命中且行号 2 能落进 3 行目标 ⇒ 正常解析、无 warn。"
        root = make_sandbox({"a.md": (
            "回放 [mvlog.csv](../../artifacts/aim-mv-20261024/mvlog.csv) 与 `mvlog.csv:2`\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["alias_doc"], 1, d)
            self.assertEqual(d["counts"]["warn"], 0, d)
        finally:
            cleanup(root)

    def test_F1_unc_external_link_rejected(self):
        "F1 负：UNC/外链目标先拒 ⇒ 仍红。"
        root = make_sandbox({"a.md": (
            "[x](file://server/share/x.csv) 与 `x.csv`\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["x.csv"], d)
            self.assertEqual(d["counts"]["alias_doc"], 0, d)
        finally:
            cleanup(root)


class FieldSemantics(unittest.TestCase):
    """D-060 原 CLI/JSON 字段语义：别名成功**不是**「已标失效」。

    自纠前 summary 把 alias_deads 混进「已标失效」，而 JSON.dead_marked 只数 deads
    ⇒ 零标记 fixture 下文字 1 / JSON 0 分叉。本组锁死两者同义同数、alias 单列。
    """

    def _run_pair(self, root):
        rj = subprocess.run([sys.executable, "-B", "-X", "utf8", str(GUARD), "--json", "--root", str(root)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        rt = subprocess.run([sys.executable, "-B", "-X", "utf8", str(GUARD), "--root", str(root)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return json.loads(rj.stdout.decode("utf-8", "replace")), rt.stdout.decode("utf-8", "replace")

    def test_F2_alias_zero_marker_fields_do_not_diverge(self):
        "F2 正：零标记 + 别名命中 ⇒ 文字「已标失效」== JSON.dead_marked，alias 单列。"
        import re
        root = make_sandbox({"m.md": (
            "回放 [mvlog.csv](../../artifacts/aim-mv-20261024/mvlog.csv) 与 `mvlog.csv`\n")})
        try:
            d, txt = self._run_pair(root)
            line = next(l for l in txt.splitlines() if "锚点守卫" in l)
            m_dead = re.search(r"已标失效\s*(\d+)", line)
            m_alias = re.search(r"别名解析\s*(\d+)", line)
            self.assertIsNotNone(m_dead, line)
            self.assertIsNotNone(m_alias, line)
            self.assertEqual(int(m_dead.group(1)), d["counts"]["dead_marked"], line)
            self.assertEqual(int(m_alias.group(1)), d["counts"]["alias_doc"], line)
            self.assertEqual(d["counts"]["dead_marked"], 0)
            self.assertEqual(d["counts"]["alias_doc"], 1)
        finally:
            cleanup(root)

    def test_F3_real_marker_counts_stay_equal(self):
        "F3 正：真有〔失效〕标记时，文字「已标失效」仍 == JSON.dead_marked（原义不回退）。"
        import re
        root = make_sandbox({"m2.md": "原物 `gone.py` 〔失效：外部仓库〕\n"})
        try:
            d, txt = self._run_pair(root)
            line = next(l for l in txt.splitlines() if "锚点守卫" in l)
            m_dead = re.search(r"已标失效\s*(\d+)", line)
            self.assertEqual(int(m_dead.group(1)), d["counts"]["dead_marked"], line)
            self.assertEqual(d["counts"]["dead_marked"], 1, line)
            self.assertEqual(d["counts"]["alias_doc"], 0, line)
        finally:
            cleanup(root)


    def test_E1_same_line_inside_fence_is_not_exempt(self):
        """E1 负（真负控）：**同一引用与 token 同在围栏内同一行** ⇒ 不得豁免。

        原版 E1 把「引用」放在围栏外、token 放在围栏内另一行 —— 那只证明「不在同一行」，
        并不能证明「引用+标签同在围栏内」不豁免（captain 判为验证空洞）。
        原 sample 作为 E1old 单独保留（不再冒充已覆盖），本用例是真正的负控；
        配对正控见 E1b（同结构但整行在围栏外 ⇒ 必须豁免）。
        """
        root = make_sandbox({"a.md": (
            "```\n"
            "原物 `gone.py` 〔证失效〕 理由在外：工作区 0 命中。\n"
            "```\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_E1b_same_line_outside_fence_exempts(self):
        """E1b 正（与 E1 配对）：同结构但整行在**围栏外** ⇒ 必须豁免。

        没有这条，E1 单独存在时无法区分「围栏导致不豁免」与「实现一律不豁免」。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 理由在外：工作区 0 命中。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_E1old_cross_line_sample_retained(self):
        """E1old：**原 sample 原样保留**（引用在围栏外、token 在围栏内另一行）。

        它只证明「不同一行」，**不再冒充「引用+标签同在围栏内」的覆盖**。保留是为了
        不丢原先的测试输入与结果；真正的负控是 E1。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py`\n"
            "```\n"
            "写 `〔证失效〕` 即可\n"
            "```\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
        finally:
            cleanup(root)

    def test_E2_explicit_example_wording_is_not_exempt(self):
        """E2 负：同行**显式**写明「本行仅展示语法，不是声明」⇒ 不得豁免。

        这条是 captain 明确要求保留的负例：显式「非声明」措辞必须继续判红，
        不能因为判据从词表改成语法位置就把它改绿。它靠的是否定式短语，
        不是「示例/写法」这类普通用词。"""
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 （本行仅展示语法，不是声明）\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_E3_real_declaration_still_exempts(self):
        """E3 正：真正的声明（不在围栏、无示例词）⇒ 豁免（与 E1/E2 成对）。"""
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 理由写在括号外：工作区 0 命中。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_E4_live_target_unaffected_by_marker(self):
        """E4 正：真实目标仍存在时，行内标记不改变 live 判定（正常检查保留）。"""
        root = make_sandbox({"a.md": (
            "`artifacts/aim-mv-20261024/mvlog.csv:1` 〔证失效〕\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)  # live 不进 deads
        finally:
            cleanup(root)

    def test_B1_inline_span_display_is_not_exempt(self):
        """B1 负：规范词只被**行内反引号**包住、同行无声明性上下文 ⇒ 展示，不豁免。

        这是 t28 F-G3-1 的 B 面：旧版靠「示例词」兜住，作者不写那两个字就静默放过。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py`；照 `〔证失效〕` 这样写就行\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_B2_fenced_display_is_not_exempt(self):
        """B2 负：规范词位于**代码围栏**内 ⇒ 展示，不豁免。"""
        root = make_sandbox({"a.md": (
            "原物 `gone.py`\n"
            "```\n"
            "写 `〔证失效〕` 即可\n"
            "```\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), ["gone.py"], d)
            self.assertEqual(d["counts"]["dead_marked"], 0, d)
        finally:
            cleanup(root)

    def test_B3_reason_containing_example_words_still_exempts(self):
        """B3 正：**括号外自由理由**里出现「例如/写法/语法/演示」属于正常书写 ⇒ 必须继续豁免。

        这是 t28 F-G3-1 的 A 面：旧版用词表裸匹配把这些真声明误判成展示。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 理由在外，例如：工作区 0 命中；写法如上。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_B4_plain_declaration_exempts(self):
        """B4 正：token 不在任何 code span 内 ⇒ 声明，豁免。"""
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 理由写在括号外：工作区 0 命中。\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_C1_glued_fullwidth_paren_is_exempt(self):
        """C1 正：合法声明**紧贴全角括号理由**（token 后无空格）⇒ 必须豁免。

        t47 F-G3-2：t46 用「前/后紧贴非空白即展示」猜展示，给规范凭空加了一个空格门槛，
        导致「路径〔证失效〕（理由）」被误判成展示。真实文档里这是常态写法。
        """
        root = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕（理由：工作区 0 命中）\n")})
        try:
            rc, d = run_guard(root)
            self.assertEqual(red_targets(d), [], d)
            self.assertEqual(d["counts"]["dead_marked"], 1, d)
        finally:
            cleanup(root)

    def test_C2_glued_vs_spaced_are_equivalent(self):
        """C2 正：C1 与「token 后加一个空格」**同语义应同结果**（只差 token 外空格）。

        这条专门区分「按语法位置」与「按字符紧邻」的实现：紧邻实现会让两者结果相反。
        """
        glued = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕（理由：工作区 0 命中）\n")})
        spaced = make_sandbox({"a.md": (
            "原物 `gone.py` 〔证失效〕 （理由：工作区 0 命中）\n")})
        try:
            rc_g, d_g = run_guard(glued)
            rc_s, d_s = run_guard(spaced)
            self.assertEqual(red_targets(d_g), red_targets(d_s), (d_g, d_s))
            self.assertEqual(d_g["counts"]["dead_marked"], d_s["counts"]["dead_marked"], (d_g, d_s))
            self.assertEqual(rc_g, rc_s)
        finally:
            cleanup(glued)
            cleanup(spaced)

if __name__ == "__main__":
    unittest.main(verbosity=2)
