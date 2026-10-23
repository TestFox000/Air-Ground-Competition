#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""t5e 单测 —— L0 裁剪（route_crop）的语法/不变式/拒绝路径 + H1/H2 bench 全过

跑法：
    python -m unittest discover -s dzsmartscout/tools/planner/tests -p "test_*.py" -v
    python dzsmartscout/tools/planner/tests/test_route_crop.py

覆盖：
  ① 路线语法：token 形态（`G10*` / `N5,y` / `R:` / `SHOOT@`）、分隔符、前缀词、reset:/load: 拒绝
  ② **纯后缀不变式**：裁剪结果 == `tokens[anchor:]`（一字不改、不重排），语义尾巴（`*` / `,y`）保留
  ③ 无回头段 / 无已占格冲突：真实 44 token 路线上逐格断言
  ④ 拒绝路径：错锚点（车格不在线里）、重复格无 yaw、phase 闸门、repress、航向矛盾
  ⑤ 负证据语义：`coverage.complete=false` / 旧报文 ⇒ 不产生 ABSENT（t10 口径）
  ⑥ car_state 解析：车侧原生报文 / supervisor 日志行 / 缺 block 抛错 / 混合行取最后一条
  ⑦ 车侧形状契约：裁剪结果过 `dzlink_supervisor.validate_route`
  ⑧ CLI：`--emit cmd/json` 输出形态 + 冲突时 exit 1
  ⑨ bench：H1a~H1e / H2a~H2h 全部 PASS（离线近似模型，见 bench 模块头）
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent          # tools/planner/tests
PLANNER = HERE.parent                                   # tools/planner
WORKSPACE = PLANNER.parents[2]
if str(PLANNER) not in sys.path:
    sys.path.insert(0, str(PLANNER))

import bench_route_crop as BENCH                        # noqa: E402
import route_crop as RC                                 # noqa: E402

ROUTE = None                                            # 惰性加载（要读 vendor config）


def route() -> list:
    global ROUTE
    if ROUTE is None:
        ROUTE = BENCH.vendor_route()
    return ROUTE


class TestSyntax(unittest.TestCase):
    def test_token_block_forms(self):
        for raw, want in [("G10", "G10"), ("g10", "G10"), ("G10*", "G10"),
                          ("N5,y", "N5"), ("R:F11", "F11"), ("R:B5*", "B5"),
                          ("SHOOT@G11", "G11"), ("F3:SCAN_STOP", "F3")]:
            self.assertEqual(RC.token_block(raw), want, raw)
        for bad in ["", "Z1", "A12", ":SCAN_STOP", "N5,x,y"]:
            self.assertIsNone(RC.token_block(bad), bad)

    def test_token_tail_preserved(self):
        self.assertEqual(RC.token_tail("G10*"), "*")
        self.assertEqual(RC.token_tail("N5,y"), ",y")
        self.assertEqual(RC.token_tail("R:F11"), "R:")
        self.assertEqual(RC.token_tail("G10"), "")

    def test_split_route_prefix_and_separators(self):
        self.assertEqual(RC.split_route("多点 N10 M10"), ["N10", "M10"])
        self.assertEqual(RC.split_route("N10;M10|L10"), ["N10", "M10", "L10"])
        self.assertEqual(RC.split_route("route A1 B2"), ["A1", "B2"])

    def test_split_route_rejects_non_route(self):
        for bad in ["", "reset:N10,y", "load:r1.txt", "多点"]:
            with self.assertRaises(RC.CropError, msg=bad):
                RC.split_route(bad)
        with self.assertRaises(RC.CropError):
            RC.split_route("N10 XX M10")               # 混入非格名 token

    def test_norm_blocks(self):
        self.assertEqual(RC.norm_blocks(["A3,B4", " ; c5 ", None]), ["A3", "B4", "C5"])
        with self.assertRaises(RC.CropError):
            RC.norm_blocks(["Z9"])


class TestCropInvariants(unittest.TestCase):
    def test_pure_suffix_not_rewritten(self):
        """★ 核心不变式：输出恒为原序 token 的一段后缀，一字不改、不重排。"""
        toks = route()
        res = RC.crop_route(toks, RC.CarState(block="A2", yaw_deg=0.0, phase="DRIVE"))
        self.assertTrue(res.ok, res.reject)
        idx = res.anchor.index
        self.assertEqual(list(res.route), toks[idx:])
        self.assertEqual(list(res.dropped), toks[:idx])

    def test_semantic_tails_survive(self):
        """`D3*`（停站打靶）与末点 `N5,y`（朝向）必须原样保留 —— 裁剪不许把语义裁掉。"""
        res = RC.crop_route(route(), RC.CarState(block="A2", yaw_deg=0.0, phase="DRIVE"))
        self.assertIn("D3*", res.route)
        self.assertEqual(res.route[-1], "N5,y")

    def test_no_backtrack_no_occupied(self):
        """无回头段 + 无已占格：后缀里不出现任何已走过格，也不出现已压格。"""
        toks = route()
        res = RC.crop_route(toks, RC.CarState(block="A2", yaw_deg=0.0, phase="DRIVE"),
                            pressed=["D9", "H10"])
        self.assertTrue(res.ok, res.reject)
        st = res.stats
        suffix = RC.blocks_of(list(res.route)[1:])
        self.assertFalse(set(suffix) & set(st["dropped_prefix"]), "后缀重走了已走过格")
        self.assertFalse(set(suffix) & {"D9", "H10"}, "后缀包含了已压格")
        self.assertEqual(st["n_dropped"], 23)
        self.assertEqual(st["n_out"], len(toks) - 23)
        self.assertFalse(res.conflicts)

    def test_anchor_variants(self):
        """锚点必须落在整线里该格的下标上（起点 0、终点末位；中间格逐格核对）。"""
        toks = route()
        blocks = RC.blocks_of(toks)
        for block in ("N10", "G10", "E10", "A2", "N5"):
            if block not in blocks:
                continue
            want = blocks.index(block)
            fix, why = RC.locate_anchor(toks, RC.CarState(block=block, yaw_deg=None))
            self.assertIsNotNone(fix, why)
            self.assertEqual(fix.index, want, block)
        self.assertEqual(blocks[0], "N10")
        self.assertTrue(toks[-1].startswith(blocks[-1]))


class TestRejections(unittest.TestCase):
    def _car(self, block="A2", yaw=0.0, phase="DRIVE"):
        return RC.CarState(block=block, yaw_deg=yaw, phase=phase)

    def test_off_route_refused_by_default(self):
        res = RC.crop_route(route(), self._car("N11"))
        self.assertFalse(res.ok)
        self.assertTrue(any("不在整线里" in r for r in res.reject), res.reject)

    def test_off_route_nearest_when_allowed(self):
        res = RC.crop_route(route(), self._car("N11"), allow_nearest=True)
        self.assertTrue(res.ok, res.reject)
        self.assertEqual(res.anchor.mode, "nearest")

    def test_duplicate_anchor_needs_yaw(self):
        dup = ["N10", "M10", "M9", "M10", "L10", "K5", "N5"]
        res = RC.crop_route(dup, RC.CarState(block="M10", yaw_deg=None, phase="DRIVE"))
        self.assertFalse(res.ok)
        self.assertTrue(any("yaw_deg" in r for r in res.reject), res.reject)

    def test_phase_gate(self):
        for ph in ("STOP", "FINISH", "SAFE", "IDLE", "HOLD"):
            res = RC.crop_route(route(), self._car(phase=ph))
            self.assertFalse(res.ok, ph)
        self.assertTrue(RC.crop_route(route(), self._car(phase="DRIVE")).ok)
        self.assertTrue(RC.crop_route(route(), self._car(phase="APPROACH")).ok)

    def test_repress_conflict(self):
        """已压格落在后缀里 ⇒ repress。冲突格**从整线里取**，不写死（换路线不失效）。"""
        blocks = RC.blocks_of(route())
        victim = blocks[blocks.index("A2") + 5]
        res = RC.crop_route(route(), self._car(), pressed=[victim])
        self.assertFalse(res.ok)
        self.assertIn("repress", [c["kind"] for c in res.conflicts])
        self.assertEqual([c["block"] for c in res.conflicts if c["kind"] == "repress"], [victim])

    def test_json_hides_rejected_route(self):
        """被拒绝时 JSON 的 `route` 必须是空串 —— 下游不可能误用一条被拒的线。"""
        blocks = RC.blocks_of(route())
        victim = blocks[blocks.index("A2") + 5]
        res = RC.crop_route(route(), self._car(), pressed=[victim])
        js = res.to_json()
        self.assertEqual(js["route"], "")
        self.assertTrue(js["route_rejected"].startswith("A2"))

    def test_revisit_conflict_and_override(self):
        dup = ["N10", "M10", "M9", "M10", "L10", "K5", "N5"]
        strict = RC.crop_route(dup, RC.CarState(block="M10", yaw_deg=90.0, phase="DRIVE"))
        self.assertFalse(strict.ok)
        self.assertIn("revisit", [c["kind"] for c in strict.conflicts])
        loose = RC.crop_route(dup, RC.CarState(block="M10", yaw_deg=90.0, phase="DRIVE"),
                              allow_revisit=True)
        self.assertTrue(loose.ok, loose.reject)

    def test_handover_reversal(self):
        res = RC.crop_route(route(), self._car("B9", yaw=180.0))
        self.assertFalse(res.ok)
        self.assertIn("handover_reversal", [c["kind"] for c in res.conflicts])

    def test_anchor_shoot_stop_warns(self):
        """锚点带 `*`（停站打靶）时必须留一条告警：车侧 `_load_tokens` 会从 index 1 起走。"""
        shoot = [t for t in route() if "*" in RC.token_tail(t)]
        if not shoot:
            self.skipTest("当前整线没有停站打靶标记")
        block = RC.token_block(shoot[0])
        res = RC.crop_route(route(), self._car(block, yaw=0.0))
        self.assertTrue(res.ok, res.reject)
        self.assertTrue(any("停站打靶" in w for w in res.warnings), res.warnings)


class TestCoverage(unittest.TestCase):
    def _payload(self, *, complete):
        return BENCH._field_verify(complete=complete, cells=RC.grid().all_blocks())

    def test_incomplete_never_absent(self):
        cleared, notes = RC.cleared_from_field_verify(self._payload(complete=False))
        self.assertIsNone(cleared)
        self.assertTrue(notes)

    def test_complete_gives_absent(self):
        cleared, _ = RC.cleared_from_field_verify(self._payload(complete=True))
        self.assertEqual(len(cleared), 154)

    def test_old_payload_without_coverage(self):
        cleared, notes = RC.cleared_from_field_verify({"type": "field_verify", "mines": []})
        self.assertIsNone(cleared)
        self.assertTrue(any("coverage" in n for n in notes), notes)

    def test_unknown_ahead_counts_only_uncleared(self):
        car = RC.CarState(block="A2", yaw_deg=0.0, phase="DRIVE")
        base = RC.crop_route(route(), car)
        full = RC.crop_route(route(), car,
                             cleared=RC.cleared_from_field_verify(
                                 self._payload(complete=True))[0])
        self.assertGreater(base.stats["n_unknown_ahead"], 0)
        self.assertEqual(full.stats["n_unknown_ahead"], 0)


class TestCarState(unittest.TestCase):
    def test_native_payload(self):
        st = RC.car_state_from_payload({"type": "car_state", "block": "e10",
                                        "yaw_deg": 90, "phase": "drive", "ts": 1.5})
        self.assertEqual((st.block, st.yaw_deg, st.phase), ("E10", 90.0, "DRIVE"))

    def test_supervisor_log_line(self):
        st = RC.car_state_from_payload({"kind": "car_state", "block": "A2", "phase": "DRIVE"})
        self.assertEqual(st.block, "A2")
        self.assertIsNone(st.yaw_deg)

    def test_missing_block_raises(self):
        with self.assertRaises(RC.CropError):
            RC.car_state_from_payload({"type": "car_state", "phase": "DRIVE"})
        with self.assertRaises(RC.CropError):
            RC.car_state_from_payload({"type": "ack", "block": "A2"})

    def test_file_takes_last_car_state(self):
        import tempfile
        lines = ['{"type":"heartbeat","ts":1}',
                 '{"type":"car_state","block":"G10","yaw_deg":90,"phase":"DRIVE"}',
                 '{"kind":"ack","of":3}',
                 '{"type":"car_state","block":"F10","yaw_deg":90,"phase":"DRIVE"}']
        with tempfile.TemporaryDirectory() as td:
            p = pathlib.Path(td) / "car.jsonl"
            p.write_text("\n".join(lines) + "\n", encoding="utf-8")
            st = RC.car_state_from_file(p)
        self.assertEqual(st.block, "F10")


class TestContract(unittest.TestCase):
    def test_cropped_route_passes_car_contract(self):
        res = RC.crop_route(route(), RC.CarState(block="A2", yaw_deg=0.0, phase="DRIVE"))
        ok, probs = RC.validate_route_contract(res.route_text())
        self.assertTrue(ok, probs)

    def test_contract_rejects_garbage(self):
        ok, probs = RC.validate_route_contract("N10 ZZZ")
        self.assertFalse(ok)
        self.assertTrue(probs)

    def test_route_crop_does_not_import_rclpy(self):
        """本模块必须能在 Windows 本机跑（无 ROS）—— 导入期不许拉 rclpy。"""
        self.assertNotIn("rclpy", sys.modules)


class TestBench(unittest.TestCase):
    def test_bench_all_pass(self):
        results = BENCH.run_bench()
        bad = [(r.name, [c for c in r.checks if not c[1]]) for r in results if not r.passed]
        self.assertFalse(bad, f"bench 有用例 FAIL：{bad}")
        self.assertGreaterEqual(len(results), 13)
        names = {r.name for r in results}
        for want in ("H1a", "H1b", "H1c", "H1d", "H1e", "H2a", "H2b", "H2c"):
            self.assertIn(want, names)


class TestCli(unittest.TestCase):
    def _run(self, *args):
        cmd = [sys.executable, str(PLANNER / "route_crop.py"), "crop", *args]
        return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                              errors="replace", cwd=str(WORKSPACE), timeout=300)

    def test_emit_cmd_is_supervisor_cmd_line(self):
        p = self._run("--route", " ".join(route()), "--car-block", "A2", "--car-yaw", "0",
                      "--emit", "cmd")
        self.assertEqual(p.returncode, 0, p.stderr[-800:])
        obj = json.loads(p.stdout.strip().splitlines()[-1])
        self.assertEqual(obj["type"], "route")
        self.assertTrue(obj["data"].startswith("A2 "), obj["data"][:40])

    def test_emit_json_and_exit1_on_repress(self):
        p = self._run("--route", " ".join(route()), "--car-block", "A2", "--car-yaw", "0",
                      "--pressed", "I5", "--emit", "json")
        self.assertEqual(p.returncode, 1, p.stderr[-800:])
        obj = json.loads(p.stdout)
        self.assertFalse(obj["ok"])
        self.assertEqual(obj["conflicts"][0]["kind"], "repress")

    def test_reset_route_is_usage_error(self):
        p = self._run("--route", "reset:N10,y", "--car-block", "N10")
        self.assertEqual(p.returncode, 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
