#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dz_send_waypoint 的单测（★ 不 import rclpy，Windows 本机可跑）。

跑法（任选）：
  python dzsmartscout/tools/link/tests/test_dz_send_waypoint.py
  python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"

覆盖（t12 修复的 t4 真机缺陷 D4，以及既有报文字段契约）：
  · 格名拆分：空格 / 逗号 / 分号 / 竖线 / 顿号 一律拆开；去空、大写、按序去重
  · CLI：`target G10 H10`（位置参数）与 `target --blocks G10,H10`（别名）等价
  · CLI：`field --mines A3,B4` 不再产出畸形格名 `"A3,B4"`
  · 非法输入的明确报错：`bridge lower=K9,K4`、`target`（无格名）、`--mines` 后无值
  · 报文字段与车侧 `tasks/drone.py` 解析面逐字段对齐（target_zone / field_verify）
"""
from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_LINK_DIR = os.path.dirname(_HERE)
if _LINK_DIR not in sys.path:
    sys.path.insert(0, _LINK_DIR)

import dz_send_waypoint as W  # noqa: E402


def dry(argv):
    """跑 CLI 的 --dry-run，返回 (rc, stdout, stderr)。"""
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        rc = W.main(argv)
    return rc, out.getvalue(), err.getvalue()


def payloads(stdout):
    """从 dry-run 输出里摘出 JSON 报文。"""
    got = []
    for line in stdout.splitlines():
        line = line.strip()
        if line.startswith("{"):
            got.append(json.loads(line))
    return got


class TestSplitBlocks(unittest.TestCase):

    def test_space_separated(self):
        self.assertEqual(W.split_blocks(["G10", "H10"]), ["G10", "H10"])

    def test_comma_inside_one_token(self):
        """★ D4 核心：以前 ["A3,B4"] 会原样变成一个畸形格名。"""
        self.assertEqual(W.split_blocks(["A3,B4"]), ["A3", "B4"])

    def test_semicolon_pipe_dunhao(self):
        self.assertEqual(W.split_blocks(["A3;B4|C5、D6"]), ["A3", "B4", "C5", "D6"])

    def test_space_inside_one_token(self):
        self.assertEqual(W.split_blocks(["A3 B4"]), ["A3", "B4"])

    def test_uppercase_and_strip(self):
        self.assertEqual(W.split_blocks([" a3 , b4 "]), ["A3", "B4"])

    def test_dedup_preserves_order(self):
        self.assertEqual(W.split_blocks(["A3,B4,A3", "B4", "K9"]), ["A3", "B4", "K9"])

    def test_empties_dropped(self):
        self.assertEqual(W.split_blocks(["", " , ", "A3", None]), ["A3"])

    def test_none_and_empty(self):
        self.assertEqual(W.split_blocks(None), [])
        self.assertEqual(W.split_blocks([]), [])

    def test_had_separator(self):
        self.assertTrue(W.had_separator(["A3,B4"]))
        self.assertFalse(W.had_separator(["A3", "B4"]))


class TestTargetCli(unittest.TestCase):

    def test_positional_space(self):
        rc, out, _ = dry(["target", "G10", "H10", "--dry-run"])
        self.assertEqual(rc, 0)
        self.assertEqual(payloads(out)[0]["blocks"], ["G10", "H10"])

    def test_blocks_alias_with_comma(self):
        """★ D4：`target --blocks G10,H10` 以前 argparse 直接 exit 2。"""
        rc, out, _ = dry(["target", "--blocks", "G10,H10", "--dry-run"])
        self.assertEqual(rc, 0)
        self.assertEqual(payloads(out)[0]["blocks"], ["G10", "H10"])

    def test_comma_split_is_announced_on_stderr(self):
        _, _, err = dry(["target", "--blocks", "G10,H10", "--dry-run"])
        self.assertIn("拆分", err)

    def test_no_blocks_is_a_clear_error(self):
        with self.assertRaises(SystemExit) as cm:
            dry(["target", "--dry-run"])
        self.assertIn("至少", str(cm.exception))

    def test_message_shape_matches_car_contract(self):
        rc, out, _ = dry(["target", "G10", "H10", "--dry-run"])
        msg = payloads(out)[0]
        self.assertEqual(msg["type"], "target_zone")
        self.assertIsInstance(msg["blocks"], list)
        self.assertIn("ts", msg)


class TestFieldCli(unittest.TestCase):

    def test_space_separated(self):
        rc, out, _ = dry(["field", "--mines", "A3", "B4", "--dry-run"])
        self.assertEqual(rc, 0)
        self.assertEqual(payloads(out)[0]["mines"], ["A3", "B4"])

    def test_comma_separated_no_malformed_block(self):
        rc, out, _ = dry(["field", "--mines", "A3,B4", "--obstacles", "K7", "--dry-run"])
        self.assertEqual(rc, 0)
        msg = payloads(out)[0]
        self.assertEqual(msg["mines"], ["A3", "B4"])
        self.assertEqual(msg["obstacles"], ["K7"])
        for key in ("mines", "obstacles"):
            for b in msg.get(key, []):
                self.assertNotIn(",", b, f"畸形格名：{b!r}")

    def test_only_obstacles(self):
        rc, out, _ = dry(["field", "--obstacles", "K7,K8", "--dry-run"])
        self.assertEqual(rc, 0)
        msg = payloads(out)[0]
        self.assertNotIn("mines", msg)
        self.assertEqual(msg["obstacles"], ["K7", "K8"])

    def test_missing_values_is_a_clear_error(self):
        with self.assertRaises(SystemExit) as cm:
            dry(["field", "--mines", "--dry-run"])
        self.assertIn("没跟格名", str(cm.exception))

    def test_neither_flag(self):
        with self.assertRaises(SystemExit) as cm:
            dry(["field", "--dry-run"])
        self.assertIn("至少", str(cm.exception))


class TestBridgeCli(unittest.TestCase):

    def test_both_halves(self):
        rc, out, _ = dry(["bridge", "lower=K9", "upper=K4", "--seq", "7", "--dry-run"])
        self.assertEqual(rc, 0)
        msgs = payloads(out)
        self.assertEqual([m["half"] for m in msgs], ["lower", "upper"])
        self.assertEqual([m["block"] for m in msgs], ["K9", "K4"])
        self.assertEqual(msgs[0]["seq"], 7)
        self.assertEqual(msgs[0]["type"], "bridge_passable")

    def test_two_blocks_in_one_half_is_refused(self):
        """★ D4：以前 `lower=K9,K4` 会静默发出畸形格名 "K9,K4"。"""
        with self.assertRaises(SystemExit) as cm:
            dry(["bridge", "lower=K9,K4", "--dry-run"])
        self.assertIn("只能给一个格名", str(cm.exception))

    def test_unknown_half(self):
        with self.assertRaises(SystemExit):
            dry(["bridge", "middle=K9", "--dry-run"])

    def test_missing_equals(self):
        with self.assertRaises(SystemExit):
            dry(["bridge", "K9", "--dry-run"])


class TestRouteCli(unittest.TestCase):

    def test_route_text_passthrough(self):
        rc, out, _ = dry(["route", "N10 M10 L10* L9 K9", "--dry-run"])
        self.assertEqual(rc, 0)
        self.assertIn("N10 M10 L10* L9 K9", out)


if __name__ == "__main__":
    unittest.main(verbosity=2)
