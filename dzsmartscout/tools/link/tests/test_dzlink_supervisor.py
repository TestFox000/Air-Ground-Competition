#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""dzlink_supervisor 的单测（★ 不 import rclpy，Windows 本机可跑）。

跑法（任选）：
  python dzsmartscout/tools/link/tests/test_dzlink_supervisor.py
  python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"
  python dzsmartscout/tools/link/dzlink_supervisor.py selftest    ← 等价的精简自检

覆盖范围（对应任务书的 ⑤ 与契约）：
  · 心跳节律（1Hz：首帧即发、周期内不重复、跨周期重发）
  · 情报翻译（字段宽容 / 方向词映射 / 格名规范化去重 / 认不出就丢弃不猜）
  · 情报时效（ttl 内才发、过期停发）
  · 报文契约（JSON 字段与车侧 drone.py 逐字段对齐、seq 单调递增）
  · 路线契约（合法/非法形状、reset/load、R: 前缀、* 后缀、朝向后缀位置）
  · 闭环台账（ack 只匹配真实发过的 seq；comma... 不匹配也算收到）
  · 文件尾随（增量、半行缓冲、截断重置、注释与空行、坏行不改状态）
  · 命令文件（route/reset/bridge/stop 执行、坏行不崩）
  · 端到端（感知文件 → 报文 → 模拟车回 ack → 台账对上）
"""
from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_LINK_DIR = os.path.dirname(_HERE)
if _LINK_DIR not in sys.path:
    sys.path.insert(0, _LINK_DIR)

import dzlink_supervisor as S  # noqa: E402


def core(**cfg) -> S.SupervisorCore:
    """新建一个不落盘的 core（测试里绝不写真实日志文件）。"""
    return S.SupervisorCore(S.CoreConfig(**cfg), S.JsonlLogger(None))


class TestDomainBootstrap(unittest.TestCase):

    def setUp(self):
        self._saved = os.environ.get("ROS_DOMAIN_ID")
        os.environ.pop("ROS_DOMAIN_ID", None)

    def tearDown(self):
        if self._saved is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = self._saved

    def test_builtin_fallback_is_118(self):
        info = S.bootstrap_domain("/definitely/not/here.env", argv=[], quiet=True)
        self.assertEqual(os.environ["ROS_DOMAIN_ID"], "118")
        self.assertEqual(info["source"], "builtin")
        self.assertTrue(info["warnings"])

    def test_env_file_wins_over_builtin(self):
        d = tempfile.mkdtemp()
        try:
            p = os.path.join(d, "dzlink.env")
            with open(p, "w", encoding="utf-8") as fh:
                fh.write("# c\nexport ROS_DOMAIN_ID=200\nexport ROS_LOCALHOST_ONLY=0\n")
            info = S.bootstrap_domain(p, argv=[], quiet=True)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "200")
            self.assertEqual(info["source"], "envfile")
            self.assertFalse(info["warnings"])
        finally:
            shutil.rmtree(d, ignore_errors=True)

    def test_cli_domain_beats_env_file(self):
        d = tempfile.mkdtemp()
        try:
            p = os.path.join(d, "dzlink.env")
            with open(p, "w", encoding="utf-8") as fh:
                fh.write("ROS_DOMAIN_ID=200\n")
            info = S.bootstrap_domain(p, argv=["run", "--domain", "7"], quiet=True)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "7")
            self.assertEqual(info["source"], "cli")
        finally:
            shutil.rmtree(d, ignore_errors=True)

    def test_external_domain_gets_warning_not_silence(self):
        d = tempfile.mkdtemp()
        try:
            p = os.path.join(d, "dzlink.env")
            with open(p, "w", encoding="utf-8") as fh:
                fh.write("ROS_DOMAIN_ID=118\n")
            os.environ["ROS_DOMAIN_ID"] = "0"
            info = S.bootstrap_domain(p, argv=[], quiet=True)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "118")
            self.assertTrue(any("外部已设" in w for w in info["warnings"]))
        finally:
            shutil.rmtree(d, ignore_errors=True)


class TestHeartbeat(unittest.TestCase):

    def test_first_tick_emits_then_period_gates(self):
        c = core(heartbeat_period_s=1.0)
        evs = c.tick(now=100.0)
        self.assertEqual(len(evs), 1)
        self.assertEqual(evs[0].kind, "heartbeat")
        self.assertEqual(evs[0].topic, S.TOPIC_TO_CAR)
        self.assertEqual(evs[0].qos, S.QOS_LINK)
        self.assertEqual(c.tick(now=100.9), [])
        evs = c.tick(now=101.0)
        self.assertEqual([e.kind for e in evs], ["heartbeat"])

    def test_heartbeat_payload_contract(self):
        c = core()
        ev = c.tick(now=5.0)[0]
        body = json.loads(ev.text)
        self.assertEqual(body["type"], "heartbeat")
        self.assertIsInstance(body["seq"], int)
        self.assertAlmostEqual(body["ts"], 5.0, places=3)

    def test_heartbeat_hz(self):
        c = core(heartbeat_period_s=0.5)
        n = 0
        for i in range(0, 400):
            n += len([e for e in c.tick(now=1000 + i * 0.05) if e.kind == "heartbeat"])
        # 20s @ 2Hz ≈ 40 条
        self.assertGreaterEqual(n, 38)
        self.assertLessEqual(n, 42)

    def test_seq_is_monotonic_across_kinds(self):
        c = core()
        seqs = []
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        for ev in c.tick(now=1.0):
            seqs.append(ev.seq)
        for ev in c.tick(now=2.0):
            if ev.seq is not None:
                seqs.append(ev.seq)
        self.assertEqual(seqs, sorted(seqs))
        self.assertEqual(len(seqs), len(set(seqs)))


class TestDecide(unittest.TestCase):

    def test_canonical_bridge(self):
        out = S.decide({"type": "bridge_passable", "half": "upper", "block": "k4",
                        "confidence": 0.42})
        self.assertEqual(out, [("bridge_passable", {"half": "upper", "block": "K4",
                                                    "confidence": 0.42})])

    def test_direction_word_maps_to_half(self):
        out = S.decide({"type": "bridge", "direction": "left", "hole": "K9"})
        self.assertEqual(out[0][1]["half"], "lower")
        out = S.decide({"type": "bridge", "direction": "right", "hole": "K4"})
        self.assertEqual(out[0][1]["half"], "upper")

    def test_direction_word_mapping_is_configurable(self):
        out = S.decide({"type": "bridge", "direction": "left", "block": "K9"},
                       left_halves="upper")
        self.assertEqual(out[0][1]["half"], "upper")

    def test_half_aliases(self):
        for raw, want in (("low", "lower"), ("下", "lower"), ("top", "upper"),
                          ("上半场", "upper"), (" LOWER ", "lower")):
            out = S.decide({"type": "bridge_passable", "half": raw, "block": "K9"})
            self.assertEqual(out[0][1]["half"], want, raw)

    def test_bridge_without_usable_half_is_dropped_not_guessed(self):
        self.assertEqual(S.decide({"type": "bridge_passable", "block": "K9"}), [])
        self.assertEqual(S.decide({"type": "bridge", "direction": "north", "block": "K9"}), [])

    def test_confidence_clamped_and_default(self):
        self.assertEqual(S.decide({"type": "bridge_passable", "half": "lower",
                                   "block": "K9"})[0][1]["confidence"], 0.9)
        self.assertEqual(S.decide({"type": "bridge_passable", "half": "lower",
                                   "block": "K9", "confidence": 5})[0][1]["confidence"], 1.0)
        self.assertEqual(S.decide({"type": "bridge_passable", "half": "lower",
                                   "block": "K9", "confidence": -3})[0][1]["confidence"], 0.0)

    def test_blocks_normalised_and_deduped(self):
        out = S.decide({"type": "target_zone", "blocks": ["g10", "H11", "g10", "K10"]})
        self.assertEqual(out[0][1]["blocks"], ["G10", "H11", "K10"])

    def test_blocks_accept_string_and_unknown_entries_skipped(self):
        out = S.decide({"type": "field_verify", "mines": "A3, B4 ; ZZ9", "obstacles": ["k9"]})
        self.assertEqual(out[0][1], {"mines": ["A3", "B4"], "obstacles": ["K9"]})

    def test_split_mine_and_obstacle_events(self):
        self.assertEqual(S.decide({"type": "mines", "blocks": ["a3"]}),
                         [("field_verify", {"mines": ["A3"]})])
        self.assertEqual(S.decide({"type": "obstacles", "blocks": ["k9"]}),
                         [("field_verify", {"obstacles": ["K9"]})])

    def test_wrapped_payload(self):
        out = S.decide({"payload": {"type": "target_zone", "blocks": ["G10"]}})
        self.assertEqual(out[0][1]["blocks"], ["G10"])
        out = S.decide({"data": {"type": "bridge_passable", "half": "lower", "block": "K9"}})
        self.assertEqual(out[0][0], "bridge_passable")

    def test_unknown_is_dropped(self):
        for bad in ({}, {"type": "wat"}, {"foo": 1}, None, "x", 5):
            self.assertEqual(S.decide(bad), [], repr(bad))

    def test_block_only_true_for_valid_grid_names(self):
        self.assertIsNone(S._norm_block("Z9"))
        self.assertIsNone(S._norm_block("K12"))
        self.assertIsNone(S._norm_block(""))
        self.assertEqual(S._norm_block("k11"), "K11")
        self.assertEqual(S._norm_block("N10,y"), "N10")

    def test_canonical_text_field_order_and_content(self):
        txt = S.canonical_text("bridge_passable",
                               {"half": "lower", "block": "K9", "confidence": 0.9},
                               seq=7, ts=1234.5)
        self.assertEqual(txt, '{"type":"bridge_passable","half":"lower","block":"K9",'
                              '"confidence":0.9,"ts":1234.5,"seq":7}')
        txt = S.canonical_text("target_zone", {"blocks": ["G10", "H11"]}, seq=None, ts=1.0)
        self.assertNotIn("seq", txt)
        self.assertIn('"blocks":["G10","H11"]', txt)
        txt = S.canonical_text("field_verify", {"mines": ["A3"]}, seq=1, ts=1.0)
        self.assertIn('"mines":["A3"]', txt)
        self.assertNotIn("obstacles", txt)
        with self.assertRaises(ValueError):
            S.canonical_text("nope", {}, None)

    def test_no_ascii_escaping_non_ascii_ok(self):
        txt = S.canonical_text("heartbeat", {}, seq=1, ts=1.0)
        json.loads(txt)  # 必须可解析


class TestIntelEmission(unittest.TestCase):

    def test_immediate_emit_then_period_gate(self):
        c = core(bridge_period_s=2.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=10.0)
        evs = c.tick(now=10.0)
        br = [e for e in evs if e.label == "bridge_passable"]
        self.assertEqual(len(br), 1)
        self.assertEqual(br[0].repeat, 3)
        self.assertEqual(c.tick(now=10.5), [])
        again = [e for e in c.tick(now=12.0) if e.label == "bridge_passable"]
        self.assertEqual(len(again), 1)

    def test_content_change_emits_immediately(self):
        c = core(bridge_period_s=60.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.tick(now=1.0)
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K4"}, now=2.0)
        evs = c.tick(now=2.0)
        self.assertTrue(any('"block":"K4"' in e.text for e in evs))

    def test_same_content_refreshes_freshness_without_respam(self):
        c = core(bridge_period_s=60.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.tick(now=1.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=50.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=55.0)
        self.assertEqual([e for e in c.tick(now=55.0) if e.kind == "intel"], [])

    def test_zero_period_means_change_only(self):
        c = core(target_period_s=0.0)
        c.ingest_sample({"type": "target_zone", "blocks": ["G10"]}, now=1.0)
        self.assertEqual(len([e for e in c.tick(now=1.0) if e.label == "target_zone"]), 1)
        self.assertEqual([e for e in c.tick(now=99.0) if e.label == "target_zone"], [])
        c.ingest_sample({"type": "target_zone", "blocks": ["G10", "H10"]}, now=100.0)
        self.assertEqual(len([e for e in c.tick(now=100.0) if e.label == "target_zone"]), 1)

    def test_ttl_expiry_stops_emission(self):
        c = core(bridge_period_s=1.0, info_ttl_s=5.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        self.assertEqual(len([e for e in c.tick(now=1.0) if e.label == "bridge_passable"]), 1)
        self.assertEqual(len([e for e in c.tick(now=4.0) if e.label == "bridge_passable"]), 1)
        # 过期后一条都不发
        for t in (10.0, 20.0, 60.0):
            self.assertEqual([e for e in c.tick(now=t) if e.label == "bridge_passable"], [],
                             f"t={t}")
        st = c.status(now=60.0)
        self.assertFalse(st["intel"]["bridge_passable:lower"]["fresh"])

    def test_ttl_refresh_keeps_alive(self):
        """★ 语义：**没变过**的内容在周期之外不重发（否则日志会被刷满），
        但每次 refresh 都会把时效推后，所以"持续有新样本"时情报一直新鲜可用。"""
        c = core(bridge_period_s=2.0, info_ttl_s=6.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=0.0)
        self.assertEqual(len([e for e in c.tick(now=0.0) if e.label == "bridge_passable"]), 1)
        for t in (1.0, 3.0, 5.0, 7.0, 9.0):   # 感知持续在报同一件事
            c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=t)
        self.assertEqual([e for e in c.tick(now=9.5) if e.label == "bridge_passable"], [])
        st = c.status(now=9.5)
        self.assertTrue(st["intel"]["bridge_passable:lower"]["fresh"])
        self.assertLessEqual(st["intel"]["bridge_passable:lower"]["age_s"], 1.0)
        # 一旦新内容进来 → 立即重发（不等周期）
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K8"}, now=9.6)
        self.assertEqual(len([e for e in c.tick(now=9.6) if e.label == "bridge_passable"]), 1)
        # 周期到点且内容没再变 → 仍会按周期重发一次（保证车侧拿到的是新鲜情报）
        self.assertEqual(len([e for e in c.tick(now=11.7) if e.label == "bridge_passable"]), 1)

    def test_ttl_expiry_is_hard_stop_even_with_period(self):
        c = core(bridge_period_s=1.0, info_ttl_s=2.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.tick(now=1.0)
        self.assertEqual([e for e in c.tick(now=5.0) if e.label == "bridge_passable"], [])

    def test_rejected_sample_does_not_create_intel(self):
        c = core()
        self.assertEqual(c.ingest_sample({"type": "garbage"}, now=1.0), [])
        self.assertEqual(c.records, {})
        self.assertEqual(c.stats.samples_rejected, 1)

    def test_lower_and_upper_are_independent_slots(self):
        # ★ 回归：车侧 bridge_of("lower")/bridge_of("upper") 是两个独立半场，
        #   本节点必须同时下发两条；后到的 upper 绝不能覆盖 lower（第一版就错了）。
        c = core(bridge_period_s=1.0)
        got = c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"},
                              now=1.0)
        self.assertEqual(got, ["bridge_passable:lower"])
        got = c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K4"},
                              now=1.0)
        self.assertEqual(got, ["bridge_passable:upper"])
        evs = [e for e in c.tick(now=1.0) if e.label == "bridge_passable"]
        self.assertEqual(len(evs), 2)
        halves = sorted(json.loads(e.text)["half"] for e in evs)
        self.assertEqual(halves, ["lower", "upper"])
        payloads = {json.loads(e.text)["half"]: json.loads(e.text)["block"] for e in evs}
        self.assertEqual(payloads, {"lower": "K9", "upper": "K4"})
        # ★ t16：两槽**不再各自独立节流** —— 任一桥洞半场到点，就把所有新鲜半场一起发。
        #   理由：车侧 `mission.bridge_info_wait_s` 只有 2.0 s，两半场相位一错开，
        #   就可能有一个落在等情报窗口之外（车侧只进遥测、路线已冻结）。
        #   旧行为在这里只发 ["upper"]、0.7 s 后才补 ["lower"]，间隔已逼近窗口上限。
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=2.0)
        evs = [e for e in c.tick(now=2.5) if e.label == "bridge_passable"]
        self.assertEqual([json.loads(e.text)["half"] for e in evs], ["lower", "upper"],
                         "lower 到点必须把 upper 一起带出来（同轮成对）")
        evs = [e for e in c.tick(now=3.2) if e.label == "bridge_passable"]
        self.assertEqual(evs, [], "成对发出后两槽的 next 一起推进 ⇒ 相位对齐，周期内不再重发")
        evs = [e for e in c.tick(now=3.6) if e.label == "bridge_passable"]
        self.assertEqual([json.loads(e.text)["half"] for e in evs], ["lower", "upper"])
        # 两槽同时在账、各自新鲜，且 block 没有被对方篡改
        st = c.status(now=3.2)
        self.assertEqual(st["intel"]["bridge_passable:lower"]["payload"]["block"], "K9")
        self.assertEqual(st["intel"]["bridge_passable:upper"]["payload"]["block"], "K4")


class TestRouteContract(unittest.TestCase):

    def test_valid_routes(self):
        for text in ("N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5",
                     "N10", "G10*", "R:F11", "R:B5*", "N5,y", "N5,x", "K9:SCAN_STOP",
                     "SHOOT@G11", "多点 N10 M10", "route N10 M10", "single N10,y",
                     "N10|M10;L10"):
            ok, probs = S.validate_route(text)
            self.assertTrue(ok, f"{text!r} → {probs}")

    def test_grid_columns_10_and_11_are_accepted(self):
        # 回归：`[1-9]|1[01]` 的交替顺序曾让所有 10/11 列被判非法
        for text in ("K10", "K11", "N10 M10", "G10*", "K11,y"):
            ok, probs = S.validate_route(text)
            self.assertTrue(ok, f"{text!r} → {probs}")

    def test_invalid_routes(self):
        for text in ("", "   ", "N12", "N0", "Z9", "N5,y M10", "N5,,y", "多点"):
            ok, _ = S.validate_route(text)
            self.assertFalse(ok, f"{text!r} 竟然通过了")

    def test_reset_and_load(self):
        self.assertTrue(S.validate_route("reset:N10,y")[0])
        self.assertTrue(S.validate_route("reset:N10")[0])
        self.assertTrue(S.validate_route("reset:K11,x")[0])
        self.assertFalse(S.validate_route("reset:N10,z")[0])
        self.assertFalse(S.validate_route("reset:")[0])
        self.assertFalse(S.validate_route("reset:N10,y M10")[0])
        self.assertTrue(S.validate_route("load:r1.txt")[0])
        self.assertFalse(S.validate_route("load:")[0])

    def test_facing_suffix_only_on_last_token(self):
        ok, probs = S.validate_route("N5,y M10")
        self.assertFalse(ok)
        self.assertTrue(any("只允许挂在最后一个" in p for p in probs))

    def test_route_payload_rejects_and_normalises_separators(self):
        self.assertEqual(S.route_payload("N10|M10;L10"), "N10 M10 L10")
        with self.assertRaises(ValueError):
            S.route_payload("N5,y M10")

    def test_reset_command_builder(self):
        self.assertEqual(S.reset_command("n10", "y"), "reset:N10,y")
        self.assertEqual(S.reset_command("N10"), "reset:N10,y")
        self.assertEqual(S.reset_command("N10", ""), "reset:N10")
        with self.assertRaises(ValueError):
            S.reset_command("ZZ9")
        with self.assertRaises(ValueError):
            S.reset_command("N10", "z")

    def test_route_blocks_extraction(self):
        self.assertEqual(S.route_blocks("R:F11 G10* N5,y M11"), ["F11", "G10", "N5", "M11"])

    def test_submit_route_queues_repeats(self):
        c = core(route_repeat=3)
        c.submit_route("N10 M10 L10")
        evs = c.tick(now=1.0)
        route = [e for e in evs if e.kind == "route"]
        self.assertEqual(len(route), 1)
        self.assertEqual(route[0].repeat, 3)
        self.assertEqual(route[0].topic, S.TOPIC_ROUTE)
        self.assertEqual(route[0].qos, "reliable")

    def test_submit_route_priority_over_intel(self):
        c = core()
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.submit_route("N10 M10")
        evs = c.tick(now=1.0)
        self.assertEqual(evs[0].kind, "route")

    def test_submit_reset(self):
        c = core()
        c.submit_reset("N10", "y")
        route = [e for e in c.tick(now=1.0) if e.kind == "route"]
        self.assertEqual(route[0].text, "reset:N10,y")

    def test_bad_route_raises_before_sending(self):
        c = core()
        with self.assertRaises(ValueError):
            c.submit_route("N5,y M10")
        self.assertEqual(c.tick(now=1.0)[0].kind, "heartbeat")  # 只剩心跳，没有路线


class TestLedgerAndCarMessages(unittest.TestCase):

    def test_ack_matches_sent_seq(self):
        c = core()
        evs = c.tick(now=1.0)
        seq = evs[0].seq
        c.ingest_car_raw(json.dumps({"type": "ack", "of": seq}), now=1.1)
        self.assertEqual(c.stats.acks, 1)
        self.assertEqual(c.stats.last_ack_seq, seq)
        # 心跳走独立计数（1Hz 高频，免得淹掉情报的回执率）
        self.assertEqual(c.ledger.summary()["hb_sent"], 1)
        self.assertEqual(c.ledger.summary()["hb_acked"], 1)
        self.assertEqual(c.ledger.pending, [])

    def test_intel_ack_tracked_in_ledger(self):
        c = core()
        c.tick(now=1.0)            # 心跳 #1（进 hb 台账）
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        evs = c.tick(now=1.0)
        intel = [e for e in evs if e.kind == "intel"]
        self.assertEqual(len(intel), 1)
        self.assertEqual(c.ledger.summary()["pending_seq"], 1)
        c.ingest_car_raw(json.dumps({"type": "ack", "of": intel[0].seq}), now=1.2)
        self.assertEqual(c.ledger.summary()["intel_acked"], 1)
        self.assertEqual(c.ledger.summary()["pending_seq"], 0)

    def test_ack_for_unknown_seq_still_counted_but_not_matched(self):
        c = core()
        c.tick(now=1.0)
        c.ingest_car_raw(json.dumps({"type": "ack", "of": 9999}), now=1.1)
        self.assertEqual(c.stats.acks, 1)
        self.assertEqual(c.ledger.summary()["intel_acked"], 0)
        self.assertEqual(c.ledger.summary()["hb_acked"], 0)
        self.assertEqual(c.ledger.summary()["hb_recent_sent"], 1)
        # 有 ack 却一条都对不上 → 必须被标成"可疑"（域号不一致的典型症状）
        self.assertTrue(c.ledger.summary()["suspicious_mismatch"])
        self.assertFalse(c.ledger.summary()["any_acked"])

    def test_car_state_parsed(self):
        c = core()
        c.ingest_car_raw(json.dumps({"type": "car_state", "block": "K9", "yaw_deg": 180.0,
                                     "phase": "DRIVE", "ts": 1.0, "seq": 3}), now=2.0)
        st = c.status(now=2.0)
        self.assertEqual(st["stats"]["car_states"], 1)
        self.assertEqual(st["stats"]["last_car_block"], "K9")
        self.assertEqual(st["stats"]["last_car_phase"], "DRIVE")

    def test_malformed_car_messages_do_not_raise(self):
        c = core()
        for raw in ("not json", "[]", "", '{"type":', "null", '{"no_type":1}'):
            msg = c.ingest_car_raw(raw, now=1.0)
            self.assertIn(msg.kind, ("non-json", "?"))
        self.assertEqual(c.stats.car_msgs, 6)

    def test_car_last_ago(self):
        c = core()
        c.ingest_car_raw('{"type":"heartbeat"}', now=10.0)
        self.assertEqual(c.status(now=13.0)["stats"]["car_last_ago_s"], 3.0)


class TestFileTailer(unittest.TestCase):

    def setUp(self):
        self.d = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def _append(self, name, text):
        with open(os.path.join(self.d, name), "a", encoding="utf-8") as fh:
            fh.write(text)

    def test_incremental_read_from_start(self):
        self._append("a.jsonl", '{"type":"target_zone","blocks":["G10"]}\n')
        t = S.FileTailer(os.path.join(self.d, "a.jsonl"), from_end=False)
        self.assertEqual(len(t.poll()), 1)
        self.assertEqual(t.poll(), [])
        self._append("a.jsonl", '{"type":"heartbeat"}\n')
        self.assertEqual(len(t.poll()), 1)

    def test_from_end_skips_existing(self):
        self._append("a.jsonl", "line1\nline2\n")
        t = S.FileTailer(os.path.join(self.d, "a.jsonl"), from_end=True)
        self.assertEqual(t.poll(), [])
        self._append("a.jsonl", "line3\n")
        self.assertEqual(t.poll(), ["line3"])

    def test_partial_line_is_buffered(self):
        p = os.path.join(self.d, "a.jsonl")
        t = S.FileTailer(p, from_end=False)
        self._append("a.jsonl", '{"type":"heart')
        self.assertEqual(t.poll(), [])
        self._append("a.jsonl", 'beat"}\n')
        self.assertEqual(t.poll(), ['{"type":"heartbeat"}'])

    def test_truncation_resets_offset(self):
        p = os.path.join(self.d, "a.jsonl")
        self._append("a.jsonl", "one\ntwo\n")
        t = S.FileTailer(p, from_end=False)
        self.assertEqual(t.poll(), ["one", "two"])
        with open(p, "w", encoding="utf-8") as fh:
            fh.write("fresh\n")
        self.assertEqual(t.poll(), ["fresh"])

    def test_blank_comment_and_bom_lines_skipped(self):
        self._append("a.jsonl", '\ufeff{"type":"heartbeat"}\n\n# comment\n   \n')
        t = S.FileTailer(os.path.join(self.d, "a.jsonl"), from_end=False)
        self.assertEqual(t.poll(), ['{"type":"heartbeat"}'])

    def test_glob_tailer_tracks_multiple_files(self):
        self._append("x1.jsonl", '{"type":"target_zone","blocks":["G10"]}\n')
        self._append("x2.jsonl", '{"type":"heartbeat"}\n')
        g = S.GlobTailer(os.path.join(self.d, "x*.jsonl"), from_end=False)
        got = g.poll()
        self.assertEqual(len(got), 2)
        self.assertEqual({os.path.basename(p) for p, _ in got}, {"x1.jsonl", "x2.jsonl"})
        self.assertEqual(g.poll(), [])

    def test_end_to_end_file_to_wire_to_ack(self):
        """感知文件 → core → 报文 → 模拟车 ack → 台账对上（本机全链路，无需 ROS）。"""
        path = os.path.join(self.d, "perception.jsonl")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write('{"type":"bridge","direction":"left","hole":"K9","confidence":0.95}\n')
            fh.write('{"type":"target_zone","blocks":["G10","H10"]}\n')
            fh.write('{"type":"field_verify","mines":["A3"],"obstacles":["K9"]}\n')
            fh.write('garbage line\n')
            fh.write('{"type":"bridge_passable","half":"upper","block":"K4"}\n')

        c = core()
        g = S.GlobTailer(path, from_end=False)
        for src, line in g.poll():
            obj, err = S.sample_from_file_line(line)
            if obj is None:
                c.stats.samples_rejected += 1
                continue
            c.ingest_sample(obj, source=os.path.basename(src))

        self.assertEqual(c.stats.samples_seen, 4)
        self.assertEqual(c.stats.samples_rejected, 1)

        evs = c.tick(now=100.0)
        intel = {e.label: e for e in evs if e.kind == "intel"}
        self.assertIn("bridge_passable", intel)
        self.assertIn("target_zone", intel)
        self.assertIn("field_verify", intel)
        # bridge 两条（lower K9 / upper K4）必须都在 —— 不得互相覆盖
        bridges = [e for e in evs if e.label == "bridge_passable"]
        self.assertEqual(len(bridges), 2, [e.text for e in bridges])
        halves = sorted(json.loads(e.text)["half"] for e in bridges)
        self.assertEqual(halves, ["lower", "upper"])

        # 车侧 ack：逐条回，台账必须全部对上
        for e in evs:
            if e.seq is not None:
                c.ingest_car_raw(json.dumps({"type": "ack", "of": e.seq}), now=100.1)
        self.assertEqual(c.ledger.summary()["pending_seq"], 0)
        self.assertEqual(c.ledger.summary()["intel_acked"], 4)


class TestCommandFile(unittest.TestCase):

    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.path = os.path.join(self.d, "cmd.jsonl")

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def _write(self, text):
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(text)

    def test_route_reset_bridge_heartbeat_stop(self):
        c = core()
        cf = S.CommandFile(self.path)
        self._write('{"type":"route","data":"N10 M10 L10"}\n')
        self._write('{"type":"bridge","half":"lower","block":"K9"}\n')
        self._write('{"type":"heartbeat"}\n')
        self._write('{"type":"reset","block":"K11"}\n')
        self.assertEqual(cf.poll(c), 4)

        evs = c.tick(now=1.0)
        self.assertEqual(evs[0].kind, "route")
        self.assertEqual([e.text for e in evs if e.kind == "route"], ["N10 M10 L10"])
        # 路线一次只放一条出队（避免同一 tick 把两条互相矛盾的路线连发）
        evs2 = c.tick(now=1.1)
        self.assertEqual([e.text for e in evs2 if e.kind == "route"], ["reset:K11,y"])
        self.assertTrue(any(e.label == "bridge_passable" for e in evs))
        # 命令只消费一次：后面再 tick 也不会重复下发路线
        self.assertEqual([e for e in c.tick(now=99.0) if e.kind == "route"], [])

    def test_bad_route_in_command_file_is_rejected_not_crash(self):
        c = core()
        cf = S.CommandFile(self.path)
        self._write('{"type":"route","data":"N5,y M10"}\n')      # 形状非法
        self._write('{"type":"reset","block":"ZZ9"}\n')          # 格名非法
        self._write('{"type":"route"}\n')                        # 缺 data
        self.assertEqual(cf.poll(c), 0)
        self.assertEqual([e for e in c.tick(now=1.0) if e.kind == "route"], [])

    def test_bad_lines_do_not_raise(self):
        c = core()
        cf = S.CommandFile(self.path)
        self._write("not json\n")
        self._write('["array"]\n')
        self._write('{"type":"nope"}\n')
        self.assertEqual(cf.poll(c), 0)      # 一条都没被执行
        self.assertEqual(c.tick(now=2.0)[0].kind, "heartbeat")

    def test_partial_command_line_is_buffered(self):
        c = core()
        cf = S.CommandFile(self.path)
        self._write('{"type":"rout')
        self.assertEqual(cf.poll(c), 0)
        self._write('e","data":"N10 M10"}\n')
        self.assertEqual(cf.poll(c), 1)
        route = [e for e in c.tick(now=1.0) if e.kind == "route"]
        self.assertEqual(route[0].text, "N10 M10")

    def test_from_start_replays_existing_commands(self):
        self._write('{"type":"route","data":"N10 M10"}\n')
        c = core()
        self.assertEqual(S.CommandFile(self.path, from_end=False).poll(c), 1)
        route = [e for e in c.tick(now=1.0) if e.kind == "route"]
        self.assertEqual(route[0].text, "N10 M10")
        # 默认（from_end=True）不该重放历史命令
        c2 = core()
        self.assertEqual(S.CommandFile(self.path).poll(c2), 0)

    def test_route_from_file(self):
        rp = os.path.join(self.d, "r1.txt")
        with open(rp, "w", encoding="utf-8") as fh:
            fh.write("N10 M10 L10 L9 K9\n")
        c = core()
        cf = S.CommandFile(self.path)
        self._write(json.dumps({"type": "route", "file": rp}) + "\n")
        cf.poll(c)
        route = [e for e in c.tick(now=1.0) if e.kind == "route"]
        self.assertEqual(route[0].text, "N10 M10 L10 L9 K9")

    def test_stop_command(self):
        c = core()
        cf = S.CommandFile(self.path)
        self._write('{"type":"stop"}\n')
        cf.poll(c)
        self.assertTrue(c.stopping)


class TestUdpSource(unittest.TestCase):
    """UDP JSON 输入（只吃 JSON，不解析 dzuav 二进制帧）。"""

    def test_json_datagrams_are_ingested(self):
        import socket
        src = S.UdpJsonSource("127.0.0.1", 0)
        port = src.sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            tx.sendto(b'{"type":"bridge_passable","half":"lower","block":"K9"}\n',
                      ("127.0.0.1", port))
            tx.sendto(b'not json', ("127.0.0.1", port))
            got = src.poll()
            self.assertEqual(len(got), 1)
            self.assertEqual(got[0][1]["block"], "K9")
            self.assertEqual(src.packets, 2)
            self.assertEqual(src.rejected, 1)
        finally:
            tx.close()
            src.close()

    def test_udp_to_wire(self):
        import socket
        src = S.UdpJsonSource("127.0.0.1", 0)
        port = src.sock.getsockname()[1]
        c = core()
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            tx.sendto(b'{"type":"target_zone","blocks":["H11"]}', ("127.0.0.1", port))
            for source, obj in src.poll():
                c.ingest_sample(obj, source=source)
            evs = [e for e in c.tick(now=1.0) if e.label == "target_zone"]
            self.assertEqual(len(evs), 1)
            self.assertIn('"blocks":["H11"]', evs[0].text)
        finally:
            tx.close()
            src.close()


class TestLoggerAndStatus(unittest.TestCase):

    def setUp(self):
        self.d = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def test_logger_writes_jsonl(self):
        p = os.path.join(self.d, "sub", "log.jsonl")
        lg = S.JsonlLogger(p)
        lg.log("hello", a=1)
        lg.close()
        with open(p, "r", encoding="utf-8") as fh:
            rec = json.loads(fh.readline())
        self.assertEqual(rec["kind"], "hello")
        self.assertEqual(rec["a"], 1)

    def test_logger_none_is_noop(self):
        lg = S.JsonlLogger(None)
        lg.log("x")
        lg.close()
        self.assertEqual(lg.written, 0)

    def test_status_shape(self):
        c = core(bridge_period_s=1.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.tick(now=1.0)
        st = c.status(now=2.0)
        for k in ("version", "domain", "config", "intel", "stats", "ack_ledger",
                  "pending_routes", "car_contract"):
            self.assertIn(k, st)
        self.assertTrue(st["intel"]["bridge_passable:lower"]["fresh"])
        self.assertEqual(st["intel"]["bridge_passable:lower"]["type"], "bridge_passable")
        self.assertEqual(st["car_contract"]["info_stale_s"], 30.0)

    def test_core_config_normalises_bad_input(self):
        cfg = S.CoreConfig(heartbeat_period_s=-5, bridge_period_s=-1, info_ttl_s=0,
                           intel_repeat=0, route_repeat=-3, left_halves="UPPER").normalise()
        self.assertGreater(cfg.heartbeat_period_s, 0)
        self.assertEqual(cfg.bridge_period_s, 0.0)
        self.assertGreaterEqual(cfg.info_ttl_s, 1.0)
        self.assertEqual(cfg.intel_repeat, 1)
        self.assertEqual(cfg.route_repeat, 1)
        self.assertEqual(cfg.left_halves, "upper")

    def test_emit_order_is_bridge_first(self):
        """桥洞最有时效性（车侧等情报窗口只有 2 s）→ 必须先发。顺序必须确定，不靠字典序。"""
        c = core()
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K4"}, now=1.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K9"}, now=1.0)
        c.ingest_sample({"type": "target_zone", "blocks": ["G10"]}, now=1.0)
        c.ingest_sample({"type": "field_verify", "mines": ["A3"]}, now=1.0)
        labels = [e.label for e in c.tick(now=1.0) if e.kind == "intel"]
        self.assertEqual(labels, ["bridge_passable", "bridge_passable",
                                  "target_zone", "field_verify"])
        evs = [e for e in c.tick(now=30.0) if e.kind == "intel"]
        self.assertEqual([json.loads(e.text)["half"] for e in evs[:2]], ["lower", "upper"])

    def test_no_rclpy_import_at_module_level(self):
        """⑤ 的硬约束：纯逻辑层不得依赖 rclpy（Windows 本机必须能 import）。"""
        self.assertNotIn("rclpy", sys.modules)


# --------------------------------------------------------------------------- #
#  t12 回归：t4 真机验证抓到的无人机侧缺陷 D1 / D2
# --------------------------------------------------------------------------- #
import ast  # noqa: E402
import builtins  # noqa: E402
import contextlib  # noqa: E402
import inspect  # noqa: E402
import io  # noqa: E402
import textwrap  # noqa: E402
import time  # noqa: E402
from unittest import mock  # noqa: E402


class TestT12Fixes(unittest.TestCase):
    """t4 真机缺陷的回归测试（两个都不需要 ROS）。

    D1：`_run_ros` 里 `listen` 分支在 try 内 return，`finally` 却引用只在 try 之后
        才赋值的 `udp_src` → 功能正常但退出时 UnboundLocalError、退出码非 0。
    D2：`run` 子命令声明了 --bridge/--target/--mines/--obstacles，但 main 的 run 分支
        没调 `_add_samples_from_args` → argparse 照收、真跑 intel_emitted=0、零提示。
    """

    def setUp(self):
        self._saved_domain = os.environ.get("ROS_DOMAIN_ID")

    def tearDown(self):
        if self._saved_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = self._saved_domain

    # ---------- D1 ----------
    def test_d1_udp_src_initialised_before_try(self):
        src = textwrap.dedent(inspect.getsource(S._run_ros))
        i_init = src.find("udp_src = None")
        i_try = src.find("try:")
        self.assertNotEqual(i_init, -1, "_run_ros 必须给 udp_src 兜底赋值")
        self.assertNotEqual(i_try, -1, "_run_ros 应该有 try/finally")
        self.assertLess(i_init, i_try,
                        "udp_src = None 必须在 try **之前**：listen 分支在 try 内 return，"
                        "finally 仍会引用它（t4 D1 = UnboundLocalError + 退出码非 0）")

    def test_d1_finally_only_uses_names_bound_before_try(self):
        """通用化的 D1 守卫：`finally` 里用到的每个局部名，都必须在进入 try 之前已绑定。

        这样以后在 try 之后（或 try 体内）新增局部变量、又在 finally 里收尾，会被本测试拦住。
        """
        src = textwrap.dedent(inspect.getsource(S._run_ros))
        fn = ast.parse(src).body[0]
        args = {a.arg for a in fn.args.args}
        allowed = set(dir(builtins)) | set(vars(S)) | args

        tries = [n for n in fn.body if isinstance(n, ast.Try)]
        self.assertTrue(tries, "_run_ros 里应该有 try/finally")
        for t in tries:
            bound_before = set()
            for st in fn.body:
                if st is t:
                    break
                for n in ast.walk(st):
                    if isinstance(n, ast.Assign):
                        bound_before |= {tg.id for tg in n.targets
                                         if isinstance(tg, ast.Name)}
                    elif isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name):
                        bound_before.add(n.target.id)
                    elif isinstance(n, (ast.Import, ast.ImportFrom)):
                        bound_before |= {(a.asname or a.name.split(".")[0])
                                         for a in n.names}
            loaded = {n.id for h in t.finalbody for n in ast.walk(h)
                      if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
            unknown = loaded - bound_before - allowed
            self.assertFalse(unknown,
                             f"finally 引用了 try 之前未绑定的局部名：{sorted(unknown)}"
                             "（listen 分支会先 return，这会造成 UnboundLocalError）")

    # ---------- D2 ----------
    def _main_capturing_core(self, argv):
        cap = {}

        def fake_run(args, core, mode):
            cap["core"] = core
            cap["mode"] = mode
            return 0

        with mock.patch.object(S, "_run_ros", fake_run):
            rc = S.main(argv)
        return rc, cap

    def test_d2_run_bridge_injects_both_halves(self):
        rc, cap = self._main_capturing_core(
            ["run", "--no-log", "--domain", "118",
             "--bridge", "lower=K9", "--bridge", "upper=K4"])
        self.assertEqual(rc, 0)
        self.assertEqual(cap.get("mode"), "run")
        c = cap["core"]
        evs = c.tick(now=time.time())
        intel = [e for e in evs if e.kind == "intel"]
        labels = [e.label for e in intel]
        self.assertEqual(labels.count("bridge_passable"), 2,
                         f"run --bridge 必须两条半场都注入，实际 labels={labels}")
        self.assertTrue(any('"half":"lower"' in e.text for e in intel))
        self.assertTrue(any('"half":"upper"' in e.text for e in intel))

    def test_d2_run_target_and_field_inject(self):
        rc, cap = self._main_capturing_core(
            ["run", "--no-log", "--domain", "118",
             "--target", "G10,H10", "--mines", "A3", "--obstacles", "K9"])
        self.assertEqual(rc, 0)
        evs = cap["core"].tick(now=time.time())
        by_label = {e.label: json.loads(e.text) for e in evs if e.kind == "intel"}
        self.assertEqual(by_label["target_zone"]["blocks"], ["G10", "H10"])
        self.assertEqual(by_label["field_verify"]["mines"], ["A3"])
        self.assertEqual(by_label["field_verify"]["obstacles"], ["K9"])

    def test_d2_run_without_intel_args_stays_empty(self):
        """不给情报参数时不能被"顺手"注入任何东西（现役行为逐字保持）。"""
        rc, cap = self._main_capturing_core(["run", "--no-log", "--domain", "118"])
        self.assertEqual(rc, 0)
        self.assertEqual([e for e in cap["core"].tick(now=time.time())
                          if e.kind == "intel"], [])

    def test_d2_run_dry_run_does_not_double_inject(self):
        """`run --dry-run` 走 cmd_dry_run（内部已注入一次）→ 不能被重复注入。"""
        def boom(*a, **k):
            raise AssertionError("run --dry-run 不应该进 _run_ros")

        buf = io.StringIO()
        with mock.patch.object(S, "_run_ros", boom):
            with contextlib.redirect_stdout(buf):
                rc = S.main(["run", "--no-log", "--domain", "118", "--dry-run",
                             "--bridge", "lower=K9"])
        out = buf.getvalue()
        self.assertEqual(rc, 0)
        # 心跳 + 1 条 bridge = 2 类事件；重复注入会是 3 类
        self.assertIn("共 2 类事件", out, out)


# --------------------------------------------------------------------------- #
#  t16：桥洞两半场「同轮成对发出」——车侧等情报窗口只有 2 s
# --------------------------------------------------------------------------- #
class TestT16BridgePairing(unittest.TestCase):
    """车侧 `mission.bridge_info_wait_s`（2.0 s）只覆盖「两半场相隔 ≤2 s 到达」。

    先到的那半场入库；超窗口才到的半场只进遥测、路线已冻结。所以 supervisor 必须保证：
    **任一新鲜桥洞半场到点时，把所有新鲜桥洞半场一起发**（同一轮、间隔远小于 2 s）。
    真机实测（t16）：同轮 lower→upper 车侧接收间隔中位 0.015 s / 最大 0.022 s。
    """

    def test_halves_emitted_as_adjacent_pair_in_fixed_order(self):
        c = core(bridge_period_s=2.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K10"}, now=100.0)
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K5"}, now=100.0)
        evs = [e for e in c.tick(now=100.0) if e.kind == "intel"]
        idx = [i for i, e in enumerate(evs) if e.label == "bridge_passable"]
        self.assertEqual(len(idx), 2, [e.text for e in evs])
        self.assertEqual(idx[1] - idx[0], 1, "两半场必须相邻发出，中间不能夹别的情报")
        self.assertEqual([json.loads(evs[i].text)["half"] for i in idx], ["lower", "upper"])

    def test_late_second_half_pulls_the_first_one_along(self):
        """★ 核心保证：第二个半场晚来（相位已错开）时，第一个半场必须**同轮**再发一次。"""
        c = core(bridge_period_s=2.0, info_ttl_s=30.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K10"}, now=0.0)
        self.assertEqual(len([e for e in c.tick(now=0.0) if e.label == "bridge_passable"]), 1)
        # 5 s 后（远超窗口）第二半场才到 —— 老实现只会单发 upper，车侧永远看不到成对
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K5"}, now=5.0)
        evs = [e for e in c.tick(now=5.0) if e.label == "bridge_passable"]
        halves = [json.loads(e.text)["half"] for e in evs]
        self.assertEqual(halves, ["lower", "upper"],
                         "晚到的 upper 必须把 lower 一起带出来（同轮成对）")

    def test_pairing_does_not_spam_within_period(self):
        c = core(bridge_period_s=2.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K10"}, now=10.0)
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K5"}, now=10.0)
        self.assertEqual(len([e for e in c.tick(now=10.0) if e.label == "bridge_passable"]), 2)
        self.assertEqual([e for e in c.tick(now=11.0) if e.label == "bridge_passable"], [])
        self.assertEqual(len([e for e in c.tick(now=12.0) if e.label == "bridge_passable"]), 2,
                         "到点后仍然成对重发")

    def test_expired_half_is_not_resurrected_by_pairing(self):
        """过期半场只进 intel_stale、不下发；不能被"成对"逻辑复活。"""
        c = core(bridge_period_s=2.0, info_ttl_s=5.0)
        c.ingest_sample({"type": "bridge_passable", "half": "lower", "block": "K10"}, now=0.0)
        c.tick(now=0.0)
        c.ingest_sample({"type": "bridge_passable", "half": "upper", "block": "K5"}, now=20.0)
        evs = [e for e in c.tick(now=20.0) if e.label == "bridge_passable"]
        self.assertEqual([json.loads(e.text)["half"] for e in evs], ["upper"],
                         "过期的 lower 不得被带出来")
        st = c.status(now=20.0)
        self.assertFalse(st["intel"]["bridge_passable:lower"]["fresh"])
        self.assertTrue(st["intel"]["bridge_passable:upper"]["fresh"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
