# -*- coding: utf-8 -*-
r"""DL1 适配器单测（生产件 · dzsmartscout/tools/link/dzudp_adapter.py；纯离线）。

覆盖卡面「必测」六项 + 四条硬边界：
  1  13 类映射完备（含与机端 ObjectType / 教员 TYPE_NAMES 三处对账）
  2  坐标往返（154 格全闭合 + 与车端公式不可互换 + 残差拒收）
  3  未知类 / 未知 msg_id 拒收
  4  坏 CRC / 截断 / 声明长度不符 / NaN 拒收
  5  缺失字段保持缺失（不补 coverage、不伪造 field_layout、空类别整键不发）
  6  0x4003 只记录、零报文（源码无路线话题串 + 把 bridge() 换成炸弹仍能跑）

复跑：
  python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py" -v
  python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_dzudp_*.py"
（解析核是仓内钉死快照 dzudp_core.py，provenance 另见 test_dzudp_core_provenance.py；
  单写者/双发布自检见 test_dzudp_single_writer.py）
"""

from __future__ import annotations

import json
import struct
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
LINK = HERE.parent
REPO = HERE.parents[3]         # .../v1/dzsmartscout/tools/link/tests -> v1
if str(LINK) not in sys.path:
    sys.path.insert(0, str(LINK))

import dzudp_adapter as A  # noqa: E402

SAMPLE_INTEL = REPO / "_tmp_session" / "sqc-20261024" / "real_intel_13cls.hex"
SAMPLE_ROUTE = REPO / "_tmp_session" / "sqc-20261024" / "real_route.hex"
#: 机端部署源码快照（只读交叉对账用；不在则可跳过该条）
DRONE_INTEL_PROTOCOL = (REPO / "_tmp_session" / "AGC3" / "cloud" / "src" / "ag_integrate"
                        / "ag_integrate" / "intel_protocol.py")
DRONE_FIELD_FRAME = (REPO / "_tmp_session" / "AGC3" / "cloud" / "src" / "ag_integrate"
                     / "ag_integrate" / "field_frame.py")

ADAPTER = A.BinaryIntelAdapter.__module__
ADAPTER_SRC = Path(sys.modules[ADAPTER].__file__).read_text(encoding="utf-8")


def hexbytes(path: Path) -> bytes:
    return bytes.fromhex("".join(path.read_text(encoding="utf-8").split()))


# --------------------------------------------------------------------------- #
#  夹具：按机上 UdpBroadcaster 字节布局编码（只为造帧；用**解析核的 crc16**）
#  正确性由 test_reencode_real_samples_byte_equal 证明（重编码真样本逐字节相等）
# --------------------------------------------------------------------------- #
def fix_intel(objects, ts_ms: int = 1792828167761) -> bytes:
    core = A.load_parse_core()
    count = len(objects)
    out = bytearray()
    out += core.MAGIC
    out.append(core.VERSION)
    out += struct.pack("<QHIH", ts_ms, core.MSG_INTEL, 2 + 25 * count, count)
    for t, x, y, ln, wd in objects:
        out += struct.pack("<Bddff", int(t), float(x), float(y), float(ln), float(wd))
    body_end = 21 + 25 * count
    out += struct.pack("<H", core.crc16(memoryview(out)[4:body_end]))
    out += b"\xfe\xfe\xfe\xfe"
    return bytes(out)


def fix_pose(x: float, y: float, yaw: float, ts_ms: int = 1792828167761) -> bytes:
    core = A.load_parse_core()
    out = bytearray()
    out += core.MAGIC
    out.append(core.VERSION)
    out += struct.pack("<QHI", ts_ms, core.MSG_POSE, 20)
    out += struct.pack("<ddf", float(x), float(y), float(yaw))
    body_end = 19 + 20
    out += struct.pack("<H", core.crc16(memoryview(out)[4:body_end]))
    out += b"\xfe\xfe\xfe\xfe"
    return bytes(out)


def fix_route(reset: str, route: str, ts_ms: int = 1792828167761) -> bytes:
    core = A.load_parse_core()
    r1, r2 = reset.encode("utf-8"), route.encode("utf-8")
    payload = struct.pack("<H", len(r1)) + r1 + struct.pack("<H", len(r2)) + r2
    out = bytearray()
    out += core.MAGIC
    out.append(core.VERSION)
    out += struct.pack("<QHI", ts_ms, core.MSG_ROUTE, len(payload))
    out += payload
    body_end = 19 + len(payload)
    out += struct.pack("<H", core.crc16(memoryview(out)[4:body_end]))
    out += b"\xfe\xfe\xfe\xfe"
    return bytes(out)


def obj_at(block: str, type_index: int, *, dx: float = 0.0, dy: float = 0.0):
    col = A.LETTERS.index(block[0].upper())
    row = int(block[1:]) - 1
    x, y = A.cell_to_world(col, row)
    return (type_index, x + dx, y + dy, 0.08, 0.06)


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a = A.BinaryIntelAdapter()


# --------------------------------------------------------------------------- #
#  1) 13 类映射完备
# --------------------------------------------------------------------------- #
class TestClassTable(Base):
    def test_table_covers_exactly_13(self):
        self.assertEqual(sorted(A.CLASS_TABLE), list(range(13)))
        self.assertEqual(len(A.CLASS_TABLE), 13)

    def test_every_role_is_known_and_all_roles_used(self):
        roles = {r.role for r in A.CLASS_TABLE.values()}
        self.assertEqual(roles, set(A.JSON_ROLES))

    def test_every_class_has_explanatory_note(self):
        for idx, rule in A.CLASS_TABLE.items():
            with self.subTest(type=idx, name=rule.name):
                self.assertTrue(rule.name.isupper())
                self.assertTrue(rule.note.strip(), f"{rule.name} 缺理由（映射必须可解释）")

    def test_names_match_teacher_parse_core(self):
        core = A.load_parse_core()
        self.assertEqual(sorted(core.TYPE_NAMES), list(range(13)))
        for idx, rule in A.CLASS_TABLE.items():
            self.assertEqual(rule.name, core.TYPE_NAMES[idx],
                             f"type {idx} 名字与解析核不一致（13 类是硬约束）")

    @unittest.skipUnless(DRONE_INTEL_PROTOCOL.is_file(), "机端源码快照不在")
    def test_names_match_drone_deployed_enum(self):
        """第二独立证据：机端部署的 intel_protocol.ObjectType 顺序必须逐项相同。"""
        import importlib.util
        spec = importlib.util.spec_from_file_location("_drone_intel", DRONE_INTEL_PROTOCOL)
        mod = importlib.util.module_from_spec(spec)
        sys.modules["_drone_intel"] = mod      # dataclass 需要模块已注册
        spec.loader.exec_module(mod)
        names = [m.name for m in mod.ObjectType]
        self.assertEqual(len(names), 13)
        self.assertEqual([A.CLASS_TABLE[i].name for i in range(13)], names)

    def test_only_paper_obstacle_target_reach_json(self):
        roles = {i: r.role for i, r in A.CLASS_TABLE.items()}
        self.assertEqual(roles[0], "paper")
        self.assertEqual(roles[5], "obstacle")
        self.assertEqual(roles[3], "target")
        self.assertEqual(roles[4], "target")
        for i in (1, 2, 6, 7, 8, 9, 10, 11, 12):
            self.assertEqual(roles[i], "record_only", f"type {i} 不许有 JSON 落点")


# --------------------------------------------------------------------------- #
#  2) 坐标往返 / 帧不可互换 / 残差
# --------------------------------------------------------------------------- #
class TestCoordinates(Base):
    def test_all_cells_roundtrip(self):
        n = 0
        for col in range(A.COLS):
            for row in range(A.ROWS):
                x, y = A.cell_to_world(col, row)
                cell = A.world_to_cell(x, y)
                self.assertEqual((cell.col, cell.row), (col, row),
                                 f"{A.LETTERS[col]}{row + 1} 往返失败")
                self.assertAlmostEqual(cell.max_residual, 0.0, places=9)
                n += 1
        self.assertEqual(n, 154)

    @unittest.skipUnless(DRONE_FIELD_FRAME.is_file(), "机端 field_frame.py 快照不在")
    def test_matches_drone_side_implementation(self):
        """与机端 field_frame 逐格同解（独立实现，同结果）。"""
        import importlib.util
        spec = importlib.util.spec_from_file_location("_drone_ff", DRONE_FIELD_FRAME)
        ff = importlib.util.module_from_spec(spec)
        sys.modules["_drone_ff"] = ff          # dataclass 需要模块已注册
        spec.loader.exec_module(ff)
        self.assertEqual((ff.X0, ff.Y0, ff.CELL_SIZE, ff.COLS, ff.ROWS),
                         (A.X0, A.Y0, A.CELL_M, A.COLS, A.ROWS))
        self.assertEqual(ff.MAX_CELL_RESIDUAL, A.MAX_CELL_RESIDUAL)
        for col in range(A.COLS):
            for row in range(A.ROWS):
                x, y = A.cell_to_world(col, row)
                self.assertEqual(ff.cell_to_world(ff.CellRef(col, row)), (x, y))
                got = ff.world_to_cell(x, y)
                self.assertEqual((got.col, got.row), (col, row))

    def test_car_frame_is_not_interchangeable(self):
        """拿车端 geometry（perception/grid.GridSpec）反解机端坐标 ⇒ 错格（且不报错）。"""
        from dzsmartscout.tools.perception.grid import GridSpec
        spec = GridSpec()
        x, y = A.cell_to_world(0, 0)          # 机端 A1
        drone_cell = A.world_to_cell(x, y)
        car_block = spec.meters_to_block(x, y)
        self.assertEqual(drone_cell.name, "A1")
        self.assertNotEqual(car_block, drone_cell.name,
                            "两套帧若等价，本适配器的帧常数就白写了（此断言是防混用的门）")
        self.assertEqual(car_block, "D11")     # 车端公式的车端结论，与真值无关

    def test_residual_reject_boundary(self):
        x, y = A.cell_to_world(4, 4)          # E5 格心
        # 0.28 格（9.8cm）以内接受
        self.assertEqual(A.world_to_cell(x + 0.28 * A.CELL_M, y).name, "E5")
        # 0.37 格（13cm）拒收：超过 0.35 格阈值
        with self.assertRaises(A.CellResidualTooLarge):
            A.world_to_cell(x + 0.37 * A.CELL_M, y)
        # y 方向同理（两个轴各自判）
        with self.assertRaises(A.CellResidualTooLarge):
            A.world_to_cell(x, y - 0.40 * A.CELL_M)

    def test_out_of_field_and_nan_rejected(self):
        for x, y in ((-0.5, 1.0), (5.0, 1.0), (1.0, -0.5), (1.0, 5.2),
                     (float("nan"), 1.0), (1.0, float("inf"))):
            with self.subTest(x=x, y=y):
                with self.assertRaises(A.CellOutOfField):
                    A.world_to_cell(x, y)

    def test_half_boundary_is_row_6(self):
        self.assertEqual(A.world_to_cell(*A.cell_to_world(10, 5)).half, "upper")  # K6
        self.assertEqual(A.world_to_cell(*A.cell_to_world(10, 6)).half, "lower")  # K7


# --------------------------------------------------------------------------- #
#  3) 未知类 / 未知 msg_id
# --------------------------------------------------------------------------- #
class TestUnknown(Base):
    def test_unknown_object_type_rejected_whole_frame(self):
        for bad in (13, 14, 200):
            with self.subTest(type=bad):
                res = self.a.adapt(fix_intel([obj_at("E5", bad)]))
                self.assertEqual(res.kind, "bad")
                self.assertEqual(res.messages, [])
                self.assertTrue(any("unknown type" in e for e in res.errors), res.errors)

    def test_unknown_msg_id_rejected(self):
        core = A.load_parse_core()
        raw = bytearray(fix_pose(1.0, 2.0, 0.0))
        struct.pack_into("<H", raw, 13, 0x4004)
        # 重算 CRC 也没用：msg_id 先被 dispatch 挡下
        out = self.a.adapt(bytes(raw))
        self.assertEqual(out.kind, "bad")
        self.assertIn("0x4004", out.errors[0])
        self.assertEqual(out.messages, [])
        self.assertIsNotNone(core)          # 用到了同一个核，不是另写的判定

    def test_non_protocol_garbage(self):
        for raw in (b"", b"\x00" * 40, b"hello world", b"\xef\xef\xef\xef\x11" + b"\x00" * 20):
            with self.subTest(raw=raw[:8]):
                res = self.a.adapt(raw)
                self.assertEqual(res.kind, "bad")
                self.assertEqual(res.messages, [])

    def test_non_bytes_input(self):
        res = self.a.adapt("not bytes")       # type: ignore[arg-type]
        self.assertEqual(res.kind, "bad")


# --------------------------------------------------------------------------- #
#  4) 坏 CRC / 截断 / 长度 / NaN
# --------------------------------------------------------------------------- #
class TestBadFrames(Base):
    def test_crc_mismatch(self):
        raw = bytearray(fix_intel([obj_at("A3", 0)]))
        raw[25] ^= 0xFF                      # 翻一个 body 字节，不重算 CRC
        res = self.a.adapt(bytes(raw))
        self.assertEqual(res.kind, "bad")
        self.assertIn("crc", " ".join(res.errors).lower())

    def test_truncated(self):
        good = fix_intel([obj_at("A3", 0), obj_at("B4", 5)])
        for cut in (1, 5, 21, len(good) // 2, len(good) - 1):
            with self.subTest(cut=cut):
                res = self.a.adapt(good[:cut])
                self.assertEqual(res.kind, "bad")
                self.assertEqual(res.messages, [])

    def test_declared_length_mismatch(self):
        raw = bytearray(fix_intel([obj_at("A3", 0)]))
        struct.pack_into("<I", raw, 15, 52)   # 谎报 declen（1 个对象应为 27）
        res = self.a.adapt(bytes(raw))
        self.assertEqual(res.kind, "bad")
        self.assertIn("len", " ".join(res.errors).lower())

    def test_count_too_big_for_length(self):
        raw = bytearray(fix_intel([obj_at("A3", 0)]))
        struct.pack_into("<H", raw, 19, 5)    # count=5 但只带 1 个对象
        res = self.a.adapt(bytes(raw))
        self.assertEqual(res.kind, "bad")

    def test_nan_object(self):
        res = self.a.adapt(fix_intel([(0, float("nan"), 1.0, 0.1, 0.1)]))
        self.assertEqual(res.kind, "bad")
        self.assertIn("NaN", " ".join(res.errors))

    def test_bad_suffix(self):
        raw = bytearray(fix_intel([obj_at("A3", 0)]))
        raw[-1] = 0x00
        self.assertEqual(self.a.adapt(bytes(raw)).kind, "bad")

    def test_bad_frames_do_not_poison_counters(self):
        a = A.BinaryIntelAdapter()
        a.adapt(fix_intel([obj_at("A3", 0)]))
        before = dict(a.stats)
        a.adapt(b"garbage")
        self.assertEqual(a.stats["intel"], before["intel"])
        self.assertEqual(a.stats["bad"], before["bad"] + 1)


# --------------------------------------------------------------------------- #
#  5) 缺失字段保持缺失
# --------------------------------------------------------------------------- #
class TestMissingStaysMissing(Base):
    def test_no_coverage_slice_scan_id_field_layout_ever(self):
        frames = [
            fix_intel([obj_at("A3", 0)]),
            fix_intel([obj_at("A3", 0), obj_at("K9", 0)]),
            fix_intel([obj_at("D4", 4), obj_at("F6", 5)]),
            fix_intel([obj_at("B2", 1)]),
        ]
        for raw in frames:
            res = self.a.adapt(raw, now=1792828167.0)
            for msg in res.messages:
                for key in ("coverage", "slice", "scan_id", "targets"):
                    self.assertNotIn(key, msg, f"{msg.get('type')} 不许补 {key}")
                self.assertNotEqual(msg.get("type"), "field_layout")
            self.assertEqual(res.records["absent_in_binary"],
                             ["coverage", "slice", "scan_id", "field_layout",
                              "per_object_confidence"])

    def test_empty_class_keys_are_omitted_not_faked(self):
        # 只有雷：obstacles 键必须**缺席**（不是 []）
        res = self.a.adapt(fix_intel([obj_at("A3", 0)]))
        self.assertEqual(res.message_types, ["field_verify"])
        fv = res.messages[0]
        self.assertEqual(fv["mines"], ["A3"])
        self.assertNotIn("obstacles", fv)

        # 只有障碍：mines 键缺席
        res = self.a.adapt(fix_intel([obj_at("F6", 5)]))
        fv = res.messages[0]
        self.assertEqual(fv["obstacles"], ["F6"])
        self.assertNotIn("mines", fv)

    def test_frame_with_nothing_to_report_emits_no_message(self):
        # 只有格号/车/桶/起终点：二进制里"没有雷障靶"，但没有 coverage ⇒ 一条都不许发
        objs = [obj_at("B2", 1), obj_at("C3", 2), obj_at("G7", 6),
                obj_at("H8", 7), obj_at("I9", 8), obj_at("J10", 9)]
        res = self.a.adapt(fix_intel(objs))
        self.assertEqual(res.messages, [])
        self.assertEqual(res.records["n_objects"], 6)

    def test_include_empty_classes_is_opt_in_only(self):
        a = A.BinaryIntelAdapter(A.AdapterConfig(include_empty_classes=True))
        res = a.adapt(fix_intel([obj_at("B2", 1)]))
        self.assertEqual(res.message_types, ["field_verify"])
        self.assertEqual(res.messages[0]["mines"], [])
        self.assertEqual(res.messages[0]["obstacles"], [])
        # 默认适配器仍然不发
        self.assertEqual(self.a.adapt(fix_intel([obj_at("B2", 1)])).messages, [])

    def test_no_heartbeat_by_default(self):
        for raw in (fix_intel([obj_at("A3", 0)]), fix_pose(1.0, 2.0, 0.0),
                    fix_route("reset:N10,x", "N10 K10 M10 L10,x")):
            res = self.a.adapt(raw)
            self.assertNotIn("heartbeat", res.message_types)


# --------------------------------------------------------------------------- #
#  6) 0x4003 只记录、零报文
# --------------------------------------------------------------------------- #
class TestRouteNeverPublished(Base):
    def test_adapter_source_has_no_route_topic_string(self):
        """源码里连路线话题的字符串都没有 ⇒ 结构上不可能有发布路径。"""
        self.assertNotIn("/nav/waypoint", ADAPTER_SRC)

    def test_route_frame_records_only(self):
        raw = fix_route("reset:N10,x", "N10 K10 M10 L10,x")
        res = self.a.adapt(raw)
        self.assertEqual(res.kind, "route")
        self.assertEqual(res.messages, [])
        self.assertEqual(res.encoded, [])
        self.assertEqual(res.records["reset"], "reset:N10,x")
        self.assertEqual(res.records["route"], "N10 K10 M10 L10,x")
        self.assertEqual(res.records["tokens"], ["N10", "K10", "M10", "L10,x"])
        self.assertEqual(res.records["n_tokens"], 4)
        self.assertIsNone(res.records["json_landing"])
        self.assertIn("只记录", res.records["publish"])

    def test_replacing_core_bridge_with_bomb_still_works(self):
        """把解析核的 bridge()/sniff_loop() 换成炸弹，路线帧仍不应触发任何发布。"""
        a = A.BinaryIntelAdapter()

        def bomb(*_a, **_k):
            raise AssertionError("适配器不许调用解析核的发布/sniff 入口")

        for name in ("bridge", "sniff", "sniff_loop"):
            if hasattr(a.core, name):
                setattr(a.core, name, bomb)
        res = a.adapt(fix_route("reset:A1", "A1 B2,x"))
        self.assertEqual(res.kind, "route")
        self.assertEqual(res.messages, [])

    def test_publish_switch_of_core_stayed_off(self):
        core = A.load_parse_core()
        self.assertIs(core.PUBLISH_ENABLED, False)
        # 适配器只借解析函数，诊断里明示
        diag = self.a.diagnostics()
        self.assertIs(diag["parse_core"]["publish_enabled"], False)
        self.assertEqual(diag["parse_core"]["used_for"],
                         ["crc16", "parse_intel", "parse_pose", "parse_route"])


# --------------------------------------------------------------------------- #
#  真帧重放（352B / 13 对象；57B / 4 航点）
# --------------------------------------------------------------------------- #
class TestRealSamples(Base):
    @unittest.skipUnless(SAMPLE_INTEL.is_file(), "真帧样本不在")
    def test_real_intel_frame(self):
        raw = hexbytes(SAMPLE_INTEL)
        self.assertEqual(len(raw), 352, "机端真帧长度必须是 352B")
        res = self.a.adapt(raw, now=1792828167.761)
        self.assertEqual(res.kind, "intel")
        self.assertEqual(res.records["n_objects"], 13)
        self.assertEqual(res.records["frame_ts_ms"], 1792828167761)

        cells = {o["class"]: o["cell"] for o in res.records["objects"]}
        self.assertEqual(cells["PAPER"], "A1")
        self.assertEqual(cells["CELL_LABEL"], "B2")
        self.assertEqual(cells["CAR"], "C3")
        self.assertEqual(cells["TARGET_RED"], "D4")
        self.assertEqual(cells["TARGET_BLUE"], "E5")
        self.assertEqual(cells["OBSTACLE"], "F6")
        self.assertEqual(cells["BUCKET_CYAN"], "I9")
        self.assertEqual(cells["CAR_END"], "K11")
        # 样本末两个对象在 x 继续增大处折回第 1/2 行（不是 L12/M13）⇒ 全 13 个都在场内
        self.assertEqual(cells["UAV_START"], "L1")
        self.assertEqual(cells["UAV_END"], "M2")
        self.assertEqual(res.rejected, [])

        self.assertEqual(res.message_types, ["target_zone", "field_verify"])
        self.assertEqual(res.messages[0]["blocks"], ["D4", "E5"])
        self.assertEqual(res.messages[1]["mines"], ["A1"])
        self.assertEqual(res.messages[1]["obstacles"], ["F6"])
        # K 列的 CAR_END 不是纸片 ⇒ 不许冒充桥洞
        self.assertEqual(res.records["bridge_passable"], {"lower": [], "upper": []})

    @unittest.skipUnless(SAMPLE_ROUTE.is_file(), "真帧样本不在")
    def test_real_route_frame(self):
        raw = hexbytes(SAMPLE_ROUTE)
        self.assertEqual(len(raw), 57, "机端真帧长度必须是 57B")
        res = self.a.adapt(raw)
        self.assertEqual(res.kind, "route")
        self.assertEqual(res.messages, [])
        self.assertEqual(res.records["reset"], "reset:N10,x")
        self.assertEqual(res.records["tokens"], ["N10", "K10", "M10", "L10,x"])

    @unittest.skipUnless(SAMPLE_INTEL.is_file() and SAMPLE_ROUTE.is_file(), "真帧样本不在")
    def test_reencode_real_samples_byte_equal(self):
        """夹具编码器与线上编码器等价：真样本解出来再编回去，逐字节相等。"""
        core = A.load_parse_core()
        intel = hexbytes(SAMPLE_INTEL)
        ts, objs = core.parse_intel(intel)
        self.assertEqual(fix_intel(objs, ts), intel)
        route = hexbytes(SAMPLE_ROUTE)
        ts2, reset_cmd, text = core.parse_route(route)
        self.assertEqual(fix_route(reset_cmd, text, ts2), route)
        pose = fix_pose(1.25, -3.5, 0.75, 1792828167761)
        self.assertEqual(fix_pose(*core.parse_pose(pose)[1:], core.parse_pose(pose)[0]), pose)


# --------------------------------------------------------------------------- #
#  语义：桥洞 / 靶 / 位姿 / 计数
# --------------------------------------------------------------------------- #
class TestSemantics(Base):
    def test_bridge_marker_only_in_k_column(self):
        res = self.a.adapt(fix_intel([obj_at("K9", 0), obj_at("K4", 0),
                                      obj_at("J9", 0)]))
        # 两条 bridge + 一条 field_verify（J9 纸片是雷，K 列两条是桥洞，不在同一键里）
        self.assertEqual(res.message_types,
                         ["bridge_passable", "bridge_passable", "field_verify"])
        got = [(m["half"], m["block"]) for m in res.messages[:2]]
        self.assertEqual(got, [("lower", "K9"), ("upper", "K4")])
        for m in res.messages[:2]:
            self.assertEqual(m["confidence"], 1.0)      # 合成常数，且是机端现行值
        self.assertIn("bridge_passable.confidence", res.records["synthetic_fields"])
        # J9 纸片是雷，不是桥洞
        self.assertEqual(res.records["mines"], ["J9"])

    def test_multiple_bridge_cells_per_half_warns_and_takes_first(self):
        """同半场多格：只报排序首格 + 告警（与机端 dzbrain_bridge 的 sorted()[0] 同口径）。

        注意排序是**字典序**（"K10" < "K9"），这是既有生产者的真实行为，
        不是本层引入的新规则——保持等价比"看起来更合理"重要。
        """
        res = self.a.adapt(fix_intel([obj_at("K9", 0), obj_at("K10", 0)]))
        self.assertEqual(len(res.messages), 1)
        self.assertEqual(res.messages[0]["block"], "K10")
        self.assertTrue(any("多个桥洞格" in w for w in res.warnings))

    def test_out_of_field_object_rejected_per_object(self):
        """单个出界目标不拖累同帧其它目标（真机误检/场地外目标必然出现）。"""
        objs = [obj_at("A3", 0),
                (0, 5.20, 1.00, 0.08, 0.06),        # x 远超场地宽（col≈14）
                obj_at("F6", 5)]
        res = self.a.adapt(fix_intel(objs))
        self.assertEqual(res.kind, "intel")
        self.assertEqual(len(res.rejected), 1)
        self.assertIn("out of field", res.rejected[0]["reject_reason"])
        self.assertEqual(res.messages[0]["mines"], ["A3"])
        self.assertEqual(res.messages[0]["obstacles"], ["F6"])

    def test_custom_bridge_confidence(self):
        a = A.BinaryIntelAdapter(A.AdapterConfig(bridge_confidence=0.9))
        res = a.adapt(fix_intel([obj_at("K9", 0)]))
        self.assertEqual(res.messages[0]["confidence"], 0.9)

    def test_red_and_blue_both_in_target_zone_sorted(self):
        res = self.a.adapt(fix_intel([obj_at("E5", 4), obj_at("D4", 3), obj_at("E5", 4)]))
        self.assertEqual(res.messages[0]["type"], "target_zone")
        self.assertEqual(res.messages[0]["blocks"], ["D4", "E5"])   # 去重 + 排序

    def test_same_cell_deduped(self):
        res = self.a.adapt(fix_intel([obj_at("A3", 0), obj_at("A3", 0)]))
        self.assertEqual(res.messages[0]["mines"], ["A3"])

    def test_pose_records_only(self):
        res = self.a.adapt(fix_pose(*A.cell_to_world(3, 3), 0.5))
        self.assertEqual(res.kind, "pose")
        self.assertEqual(res.messages, [])
        self.assertEqual(res.records["cell"]["cell"], "D4")
        self.assertAlmostEqual(res.records["yaw_deg"], 28.65, places=2)
        self.assertIsNone(res.records["json_landing"])
        self.assertIn("car_pose", res.records["note"])

    def test_seq_optional(self):
        counter = iter(range(1, 100))
        res = self.a.adapt(fix_intel([obj_at("A3", 0), obj_at("K9", 0), obj_at("D4", 3)]),
                           next_seq=lambda: next(counter))
        self.assertEqual([m.get("seq") for m in res.messages], [1, 2, 3])
        # 不给 next_seq 就不塞 seq（不伪造 ack 语义）
        self.assertNotIn("seq", self.a.adapt(fix_intel([obj_at("A3", 0)])).messages[0])

    def test_stats_and_diagnostics(self):
        a = A.BinaryIntelAdapter()
        a.adapt(fix_intel([obj_at("A3", 0)]))
        a.adapt(fix_pose(1.0, 1.0, 0.0))
        a.adapt(fix_route("r", "A1,x"))
        a.adapt(b"junk")
        self.assertEqual((a.stats["intel"], a.stats["pose"], a.stats["route"],
                          a.stats["bad"]), (1, 1, 1, 1))
        diag = a.diagnostics()
        self.assertEqual(sorted(diag["class_table"]), list(range(13)))
        self.assertEqual(diag["frame_constants"]["max_cell_residual"], 0.35)


# --------------------------------------------------------------------------- #
#  接入既有入口：编出来的报文必须被车侧口径"吃得下"
# --------------------------------------------------------------------------- #
class TestExistingEntryCompatibility(Base):
    def test_encoded_messages_pass_car_side_mirror(self):
        """用仓内 protocol.mirror_car_side_parse 复述车侧判定：不得出现 missing_fields。"""
        proto = self.a.protocol
        frames = [fix_intel([obj_at("A3", 0), obj_at("F6", 5)]),
                  fix_intel([obj_at("K9", 0)]),
                  fix_intel([obj_at("D4", 4)])]
        seen = set()
        for raw in frames:
            res = self.a.adapt(raw, now=1792828167.0)
            for body in res.encoded:
                verdict = proto.mirror_car_side_parse(body)
                self.assertTrue(verdict["raw_ok"], verdict)
                self.assertNotIn("missing_fields", verdict["action"], verdict)
                self.assertIn(verdict["type"], proto.PAYLOAD_TYPES)
                seen.add(verdict["type"])
        self.assertEqual(seen, {"field_verify", "bridge_passable", "target_zone"})

    def test_payloads_validate_with_existing_validator(self):
        proto = self.a.protocol
        res = self.a.adapt(fix_intel([obj_at("A3", 0), obj_at("K9", 0),
                                      obj_at("D4", 4), obj_at("F6", 5)]))
        self.assertEqual(len(res.messages), 3)
        for msg in res.messages:
            self.assertEqual(proto.validate_payload(msg), [], msg)

    def test_encoding_is_compact_utf8_json(self):
        res = self.a.adapt(fix_intel([obj_at("A3", 0)]))
        body = res.encoded[0]
        self.assertEqual(body, json.dumps(json.loads(body), ensure_ascii=False,
                                          separators=(",", ":")))
        self.assertNotIn(" ", body)


# --------------------------------------------------------------------------- #
#  7) 输出面边界：唯一话题 / route·pose 零发布 / 单写者声明
# --------------------------------------------------------------------------- #
CAR_CONTRACT = REPO / "dzrobot_v2" / "src" / "dzbrain" / "dzbrain" / "io" / "contract.py"


class TestPublishBoundary(Base):
    def test_single_output_topic_is_declared(self):
        self.assertEqual(A.DRONE_TO_CAR_TOPIC, "/dzlink/drone_to_car")
        self.assertEqual(A.DRONE_TO_CAR_QOS, {"depth": 10, "reliability": "best_effort"})
        pub = self.a.diagnostics()["publish"]
        self.assertEqual(pub["topic"], A.DRONE_TO_CAR_TOPIC)
        self.assertEqual(pub["topics_total"], 1, "本层只许有一个输出话题")
        self.assertEqual(pub["max_publishers"], 1, "同话题只许一个生产者（机端 dzbrain_bridge 已在）")

    @unittest.skipUnless(CAR_CONTRACT.is_file(), "车端契约源码不在")
    def test_topic_and_qos_match_car_side_contract(self):
        text = CAR_CONTRACT.read_text(encoding="utf-8")
        self.assertIn('"drone_in": Topic("/dzlink/drone_to_car", "std_msgs/String", QOS_LINK', text)
        self.assertIn('QOS_LINK = QosSpec(depth=10, reliability="best_effort")', text)

    def test_topic_is_none_when_nothing_may_be_published(self):
        self.assertEqual(self.a.adapt(fix_intel([obj_at("A3", 0)])).topic,
                         A.DRONE_TO_CAR_TOPIC)
        for raw in (fix_route("reset:A1,x", "A1 B2,x"), fix_pose(1.0, 2.0, 0.0), b"junk"):
            with self.subTest(kind=raw[:8]):
                self.assertIsNone(self.a.adapt(raw).topic)

    def test_payload_types_are_within_car_side_whitelist(self):
        """编出来的报文类型必须已在车侧白名单里（未知 type 会被静默忽略）。"""
        proto = self.a.protocol
        frames = [fix_intel([obj_at("A3", 0)]), fix_intel([obj_at("K9", 0)]),
                  fix_intel([obj_at("D4", 3)])]
        for raw in frames:
            for msg in self.a.adapt(raw).messages:
                self.assertIn(msg["type"], proto.PAYLOAD_TYPES)
                self.assertNotEqual(msg["type"], "field_layout", "布局必须由机端规划侧发 JSON")


# --------------------------------------------------------------------------- #
#  8) 单写者声明面（自检本体在 test_dzudp_single_writer.py）
# --------------------------------------------------------------------------- #
class TestSingleWriterDeclared(Base):
    def test_runtime_constraint_is_machine_readable(self):
        self.assertIn("禁止同 topic 双写", A.RUNTIME_CONSTRAINT)
        self.assertIn("/dzlink/drone_to_car", A.RUNTIME_CONSTRAINT)
        self.assertEqual(self.a.diagnostics()["runtime_constraint"], A.RUNTIME_CONSTRAINT)

    def test_diagnostics_points_at_selfcheck(self):
        pub = self.a.diagnostics()["publish"]
        self.assertIn("--selfcheck", pub["selfcheck"])
        self.assertEqual(pub["max_publishers"], 1)

    def test_guard_and_error_are_importable(self):
        self.assertTrue(callable(A.single_writer_preflight))
        g = A.SingleWriterGuard()
        self.assertEqual(g.topic, A.DRONE_TO_CAR_TOPIC)
        self.assertTrue(issubclass(A.DoublePublishError, RuntimeError))
        # 默认 claim 落系统临时目录（不同话题互不干扰）
        self.assertIn("dzudp_adapter.", Path(g.claim_path).name)


if __name__ == "__main__":
    unittest.main(verbosity=2)
