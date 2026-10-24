# -*- coding: utf-8 -*-
r"""dzudp_core.py 快照溯源单测：钉死"仓内解析核 == 教员 udp_bridge.py"。

背景（2026-10-24 Lead 裁决①）：解析核**入仓一份**，但必须是钉死快照。
教员交接目录会搬走、机上没有这个文件 ⇒ 永久 importlib 外部路径 = 静默漂移。
所以入仓一份 + 这条 provenance 单测，把"全队永远用同一份解析核"变成机器判据。

【为什么是"去掉文件头后的 sha256"】
裁决要求的文件头（来源路径 / sha256 / 抓取日期 / 只同步不改）本身会改变文件字节，
所以**整个文件**的 sha256 不可能等于教员原件。本测试的做法与裁决等价且更强：

    sha256(本文件去掉文件头，即 PINNED_MARKER 行之后的全部字节) == PIN
    len(同一段) == PINNED_BYTES
    PIN 是**写死在本测试里的独立常量**（不读被测模块的常量自证）

额外再核：去掉文件头后的字节与教员原件 / 取证镜像**逐字节相等**（文件在时）。

复跑：
  python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_dzudp_core_provenance.py" -v
"""
from __future__ import annotations

import hashlib
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
LINK = HERE.parent
REPO = HERE.parents[3]
if str(LINK) not in sys.path:
    sys.path.insert(0, str(LINK))

import dzudp_adapter as A  # noqa: E402
import dzudp_core as CORE  # noqa: E402

#: 独立常量（本测试的唯一锚点；与模块内常量分开，防"自证"）
PIN = "9ff259a36ce40f70437b713d0911d44a122f678909206ad734f0cc94a85b64e8"
PIN_BYTES = 11821
PIN_DATE = "2026-10-24"
#: 分界行：与模块里的 PINNED_MARKER 必须一字不差
MARKER = b"# ==== BEGIN PINNED SNAPSHOT (udp_bridge.py verbatim) ===="

TEACHER = Path("D:/User/Desktop/教员交接/udp_bridge.py")
MIRROR = REPO / "_tmp_session" / "sqc-20261024" / "tools" / "udp_bridge.py"


def local_bytes() -> bytes:
    return Path(CORE.__file__).read_bytes()


def snapshot_payload() -> bytes:
    """去掉文件头：MARKER 行之后的全部字节（保留原始行尾）。"""
    data = local_bytes()
    needle = MARKER + b"\n"
    idx = data.find(needle)
    if idx < 0:
        raise AssertionError("dzudp_core.py 里找不到 PINNED_MARKER 分界行")
    return data[idx + len(needle):]


class TestPinnedSnapshot(unittest.TestCase):
    def test_local_snapshot_sha256_equals_pin(self):
        """核心断言：快照段 sha256 == PIN（文件头除外）。"""
        got = hashlib.sha256(snapshot_payload()).hexdigest()
        self.assertEqual(got, PIN, "dzudp_core.py 的快照段已漂移（不是教员原件了）")

    def test_local_snapshot_length_equals_pin(self):
        self.assertEqual(len(snapshot_payload()), PIN_BYTES)

    def test_module_constants_match_pin(self):
        """模块自己记的常量不许与独立 PIN 不一致（防手抄错）。"""
        self.assertEqual(CORE.PINNED_SHA256, PIN)
        self.assertEqual(CORE.PINNED_BYTES, PIN_BYTES)
        self.assertEqual(CORE.PINNED_DATE, PIN_DATE)
        self.assertEqual(CORE.PINNED_MARKER, MARKER)

    def test_marker_appears_exactly_once_as_separator(self):
        self.assertEqual(local_bytes().count(MARKER + b"\n"), 1)

    @unittest.skipUnless(TEACHER.is_file(), "教员原件不在（搬走了）：只核 PIN 与镜像")
    def test_bytes_equal_to_teacher_original(self):
        self.assertEqual(snapshot_payload(), TEACHER.read_bytes(),
                         "快照段与教员原件不是逐字节相同")

    @unittest.skipUnless(MIRROR.is_file(), "取证镜像不在")
    def test_bytes_equal_to_forensic_mirror(self):
        self.assertEqual(snapshot_payload(), MIRROR.read_bytes())

    def test_header_states_provenance(self):
        """裁决要求文件头写清：来源 / sha256 / 日期 / 只同步不改 / 改后重跑 51+105。"""
        head = Path(CORE.__file__).read_text(encoding="utf-8").split(
            MARKER.decode("ascii"), 1)[0]
        for token in ("来源路径", PIN, PIN_DATE, "只同步不改",
                      "test_dzudp_core_provenance.py"):
            with self.subTest(token=token):
                self.assertIn(token, head)
        # 重跑口径：两条命令都要出现（51 条 dl1 + 105 条 link）
        self.assertIn('discover -s dzsmartscout/tools/link/tests', head)
        self.assertIn('discover -s _tmp_session/agc-1024/dl1/tests', head)


class TestBorrowedSurface(unittest.TestCase):
    def test_publish_switch_must_stay_off(self):
        self.assertIs(CORE.PUBLISH_ENABLED, False, "解析核的发布开关被打开了")

    def test_borrowed_api_is_present(self):
        for name in ("crc16", "parse_intel", "parse_pose", "parse_route",
                     "TYPE_NAMES", "MAGIC", "VERSION", "MSG_INTEL", "MSG_POSE",
                     "MSG_ROUTE"):
            with self.subTest(name=name):
                self.assertTrue(hasattr(CORE, name), "解析核缺 " + name)
        self.assertEqual(sorted(CORE.TYPE_NAMES), list(range(13)))

    def test_adapter_resolves_to_in_repo_snapshot(self):
        """生产路径必须优先命中仓内快照（不是 _tmp_session、更不是教员目录）。"""
        a = A.BinaryIntelAdapter()
        self.assertEqual(Path(a.core.__file__).resolve(), Path(CORE.__file__).resolve())
        diag = a.diagnostics()["parse_core"]
        self.assertIs(diag["publish_enabled"], False)
        self.assertEqual(Path(diag["path"]).resolve(), Path(CORE.__file__).resolve())

    def test_candidate_order_puts_repo_snapshot_first(self):
        cands = A._core_candidates(REPO)          # noqa: SLF001 - 顺序是本卡契约
        names = [Path(c).name for c in cands]
        self.assertEqual(names[0], "dzudp_core.py")
        self.assertEqual(names[1], "udp_bridge.py")


if __name__ == "__main__":
    unittest.main(verbosity=2)
