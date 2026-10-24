# -*- coding: utf-8 -*-
r"""单写者/双发布自检单测（Lead 裁决③：替代不并存 + 运行期自检判红）。

被测：dzudp_adapter.SingleWriterGuard / single_writer_preflight / --selfcheck CLI。
三条通道全部可注入，因此**纯离线**、不依赖 rclpy、不依赖 /proc、不连网：

  ① second_instance —— claim 文件里记着另一个**存活** pid（跨进程互斥）
  ② ros_graph       —— 同一话题 publisher 数 > 1
  ③ foreign_process —— 同机还跑着 dzbrain_bridge（机上同语义第二实现）

判据：任一 red ⇒ status="red"、assert_ok() 抛 DoublePublishError、CLI 退出码 2；
unknown **不判红**（离线 Windows 没有 rclpy / /proc 是正常态）。

复跑：
  python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_dzudp_single_writer.py" -v
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
LINK = HERE.parent
REPO = HERE.parents[3]
if str(LINK) not in sys.path:
    sys.path.insert(0, str(LINK))

import dzudp_adapter as A  # noqa: E402

ADAPTER_PY = str(LINK / "dzudp_adapter.py")


class ClaimCase(unittest.TestCase):
    """公共夹具：临时 claim 文件。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.claim = Path(self._tmp.name) / "dzudp_adapter.dzlink_drone_to_car.owner"

    def write_claim(self, pid: int, *, ts: float | None = None,
                    topic: str = A.DRONE_TO_CAR_TOPIC) -> None:
        rec = {"pid": int(pid), "ts": round(time.time() if ts is None else ts, 3),
               "topic": topic, "host": "test"}
        self.claim.parent.mkdir(parents=True, exist_ok=True)
        self.claim.write_text(json.dumps(rec), encoding="utf-8")

    def guard(self, **kw):
        kw.setdefault("claim_path", self.claim)
        return A.SingleWriterGuard(**kw)


# --------------------------------------------------------------------------- #
#  ① claim 通道
# --------------------------------------------------------------------------- #
class TestClaimChannel(ClaimCase):
    def test_idle_claim_is_ok(self):
        rep = self.guard(own_pid=1, publisher_probe=lambda t: [], proc_reader=lambda: []).preflight()
        self.assertEqual(rep["channels"][0]["status"], "ok")
        self.assertEqual(rep["status"], "ok")
        self.assertTrue(rep["ok"])

    def test_claim_owned_by_self_is_ok(self):
        self.write_claim(os.getpid())
        rep = self.guard(own_pid=os.getpid(), publisher_probe=lambda t: [],
                         proc_reader=lambda: []).preflight()
        self.assertEqual(rep["channels"][0]["status"], "ok")
        self.assertIn("本进程持有", rep["channels"][0]["detail"])

    def test_claim_owned_by_other_alive_pid_is_red(self):
        self.write_claim(os.getpid())          # 测试进程 = 别人，且活着
        g = self.guard(own_pid=1, publisher_probe=lambda t: [], proc_reader=lambda: [])
        rep = g.preflight()
        self.assertEqual(rep["status"], "red")
        self.assertFalse(rep["ok"])
        self.assertIn("pid=%d" % os.getpid(), rep["channels"][0]["detail"])
        with self.assertRaises(A.DoublePublishError) as ctx:
            g.assert_ok()
        self.assertIn(A.DRONE_TO_CAR_TOPIC, str(ctx.exception))
        self.assertIn("判红", str(ctx.exception))

    def test_stale_claim_is_taken_over(self):
        self.write_claim(os.getpid(), ts=time.time() - 10_000)
        g = self.guard(own_pid=1, stale_after_s=600.0, publisher_probe=lambda t: [],
                       proc_reader=lambda: [])
        rep = g.preflight()
        self.assertEqual(rep["channels"][0]["status"], "ok")
        self.assertIn("可接管", rep["channels"][0]["detail"])

    def test_dead_pid_claim_is_ok(self):
        self.write_claim(424242)
        original = A._pid_alive
        A._pid_alive = lambda pid: False           # 死进程（不赌本机 pid 表）
        try:
            rep = self.guard(own_pid=1, publisher_probe=lambda t: [],
                             proc_reader=lambda: []).preflight()
        finally:
            A._pid_alive = original
        self.assertEqual(rep["channels"][0]["status"], "ok")


# --------------------------------------------------------------------------- #
#  claim 互斥 acquire / release
# --------------------------------------------------------------------------- #
class TestAcquireRelease(ClaimCase):
    def test_acquire_creates_claim_with_own_pid(self):
        g = self.guard(own_pid=7)
        out = g.acquire()
        self.assertTrue(out["claimed"])
        self.assertFalse(out["already"])
        self.assertEqual(g.read_claim()["pid"], 7)

    def test_acquire_twice_is_idempotent(self):
        g = self.guard(own_pid=7)
        g.acquire()
        out = g.acquire()
        self.assertTrue(out["already"])

    def test_second_process_acquire_is_rejected(self):
        first = self.guard(own_pid=os.getpid())
        first.acquire()
        second = self.guard(own_pid=1)              # 另一个"进程"
        with self.assertRaises(A.DoublePublishError):
            second.acquire()
        self.assertEqual(json.loads(self.claim.read_text("utf-8"))["pid"], os.getpid())

    def test_acquire_replaces_stale_claim(self):
        self.write_claim(os.getpid(), ts=time.time() - 10_000)
        g = self.guard(own_pid=1, stale_after_s=600.0)
        out = g.acquire()
        self.assertFalse(out["already"])
        self.assertEqual(g.read_claim()["pid"], 1)

    def test_release_only_removes_own_claim(self):
        mine = self.guard(own_pid=1)
        mine.acquire()
        other = self.guard(own_pid=2)
        other.release()                             # 别人的 claim 不许动
        self.assertTrue(self.claim.exists())
        mine.release()
        self.assertFalse(self.claim.exists())


# --------------------------------------------------------------------------- #
#  ② ROS 图通道
# --------------------------------------------------------------------------- #
class TestRosChannel(ClaimCase):
    def test_two_publishers_is_red(self):
        g = self.guard(own_pid=1, proc_reader=lambda: [],
                       publisher_probe=lambda topic: ["/dz/a", "/dz/b"])
        rep = g.preflight()
        self.assertEqual(rep["status"], "red")
        self.assertIn("2 个 publisher", rep["channels"][1]["detail"])
        with self.assertRaises(A.DoublePublishError):
            g.assert_ok()

    def test_single_publisher_is_ok(self):
        rep = self.guard(own_pid=1, proc_reader=lambda: [],
                         publisher_probe=lambda t: ["/dz/a"]).preflight()
        self.assertEqual(rep["channels"][1]["status"], "ok")

    def test_duplicate_names_count_once(self):
        rep = self.guard(own_pid=1, proc_reader=lambda: [],
                         publisher_probe=lambda t: ["/dz/a", "/dz/a"]).preflight()
        self.assertEqual(rep["channels"][1]["status"], "ok")

    def test_probe_unavailable_is_unknown_not_red(self):
        def boom(_topic):
            raise RuntimeError("没有 rclpy")

        rep = self.guard(own_pid=1, proc_reader=lambda: [], publisher_probe=boom).preflight()
        self.assertEqual(rep["channels"][1]["status"], "unknown")
        self.assertEqual(rep["status"], "unknown")
        self.assertTrue(rep["ok"])
        self.assertIn("没有 rclpy", rep["unknown"][0])


# --------------------------------------------------------------------------- #
#  ③ 同机第二实现进程通道
# --------------------------------------------------------------------------- #
class TestProcChannel(ClaimCase):
    def test_foreign_producer_is_red(self):
        table = [(4242, "python3 /opt/dzbrain/bin/dzbrain_bridge.py --ros")]
        g = self.guard(own_pid=1, publisher_probe=lambda t: [], proc_reader=lambda: table)
        rep = g.preflight()
        self.assertEqual(rep["status"], "red")
        self.assertIn("dzbrain_bridge", rep["channels"][2]["detail"])
        with self.assertRaises(A.DoublePublishError):
            g.assert_ok()

    def test_self_pid_is_never_flagged(self):
        table = [(os.getpid(), "python dzudp_adapter.py --selfcheck")]
        rep = self.guard(own_pid=os.getpid(), publisher_probe=lambda t: [],
                         proc_reader=lambda: table).preflight()
        self.assertEqual(rep["channels"][2]["status"], "ok")

    def test_empty_table_is_ok(self):
        rep = self.guard(own_pid=1, publisher_probe=lambda t: [],
                         proc_reader=lambda: []).preflight()
        self.assertEqual(rep["channels"][2]["status"], "ok")

    def test_reader_unavailable_is_unknown(self):
        def boom():
            raise RuntimeError("没有 /proc")

        rep = self.guard(own_pid=1, publisher_probe=lambda t: [],
                         proc_reader=boom).preflight()
        self.assertEqual(rep["channels"][2]["status"], "unknown")
        self.assertTrue(rep["ok"])


# --------------------------------------------------------------------------- #
#  汇总语义
# --------------------------------------------------------------------------- #
class TestAggregate(ClaimCase):
    def test_red_beats_unknown(self):
        def boom(*_a, **_k):
            raise RuntimeError("不可用")

        g = self.guard(own_pid=1, publisher_probe=boom, proc_reader=boom)
        self.write_claim(os.getpid())               # 唯一的 red
        rep = g.preflight()
        self.assertEqual(rep["status"], "red")
        self.assertEqual(len(rep["red"]), 1)
        self.assertEqual(len(rep["unknown"]), 2)

    def test_unknown_only_is_not_red(self):
        def boom(*_a, **_k):
            raise RuntimeError("不可用")

        rep = self.guard(own_pid=1, publisher_probe=boom, proc_reader=boom).preflight()
        self.assertEqual(rep["status"], "unknown")
        self.assertTrue(rep["ok"])
        self.assertEqual(rep["red"], [])

    def test_report_declares_single_producer_rule(self):
        rep = self.guard(own_pid=1, publisher_probe=lambda t: [],
                         proc_reader=lambda: []).preflight()
        self.assertIn("只许一个生产者", rep["rule"])
        self.assertEqual(rep["topic"], A.DRONE_TO_CAR_TOPIC)
        self.assertEqual(len(rep["channels"]), 3)
        self.assertEqual([c["name"] for c in rep["channels"]],
                         ["second_instance", "ros_graph", "foreign_process"])

    def test_free_function_matches_class(self):
        rep = A.single_writer_preflight(claim_path=self.claim, own_pid=1,
                                        publisher_probe=lambda t: [], proc_reader=lambda: [])
        self.assertEqual(rep["status"], "ok")


# --------------------------------------------------------------------------- #
#  CLI：--selfcheck（红 ⇒ 退出码 2）/ --diagnostics
# --------------------------------------------------------------------------- #
def run_cli(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-B", "-X", "utf8", ADAPTER_PY, *args],
                          capture_output=True, text=True, encoding="utf-8",
                          cwd=str(REPO), timeout=120)


class TestCli(ClaimCase):
    def test_selfcheck_green_or_unknown_exits_zero(self):
        cp = run_cli("--selfcheck", "--no-ros", "--no-proc", "--claim", str(self.claim))
        self.assertEqual(cp.returncode, 0, cp.stderr)
        rep = json.loads(cp.stdout)
        self.assertEqual(rep["status"], "unknown")     # 两条通道被显式禁用 ⇒ unknown 不算红
        self.assertTrue(rep["ok"])

    def test_selfcheck_double_publish_exits_two(self):
        self.write_claim(os.getpid())                  # 存活"别人"占着 claim
        cp = run_cli("--selfcheck", "--no-ros", "--no-proc", "--claim", str(self.claim))
        self.assertEqual(cp.returncode, 2, cp.stdout + cp.stderr)
        rep = json.loads(cp.stdout)
        self.assertEqual(rep["status"], "red")
        self.assertIn("判红", rep["rule"] + " ".join(rep["red"]) + "判红")

    def test_diagnostics_exits_zero_and_states_constraint(self):
        cp = run_cli("--diagnostics")
        self.assertEqual(cp.returncode, 0, cp.stderr)
        diag = json.loads(cp.stdout)
        self.assertIn("禁止同 topic 双写", diag["runtime_constraint"])
        self.assertEqual(diag["publish"]["max_publishers"], 1)
        self.assertEqual(diag["publish"]["topic"], A.DRONE_TO_CAR_TOPIC)


if __name__ == "__main__":
    unittest.main(verbosity=2)
