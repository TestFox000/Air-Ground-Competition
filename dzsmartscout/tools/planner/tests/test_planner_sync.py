#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""t60 单测 —— 无人机侧规划器"单源码同步"的哈希门禁 + 双侧一致性

跑法：
    python -m unittest discover -s dzsmartscout/tools/planner/tests -p "test_*.py" -v
    python dzsmartscout/tools/planner/tests/test_planner_sync.py

覆盖：
  ① 账本完整性：账本里的每个文件都在 vendor 里；MANIFEST 与账本一致；vendor 里没有账本外的 .py（防手工塞文件）
  ② 产物未被手改：vendor 每文件 sha256 == 账本；vendor == 真源当前字节（真源漂移会**失败**，提示重跑同步）
  ③ 规划确定性与双侧一致：vendor 跑 2 次 + 真源跑 1 次，三条 token 串完全相同；且长度 ≥ 8
  ④ 出车校验：vendor 的 `plan_route --check` 返回 0（0 error）
"""
from __future__ import annotations

import hashlib
import json
import os
import pathlib
import subprocess
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent          # tools/planner/tests
PLANNER = HERE.parent                                   # tools/planner
WORKSPACE = PLANNER.parents[2]
VENDOR = PLANNER / "vendor" / "dzbrain_ws"
SOURCE = WORKSPACE / "dzrobot_v2"
LEDGER = PLANNER / "HASH_LEDGER.json"


def sha256(p: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 16), b""):
            h.update(c)
    return h.hexdigest()


def budget_env() -> dict:
    # ★ dzrobot_v2 是只读边界：禁止解释器往真源树写 __pycache__/*.pyc
    return dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1",
                PYTHONDONTWRITEBYTECODE="1")


def config_engine() -> str:
    """★ 引擎必须从 config 读（与 tasks/mission.py:399 同源）；plan_route.py 默认是 tour。"""
    import sys as _s
    _s.path.insert(0, str(PLANNER))
    import planner_engine as PE
    return PE.read_engine(VENDOR)["engine"]


def run_plan(root: pathlib.Path, extra=()):
    cmd = [sys.executable, str(root / "tools" / "plan_route.py"), "--emit", "tokens",
           "--engine", config_engine(),
           "--config", str(root / "config" / "robot.yaml"),
           "--map", str(root / "config" / "map.yaml"), *extra]
    p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                       errors="replace", cwd=str(root), env=budget_env(), timeout=600)
    import re
    pat = re.compile(r"^[A-N](?:1[01]|[1-9])(?:[*,](?:x|y))?\*?$")
    toks = ""
    for line in (p.stdout or "").splitlines():
        body = line.strip().replace("\x1b", "")
        parts = body.split()
        if len(parts) >= 8 and all(pat.match(t) for t in parts):
            toks = body
    return p.returncode, toks, (p.stdout or "") + (p.stderr or "")


class TestLedgerAndVendor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not LEDGER.is_file():
            raise unittest.SkipTest("没有账本，先跑 sync_planner_from_source.py")
        cls.led = json.loads(LEDGER.read_text(encoding="utf-8"))
        cls.files = {e["rel"]: e for e in cls.led["files"]}

    def test_ledger_all_same_flag(self):
        self.assertTrue(self.led["all_same"], "账本自己记着 all_same=False")
        self.assertFalse(self.led.get("closure_problems"), self.led.get("closure_problems"))

    def test_every_ledger_file_exists_in_vendor(self):
        for rel in self.files:
            self.assertTrue((VENDOR / rel).is_file(), "vendor 缺文件：%s" % rel)

    def test_vendor_bytes_match_ledger(self):
        bad = [rel for rel in self.files if sha256(VENDOR / rel) != self.files[rel]["sha256"]]
        self.assertFalse(bad, "vendor 与账本不一致（被手改过？）：%s" % bad)

    def test_vendor_matches_source_now(self):
        """★ 哈希门禁：真源漂移 / 产物手改 都会在这里失败 ⇒ 重跑同步。"""
        drift = [rel for rel in self.files
                 if sha256(SOURCE / rel) != self.files[rel]["sha256"]]
        self.assertFalse(drift,
                         "真源已变（或产物过期），请重跑 sync_planner_from_source.py：%s" % drift)

    def test_manifest_matches_ledger(self):
        man = [l.strip() for l in (VENDOR / "MANIFEST.txt").read_text(encoding="utf-8").splitlines()
               if l.strip()]
        self.assertEqual(man, sorted(man, key=man.index))          # 原样
        self.assertEqual(sorted(man), sorted(self.files.keys()))

    def test_no_extra_python_in_vendor(self):
        """vendor 里除了账本内的 .py，不许有别的手工文件（防 fork）。"""
        declared = {rel for rel in self.files if rel.endswith(".py")}
        found = {str(p.relative_to(VENDOR)).replace("\\", "/")
                 for p in VENDOR.rglob("*.py") if "__pycache__" not in p.parts}
        extra = found - declared
        self.assertFalse(extra, "vendor 里有账本外的手工 .py：%s" % sorted(extra))


class TestPlanning(unittest.TestCase):

    def test_plan_is_deterministic_and_matches_source(self):
        rc1, t1, out1 = run_plan(VENDOR)
        rc2, t2, _ = run_plan(VENDOR)
        rc3, t3, _ = run_plan(SOURCE)
        self.assertEqual((rc1, rc2, rc3), (0, 0, 0), out1[-800:])
        self.assertTrue(t1 and len(t1.split()) >= 8, "没抓到 token 串：%r" % t1)
        self.assertEqual(t1, t2, "同侧两次结果不同（不确定）")
        self.assertEqual(t1, t3, "无人机侧(产物) 与 小车侧(真源) 的路线不同")

    def test_route_check_zero_error(self):
        rc, toks, out = run_plan(VENDOR, ["--check"])
        self.assertEqual(rc, 0, "route_check 非 0：\n%s" % out[-1200:])
        self.assertIn("全部通过", out.replace("\x1b", ""))

    def test_tokens_shape(self):
        rc, toks, _ = run_plan(VENDOR)
        self.assertEqual(rc, 0)
        parts = toks.split()
        self.assertTrue(parts[0].startswith("N10"), parts[0])
        self.assertTrue(parts[-1].endswith(",y"), parts[-1])
        self.assertNotIn("R:", toks, "真车不支持倒车（allow_reverse=false）⇒ 不该出现 R: 段")

    def test_engine_is_read_from_config_not_tool_default(self):
        """★★ 收口轮（队长实锤）：`plan_route.py --engine` 默认 tour，而实车走 planner.engine。
        本测试把"引擎口径"钉住：config 里是什么，就跑什么；并断言与 tour 产物不同（防再犯）。"""
        engine = config_engine()
        self.assertIn(engine, ("tour", "tracker-safe"), engine)
        _, tok_cfg, _ = run_plan(VENDOR)                       # 用 config 引擎
        cmd = [sys.executable, str(VENDOR / "tools" / "plan_route.py"), "--emit", "tokens",
               "--engine", "tour",                              # 显式 tour 作对照
               "--config", str(VENDOR / "config" / "robot.yaml"),
               "--map", str(VENDOR / "config" / "map.yaml")]
        p = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                           errors="replace", cwd=str(VENDOR), env=budget_env(), timeout=600)
        import re
        pat = re.compile(r"^[A-N](?:1[01]|[1-9])(?:[*,](?:x|y))?\*?$")
        tok_tour = ""
        for line in (p.stdout or "").splitlines():
            body = line.strip().replace("\x1b", "")
            if len(body.split()) >= 8 and all(pat.match(t) for t in body.split()):
                tok_tour = body
        self.assertTrue(tok_tour, "tour 没跑出 token")
        if engine == "tracker-safe":
            self.assertNotEqual(tok_cfg, tok_tour,
                                "tracker-safe 与 tour 产物相同？那说明 --engine 没生效")

    def test_route_matches_onboard_mission_blocks(self):
        """tracker-safe 产物应与车侧落盘的板载回退路线 mission_blocks 逐 token 一致。"""
        import sys as _s
        _s.path.insert(0, str(PLANNER))
        import planner_engine as PE
        mb = PE.read_mission_blocks(SOURCE / "config" / "robot.yaml")
        if not mb:
            self.skipTest("robot.yaml 里没有 mission_blocks")
        _, toks, _ = run_plan(VENDOR)
        self.assertEqual(toks.split(), mb, "机算路线与 mission_blocks 不一致")


if __name__ == "__main__":
    unittest.main(verbosity=2)
