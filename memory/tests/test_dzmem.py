"""dzmem 自测：事实/时间线/偏好/人/检索/摘要/导出。"""
import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import dzmem  # noqa: E402


class DzmemTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="dzmem_")
        self.db = Path(self.tmp) / "t.db"
        self.con = dzmem.connect(self.db)

    def test_fact_add_and_search_chinese(self):
        dzmem.add_fact(self.con, "T3 审计结论 NO-GO", topic="T3", tags="audit")
        rows, evs = dzmem.search(self.con, "审计")
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["topic"], "T3")
        rows2, _ = dzmem.search(self.con, "NO-GO")
        self.assertEqual(len(rows2), 1)
        rows3, _ = dzmem.search(self.con, "不存在的词")
        self.assertEqual(rows3, [])

    def test_events_recent_and_digest(self):
        now = time.time()
        dzmem.add_event(self.con, "心跳器上线", kind="milestone", ts=now - 60)
        dzmem.add_event(self.con, "很久以前的事", kind="note", ts=now - 5 * 86400)
        f, e = dzmem.recent(self.con, days=1)
        self.assertEqual(len(e), 1)
        md = dzmem.call_digest(self.con, days=1)
        self.assertIn("心跳器上线", md)
        self.assertNotIn("很久以前的事", md)

    def test_pref_upsert_and_person_upsert(self):
        dzmem.set_pref(self.con, "报告风格", "中文")
        dzmem.set_pref(self.con, "报告风格", "中文 + 表格")
        row = self.con.execute("SELECT value FROM preferences WHERE key='报告风格'").fetchone()
        self.assertEqual(row["value"], "中文 + 表格")
        self.assertEqual(len(list(self.con.execute("SELECT * FROM preferences"))), 1)

        dzmem.upsert_person(self.con, "用户", "伙伴", "喜欢大肥鱼")
        dzmem.upsert_person(self.con, "用户", "伙伴", "喜欢大肥鱼，在意证据")
        rows = list(self.con.execute("SELECT * FROM people"))
        self.assertEqual(len(rows), 1)
        self.assertIn("证据", rows[0]["notes"])

    def test_export_json_roundtrip(self):
        dzmem.add_fact(self.con, "一条事实", topic="x")
        dzmem.add_event(self.con, "一条事件", kind="note")
        out = Path(self.tmp) / "dump.json"
        nf, ne = dzmem.export_json(self.con, out)
        self.assertEqual((nf, ne), (1, 1))
        data = json.loads(out.read_text(encoding="utf-8"))
        self.assertEqual(data["facts"][0]["text"], "一条事实")
        self.assertEqual(data["events"][0]["text"], "一条事件")

    def test_stats(self):
        dzmem.add_fact(self.con, "f")
        dzmem.add_event(self.con, "e")
        dzmem.set_pref(self.con, "k", "v")
        dzmem.upsert_person(self.con, "p")
        self.assertEqual(dzmem.stats(self.con), {"facts": 1, "events": 1, "prefs": 1, "people": 1})


if __name__ == "__main__":
    unittest.main()
