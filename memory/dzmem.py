#!/usr/bin/env python3
"""dzmem —— 大肥鱼的本地长期记忆（单文件 SQLite，零依赖）。

facts 事实 / events 时间线 / preferences 偏好 / people 人。
"""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_DB = HERE / "dzmem.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS facts (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts REAL NOT NULL,
  topic TEXT NOT NULL DEFAULT '',
  text TEXT NOT NULL,
  source TEXT NOT NULL DEFAULT 'lead',
  tags TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_facts_topic ON facts(topic);
CREATE INDEX IF NOT EXISTS idx_facts_ts ON facts(ts);
CREATE TABLE IF NOT EXISTS events (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  ts REAL NOT NULL,
  kind TEXT NOT NULL DEFAULT 'note',
  text TEXT NOT NULL,
  meta TEXT NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts);
CREATE TABLE IF NOT EXISTS preferences (
  key TEXT PRIMARY KEY,
  value TEXT NOT NULL,
  ts REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS people (
  name TEXT PRIMARY KEY,
  relation TEXT NOT NULL DEFAULT '',
  notes TEXT NOT NULL DEFAULT '',
  ts REAL NOT NULL
);
"""


def connect(db=None):
    con = sqlite3.connect(str(db or DEFAULT_DB))
    con.row_factory = sqlite3.Row
    con.executescript(SCHEMA)
    return con


def _now(ts=None):
    return float(ts) if ts is not None else time.time()


def add_fact(con, text, topic="", tags="", source="lead", ts=None):
    con.execute("INSERT INTO facts(ts, topic, text, source, tags) VALUES (?,?,?,?,?)",
                (_now(ts), topic, text, source, tags))
    con.commit()


def add_event(con, text, kind="note", meta="", ts=None):
    con.execute("INSERT INTO events(ts, kind, text, meta) VALUES (?,?,?,?)",
                (_now(ts), kind, text, meta))
    con.commit()


def set_pref(con, key, value, ts=None):
    con.execute("INSERT INTO preferences(key, value, ts) VALUES (?,?,?) "
                "ON CONFLICT(key) DO UPDATE SET value=excluded.value, ts=excluded.ts",
                (key, value, _now(ts)))
    con.commit()


def upsert_person(con, name, relation="", notes="", ts=None):
    con.execute("INSERT INTO people(name, relation, notes, ts) VALUES (?,?,?,?) "
                "ON CONFLICT(name) DO UPDATE SET relation=excluded.relation, "
                "notes=excluded.notes, ts=excluded.ts",
                (name, relation, notes, _now(ts)))
    con.commit()


def search(con, q, limit=20):
    like = "%" + q + "%"
    rows = con.execute(
        "SELECT id, ts, topic, text, source, tags FROM facts "
        "WHERE text LIKE ? OR topic LIKE ? OR tags LIKE ? "
        "ORDER BY ts DESC LIMIT ?", (like, like, like, limit)).fetchall()
    evs = con.execute(
        "SELECT id, ts, kind, text FROM events WHERE text LIKE ? OR kind LIKE ? "
        "ORDER BY ts DESC LIMIT ?", (like, like, limit)).fetchall()
    return rows, evs


def recent(con, days=1, limit=200):
    cut = time.time() - days * 86400
    facts = con.execute("SELECT * FROM facts WHERE ts >= ? ORDER BY ts ASC LIMIT ?",
                        (cut, limit)).fetchall()
    events = con.execute("SELECT * FROM events WHERE ts >= ? ORDER BY ts ASC LIMIT ?",
                         (cut, limit)).fetchall()
    return facts, events


def fmt_ts(ts):
    return time.strftime("%Y-%m-%d %H:%M", time.localtime(ts))


def stats(con):
    q = lambda sql: con.execute(sql).fetchone()[0]
    return {"facts": q("SELECT COUNT(*) FROM facts"),
            "events": q("SELECT COUNT(*) FROM events"),
            "prefs": q("SELECT COUNT(*) FROM preferences"),
            "people": q("SELECT COUNT(*) FROM people")}

def call_digest(con, days=1):
    facts, events = recent(con, days=days, limit=500)
    out = ["# 记忆摘要（近 %s 天）" % days, ""]
    out.append("- 事实 %d 条 / 事件 %d 条" % (len(facts), len(events)))
    out.append("")
    if facts:
        out.append("## 事实")
        for f in facts:
            tag = (" #" + f["tags"]) if f["tags"] else ""
            out.append("- [%s]%s **%s**：%s" % (fmt_ts(f["ts"]), tag, f["topic"] or "—", f["text"]))
        out.append("")
    if events:
        out.append("## 时间线")
        for e in events:
            out.append("- [%s] (%s) %s" % (fmt_ts(e["ts"]), e["kind"], e["text"]))
    prefs = con.execute("SELECT * FROM preferences ORDER BY key").fetchall()
    if prefs:
        out.append("")
        out.append("## 偏好")
        for p in prefs:
            out.append("- %s：%s" % (p["key"], p["value"]))
    ppl = con.execute("SELECT * FROM people ORDER BY name").fetchall()
    if ppl:
        out.append("")
        out.append("## 人")
        for p in ppl:
            out.append("- %s（%s）：%s" % (p["name"], p["relation"], p["notes"]))
    return "\n".join(out) + "\n"


def export_json(con, path):
    data = {
        "facts": [dict(r) for r in con.execute("SELECT * FROM facts ORDER BY ts")],
        "events": [dict(r) for r in con.execute("SELECT * FROM events ORDER BY ts")],
        "preferences": [dict(r) for r in con.execute("SELECT * FROM preferences")],
        "people": [dict(r) for r in con.execute("SELECT * FROM people")],
    }
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return len(data["facts"]), len(data["events"])


def main(argv=None):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass
    ap = argparse.ArgumentParser(prog="dzmem", description="大肥鱼的本地长期记忆")
    ap.add_argument("--db", default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("add")
    p.add_argument("text")
    p.add_argument("--topic", default="")
    p.add_argument("--tag", default="")
    p.add_argument("--source", default="lead")
    p.add_argument("--ts", type=float, default=None)

    p = sub.add_parser("event")
    p.add_argument("text")
    p.add_argument("--kind", default="note")
    p.add_argument("--meta", default="")
    p.add_argument("--ts", type=float, default=None)

    p = sub.add_parser("diary")
    p.add_argument("text")

    p = sub.add_parser("pref")
    p.add_argument("key")
    p.add_argument("value")

    p = sub.add_parser("person")
    p.add_argument("name")
    p.add_argument("--relation", default="")
    p.add_argument("--notes", default="")

    p = sub.add_parser("search")
    p.add_argument("q")
    p.add_argument("--limit", type=int, default=20)

    p = sub.add_parser("recent")
    p.add_argument("--days", type=float, default=1)

    sub.add_parser("today")
    p = sub.add_parser("digest")
    p.add_argument("--days", type=float, default=1)
    p.add_argument("--out", default="")
    sub.add_parser("stats")
    p = sub.add_parser("export")
    p.add_argument("path")

    a = ap.parse_args(argv)
    con = connect(a.db)

    if a.cmd == "add":
        add_fact(con, a.text, a.topic, a.tag, a.source, a.ts)
        print("已记住事实。")
    elif a.cmd == "event":
        add_event(con, a.text, a.kind, a.meta, a.ts)
        print("已记入时间线。")
    elif a.cmd == "diary":
        add_event(con, a.text, "diary")
        print("日记已记下。")
    elif a.cmd == "pref":
        set_pref(con, a.key, a.value)
        print("偏好已更新。")
    elif a.cmd == "person":
        upsert_person(con, a.name, a.relation, a.notes)
        print("人物已记下。")
    elif a.cmd == "search":
        rows, evs = search(con, a.q, a.limit)
        if not rows and not evs:
            print("没有找到。")
        for r in rows:
            print("[事实 %s] (%s) %s" % (fmt_ts(r["ts"]), r["topic"], r["text"]))
        for e in evs:
            print("[事件 %s] (%s) %s" % (fmt_ts(e["ts"]), e["kind"], e["text"]))
    elif a.cmd == "recent":
        f, e = recent(con, a.days)
        for r in f:
            print("[事实 %s] (%s) %s" % (fmt_ts(r["ts"]), r["topic"], r["text"]))
        for r in e:
            print("[事件 %s] (%s) %s" % (fmt_ts(r["ts"]), r["kind"], r["text"]))
    elif a.cmd == "today":
        print(call_digest(con, days=1))
    elif a.cmd == "digest":
        md = call_digest(con, days=a.days)
        if a.out:
            Path(a.out).write_text(md, encoding="utf-8")
            print("已写入 %s" % a.out)
        else:
            print(md)
    elif a.cmd == "stats":
        print(json.dumps(stats(con), ensure_ascii=False))
    elif a.cmd == "export":
        nf, ne = export_json(con, a.path)
        print("已导出 %s（facts %d / events %d）" % (a.path, nf, ne))
    return 0


if __name__ == "__main__":
    sys.exit(main())
