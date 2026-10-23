#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_matrix.py —— QoS 反向组合变异测试总控（在 Windows 本地跑，指挥两端真机）

对每个 (方向, 发布端 QoS, 订阅端 QoS) 组合：
  1. 订阅端后台起 t4_qos_probe.py sub（--out /tmp/t4_qos_sub.json）
  2. 3.5 s 后发布端跑 t4_qos_probe.py pub（--out /tmp/t4_qos_pub.json）
  3. 两端结果 JSON 取回本地，合并成矩阵

用法:
  python t4_matrix.py                  # 全部 8 格（题面测试主题）
  python t4_matrix.py --only d2c-re-be # 只跑一格
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
OUTDIR = os.path.join(ROOT, "_tmp_session", "t4", "qos_matrix")
TOPIC = "/t4qos/probe"


def cell(direction, pub_qos, sub_qos, outdir, topic=TOPIC, sub_dur=16, pub_dur=8):
    sender, receiver = ("drone", "car") if direction == "d2c" else ("car", "drone")
    label = "%s_pub-%s_sub-%s" % (direction, pub_qos[:4], sub_qos[:4])
    sub_json = "/tmp/t4_matrix_sub_%s.json" % label
    pub_json = "/tmp/t4_matrix_pub_%s.json" % label
    sub_log = "/tmp/t4_matrix_sub_%s.log" % label
    pub_log = "/tmp/t4_matrix_pub_%s.log" % label

    cs, cr = t4_ssh.connect(sender), t4_ssh.connect(receiver)
    rec = {"cell": label, "direction": direction, "sender": sender,
           "receiver": receiver, "topic": topic,
           "pub_qos": pub_qos, "sub_qos": sub_qos}
    try:
        sub_cmd = ("setsid nohup bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py sub "
                   "--topic %s --qos %s --duration %d --out %s > %s 2>&1 < /dev/null & "
                   "echo SUB_PID=$!" % (topic, sub_qos, sub_dur, sub_json, sub_log))
        rc, out = t4_ssh.run(cr, sub_cmd)
        print("  [%s] sub started: %s" % (label, out.strip()))
        time.sleep(3.5)
        pub_cmd = ("bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub "
                   "--topic %s --qos %s --rate 5 --duration %d --tag %s-%s --out %s"
                   % (topic, pub_qos, pub_dur, sender, label, pub_json))
        rc, out = t4_ssh.run(cs, pub_cmd, timeout=120)
        print("  [%s] pub done rc=%d" % (label, rc))
        time.sleep(sub_dur - pub_dur - 3.5 + 4.0)
        # 取回结果
        for dev, conn, remote in ((receiver, cr, sub_json), (sender, cs, pub_json)):
            local = os.path.join(outdir, "%s.%s.json" % (label, dev))
            try:
                t4_ssh.get(conn, remote, local)
            except Exception as e:
                print("  [%s] get %s failed: %s" % (label, remote, e))
        for dev, conn, remote in ((receiver, cr, sub_log), (sender, cs, pub_log)):
            local = os.path.join(outdir, "%s.%s.log" % (label, dev))
            try:
                t4_ssh.get(conn, remote, local)
            except Exception as e:
                pass
        # 汇总
        sub = _load(os.path.join(outdir, "%s.%s.json" % (label, receiver)))
        pub = _load(os.path.join(outdir, "%s.%s.json" % (label, sender)))
        rec["sub_verdict"] = (sub or {}).get("verdict")
        rec["pub_verdict"] = (pub or {}).get("verdict")
        rec["received"] = (sub or {}).get("received")
        rec["peer_count_max_sub_side"] = (sub or {}).get("peer_count_max")
        rec["sent"] = (pub or {}).get("sent")
        rec["peer_count_max_pub_side"] = (pub or {}).get("peer_count_max")
        rec["sub_domain"] = (sub or {}).get("domain_id")
        rec["pub_domain"] = (pub or {}).get("domain_id")
        for k in ("verdict", "received", "peer_count_max", "sent", "domain_id"):
            sub and sub.pop("peer_count_timeline", None)
            pub and pub.pop("peer_count_timeline", None)
        rec["sub_samples"] = (sub or {}).get("samples")
        rec["pub_first_payload"] = (pub or {}).get("samples")
    finally:
        cs.close(); cr.close()
    return rec


def _load(p):
    try:
        with open(p) as f:
            return json.load(f)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default=None)
    ap.add_argument("--outdir", default=OUTDIR)
    ap.add_argument("--topic", default=TOPIC)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    cells = []
    for direction in ("d2c", "c2d"):
        for pq in ("reliable", "best_effort"):
            for sq in ("reliable", "best_effort"):
                cells.append((direction, pq, sq))
    if a.only:
        cells = [c for c in cells if "-".join(
            [c[0], "pub-" + c[1][:4], "sub-" + c[2][:4]]) == a.only]
        if not cells:
            raise SystemExit("no such cell: %s" % a.only)

    results = []
    t0 = time.time()
    for (d, pq, sq) in cells:
        print("=== cell %s pub=%s sub=%s ===" % (d, pq, sq))
        r = cell(d, pq, sq, a.outdir, topic=a.topic)
        print("    -> " + json.dumps({k: r[k] for k in
              ("pub_verdict", "sub_verdict", "received", "sent",
               "peer_count_max_pub_side", "peer_count_max_sub_side")}))
        results.append(r)
        with open(os.path.join(a.outdir, "matrix.json"), "w") as f:
            json.dump(results, f, ensure_ascii=False, indent=2)
    print("=== matrix done in %.0fs, %d cells ===" % (time.time() - t0, len(results)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
