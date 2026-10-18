# -*- coding: utf-8 -*-
import csv
import statistics as st
from collections import Counter


def an(name, path):
    rows = list(csv.DictReader(open(path, encoding="utf-8")))

    def f(x):
        try:
            return int(x)
        except Exception:
            return None

    offs = [r for r in rows if r["kind"] == "off"]
    cmds = [r for r in rows if r["kind"] == "cmd"]
    sts = [r for r in rows if r["kind"] == "st"]
    tg = [r for r in offs if f(r["c"]) in (1, 2)]
    dx = [abs(f(r["a"])) for r in tg if f(r["a"]) is not None]
    if not dx:
        print(name, ": 无靶帧")
        return
    seq = [f(r["a"]) for r in tg]
    zc = sum(1 for a, b in zip(seq, seq[1:])
             if a is not None and b is not None and (a > 0) != (b > 0))
    spd = [abs(f(r["d"])) for r in cmds if f(r["d"]) is not None]
    yaw = [f(r["b"]) for r in cmds if f(r["b"]) is not None]

    def q(v, p):
        s = sorted(v)
        return s[min(len(s) - 1, int(len(s) * p))]

    span = float(tg[-1]["t"]) - float(tg[0]["t"])
    print("%s" % name)
    print("  帧=%d 有靶=%d 状态=%s" % (len(offs), len(tg),
                                      Counter(r["a"] for r in sts).most_common()))
    print("  |dx| 中位=%.0f p90=%.0f 最大=%d  死区<=5px=%.0f%%  过零=%d 次/%.1fs"
          % (st.median(dx), q(dx, 0.9), max(dx),
             100.0 * sum(1 for d in dx if d <= 5) / len(dx), zc, span))
    if spd:
        print("  yaw速度 均值=%.0f 峰值=%d 撞上限2000=%.0f%%"
              % (sum(spd) / len(spd), max(spd),
                 100.0 * sum(1 for s in spd if s >= 1999) / len(spd)))
    if yaw:
        ch = sum(1 for a, b in zip(yaw, yaw[1:]) if a is not None and b is not None and a != b)
        print("  yaw指令 范围=%d..%d 变化=%d 次" % (min(yaw), max(yaw), ch))
    print()


an("kp15 / ki1.5 / kd1.5（上一档，样本 aim_kp12.csv）", "_tmp_session/aim_kp12.csv")
an("kp15 / ki1.0 / kd1.2", "_tmp_session/aim_k1512.csv")
an("kp12 / ki1.0 / kd1.2（本轮）", "_tmp_session/aim_k1212.csv")
