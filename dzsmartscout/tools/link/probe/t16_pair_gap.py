#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t16_pair_gap.py —— 实测「桥洞两半场（lower/upper）到达间隔」

为什么量这个（t13 现场口径提醒）：车侧 `mission.bridge_info_wait_s = 2.0 s` 只覆盖
「两半场情报**相隔 ≤ 2 s** 到达」；超隔的那个半场只进遥测、路线已冻结。
所以必须证明 supervisor 把 lower/upper **成对**发出且间隔远小于 2 s。

两个相位（都在真机跑，车侧只用 /tmp 探针）：
  P1 CLI 注入：`run --bridge lower=K10 --bridge upper=K5 …`（两半场同一 tick 首次发出）
  P2 cmd-file：运行中往 `--cmd-file` **一次写入两行**（模拟现场预置情报的做法）

度量：
  · 车侧接收间隔（权威）：t4_car_consumer.py 落的 JSONL 里每条 bridge_passable 的 t_recv
  · 机侧发送间隔（交叉验证）：supervisor JSONL 里 intel_emit 的 ts
输出：每轮的 lower→upper / upper→lower 间隔（秒），以及 min/median/max。

用法: python dzsmartscout/tools/link/probe/t16_pair_gap.py
"""
import json
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh as S  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
RAW = os.path.join(ROOT, "_tmp_session", "t16", "raw")
os.makedirs(RAW, exist_ok=True)
DRONE_DIR = "/home/duzhong/dzlink_tools"


def save(name, text):
    with open(os.path.join(RAW, name), "w", encoding="utf-8", errors="replace") as f:
        f.write(text)
    print("    [saved] %s" % name)


def ex(dev, cmd, timeout=240):
    c = S.connect(dev)
    try:
        return S.run(c, cmd, timeout=timeout)
    finally:
        c.close()


def bg(dev, inner, log):
    return ex(dev, "setsid nohup bash -c %s > %s 2>&1 < /dev/null & echo BG_PID=$!; sleep 1"
             % (S.shell_quote(inner), log))


def gaps_from(items, key_half="half", key_t="t"):
    """items 已按时间排序；返回 [(前一个 half, 后一个 half, Δ)]"""
    out = []
    for a, b in zip(items, items[1:]):
        out.append((a[key_half], b[key_half], round(b[key_t] - a[key_t], 3)))
    return out


def summarize(rows, tag):
    lower_to_upper = [g for (h1, h2, g) in rows if h1 == "lower" and h2 == "upper"]
    upper_to_lower = [g for (h1, h2, g) in rows if h1 == "upper" and h2 == "lower"]
    res = {"tag": tag, "pairs": len(rows),
           "lower→upper": _stat(lower_to_upper),
           "upper→lower": _stat(upper_to_lower)}
    print("  [%s] 样本 %d 组" % (tag, len(rows)))
    print("      lower→upper : %s" % res["lower→upper"])
    print("      upper→lower : %s" % res["upper→lower"])
    return res


def _stat(v):
    if not v:
        return None
    return {"n": len(v), "min": min(v), "median": round(statistics.median(v), 3),
            "max": max(v), "all": v}


def phase(name, launch, extra_after=None, wait_after=0.0):
    """跑一个相位：起车侧消费者 → 起无人机 supervisor →（可选）运行中按时序注入 → 收账。

    extra_after: [(延迟秒, 远端命令), ...]，按顺序执行（延迟是相对本相位起点）。
    """
    print("\n=== %s ===" % name)
    car_out = "/tmp/t16_car_recv_%s.jsonl" % name
    sup_log = "/tmp/t16_sup_%s.jsonl" % name
    sp = S.connect("car")
    try:
        S.put(sp, os.path.join(HERE, "t4_car_consumer.py"), "/tmp/t16_car_consumer.py")
        # ★ 先删旧文件：否则上一次残留的消费者会往同一个文件追加，行会交错损坏
        S.run(sp, "rm -f %s" % car_out)
        rc, out = S.run(sp, "setsid nohup bash -c %s > /tmp/t16_car_outer_%s.log 2>&1 "
                            "< /dev/null & echo CAR_PID=$!"
                        % (S.shell_quote("bash /tmp/t4_rosenv.sh python3 "
                                         "/tmp/t16_car_consumer.py --qos best_effort "
                                         "--duration 40 --out %s" % car_out), name))
        print("  car consumer: %s" % out.strip())
    finally:
        sp.close()

    # ★ 先清机侧日志：`run --log-file` 是**追加**的，上一次同名的 run 会把记录留在文件里，
    #   导致统计出现假的超大间隔（实测出现过 221 s 的伪 outlier）。
    #   必须在启动之前清 —— 启动后再 truncate 会让进程按旧偏移续写，产出带 NUL 的坏行。
    ex("drone", ": > %s" % sup_log)
    bg("drone", "bash /tmp/t4_rosenv.sh " + launch.format(log=sup_log),
       "/tmp/t16_sup_outer_%s.log" % name)
    if extra_after:
        t0 = time.time()
        for delay, cmd in extra_after:
            while time.time() - t0 < delay:
                time.sleep(0.2)
            print("  [inject @%.1fs] %s" % (time.time() - t0, cmd[:120]))
            ex("drone", cmd)
    time.sleep(wait_after)

    rc, supout = ex("drone", "tail -6 /tmp/t16_sup_outer_%s.log 2>/dev/null; echo '--- jsonl ---'; "
                             "tail -4 /tmp/t16_sup_%s.jsonl 2>/dev/null" % (name, name))
    save("pair_%s_drone_tail.txt" % name, supout)
    time.sleep(3)
    sc = S.connect("car")
    try:
        S.get(sc, car_out, os.path.join(RAW, "pair_%s_car_recv.jsonl" % name))
    except Exception as e:
        print("  !! 取车侧记录失败：%s" % e)
    finally:
        sc.close()
    try:
        S.get(S.connect("drone"), "/tmp/t16_sup_%s.jsonl" % name,
              os.path.join(RAW, "pair_%s_sup.jsonl" % name))
    except Exception as e:
        print("  !! 取机侧日志失败：%s" % e)

    # 车侧（权威）
    car_rows = []
    try:
        for line in open(os.path.join(RAW, "pair_%s_car_recv.jsonl" % name), encoding="utf-8"):
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if d.get("type") == "bridge_passable" and d.get("half"):
                car_rows.append({"half": d["half"], "t": d["t_recv"]})
    except Exception as e:
        print("  !! 读车侧记录失败：%s" % e)
    car_rows.sort(key=lambda r: r["t"])
    # 机侧（交叉验证）
    sup_rows = []
    try:
        for line in open(os.path.join(RAW, "pair_%s_sup.jsonl" % name), encoding="utf-8"):
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if d.get("kind") == "intel_emit" and d.get("type") == "bridge_passable":
                half = str(d.get("slot", "")).split(":")[-1]
                sup_rows.append({"half": half, "t": d["ts"]})
    except Exception as e:
        print("  !! 读机侧日志失败：%s" % e)
    sup_rows.sort(key=lambda r: r["t"])

    return {"car": gaps_from(car_rows), "drone": gaps_from(sup_rows),
            "car_samples": car_rows[:12], "drone_samples": sup_rows[:12]}


def main():
    out = {}

    def _inj(line, path="/tmp/t16_cmd.jsonl"):
        return ("bash /tmp/t4_rosenv.sh bash -c %s"
                % S.shell_quote("printf '%s\\n' " + repr(line).replace("'", '"')
                                + " >> " + path))

    # P1：CLI 注入（两半场同一 tick）
    res = phase("p1_cli",
                "bash -c 'cd %s && python3 dzlink_supervisor.py run "
                "--bridge lower=K10 --bridge upper=K5 --duration 22 "
                "--heartbeat-hz 1 --bridge-period 2 --log-file {log}'" % DRONE_DIR,
                wait_after=24)
    out["p1_cli"] = {"car": summarize(res["car"], "P1 车侧接收"),
                     "drone": summarize(res["drone"], "P1 机侧发送"),
                     "car_samples": res["car_samples"], "drone_samples": res["drone_samples"]}

    # P2：运行中经 cmd-file 一次写入两行
    inject = ("bash /tmp/t4_rosenv.sh bash -c %s"
              % S.shell_quote(
                  "printf '%s\\n%s\\n' "
                  "'{\"type\":\"bridge_passable\",\"half\":\"lower\",\"block\":\"K10\"}' "
                  "'{\"type\":\"bridge_passable\",\"half\":\"upper\",\"block\":\"K5\"}' "
                  ">> /tmp/t16_cmd.jsonl"))
    res = phase("p2_cmdfile",
                "bash -c 'cd %s && rm -f /tmp/t16_cmd.jsonl && "
                "python3 dzlink_supervisor.py run --cmd-file /tmp/t16_cmd.jsonl "
                "--duration 26 --heartbeat-hz 1 --bridge-period 2 --log-file {log}'" % DRONE_DIR,
                extra_after=[(8.0, inject)], wait_after=28)
    out["p2_cmdfile"] = {"car": summarize(res["car"], "P2 车侧接收"),
                         "drone": summarize(res["drone"], "P2 机侧发送"),
                         "car_samples": res["car_samples"], "drone_samples": res["drone_samples"]}

    # P3：★ 两个半场**相隔 5 s** 分两次到达（模拟感知层分开报出）→ t16 成对保证生效？
    low = ("bash /tmp/t4_rosenv.sh bash -c %s"
           % S.shell_quote("printf '%s\\n' '{\"type\":\"bridge_passable\",\"half\":\"lower\","
                           "\"block\":\"K10\"}' >> /tmp/t16_cmd3.jsonl"))
    up = ("bash /tmp/t4_rosenv.sh bash -c %s"
          % S.shell_quote("printf '%s\\n' '{\"type\":\"bridge_passable\",\"half\":\"upper\","
                          "\"block\":\"K5\"}' >> /tmp/t16_cmd3.jsonl"))
    res = phase("p3_split5s",
                "bash -c 'cd %s && rm -f /tmp/t16_cmd3.jsonl && "
                "python3 dzlink_supervisor.py run --cmd-file /tmp/t16_cmd3.jsonl "
                "--duration 34 --heartbeat-hz 1 --bridge-period 2 --log-file {log}'" % DRONE_DIR,
                extra_after=[(9.0, low), (14.0, up)], wait_after=36)
    out["p3_split5s"] = {"car": summarize(res["car"], "P3 车侧接收（半场相隔 5 s 注入）"),
                         "drone": summarize(res["drone"], "P3 机侧发送"),
                         "car_samples": res["car_samples"], "drone_samples": res["drone_samples"]}

    with open(os.path.join(RAW, "pair_gap_summary.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("\n### 汇总写入 %s/pair_gap_summary.json" % RAW)
    # 结论
    tail = []
    for k in ("p1_cli", "p2_cmdfile", "p3_split5s"):
        s = out[k]["car"]["lower→upper"]
        if s:
            tail.append("%s: 车侧 lower→upper max=%.3fs median=%.3fs (n=%d)"
                        % (k, s["max"], s["median"], s["n"]))
        else:
            tail.append("%s: 车侧 lower→upper 无样本" % k)
    print("### " + " | ".join(tail))
    return 0


if __name__ == "__main__":
    sys.exit(main())
