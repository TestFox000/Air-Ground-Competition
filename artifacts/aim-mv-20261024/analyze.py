# -*- coding: utf-8 -*-
"""mvlog.csv 复算：2026-10-24 移动靶 75 s 录制（_tmp_session/mv_test/mvlog.csv）的关键数字。

用法（在 v1 根）：  python _tmp_session/mv_test/analyze.py [csv路径]
输出：一份 JSON（可直接对照交接手册 §1.2）。

/offset_center 8 字段语义（docs/20_资料索引库/视觉与云台/01_云台瞄准与开火链路.md §2）：
  [0] dx 框中心横向偏移 px（右为正）   [1] dy 纵向偏移 px（下为正）
  [2] 类别+1（1=蓝 2=红 0=无目标）     [3]/[4] 框宽/框高 px
  [5]/[6] 靶心关键点 dx/dy（云台不用）  [7] 置信度×100
开火框：config/robot.yaml fire_dz_far_px=25 / fire_dz_near_px=26（随框宽线性插值）
"""
import sys, json, statistics as st

path = sys.argv[1] if len(sys.argv) > 1 else "_tmp_session/mv_test/mvlog.csv"
off, con, hp, cnt, shot, status = [], [], [], [], [], []
lead_n = ready_n = 0
for line in open(path, encoding="utf-8", errors="replace"):
    p = line.rstrip("\n").split(",")
    if len(p) != 4:
        continue
    t, kind, v = float(p[0]), p[2], p[3]
    if kind == "off":
        off.append((t, [int(x) for x in v.split(";")]))
    elif kind == "con":
        con.append((t, [int(x) for x in v.split(";")]))
    elif kind == "hp":
        hp.append((t, [int(x) for x in v.split(";")]))
    elif kind == "cnt":
        cnt.append((t, int(v)))
    elif kind == "shot":
        shot.append((t, int(v)))
    elif kind == "status":
        status.append((t, int(v)))
    elif kind == "lead":
        lead_n += 1 if float(v) != 0 else 0
    elif kind == "ready":
        ready_n += 1 if int(v) != 0 else 0

med = lambda xs: st.median(xs) if xs else float("nan")
def p90(xs):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(len(xs) * 0.9))] if xs else float("nan")

R = {"file": path, "off_frames": len(off), "con": len(con), "hp": len(hp)}
if off:
    t0, t1 = off[0][0], off[-1][0]
    R["duration_s"] = round(t1 - t0, 2)
    R["off_hz"] = round(len(off) / max(1e-6, t1 - t0), 1)
    for i, name in ((0, "dx_box"), (1, "dy_box"), (5, "dx_kp"), (6, "dy_kp")):
        v = [abs(a[i]) for _, a in off]
        R["abs_" + name] = {"median": med(v), "p90": p90(v), "max": max(v)}
    R["dx_signed_range"] = [min(a[0] for _, a in off), max(a[0] for _, a in off)]
    R["cls_hist"] = {str(c): sum(1 for _, a in off if a[2] == c) for c in sorted({a[2] for _, a in off})}
    R["box_w_median"] = med([a[3] for _, a in off])
    v = [abs(off[i + 1][1][0] - off[i][1][0]) for i in range(len(off) - 1)]
    R["target_speed_px_per_frame"] = {"median": med(v), "p90": p90(v), "max": max(v)}
    for thr in (1.5, 3.0, 4.0, 6.0):
        R["frac_frames_speed_le_%.1f" % thr] = round(sum(1 for x in v if x <= thr) / len(v), 4)
    R["target_speed_px_per_s"] = round(med(v) * R["off_hz"], 1)
if con:
    # CameMo_Conl 4 元素 = [Pos_0(pitch), Pos_1(yaw), Spd_0(pitch), Spd_1(yaw)]
    # 出处：dzrobot_v2/src/history/log/log_2026-10-02_1914.md:188
    names = ("pos_pitch", "pos_yaw", "spd_pitch", "spd_yaw")
    for i, nm in enumerate(names):
        vals = [c[i] for _, c in con]
        row = {"min": min(vals), "max": max(vals), "span": max(vals) - min(vals)}
        if i >= 2:  # 只有速度指令有 800 上限
            row["at_cap800"] = sum(1 for x in vals if abs(x) >= 800)
        else:
            row["span_deg"] = round(row["span"] * 360.0 / 4096.0, 1)  # 4096 刻度/圈
        R["con_" + nm] = row
if hp:
    R["judge_first"] = hp[0][1]
    R["judge_last"] = hp[-1][1]
if cnt:
    R["shoot_count_events"] = cnt
if status:
    R["aim_status_hist"] = {}
    for _, s in status:
        R["aim_status_hist"][str(s)] = R["aim_status_hist"].get(str(s), 0) + 1
R["lead_topic_msgs_nonzero"] = lead_n
R["ready_topic_msgs_nonzero"] = ready_n
if cnt and off:
    R["fire_snapshots"] = []
    for tc, n in cnt:
        near = min(off, key=lambda o: abs(o[0] - tc))
        a = near[1]
        R["fire_snapshots"].append({"shoot_count": n, "t_cnt": tc, "nearest_off_t": near[0],
                                    "dx": a[0], "dy": a[1], "cls": a[2], "box_w": a[3], "conf": a[7]})
print(json.dumps(R, ensure_ascii=False, indent=1))
