#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""瞄准开火参数审计：把车上真实参数与"建议值"对表，直接指出哪些没改。

用法（车上或本地）：
    ros2 param dump /dzrknn_dataprocess > dump.yaml
    python aim_param_audit.py dump.yaml
    # 或直接管道： ros2 param dump /dzrknn_dataprocess | python aim_param_audit.py -

为什么需要：现场症状（"各种转 / 放不在中心 / 打不到靶子"）多数并非算法错，
而是**这些参数仍停在代码默认值**（尤其 px_per_tick 未标定 = 收敛命门）。
"""
from __future__ import annotations

import sys

# 代码里 declare_parameter 的真实默认值（本文件随机核对于 dzrknn_dataprocess.cpp）
CODE_DEFAULT = {
    "px_per_tick": 2.0,
    "gyro_ff_gain": 0.0,
    "gyro_ff_sign": 1.0,
    "predict_gain": 0.5,
    "vision_latency_ms": 0.0,
    "use_external_fire_gate": False,
    "lead_apply_gain": 0.0,
    "ema_alpha": 0.3,
    "patrol_speed_ratio": 0.3,
    "track_deadzone_px": 2,
    "track_kp": 8.0,
    "track_kd": 3.0,
    "fire_dz_far_px": 3.5,
    "fire_dz_near_px": 3.5,
    "fire_use_image_stable": True,
    "target_confirm_frames": 4,
    "momentum_comp_on": True,
    "momentum_sign": 1.0,
    "scan_yaw_min_deg": -160.0,
    "scan_yaw_max_deg": 160.0,
    "fire_hold_frames": 5,
    "fire_stable_px_per_frame": 1.5,
    "shot_cooldown_ms": 500,
    "max_shots": 10,
    "max_shots_per_target": 3,
    "fire_only_blue": True,
    "laser_offset_x": 0.0,
    "laser_offset_y": 0.0,
    "box_w_far": 30.0,
    "box_w_near": 120.0,
}

# 建议值（来源：锁定增强补丁说明 §3.4 / 提前量启用与标定流程 / 扫描与EMA定值 /
#        ★2026-10-18 车上实测复核：车端为增量式PID版本，px_per_tick/track_kp 口径已换算）
#   (建议值, 若不符会造成的症状)
RECOMMEND = {
    "px_per_tick":         ("车端pid模式无此键→看pid_kp", "车端(增量式PID)不存在此键。口径换算：每帧修正比例=pid_kp/31.9；kp=10→31%(稳态滞后≈13.8px)。267ms链路延迟下kp上限≈10~12，追死拍(20~32)会自激震荡"),
    "pid_kp":              ("10.0~12.0(车端口径)", "车端现役增量式PID真源。kp=10 修正31%；勿按旧口径(本地副本px_per_tick)整定，增益顶到20+会因267ms纯延迟自激"),
    "max_shots":           (10,    "★P0：试射遗留=1 ⇒ 全程只能打1发且打满即永久禁火(须重启节点)。出车前必须改回10"),
    "gyro_ff_gain":        (1.0,   "车体一转靶就漂出画面 → 微找/重扫循环 =『各种转』"),
    "gyro_ff_sign":        ("±1(用标定脚本)", "填错会把漂移翻倍（见 aim_ff_calib_sim）"),
    "predict_gain":        (0.0,   "上游lead(dzbrain /aim/lead_x)已做全链路补偿时必须归零，否则双重补偿过冲；若上游只做部分补偿可保留0.5(≈16ms)"),
    "vision_latency_ms":   (0.0,   "同上（预测器接管）"),
    "use_external_fire_gate": (True, "不开则移动靶几乎打不出去（画面静止判据不可能满足）"),
    "lead_apply_gain":     (0.6,   "不施加提前量 → 延迟 0.26s 的 33px 飘移全落到靶外"),
    "ema_alpha":           (0.5,   "0.3 滞后 78ms → 移动靶永远追不到中心（可到 0.7）"),
    "patrol_speed_ratio":  (0.25,  "扫描过快 → 单趟可见帧少 → 锁定概率低"),
    "track_kp":            ("pid模式不生效(车端看pid_kp)", "车端track_mode=pid时此键与track_kd/track_inc_gain均不生效(横幅仍打印会误导)；真源是pid_kp"),
    "fire_dz_far_px":      (14.0,  "开火框须 > 机械抖动+余量：车端机械精度≈12px ⇒ 14/26 合理(车上实测口径)；过小则hold 5帧难满足"),
    "fire_dz_near_px":     (26.0,  "同上；far==near 时动态框等于没启用"),
    "fire_hold_frames":    (3,     "需连续 N 帧全条件成立；条件闪烁时 N 越大越难兑现"),
    "fire_stable_px_per_frame": (2.5, "1.5px/帧 对移动靶几乎不可满足（超前生效后才自然满足）"),
    "laser_offset_x":      ("标定值", "激光与相机不同轴 → 开火零点非画面中心；不标则『中心对准也打偏』"),
    "laser_offset_y":      ("标定值", "同上（竖直）"),
    "max_shots_per_target": (2,     "10 发硬预算：k=3 只能交战 3 个目标；k=2 用 5 个且 p≥0.9 时打全率仍 96~98%"),
    "shot_cooldown_ms":     (1200,  "500ms 会让弹匣 4~5s 打光；拉长以分摊到多个停点并给伺服从后坐恢复"),
    "target_confirm_frames": (6,    "10 发不允许误检浪费：提高确认质量"),
}


def parse_dump(text: str) -> dict:
    """从 ros2 param dump 文本里取 ros__parameters 段的标量键值（容错解析）。"""
    out = {}
    inside = False
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if "ros__parameters:" in line:
            inside = True
            continue
        if not inside:
            continue
        if ":" not in line:
            continue
        key, _, val = line.strip().partition(":")
        key, val = key.strip(), val.strip()
        if not key or not val:
            continue
        val = val.split("#")[0].strip()
        if val.startswith(("'", '"')) and val.endswith(("'", '"')) and len(val) >= 2:
            val = val[1:-1]
        out[key] = val
    return out


def as_num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def main() -> int:
    args = [a for a in sys.argv[1:] if a != "-"]
    if not args:
        print(__doc__)
        return 2
    text = sys.stdin.read() if "-" in sys.argv[1:] else open(args[0], encoding="utf-8").read()
    got = parse_dump(text)
    if not got:
        print("[ERR] 没解析到 ros__parameters（确认 dump 里有该节点名）")
        return 2

    print(f"解析到 {len(got)} 个参数\n")
    print(f"{'参数':<26s} {'车上现值':>12s} {'代码默认':>10s} {'建议':>18s}  判定")
    print("-" * 104)
    n_bad = 0
    for key, (rec, symptom) in RECOMMEND.items():
        cur = got.get(key, "(未dump到)")
        dflt = CODE_DEFAULT.get(key, "-")
        cur_n, rec_n = as_num(cur), as_num(rec if not isinstance(rec, str) or rec.lstrip('-').replace('.', '').isdigit() else None)
        if key in ("gyro_ff_sign", "px_per_tick", "laser_offset_x",
                   "laser_offset_y") and isinstance(rec, str):
            bad = True
            verdict = "★需标定/复核"
        elif cur_n is None or rec_n is None:
            bad = str(cur).strip().lower() != str(rec).strip().lower()
            verdict = "★不符" if bad else "OK"
        else:
            bad = abs(cur_n - rec_n) > 1e-6
            verdict = ("★不符" if bad else "OK")
        if bad:
            n_bad += 1
        print(f"{key:<26s} {str(cur):>12s} {str(dflt):>10s} {str(rec):>18s}  {verdict}")
        if bad:
            print(f"{'':<26s}   └─ 症状：{symptom}")

    print("-" * 104)
    print(f"合计需处理：{n_bad} 项")
    print("\n★ 优先顺序（2026-10-18 车上实测复核后的口径）：")
    print("  1) max_shots=1 是 P0：出车前必须改回 10（打满即永久禁火，只能重启节点恢复）")
    print("  2) use_external_fire_gate=true + lead_apply_gain 0.5~0.6 —— 治『移动靶打不到』")
    print("     （车端 /aim/lead_x 已由 dzbrain 发布，两键链路在二进制里已就绪）")
    print("  3) gyro_ff_gain 0.8~1.0 + 用 gimbal_ff_calib.py 标符号 —— 治『车一转靶就飞』")
    print("  4) laser_offset_x/y 标定 —— 治『瞄得正也打偏』（『发10/中0』的隐藏根因）")
    print("  5) 车端 pid_kp 勿超 12（267ms 链路延迟的自激上限）；稳态滞后用 lead 消，不要加 kp")
    print("  6) 规程：每次改参/换靶/打满后 重启 5_gimbal —— shots_fired_ 是进程级状态不随参数重载清零")
    return 0


if __name__ == "__main__":
    sys.exit(main())
