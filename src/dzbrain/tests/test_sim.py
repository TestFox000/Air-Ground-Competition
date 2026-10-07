# -*- coding: utf-8 -*-
"""离线纯追踪仿真器回归测试（**纯脚本，无 pytest**；全过 exit 0，有失败 exit 1）。

运行::

    python src\\dzbrain\\tests\\test_sim.py

★ 这个测试文件存在的**唯一理由**：仿真器的结论会被拿去改现场参数，
  所以它自己必须先被证明"模型是对的"。三类断言，按可信度从高到低：

1. **模型不变量**（不可协商、必须精确成立）
   · 零注入时**信念与真值逐位相同**（差 == 0，不是"很小"）—— 这是"模型可信"的定义；
   · 舵角速率限幅在**指令侧**与**执行侧**都真的成立（倒车符号翻转单独归类）；
   · 直线路线零误差能跑完；分组指标口径完备（拍数可加、RMS 可合成）。
2. **定性物理**（只断言方向，不写死数值）
   · 注入 ε=+0.18 → 信念误差显著变大；开锚定 → 最大偏差被夹住、路线跑得完；
   · 延迟变大 → 拐角误差变大。
   ⚠️ 这类断言**不写死数值**：数值随参数变，写死等于教后来的人改测试。
3. **契约**（指标键、CSV/SVG/分段 CSV 落盘、逐拍序列等长）。

★ 关于两条**"规格与现实的偏差"**（本文件把偏差原样打印，不偷偷放宽阈值）：
  · 现场图 `cross_track_rms_m` 的物理地板约 0.10 m（不是"越小越好"的调参问题）：
    `max_steer_deg=30°` + `wheelbase=0.157 m` ⇒ 最小转弯半径 `L/tan30° = 0.272 m`，
    而现场图一个 90° 弯只给 0.35 m 段长 ⇒ 车必然从顶点外侧掠过。
  · 现场图在**出厂参数**（`lookahead_min_m=0.18` / `arrival_m=0.12`）下**跑不完**：
    `R:B6 → R:D6` 要求在 0.70 m 内完成掉头（需要 R≈0.35 m），而物理下限是 0.272 m，
    纯追踪会在 A6 外圈反复绕。测试把这一条**如实钉住**（4.10），
    另用实测可跑的一组参数（`RUNNABLE`）覆盖"跑完 + 无越界"。

⚠️ 本文件只**读**仿真器，不修改 `sim.py`/`sim_cli.py`/`core`/`route`/`pursuit`/`config`。
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import tempfile

_HERE = pathlib.Path(__file__).resolve()
_TESTS = _HERE.parent
_SRC = _TESTS.parent
for _p in (str(_SRC), str(_TESTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _find_src() -> pathlib.Path:
    """位置无关地找到 `src/dzbrain`（本文件被复制到草稿目录也能跑）。"""
    for parent in _HERE.parents:
        cand = parent / "src" / "dzbrain"
        if (cand / "dzbrain" / "__init__.py").is_file():
            return cand
    return _SRC


_SRC = _find_src()
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
_WS = _SRC.parents[1]

try:                                    # Windows 控制台 GBK：° / ε / ↔ 编不出来会打断输出
    sys.stdout.reconfigure(errors="replace")   # type: ignore[union-attr]
except Exception:                       # pragma: no cover
    pass

from harness import check, check_close, finish, section    # noqa: E402

from dzbrain.core.config import Config                     # noqa: E402
from dzbrain.pursuit.path import Path                      # noqa: E402
from dzbrain.route.parser import parse_mission             # noqa: E402
from dzbrain.tools import sim as S                         # noqa: E402

_CONFIG = _WS / "config" / "robot.yaml"
_ROUTE_FIELD = _TESTS / "fixtures" / "route_field.txt"


# --------------------------------------------------------------------------- #
#  夹具
# --------------------------------------------------------------------------- #
def load_cfg(**over) -> Config:
    """读现役 `robot.yaml` 并覆盖若干点号键（**不写磁盘**）。

    ⚠️ 仿真器必须能"不改仓库文件"就换参数，否则"哪组参数跑得完"的结论无法复现。
    """
    base = Config.load(str(_CONFIG))
    if not over:
        return base
    flat = dict(base._flat)
    flat.update(over)
    cfg = Config(flat, source=f"{base.source} + {len(over)} overrides")
    cfg._migrate_legacy()
    return cfg


#: 现场图路线**实测可跑完**的一组参数（出厂值在 A6 折返处死锁，见 4.10）。
#: 只改两个键：前视距离下限 0.18→0.26 m（转角更早开始转、顶点超调降到 0），
#: 到位半径 0.12→0.20 m（承认"最小转弯半径 0.272 m > 半格 0.175 m"这个几何事实）。
RUNNABLE = {"pursuit.arrival_m": 0.20, "pursuit.lookahead_min_m": 0.26}


def field_route():
    return parse_mission(str(_ROUTE_FIELD), name="route_field.txt")


def straight_route():
    """一条 1.05 m 的直线（3 格）：N10 → M10 → L10（沿 +y）。"""
    return parse_mission(["N10", "M10", "L10"], name="straight")


def gaps(result: S.SimResult) -> list[float]:
    """逐拍"信念↔真值"距离。"""
    return [math.hypot(result.x[i] - result.truth_x[i], result.y[i] - result.truth_y[i])
            for i in range(len(result.t))]


# --------------------------------------------------------------------------- #
#  1. 零注入 → 闭环不变量（信念 == 真值）
# --------------------------------------------------------------------------- #
section("1. 零注入不变量：信念与真值逐拍相同（这是"模型可信"的定义）")

cfg0 = load_cfg()
res0 = S.run_sim(field_route(), cfg0, S.SimConfig(anchor_on_arrival=False))
g0 = gaps(res0)
check("1.1 零注入、无锚定：belief↔truth 最大偏差 == 0（不是"很小"）",
      max(g0) == 0.0,
      f"max={max(g0):.3e} rms={S._rms(g0):.3e} 拍数={len(g0)}")
check("1.2 逐拍都相同（不存在"只有开头对得上"）",
      all(v == 0.0 for v in g0),
      f"非零拍数={sum(1 for v in g0 if v != 0.0)}/{len(g0)}")
check_close("1.3 信念误差 RMS 指标也是 0", res0.metrics["belief_error_rms_m"], 0.0, 0.0)

res0b = S.run_sim(field_route(), cfg0, S.SimConfig(anchor_on_arrival=True))
g0b = gaps(res0b)
free0b = [g0b[i] for i in range(len(g0b)) if not res0b.anchored[i]]
check("1.4 零注入、**开**锚定：非锚定拍仍逐拍相同（吸附的修正量恒为 0）",
      max(free0b) == 0.0,
      f"非锚定拍 max={max(free0b):.3e}（{len(free0b)}/{len(g0b)} 拍），"
      f"吸附 {int(res0b.metrics['anchors'])} 次")
check("1.5 零注入下开/关锚定是**同一条轨迹**（吸附是恒等变换）",
      all(abs(res0b.x[i] - res0.x[i]) == 0.0 and abs(res0b.y[i] - res0.y[i]) == 0.0
          for i in range(min(len(res0.t), len(res0b.t)))),
      f"末点误差 开={res0b.metrics['final_error_m']:.6f} 关={res0.metrics['final_error_m']:.6f}")

# ---- 单步运动学（单元级，与闭环解耦）----
x1, y1, yaw1 = S.step_bicycle(0.0, 0.0, 0.0, 0.15, 0.0, 0.02, 0.157)
check("1.6 直线前进 1 拍：y 不变、x = v·dt", y1 == 0.0 and x1 == 0.15 * 0.02,
      f"({x1!r}, {y1!r}, {yaw1!r})")
x2, y2, yaw2 = S.step_bicycle(0.5, 0.5, 1.234, 0.0, 0.0, 0.02, 0.157)
check("1.7 v=0 且 δ=0 → 原地不动（含航向）",
      (x2, y2, yaw2) == (0.5, 0.5, 1.234), f"({x2}, {y2}, {yaw2})")
x3, y3, yaw3 = S.step_bicycle(0.0, 0.0, 0.0, 0.15, 30.0, 0.02, 0.157)
check_close("1.8 δ=30° 的偏航率 = v/L·tanδ（与下位机同构）",
            yaw3, 0.15 / 0.157 * math.tan(math.radians(30.0)) * 0.02, 1e-12)
x4, y4, yaw4 = S.step_bicycle(0.0, 0.0, 0.0, 0.15, 30.0, 0.02, 0.157)
_, _, yaw5 = S.step_bicycle(0.0, 0.0, 0.0, 0.15, -30.0, 0.02, 0.157)
check("1.9 正舵角 ⇒ 左转、负舵角 ⇒ 右转（符号契约）", yaw4 > 0.0 > yaw5,
      f"δ=+30° → yaw={yaw4:+.6f}；δ=-30° → yaw={yaw5:+.6f}")


# --------------------------------------------------------------------------- #
#  2. 舵角速率限幅真的生效（指令侧 + 执行侧）
# --------------------------------------------------------------------------- #
section("2. 舵角速率限幅：指令侧与执行侧都必须成立")

cfg_rate = load_cfg(**RUNNABLE)
res_rate = S.run_sim(field_route(), cfg_rate, S.SimConfig(anchor_on_arrival=False))
dt_v = res_rate.metrics["dt_s"]
limit = cfg_rate.get("pursuit.max_steer_rate_deg_s")
tol = limit * dt_v * (1.0 + 1e-9)

cmd = res_rate.steer_cmd
deltas = [abs(cmd[i] - cmd[i - 1]) for i in range(1, len(cmd))]
worst = max(range(len(deltas)), key=lambda i: deltas[i]) + 1
check("2.1 **指令**舵角相邻两拍变化 ≤ max_steer_rate_deg_s × dt",
      max(deltas) <= tol,
      f"最大 Δδ={max(deltas):.6f}° ≤ 阈值 {tol:.6f}°（第 {worst} 拍）；"
      f"实测峰值 {max(deltas)/dt_v:.1f} °/s ≤ 限幅 {limit:.0f} °/s")

flip_pts = [i for i in range(len(res_rate.reverse) - 1)
            if res_rate.reverse[i] != res_rate.reverse[i + 1]]
check("2.2 倒车符号翻转被**单独计数**（不混进速率判定、不伪装成超限）",
      int(res_rate.metrics["steer_sign_flip_steps"]) <= len(flip_pts) + 1,
      f"指标 flip={int(res_rate.metrics['steer_sign_flip_steps'])}，"
      f"倒车段标记切换点={len(flip_pts)}（段索引 {flip_pts}）；"
      f"翻转拍速率峰值={res_rate.metrics['steer_rate_at_flip_deg_s']:.1f} °/s（如实记录）")

exec_d = [abs(res_rate.steer[i] - res_rate.steer[i - 1]) for i in range(1, len(res_rate.steer))]
check("2.3 **执行**舵角相邻两拍变化 ≤ 同一限幅（执行侧是斜坡跟随，不是阶跃）",
      max(exec_d) <= tol,
      f"最大 Δδ={max(exec_d):.6f}°（= {max(exec_d)/dt_v:.1f} °/s）")
check("2.4 执行侧"越限跳变"计数为 0（模型自检指标，正常必须为 0）",
      int(res_rate.metrics["steer_delivered_jump_steps"]) == 0,
      f"jump_steps={int(res_rate.metrics['steer_delivered_jump_steps'])}")
check("2.5 速率分位数 ≤ 峰值 ≤ 限幅（三者自洽）",
      res_rate.metrics["steer_rate_p95_deg_s"] <= res_rate.metrics["steer_rate_max_deg_s"] + 1e-9
      and res_rate.metrics["steer_rate_max_deg_s"] <= limit * (1 + 1e-9),
      f"P95={res_rate.metrics['steer_rate_p95_deg_s']:.1f} "
      f"peak={res_rate.metrics['steer_rate_max_deg_s']:.1f} limit={limit:.0f} °/s")

res_nolim = S.run_sim(field_route(), cfg_rate,
                      S.SimConfig(anchor_on_arrival=False, command_rate_limit=False))
cmdnl = res_nolim.steer_cmd
d_nl = max(abs(cmdnl[i] - cmdnl[i - 1]) for i in range(1, len(cmdnl)))
check("2.6 负例：关掉限幅后**确实**出现超限跳变（证明 2.1 不是恒真）",
      d_nl > tol,
      f"无限幅最大 Δδ={d_nl:.4f}° = {d_nl/dt_v:.0f} °/s > 阈值 {tol:.4f}°")


# --------------------------------------------------------------------------- #
#  3. 直线路线零误差
# --------------------------------------------------------------------------- #
section("3. 直线路线（1.05 m）零误差")

res_line = S.run_sim(straight_route(), cfg0, S.SimConfig(anchor_on_arrival=False))
check("3.1 跑完全程", bool(res_line.metrics["completed"]) is True,
      f"completed={res_line.metrics['completed']} t={res_line.metrics['time_s']:.2f}s")
check("3.2 横向 RMS < 0.01 m", res_line.metrics["cross_track_rms_m"] < 0.01,
      f"实测 rms={res_line.metrics['cross_track_rms_m']:.6f} m")
check("3.3 末点误差 < 0.15 m", res_line.metrics["final_error_m"] < 0.15,
      f"实测末点误差={res_line.metrics['final_error_m']:.4f} m")
check("3.4 全程无越界", int(res_line.metrics["out_of_bounds_steps"]) == 0,
      f"越界拍数={int(res_line.metrics['out_of_bounds_steps'])}")
check("3.5 零误差信念仍与真值逐拍相同", max(gaps(res_line)) == 0.0,
      f"max gap={max(gaps(res_line)):.3e}")


# --------------------------------------------------------------------------- #
#  4. 现场图 34 航点零误差
# --------------------------------------------------------------------------- #
section("4. 现场图 34 航点、零误差（**实测值原样记录，不为通过而放宽阈值**）")

res_field = S.run_sim(field_route(), cfg_rate, S.SimConfig(anchor_on_arrival=False))
m = res_field.metrics
check("4.1 跑完全程", bool(m["completed"]) is True,
      f"completed={m['completed']} t={m['time_s']:.2f}s steps={int(m['steps'])}")
check("4.2 无越界拍", int(m["out_of_bounds_steps"]) == 0,
      f"越界={int(m['out_of_bounds_steps'])} 拍，越界余量={m['bounds_margin_m']} m")
check("4.3 信念↔真值 RMS == 0", m["belief_error_rms_m"] == 0.0,
      f"rms={m['belief_error_rms_m']:.3e} max={m['belief_error_max_m']:.3e}")
check("4.4 直段横向 RMS < 0.08 m（实测记录见 detail）",
      m["cross_track_rms_normal_m"] < 0.08,
      f"实测 直段={m['cross_track_rms_normal_m']:.4f} "
      f"拐角={m['cross_track_rms_corner_m']:.4f} "
      f"倒车={m['cross_track_rms_reverse_m']:.4f} 总={m['cross_track_rms_m']:.4f} m")
check("4.5 拐角横向 RMS < 0.20 m（几何地板约 0.10 m，见文件头）",
      m["cross_track_rms_corner_m"] < 0.20,
      f"实测 拐角={m['cross_track_rms_corner_m']:.4f} m，"
      f"拐角峰值={m['cross_track_max_corner_m']:.4f} m")
check("4.6 顶点超调 ≈ 0（非倒车段单独统计）", m["vertex_overshoot_normal_m"] < 0.02,
      f"实测 非倒车={m['vertex_overshoot_normal_m']:.4f} m "
      f"倒车={m['vertex_overshoot_reverse_m']:.4f} m")
check("4.7 路径长度比接近 1（没有反复绕圈）", 0.85 <= m["path_len_ratio"] <= 1.30,
      f"实测 {m['path_len_ratio']:.4f} ×（<1 = 过弯切角省了路程）")

tot = int(m["steps_normal"]) + int(m["steps_reverse"]) + int(m["steps_corner"])
check("4.8 分组拍数之和 == 总拍数（分组口径完备，不漏不重）", tot == int(m["steps"]),
      f"{int(m['steps_normal'])}+{int(m['steps_reverse'])}+{int(m['steps_corner'])}"
      f"={tot} vs steps={int(m['steps'])}")
rms_join = math.sqrt((m["cross_track_rms_normal_m"] ** 2 * int(m["steps_normal"])
                      + m["cross_track_rms_reverse_m"] ** 2 * int(m["steps_reverse"])
                      + m["cross_track_rms_corner_m"] ** 2 * int(m["steps_corner"]))
                     / max(1, tot))
check("4.9 三个分组 RMS 的加权平方和 == 总 RMS（同一距离定义）",
      abs(rms_join - m["cross_track_rms_m"]) < 1e-9,
      f"合成={rms_join:.6f} vs 总={m['cross_track_rms_m']:.6f}")

res_factory = S.run_sim(field_route(), cfg0, S.SimConfig(anchor_on_arrival=False))
check("4.10 出厂参数（ld=0.18/arrival=0.12）在 A6 折返处**跑不完**（如实钉住）",
      not res_factory.metrics["completed"],
      f"completed={res_factory.metrics['completed']} "
      f"len_ratio={res_factory.metrics['path_len_ratio']:.2f} "
      f"末点误差={res_factory.metrics['final_error_m']:.2f} m —— 根因是几何："
      f"0.70 m 内做掉头需要 R≥0.35 m，物理下限 L/tan30°=0.272 m")
check("4.11 只改两个键（前视下限 0.26 + 到位 0.20）就能跑完 → 问题在参数不在模型",
      bool(res_factory.metrics["completed"]) is False and m["completed"] is True,
      f"出厂={res_factory.metrics['time_s']:.0f}s 未完成 → 实测参数组={m['time_s']:.1f}s 完成")


# --------------------------------------------------------------------------- #
#  5. 逐段峰值表
# --------------------------------------------------------------------------- #
section("5. 逐段横向误差峰值表（调参定位用）")

n_wp = len(field_route().waypoints)
check("5.1 段数 == 航点数 − 1", len(res_field.segments) == n_wp - 1,
      f"segments={len(res_field.segments)} waypoints={n_wp}")
check("5.2 每段峰值 ≥ 0 且 ≤ 全局横向 MAX",
      all(0.0 <= s.peak_m <= m["cross_track_max_m"] + 1e-12 for s in res_field.segments),
      f"全局 MAX={m['cross_track_max_m']:.4f}，段峰值最大="
      f"{max(s.peak_m for s in res_field.segments):.4f}")
check("5.3 每段都有起止格名与终点转角（可用于定位）",
      all(s.from_block and s.to_block and s.corner_deg >= 0.0 for s in res_field.segments),
      f"例 seg0 {res_field.segments[0].from_block}→{res_field.segments[0].to_block} "
      f"转角={res_field.segments[0].corner_deg:.1f}°")
rev_segs = [s for s in res_field.segments if s.kind == "reverse"]
check("5.4 倒车段被正确标类型（现场图有 R:F11 / R:B6 / R:D6）",
      len(rev_segs) == int(m["reverse_segment_count"]) and len(rev_segs) == 3,
      f"reverse 段索引={[s.index for s in rev_segs]}（共 {len(rev_segs)}）")
peaks = sorted(res_field.segments, key=lambda s: -s.peak_m)[:3]
check("5.5 峰值最大的 3 段可定位（含起点/终点/峰值时刻）",
      all(s.peak_t > 0.0 and s.peak_m > 0.0 for s in peaks),
      " | ".join(f"seg{s.index} {s.from_block}→{s.to_block} "
                 f"{s.peak_m:.4f}m@{s.peak_t:.1f}s" for s in peaks))


# --------------------------------------------------------------------------- #
#  6. 误差注入对照（仿真器存在的核心价值）
# --------------------------------------------------------------------------- #
section("6. 误差注入：ε=+0.18 的里程比例误差（有/无锚定）")

EPS = 0.18
res_no_anchor = S.run_sim(field_route(), cfg_rate,
                          S.SimConfig(anchor_on_arrival=False, speed_scale_err=EPS))
res_anchor = S.run_sim(field_route(), cfg_rate,
                       S.SimConfig(anchor_on_arrival=True, speed_scale_err=EPS))
mn, ma = res_no_anchor.metrics, res_anchor.metrics

check("6.1 注入后信念误差显著大于零注入（ε 真的进了模型）",
      mn["belief_error_rms_m"] > 0.05,
      f"ε=+0.18 无锚定 信念RMS={mn['belief_error_rms_m']:.4f} m；"
      f"零注入 信念RMS={m['belief_error_rms_m']:.4f} m")
check("6.2 ε 主要放大的是**信念误差**，不是横向误差（建模结论，显式记录）",
      mn["belief_error_rms_m"] > mn["cross_track_rms_m"],
      f"信念RMS={mn['belief_error_rms_m']:.4f} > 横向RMS={mn['cross_track_rms_m']:.4f} m "
      f"（信念沿路线"多跑" ε×里程，真值仍在同一条折线上）")
check("6.3 锚定把**最大**信念偏差夹住（不锚定无界累积）",
      ma["belief_error_max_m"] < mn["belief_error_max_m"],
      f"有锚定 max={ma['belief_error_max_m']:.4f} m < 无锚定 max={mn['belief_error_max_m']:.4f} m")
check("6.4 锚定让路线**跑得完**（无锚定反复绕圈：长度比 ≫ 1）",
      bool(ma["completed"]) and ma["path_len_ratio"] < mn["path_len_ratio"],
      f"有锚定 completed={ma['completed']} len={ma['path_len_ratio']:.2f} t={ma['time_s']:.1f}s"
      f" | 无锚定 completed={mn['completed']} len={mn['path_len_ratio']:.2f} t={mn['time_s']:.1f}s")

res_e3n = S.run_sim(field_route(), cfg_rate,
                    S.SimConfig(anchor_on_arrival=False, speed_scale_err=0.03))
res_e3a = S.run_sim(field_route(), cfg_rate,
                    S.SimConfig(anchor_on_arrival=True, speed_scale_err=0.03))
m3n, m3a = res_e3n.metrics, res_e3a.metrics
check("6.5 ε=+0.03：两组都能跑完（对照才有意义）",
      bool(m3a["completed"]) and bool(m3n["completed"]),
      f"锚定={m3a['completed']} 无锚定={m3n['completed']}")
check("6.6 ε=+0.03：有锚定**更省路**（无锚定靠绕圈把里程补回来）",
      m3a["path_len_ratio"] < m3n["path_len_ratio"],
      f"有锚定 len={m3a['path_len_ratio']:.2f} t={m3a['time_s']:.1f}s | "
      f"无锚定 len={m3n['path_len_ratio']:.2f} t={m3n['time_s']:.1f}s")

res_noise = S.run_sim(field_route(), cfg_rate,
                      S.SimConfig(anchor_on_arrival=True, pos_noise_m=0.01))
res_drift = S.run_sim(field_route(), cfg_rate,
                      S.SimConfig(anchor_on_arrival=True, yaw_drift_deg_per_min=2.0))
check("6.7 位置白噪声入口有效（信念误差被噪声抬起）",
      res_noise.metrics["belief_error_rms_m"] > 0.0,
      f"噪声 0.01 m/拍 → 信念RMS={res_noise.metrics['belief_error_rms_m']:.4f} m "
      f"（零注入=0）")
check("6.8 航向零漂入口有效（信念↔真值出现偏差）",
      res_drift.metrics["belief_error_max_m"] > 0.0,
      f"零漂 2°/min → 信念MAX={res_drift.metrics['belief_error_max_m']:.4f} m")
check("6.9 确定性：同参数两次运行**逐位相同**（仿真器必须是可复现的）",
      all(S.run_sim(field_route(), cfg_rate,
                    S.SimConfig(speed_scale_err=EPS, pos_noise_m=0.01)).x[i]
          == S.run_sim(field_route(), cfg_rate,
                       S.SimConfig(speed_scale_err=EPS, pos_noise_m=0.01)).x[i]
          for i in range(0, 200)),
      "同 seed/同参数 → 前 200 拍逐位相同")


# --------------------------------------------------------------------------- #
#  7. 指令延迟
# --------------------------------------------------------------------------- #
section("7. 指令延迟：0 s / 0.06 s（现场估计）/ 0.2 s")

res_l0 = S.run_sim(field_route(), cfg_rate, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0))
res_l06 = S.run_sim(field_route(), cfg_rate, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.06))
res_l2 = S.run_sim(field_route(), cfg_rate, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.2))
check("7.1 零延迟能跑完", bool(res_l0.metrics["completed"]),
      f"t={res_l0.metrics['time_s']:.1f}s rms={res_l0.metrics['cross_track_rms_m']:.4f} m")
check("7.2 0.2 s 延迟不发散（横向误差有界、未被看门狗判跑飞）",
      res_l2.metrics["cross_track_rms_m"] < 0.5 and not res_l2.metrics["diverged"],
      f"t={res_l2.metrics['time_s']:.1f}s rms={res_l2.metrics['cross_track_rms_m']:.4f} m "
      f"len={res_l2.metrics['path_len_ratio']:.2f} diverged={res_l2.metrics['diverged']}")
check("7.3 延迟越大 → **拐角**横向误差越大（延迟的物理后果，单调）",
      res_l2.metrics["cross_track_rms_corner_m"] > res_l06.metrics["cross_track_rms_corner_m"]
      and res_l06.metrics["cross_track_rms_corner_m"] >= res_l0.metrics["cross_track_rms_corner_m"] - 1e-12,
      f"拐角RMS: 0s={res_l0.metrics['cross_track_rms_corner_m']:.4f} "
      f"0.06s={res_l06.metrics['cross_track_rms_corner_m']:.4f} "
      f"0.2s={res_l2.metrics['cross_track_rms_corner_m']:.4f} m")
check("7.4 延迟 0.2 s 时路线跑不完（**如实记录**：现场若真有此延迟，先修链路再谈调参）",
      not res_l2.metrics["completed"],
      f"0.2 s 延迟 completed={res_l2.metrics['completed']} "
      f"len={res_l2.metrics['path_len_ratio']:.2f} 末点误差={res_l2.metrics['final_error_m']:.2f} m")


# --------------------------------------------------------------------------- #
#  8. 契约：指标键 / 序列等长 / CSV / SVG / 分段 CSV
# --------------------------------------------------------------------------- #
section("8. 契约：指标键、序列等长、CSV/SVG/分段 CSV 落盘")

missing = [k for k in S.METRIC_KEYS if k not in m]
check("8.1 契约指标键一个不少（`docs/P0接口约定.md` §5）", not missing,
      f"缺={missing or '无'}，共 {len(S.METRIC_KEYS)} 个")
gmissing = [k for k in S.GROUPED_METRIC_KEYS if k not in m]
check("8.2 分组指标键一个不少（本次新增，调参必需）", not gmissing,
      f"缺={gmissing or '无'}，共 {len(S.GROUPED_METRIC_KEYS)} 个")

series = ["t", "x", "y", "yaw", "v", "steer", "truth_x", "truth_y", "truth_yaw",
          "seg_index", "s", "truth_seg_index", "truth_s", "steer_cmd",
          "anchored", "near_vertex", "reverse"]
lens = {name: len(res_field.series(name)) for name in series}
check("8.3 全部逐拍序列等长（列错位是最难查的 bug）",
      len(set(lens.values())) == 1,
      f"长度集合={set(lens.values())}，序列数={len(series)}")
check("8.4 诊断列语义正确：steer = 执行舵角，steer_cmd = 指令舵角，两者不同",
      res_field.steer_cmd != res_field.steer
      and all(abs(res_field.steer[i] - res_field.steer_cmd[i]) <= limit + 1e-9
              for i in range(len(res_field.steer))),
      f"最大 |执行−指令| = {max(abs(res_field.steer[i] - res_field.steer_cmd[i]) "
                                f"for i in range(len(res_field.steer))):.4f}°（≤ 限幅 {limit:.0f}°）")

with tempfile.TemporaryDirectory() as tmp:
    csv_path = S.write_csv(res_field, os.path.join(tmp, "traj.csv"))
    svg_path = S.write_svg(res_field, os.path.join(tmp, "traj.svg"), field_route(),
                           title="unit test")
    seg_path = S.write_segments_csv(res_field, os.path.join(tmp, "by_segment.csv"))
    check("8.5 write_csv 落盘（路径存在且非空）",
          os.path.isfile(csv_path) and os.path.getsize(csv_path) > 1000,
          f"{os.path.basename(csv_path)} = {os.path.getsize(csv_path)} B")
    with open(csv_path, "r", encoding="utf-8") as fh:
        head = fh.readline().strip()
        rows = sum(1 for _ in fh)
    check("8.6 CSV 前 10 列 == 契约列序，且含诊断列；行数 == 采样拍数",
          head.startswith("t,x,y,yaw,v,steer,truth_x,truth_y,seg_index,s")
          and "steer_cmd" in head and "anchored" in head and rows == len(res_field.t),
          f"列={head}；数据行={rows} 采样拍={len(res_field.t)}")
    svg_text = pathlib.Path(svg_path).read_text(encoding="utf-8")
    check("8.7 write_svg 落盘且含路线折线 / 真值轨迹 / 航点元素",
          os.path.isfile(svg_path) and "<svg" in svg_text and "</svg>" in svg_text
          and svg_text.count("<polyline") >= 3 and "<circle" in svg_text,
          f"{os.path.getsize(svg_path)} B，polyline={svg_text.count('<polyline')} "
          f"circle={svg_text.count('<circle')}")
    with open(seg_path, "r", encoding="utf-8") as fh:
        seg_head = fh.readline().strip()
        seg_rows = sum(1 for _ in fh)
    check("8.8 write_segments_csv 落盘且段数正确",
          seg_head.startswith("index,from_block,to_block") and seg_rows == len(res_field.segments),
          f"列={seg_head}；行={seg_rows} 段={len(res_field.segments)}")

fmt = S.format_metrics(res_field, S.SimConfig(), title="t")
check("8.9 指标表格式化含主表 + 分组表，不抛异常",
      "分组指标" in fmt and "横向RMS 拐角" in fmt and "信念" in fmt,
      f"输出 {len(fmt)} 字符")
check("8.10 逐段表格式化不抛异常",
      "峰值" in S.format_segment_table(res_field, top=5),
      f"TOP5 表 {len(S.format_segment_table(res_field, top=5))} 字符")

# 路线几何自检：现场图 34 点 / 16.45 m
path_field = Path(field_route())
check("8.11 现场路线几何：34 航点 / 16.45 m / 3 个倒车段",
      len(path_field.waypoints) == 34 and abs(path_field.total_length - 16.45) < 0.01
      and sum(1 for i in range(path_field.segment_count)
              if path_field.segment_is_reverse(i)) == 3,
      f"航点={len(path_field.waypoints)} 总长={path_field.total_length:.3f} m "
      f"倒车段={[i for i in range(path_field.segment_count) if path_field.segment_is_reverse(i)]}")


# --------------------------------------------------------------------------- #
#  9. SimConfig 自检（非法参数必须立刻报错，而不是跑完才看出来）
# --------------------------------------------------------------------------- #
section("9. SimConfig 参数自检")

for i, (bad, why) in enumerate((
        (dict(dt=0.0), "dt=0"),
        (dict(dt=-0.02), "dt<0"),
        (dict(cmd_latency_s=-1.0), "负延迟"),
        (dict(speed_scale_err=-1.0), "ε=-1（里程计完全不动）"),
        (dict(pos_noise_m=0.8), "白噪声超注入上限"),
        (dict(anchor_radius_m=0.0), "吸附半径 0"),
        (dict(trace_every=0), "抽稀间隔 0"),
        (dict(max_steps=-1), "步数上限为负"),
        (dict(init_yaw="foo"), "未知初值朝向"),
), start=1):
    try:
        S.SimConfig(**bad).validate()
        check(f"9.{i} {why} 必须抛 ValueError", False, "没有抛")
    except ValueError as exc:
        check(f"9.{i} {why} → ValueError", True, str(exc)[:64])


finish()
