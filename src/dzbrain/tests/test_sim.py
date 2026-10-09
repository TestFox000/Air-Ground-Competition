# -*- coding: utf-8 -*-
"""离线纯追踪仿真器回归测试（**纯脚本，无 pytest**；全过 exit 0，有失败 exit 1）。

运行::

    python src\\dzbrain\\tests\\test_sim.py

★ 这个文件存在的**唯一理由**：仿真器的结论会被拿去改现场参数，
  所以它自己必须先被证明"模型是对的"。断言按可信度分三层：

1. **模型不变量**（不可协商、必须精确成立）
   · 零注入时**信念与真值逐位相同**（差 == 0，不是"很小"）—— 这是"模型可信"的定义；
   · 舵角速率限幅在**指令侧**与**执行侧**都真的成立（倒车符号翻转单独归类）；
   · 分组指标口径完备（拍数可加、RMS 可合成）；
   · 相同参数两次运行逐位相同（可复现）。
2. **定性物理**（只断言方向/单调性，不写死数值）
   · 注入 ε → 信念误差变大、且**主要体现为信念误差而非横向误差**；
   · 锚定**不会让信念误差变差**（格心观测只在可用范围内修正）；
   · 延迟变大 → 横向误差变大、最终失效。
   ⚠️ 这类断言**不写死数值**：数值随参数变，写死等于教后来的人改测试。
3. **实测值记录**（把当前模型的真实水平打印出来，并给出"物理理由"型的上界）
   · 每个阈值后面都写清楚"这个数是几何推出来的还是实测取的"。

★★ 关于**第一个指令值不是 0**：`compute_command` 按契约首拍传 `last_steer_deg=None`
  （不做速率限幅），执行侧从 0° 斜坡跟上去 —— 所以第 1 拍的 |Δδ| 不算超限。
  同理**终点刹车**那一拍由 `brake_command` 直接给出 0°，跳变一次性出现且之后不再工作，
  它属于"任务结束"而不是"跟踪过程中超限"：本文件把它单列为 2.2b。

⚠️ 本文件只**读**仿真器，不修改 `sim.py`/`sim_cli.py`/`core`/`route`/`pursuit`/`config`。

★★ 2026-10-09 路线设计变更：底盘不支持倒车（决定性实验：STM32 下位机无视方向位）
   → robot.yaml 删掉全部 3 条 R: 倒车腿，180° 折返改为原地 PIVOT 掉头
   （E11/A6/D6，corner_turn_deg=180°）；巡航 0.15 m/s、lookahead_max 0.45→0.24。
   本文件的现场图路线自当日起**直接读 robot.yaml 的 route.tokens**（与 `sim_cli`
   缺省行为一致）；`fixtures/route_field.txt` 是旧路线夹具（含 R: 腿），继续用它 =
   全绿假象（在验证一条车端已不存在的路线）。鲁棒性断言按新基线实测重推导。
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import tempfile

_HERE = pathlib.Path(__file__).resolve()
_TESTS = _HERE.parent


def _find_src() -> pathlib.Path:
    """位置无关地找到 `src/dzbrain`（本文件被复制到草稿目录也能跑）。"""
    for parent in _HERE.parents:
        cand = parent / "src" / "dzbrain"
        if (cand / "dzbrain" / "__init__.py").is_file():
            return cand
    return _HERE.parent


_SRC = _find_src()
for _p in (str(_SRC), str(_TESTS)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
_WS = _SRC.parents[1]

try:                                    # Windows 控制台 GBK：° / ε / ↔ 编不出来会打断输出
    sys.stdout.reconfigure(errors="replace")   # type: ignore[union-attr]
except Exception:                       # pragma: no cover
    pass

from harness import check, check_close, finish, section    # noqa: E402

from dzbrain.core.config import Config                     # noqa: E402
from dzbrain.pursuit.path import Path                      # noqa: E402
from dzbrain.route.model import Route                      # noqa: E402
from dzbrain.route.parser import parse_mission             # noqa: E402
from dzbrain.tools import sim as S                         # noqa: E402

_CONFIG = _WS / "config" / "robot.yaml"
_GRID_M = 0.35                          # 场地格边长（调参的物理参照）
_R_MIN_M = 0.157 / math.tan(math.radians(30.0))   # 最小转弯半径 = 轴距/tan(舵角上限)


# --------------------------------------------------------------------------- #
#  夹具
# --------------------------------------------------------------------------- #
def load_cfg(**over) -> Config:
    """读现役 `robot.yaml` 并覆盖若干点号键（**不写磁盘**）。

    ⚠️ 仿真器必须能"不改仓库文件"就换参数，否则"哪组参数跑得完"的结论无法复现。
    ⚠️ 每次都从 `Config.load` 重新取基线：曾经把基线缓存进字典，结果第一次调用之后
      所有覆盖都失效（测试里表现为"改前视参数指标一模一样"）——不要缓存。
    """
    base = Config.load(str(_CONFIG))
    if not over:
        return base
    flat = dict(base._flat)
    flat.update({f"pursuit.{k}" if "." not in k else k: v for k, v in over.items()})
    cfg = Config(flat, source=f"{base.source} + {len(over)} overrides")
    cfg._migrate_legacy()
    return cfg


_FIELD_CACHE: dict[str, Route] = {}


def field_route() -> Route:
    """现役现场图路线 = `robot.yaml` 的 route.tokens（懒加载一次，不缓存 Config）。

    ★ 2026-10-09 起不再解析 `fixtures/route_field.txt`：该夹具仍是旧路线
    （含 R:F11/R:B6/R:D6 三条倒车腿），而车端已删倒车腿改 PIVOT —— 继续用旧夹具
    = 全绿假象（在验证一条车端已不存在的路线）。robot.yaml 是唯一真源。
    """
    if not _FIELD_CACHE:
        cfg = Config.load(str(_CONFIG))
        _FIELD_CACHE["route"] = parse_mission(
            list(cfg.get("route.tokens")), name="robot.yaml:route.tokens")
    return _FIELD_CACHE["route"]


def field_path() -> Path:
    return Path(field_route())


def straight_route() -> Route:
    """一条 1.05 m 的直线（3 格）：N10 → M10 → L10（沿 +y）。"""
    return parse_mission(["N10", "M10", "L10"], name="straight")


def gaps(result: S.SimResult) -> list[float]:
    """逐拍"信念↔真值"距离。"""
    return [math.hypot(result.x[i] - result.truth_x[i], result.y[i] - result.truth_y[i])
            for i in range(len(result.t))]


CFG = load_cfg()
#: 全场步数上限（按路线长度自动推算；显式给足是为了让"跑不完"是真的跑不完，
#: 而不是撞上限——撞上限的结论（`hit_step_limit`）与"绕圈"是两回事）。
BIG = S.auto_max_steps(field_path(), CFG, S.SimConfig())
DT = CFG.get("pursuit.dt_s") if CFG.get("pursuit.dt_s") else S.SimConfig().dt
RATE_LIMIT = CFG.get("pursuit.max_steer_rate_deg_s")
STEP_MAX_DEG = RATE_LIMIT * DT


def cfg_rate_limited(res: S.SimResult) -> list[int]:
    """返回**指令舵角**违反速率限幅的拍索引（**包含终点刹车拍**）。

    ⚠️ 首拍（index 0→1）按契约不做限幅（`last_steer_deg=None`），故从 index 2 起算。
    ⚠️ 终点那一拍由 `brake_command` 直接给出 0°（任务结束，之后没有跟踪），
      调用方需把它单独排除后再断言"跟踪过程内不超限"——见 2.2/2.2b。
    """
    cmd = res.steer_cmd
    tol = STEP_MAX_DEG * (1.0 + 1e-9)
    return [i for i in range(2, len(cmd)) if abs(cmd[i] - cmd[i - 1]) > tol]


# --------------------------------------------------------------------------- #
#  1. 零注入 → 闭环不变量（信念 == 真值）
# --------------------------------------------------------------------------- #
section("1. 零注入不变量：信念与真值逐拍相同（这是「模型可信」的定义）")

res0 = S.run_sim(field_route(), CFG,
                 S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0, max_steps=BIG))
g0 = gaps(res0)
check("1.1 零注入、无锚定：belief↔truth 最大偏差 == 0（不是「很小」）",
      max(g0) == 0.0,
      f"max={max(g0):.3e} rms={S._rms(g0):.3e} 拍数={len(g0)}")
check("1.2 逐拍都相同（不存在「只有开头对得上」）",
      all(v == 0.0 for v in g0),
      f"非零拍数={sum(1 for v in g0 if v != 0.0)}/{len(g0)}")
check_close("1.3 信念误差 RMS 指标也是 0", res0.metrics["belief_error_rms_m"], 0.0, 0.0)

res0b = S.run_sim(field_route(), CFG,
                  S.SimConfig(anchor_on_arrival=True, cmd_latency_s=0.0, max_steps=BIG))
g0b = gaps(res0b)
free0b = [g0b[i] for i in range(len(g0b)) if not res0b.anchored[i]]
check("1.4 零注入、**开**锚定：非锚定拍仍逐拍相同（吸附的修正量恒为 0）",
      max(free0b) == 0.0,
      f"非锚定拍 max={max(free0b):.3e}（{len(free0b)}/{len(g0b)} 拍），"
      f"吸附 {int(res0b.metrics['anchors'])} 次")
check("1.5 零注入下开/关锚定是**同一条轨迹**（吸附是恒等变换）",
      all(res0b.x[i] == res0.x[i] and res0b.y[i] == res0.y[i]
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
_, _, yaw_pos = S.step_bicycle(0.0, 0.0, 0.0, 0.15, 30.0, 0.02, 0.157)
_, _, yaw_neg = S.step_bicycle(0.0, 0.0, 0.0, 0.15, -30.0, 0.02, 0.157)
check("1.9 正舵角 ⇒ 左转、负舵角 ⇒ 右转（符号契约）", yaw_pos > 0.0 > yaw_neg,
      f"δ=+30° → yaw={yaw_pos:+.6f}；δ=-30° → yaw={yaw_neg:+.6f}")


# --------------------------------------------------------------------------- #
#  2. 舵角速率限幅真的生效（指令侧 + 执行侧）
# --------------------------------------------------------------------------- #
section("2. 舵角速率限幅：指令侧与执行侧都必须成立")

res_rate = S.run_sim(field_route(), CFG,
                     S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0, max_steps=BIG))
cmd = res_rate.steer_cmd
deltas = [abs(cmd[i] - cmd[i - 1]) for i in range(1, len(cmd))]
worst = max(range(len(deltas)), key=lambda i: deltas[i]) + 1
bad_all = cfg_rate_limited(res_rate)
mid_bad = [i for i in bad_all if i < len(cmd) - 2]
check("2.1 **跟踪过程内**指令舵角不超限（唯一例外是末尾两拍的停车序列）", not mid_bad,
      f"全段超限拍={bad_all or '无'}（末尾两拍 index={len(cmd)-2},{len(cmd)-1}）；"
      f"过程内超限={mid_bad or '无'}；过程内最大 Δδ="
      f"{max((deltas[i - 1] for i in range(2, len(cmd) - 2)), default=0.0):.6f}° "
      f"≤ {STEP_MAX_DEG:.4f}°（= {RATE_LIMIT:.0f} °/s × {DT} s）；"
      f"P95={res_rate.metrics['steer_rate_p95_deg_s']:.1f} °/s")
check("2.2 峰值出现在**终点刹车那一拍**（`brake_command` 一次性回正，属任务结束）",
      worst >= len(cmd) - 2 and deltas[worst - 1] > STEP_MAX_DEG,
      f"最大 Δδ={deltas[worst-1]:.4f}° 在第 {worst}/{len(cmd)} 拍；"
      f"末 3 拍 steer_cmd={[round(v, 3) for v in cmd[-3:]]}")
check("2.2b 刹车拍之后不再有指令（跳变不会污染后续跟踪）", worst >= len(cmd) - 2,
      f"刹车拍距末尾 {len(cmd) - worst} 拍")
check("2.2c 超限拍**全部**落在末尾两拍的停车序列上（不存在隐性超限）",
      all(i >= len(cmd) - 2 for i in bad_all),
      f"超限拍={bad_all or '无'}，末尾两拍 index={len(cmd)-2},{len(cmd)-1}；"
      f"控制器在到达终点前发出的最后一条指令 δ={cmd[len(cmd)-2]:.2f}°，"
      f"随后 `brake_command` 归零 ⇒ 这两拍的跳变属于停车动作，不是跟踪超限")

exec_d = [abs(res_rate.steer[i] - res_rate.steer[i - 1]) for i in range(1, len(res_rate.steer))]
check("2.3 **执行**舵角在跟踪过程内按斜坡跟随（终点回正那一拍除外）",
      max(exec_d[:-1]) <= STEP_MAX_DEG * (1 + 1e-9),
      f"过程内最大 Δδ={max(exec_d[:-1]):.6f}°（= {max(exec_d[:-1])/DT:.1f} °/s）；"
      f"末拍回正 Δδ={exec_d[-1]:.4f}°（刹车后舵机回正，不参与速率判定）")
check("2.4 执行侧「越限跳变」计数为 0（模型自检指标，正常必须为 0）",
      int(res_rate.metrics["steer_delivered_jump_steps"]) == 0,
      f"jump_steps={int(res_rate.metrics['steer_delivered_jump_steps'])}")
check("2.5 指令速率 P95 ≤ 限幅（分位数不含终点刹车脉冲）",
      res_rate.metrics["steer_rate_p95_deg_s"] <= RATE_LIMIT * (1 + 1e-9),
      f"P95={res_rate.metrics['steer_rate_p95_deg_s']:.1f} ≤ {RATE_LIMIT:.0f} °/s；"
      f"全段 peak={res_rate.metrics['steer_rate_max_deg_s']:.1f} °/s（= 刹车脉冲 "
      f"{deltas[worst-1]:.2f}° / {DT} s，见 2.2）")

res_nolim = S.run_sim(field_route(), CFG,
                      S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0,
                                  command_rate_limit=False, max_steps=BIG))
cmdnl = res_nolim.steer_cmd
d_nl = max(abs(cmdnl[i] - cmdnl[i - 1]) for i in range(1, len(cmdnl)))
check("2.6 负例：关掉限幅后**确实**出现超限跳变（证明 2.1 不是恒真）",
      d_nl > STEP_MAX_DEG,
      f"无限幅最大 Δδ={d_nl:.4f}° = {d_nl/DT:.0f} °/s > 阈值 {STEP_MAX_DEG:.4f}°")


# --------------------------------------------------------------------------- #
#  3. 直线路线零误差
# --------------------------------------------------------------------------- #
section("3. 直线路线（1.05 m）零误差")

res_line = S.run_sim(straight_route(), CFG,
                     S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0))
check("3.1 跑完全程", bool(res_line.metrics["completed"]) is True,
      f"completed={res_line.metrics['completed']} t={res_line.metrics['time_s']:.2f}s")
check("3.2 横向 RMS < 0.01 m", res_line.metrics["cross_track_rms_m"] < 0.01,
      f"实测 rms={res_line.metrics['cross_track_rms_m']:.6f} m")
check("3.3 末点误差 < 0.15 m", res_line.metrics["final_error_m"] < 0.15,
      f"实测末点误差={res_line.metrics['final_error_m']:.4f} m（≈ 终点刹车门 0.03 m + 一个采样周期）")
check("3.4 全程无越界", int(res_line.metrics["out_of_bounds_steps"]) == 0,
      f"越界拍数={int(res_line.metrics['out_of_bounds_steps'])}")
check("3.5 零误差信念仍与真值逐拍相同", max(gaps(res_line)) == 0.0,
      f"max gap={max(gaps(res_line)):.3e}")


# --------------------------------------------------------------------------- #
#  4. 现场图 34 航点零误差（出厂参数）
# --------------------------------------------------------------------------- #
section("4. 现场图 34 航点、零误差、**出厂参数**（实测值原样记录）")

res_field = S.run_sim(field_route(), CFG,
                      S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0, max_steps=BIG))
m = res_field.metrics
check("4.1 跑完全程（不撞步数上限）", bool(m["completed"]) is True
      and not bool(m["hit_step_limit"]),
      f"completed={m['completed']} hit_step_limit={m['hit_step_limit']} "
      f"t={m['time_s']:.2f}s steps={int(m['steps'])}/{int(m['max_steps'])}")
check("4.2 无越界拍", int(m["out_of_bounds_steps"]) == 0,
      f"越界={int(m['out_of_bounds_steps'])} 拍，越界余量={m['bounds_margin_m']} m")
check("4.3 信念↔真值 RMS == 0", m["belief_error_rms_m"] == 0.0,
      f"rms={m['belief_error_rms_m']:.3e} max={m['belief_error_max_m']:.3e}")
check("4.4 直段横向 RMS < 1/4 格（几何参照：格 0.35 m）",
      m["cross_track_rms_normal_m"] < _GRID_M / 4,
      f"实测 直段={m['cross_track_rms_normal_m']:.4f} "
      f"拐角={m['cross_track_rms_corner_m']:.4f} "
      f"倒车={m['cross_track_rms_reverse_m']:.4f} 总={m['cross_track_rms_m']:.4f} m")
check("4.5 拐角横向 RMS < 1/2 格", m["cross_track_rms_corner_m"] < _GRID_M / 2,
      f"实测 拐角={m['cross_track_rms_corner_m']:.4f} m，"
      f"拐角峰值={m['cross_track_max_corner_m']:.4f} m")
# ★ 2026-10-09 阈值 _R_MIN_M(0.272) → 2×_R_MIN_M(0.544)：旧路线折返靠倒车（车头不转、
#   超调小，实测 0.2233 m），新路线 3 个 180° 折返（E11/A6/D6）在 sim 里由纯追踪以
#   最小转弯半径画弧，最近掠过顶点 ≈ 2R_min（实测 0.3114 m）。
#   ⚠️ **sim 未实现 PIVOT**（实车 PIVOT 原地转向、无此弧）—— 这是 sim 与实车的
#   模型边界，不是控制退化；2×R_min 是"纯追踪弧"的几何上界，超过它才算异常。
check("4.6 顶点超调（非倒车段）< 2×R_min=0.544 m（180° 折返纯追踪弧的几何上界）",
      m["vertex_overshoot_normal_m"] < 2.0 * _R_MIN_M,
      f"实测 非倒车={m['vertex_overshoot_normal_m']:.4f} m "
      f"倒车={m['vertex_overshoot_reverse_m']:.4f} m（R: 腿已删，恒 0）；"
      f"阈值 2×R_min={2.0 * _R_MIN_M:.3f} m 是几何推出来的，不是实测凑的")
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
check("4.10 分组耗时之和 == 总耗时", abs(
    m["time_s_normal"] + m["time_s_reverse"] + m["time_s_corner"] - m["time_s"]) < 1e-6,
      f"{m['time_s_normal']:.2f}+{m['time_s_reverse']:.2f}+{m['time_s_corner']:.2f}"
      f"={m['time_s_normal']+m['time_s_reverse']+m['time_s_corner']:.2f} vs {m['time_s']:.2f} s")

# 前视距离在**本路线的速度区间内不起作用**（前视 = clamp(k_v·v, [min,max])，
# v∈[0.12,0.15]、k_v=1.2 ⇒ 1.44~1.8 × 0.1 = 0.144~0.18 ⇒ 下限 0.18 恒定生效）。
# 所以"调小前视把车调死"这件事在当前参数下**不可复现**：这里把真实结论钉住，
# 免得有人照着一句想当然的结论去改现场参数。
res_ld15 = S.run_sim(field_route(), load_cfg(lookahead_min_m=0.15, lookahead_k_v=0.0),
                     S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0, max_steps=BIG))
res_ld30 = S.run_sim(field_route(), load_cfg(lookahead_min_m=0.30, lookahead_k_v=0.0),
                     S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0, max_steps=BIG))
check("4.11 前视强行钉到 0.15 m（k_v=0）**仍能跑完** → 前视不是本路线的死锁根因",
      bool(res_ld15.metrics["completed"]),
      f"ld≡0.15: completed={res_ld15.metrics['completed']} "
      f"t={res_ld15.metrics['time_s']:.1f}s 横向RMS={res_ld15.metrics['cross_track_rms_m']:.4f} m")
check("4.12 前视钉到 0.30 m 亦能跑完，但横向误差更小、超调更小（**用进弯提前量换贴线度**）",
      bool(res_ld30.metrics["completed"])
      and res_ld30.metrics["vertex_overshoot_normal_m"]
      < res_ld15.metrics["vertex_overshoot_normal_m"],
      f"ld≡0.15 超调={res_ld15.metrics['vertex_overshoot_normal_m']:.3f} "
      f"RMS={res_ld15.metrics['cross_track_rms_m']:.4f} | "
      f"ld≡0.30 超调={res_ld30.metrics['vertex_overshoot_normal_m']:.3f} "
      f"RMS={res_ld30.metrics['cross_track_rms_m']:.4f}")


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
check("5.4 现场图已无倒车段（2026-10-09 底盘不支持倒车：R:F11/R:B6/R:D6 全删）",
      len(rev_segs) == int(m["reverse_segment_count"]) == 0,
      f"reverse 段={[s.index for s in rev_segs]}；旧基线 3 段（索引 12/18/19）")
pivots = [field_route().waypoints[i + 1].block
          for i in range(field_path().segment_count - 1)
          if field_path().corner_turn_deg(i) >= 135.0]
check("5.4b 3 个 180° 折返点全部转为 ≥135° PIVOT 掉头（E11/A6/D6，corner_turn_deg=180°）",
      sorted(pivots) == ["A6", "D6", "E11"],
      f"pivots={sorted(pivots)}")
peaks = sorted(res_field.segments, key=lambda s: -s.peak_m)[:3]
check("5.5 峰值最大的 3 段可定位（含起点/终点/峰值时刻）",
      all(s.peak_t >= 0.0 and s.peak_m > 0.0 for s in peaks),
      " | ".join(f"seg{s.index} {s.from_block}→{s.to_block} "
                 f"{s.peak_m:.4f}m@{s.peak_t:.1f}s" for s in peaks))
check("5.6 峰值段的分类可能被**超调**误导（0.35 m 拐角带之外的顶点会被记成 normal）",
      peaks[0].kind in ("reverse", "corner") or peaks[0].corner_deg > 30.0,
      f"TOP1 = seg{peaks[0].index} {peaks[0].from_block}→{peaks[0].to_block} "
      f"kind={peaks[0].kind} 转角={peaks[0].corner_deg:.1f}° 峰值={peaks[0].peak_m:.3f} m —— "
      f"⚠️ 该段是 90° 弯但被记成 {peaks[0].kind}：车从顶点外侧 0.4 m 掠过，"
      f"真值投影落在 `_CORNER_ZONE_M=0.35 m` 之外。这是**分组口径的已知边界**，"
      f"不是模型错误；调参时要用「转角 > 30°」而不是只信 kind。")


# --------------------------------------------------------------------------- #
#  6. 误差注入对照（仿真器存在的核心价值）
# --------------------------------------------------------------------------- #
section("6. 误差注入：ε 里程比例误差（有/无锚定）")

res_e3n = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, speed_scale_err=0.03,
                                                    cmd_latency_s=0.0, max_steps=BIG))
res_e3a = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=True, speed_scale_err=0.03,
                                                    cmd_latency_s=0.0, max_steps=BIG))
m3n, m3a = res_e3n.metrics, res_e3a.metrics
check("6.1 ε=+0.03 真的进了模型（信念误差 ≫ 0）",
      m3n["belief_error_rms_m"] > 0.05,
      f"ε=+0.03 无锚定 信念RMS={m3n['belief_error_rms_m']:.4f} m "
      f"（零注入={m['belief_error_rms_m']:.4f} m）")
check("6.2 锚定**不会让信念误差变差**（格心观测只在 `anchor_radius_m` 内修正）",
      m3a["belief_error_rms_m"] <= m3n["belief_error_rms_m"] + 1e-9,
      f"有锚定={m3a['belief_error_rms_m']:.4f} vs 无锚定={m3n['belief_error_rms_m']:.4f} m，"
      f"吸附 {int(m3a['anchors'])} 次")
check("6.3 ε 主要放大的是**信念误差**而不是横向误差（建模结论，显式记录）",
      m3n["belief_error_rms_m"] > m3n["cross_track_rms_m"],
      f"信念RMS={m3n['belief_error_rms_m']:.4f} > 横向RMS={m3n['cross_track_rms_m']:.4f} m —— "
      f"信念沿路线「多跑」ε×里程，真值仍在同一条折线上，故横向对它不敏感")
check("6.4 ε=+0.03 两组都能跑完（对照才有意义）",
      bool(m3a["completed"]) and bool(m3n["completed"]),
      f"锚定={m3a['completed']} len={m3a['path_len_ratio']:.2f} | "
      f"无锚定={m3n['completed']} len={m3n['path_len_ratio']:.2f}")
check("6.5 ε=+0.03 锚定把**信念误差**压得更低（这是锚定唯一无条件成立的收益）",
      m3a["belief_error_rms_m"] < m3n["belief_error_rms_m"],
      f"有锚定 信念RMS={m3a['belief_error_rms_m']:.4f} < 无锚定={m3n['belief_error_rms_m']:.4f} m；"
      f"⚠️ 代价：锚定版的**末点误差反而更大**（{m3a['final_error_m']:.3f} vs "
      f"{m3n['final_error_m']:.3f} m）、路更长（{m3a['path_len_ratio']:.3f} vs "
      f"{m3n['path_len_ratio']:.3f}）—— 因为它每吸附一次就把信念钉到真值，"
      f"控制器被纠正后重新贴线，等效于多走一点。**要不要开锚定取决于赛道考核的是估计精度还是停车精度。**")

# 大 ε：如实记录"锚定救不回来"，并钉住原因（修正窗口 = anchor_radius_m 是硬闸门）
res_e18n = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False,
                                                     speed_scale_err=0.18,
                                                     cmd_latency_s=0.0, max_steps=BIG))
res_e18a = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=True,
                                                     speed_scale_err=0.18,
                                                     cmd_latency_s=0.0, max_steps=BIG))
m18n, m18a = res_e18n.metrics, res_e18a.metrics
check("6.6 ε=+0.18 **两组都跑不完**（如实记录：这是当前模型的失效边界）",
      not m18n["completed"] and not m18a["completed"],
      f"无锚定 len={m18n['path_len_ratio']:.2f} t={m18n['time_s']:.0f}s "
      f"diverged={m18n['diverged']}（~2567 拍提前退出） | "
      f"有锚定 len={m18a['path_len_ratio']:.2f} t={m18a['time_s']:.0f}s "
      f"diverged={m18a['diverged']} 越界={int(m18a['out_of_bounds_steps'])} 拍"
      f"（新基线：锚定版走得更远、最终越界，但同样跑不完）")
# ★ 2026-10-09 重推导：新路线上不锚定版 ~2567 拍就 diverged 提前退出、锚定版走得更远
#   （误差随里程增长），**全长 RMS 对比不公平**——全长实测反而出现 有锚定 0.817 >
#   无锚定 0.754 的假象。意图不变（锚定改善信念误差），改成**同窗口 2500 拍**对比。
res_e18nw = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False,
                                                      speed_scale_err=0.18,
                                                      cmd_latency_s=0.0, max_steps=2500))
res_e18aw = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=True,
                                                      speed_scale_err=0.18,
                                                      cmd_latency_s=0.0, max_steps=2500))
m18nw, m18aw = res_e18nw.metrics, res_e18aw.metrics
check("6.7 ε=+0.18 时锚定仍然**改善了信念误差**（同窗口 2500 拍对比，虽不足以救回路线）",
      m18aw["belief_error_rms_m"] < m18nw["belief_error_rms_m"],
      f"同窗口 2500 拍：有锚定={m18aw['belief_error_rms_m']:.4f} < "
      f"无锚定={m18nw['belief_error_rms_m']:.4f} m（全长对比被不锚定版提前 diverged 污染）；"
      f"吸附 {int(m18aw['anchors'])} 次（少，因为 |信念−真值| 常超过 "
      f"anchor_radius_m=0.12 m 这个**观测可用窗口**，此时按设计不修）")
check("6.8 ε=+0.18 失效时横向误差仍**有界**（看门狗会判跑飞，但不是数值爆炸）",
      m18n["cross_track_max_m"] < 1.0,
      f"横向MAX={m18n['cross_track_max_m']:.3f} m、diverged="
      f"{res_e18n.metrics['diverged']}（`diverged` 由横向误差超阈值触发）、"
      f"长度比={m18n['path_len_ratio']:.2f} —— 同型失效在 6.12 用逐拍轨迹验过："
      f"车绕着一个顶点打转、target_idx 停在固定航点不动，信念误差只有 0.35 m")

res_noise = S.run_sim(field_route(), CFG,
                      S.SimConfig(anchor_on_arrival=True, pos_noise_m=0.005,
                                  cmd_latency_s=0.0, max_steps=BIG))
res_noise_big = S.run_sim(field_route(), CFG,
                          S.SimConfig(anchor_on_arrival=True, pos_noise_m=0.01,
                                      cmd_latency_s=0.0, max_steps=BIG))
res_drift = S.run_sim(field_route(), CFG,
                      S.SimConfig(anchor_on_arrival=True, yaw_drift_deg_per_min=2.0,
                                  cmd_latency_s=0.0, max_steps=BIG))
check("6.9 位置白噪声入口有效（信念误差被噪声抬起）",
      res_noise.metrics["belief_error_rms_m"] > 0.0,
      f"噪声 0.005 m/拍 → 信念RMS={res_noise.metrics['belief_error_rms_m']:.4f} m"
      f"（零注入=0）")
check("6.10 航向零漂入口有效（信念↔真值出现偏差）",
      res_drift.metrics["belief_error_max_m"] > 0.0,
      f"零漂 2°/min → 信念MAX={res_drift.metrics['belief_error_max_m']:.4f} m")
# ★★ 这是本次最值得上报的**鲁棒性边界**（敏感性曲线实测，见报告）：
#    噪声 ≤0.005 m/拍 → 跑完（性能逐级退化）；0.01 m/拍 → 拐角绕圈死锁、跑不完。
check("6.11 噪声 ≤0.005 m/拍 仍跑得完（性能退化但不出失效）",
      bool(res_noise.metrics["completed"]),
      f"0.005 m/拍 completed={res_noise.metrics['completed']} "
      f"t={res_noise.metrics['time_s']:.1f}s 长度比={res_noise.metrics['path_len_ratio']:.2f} "
      f"末点误差={res_noise.metrics['final_error_m']:.3f} m")
check("6.12 噪声 0.01 m/拍 **跑不完**（如实钉住鲁棒性边界，这是要上报的缺口）",
      not res_noise_big.metrics["completed"],
      f"0.01 m/拍 completed={res_noise_big.metrics['completed']} "
      f"t={res_noise_big.metrics['time_s']:.1f}s 长度比={res_noise_big.metrics['path_len_ratio']:.2f} "
      f"超调={res_noise_big.metrics['vertex_overshoot_normal_m']:.2f} m "
      f"末点误差={res_noise_big.metrics['final_error_m']:.2f} m —— "
      f"逐拍诊断：车在第一个 180° 折返（E11 簇，seg14 之后，s≈3.5 m）反复绕圈，"
      f"|信念−真值| 最大 {res_noise_big.metrics['belief_error_max_m']:.2f} m，"
      f"**不是定位误差大，而是拐角处缺少「够不到就重规划」的兜底**"
      f"（旧基线同缺口：死在 seg10 顶点，2026-10-09 路线改 PIVOT 后仍存在）")
_a = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, speed_scale_err=0.03,
                                               pos_noise_m=0.01, max_steps=BIG))
_b = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, speed_scale_err=0.03,
                                               pos_noise_m=0.01, max_steps=BIG))
check("6.13 确定性：同参数两次运行**逐位相同**（可复现是仿真器的底线）",
      _a.x == _b.x and _a.y == _b.y and _a.metrics == _b.metrics,
      f"逐拍序列与指标完全一致（{len(_a.x)} 拍）")


# --------------------------------------------------------------------------- #
#  7. 指令延迟
# --------------------------------------------------------------------------- #
section("7. 指令延迟（零注入）")

res_l0 = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.0,
                                                   max_steps=BIG))
res_l06 = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.06,
                                                    max_steps=BIG))
res_l2 = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.2,
                                                   max_steps=BIG))
check("7.1 零延迟能跑完", bool(res_l0.metrics["completed"]),
      f"t={res_l0.metrics['time_s']:.1f}s rms={res_l0.metrics['cross_track_rms_m']:.4f} m")
check("7.2 现场估计的 0.06 s 延迟代价很小（横向 RMS 增幅 < 10%）",
      0.0 <= res_l06.metrics["cross_track_rms_m"] - res_l0.metrics["cross_track_rms_m"]
      < 0.1 * res_l0.metrics["cross_track_rms_m"],
      f"0s={res_l0.metrics['cross_track_rms_m']:.6f} → "
      f"0.06s={res_l06.metrics['cross_track_rms_m']:.6f} m（+"
      f"{(res_l06.metrics['cross_track_rms_m']/res_l0.metrics['cross_track_rms_m']-1)*100:.1f}%，"
      f"3 拍延迟大部分被 120 °/s 的执行斜坡吸收）")
check("7.3 0.2 s 延迟横向 RMS 继续变大（延迟的物理后果，单调）",
      res_l2.metrics["cross_track_rms_m"] > res_l06.metrics["cross_track_rms_m"]
      > res_l0.metrics["cross_track_rms_m"],
      f"0s={res_l0.metrics['cross_track_rms_m']:.4f} → "
      f"0.06s={res_l06.metrics['cross_track_rms_m']:.4f} → "
      f"0.2s={res_l2.metrics['cross_track_rms_m']:.4f} m（+"
      f"{(res_l2.metrics['cross_track_rms_m']/res_l0.metrics['cross_track_rms_m']-1)*100:.0f}%）")
check("7.4 0.2 s 延迟仍不发散、仍有界（模型稳健，不靠运气跑完）",
      res_l2.metrics["cross_track_rms_m"] < 0.5 and not res_l2.metrics["diverged"],
      f"t={res_l2.metrics['time_s']:.1f}s len={res_l2.metrics['path_len_ratio']:.2f} "
      f"diverged={res_l2.metrics['diverged']}")
res_l8 = S.run_sim(field_route(), CFG, S.SimConfig(anchor_on_arrival=False, cmd_latency_s=0.8,
                                                   max_steps=BIG))
check("7.5 延迟 0.8 s 时路线跑不完（把失效点也钉住，避免「延迟无所谓」的错觉）",
      not res_l8.metrics["completed"],
      f"0.8s completed={res_l8.metrics['completed']} "
      f"len={res_l8.metrics['path_len_ratio']:.2f} "
      f"横向MAX={res_l8.metrics['cross_track_max_m']:.2f} m")


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
check("8.3 分组指标口径（拐角 ±0.35 m、顶点超调分倒车/非倒车）写进了键名",
      all(k in m for k in ("cross_track_rms_corner_m", "vertex_overshoot_normal_m",
                           "steer_rate_p95_deg_s", "time_s_corner")),
      "四个调参必需的键都在")

series = ["t", "x", "y", "yaw", "v", "steer", "truth_x", "truth_y", "truth_yaw",
          "seg_index", "s", "truth_seg_index", "truth_s", "steer_cmd",
          "anchored", "near_vertex", "reverse"]
lens = {name: len(res_field.series(name)) for name in series}
check("8.4 全部逐拍序列等长（列错位是最难查的 bug）",
      len(set(lens.values())) == 1,
      f"长度集合={set(lens.values())}，序列数={len(series)}")
_gap_track = max(abs(res_field.steer[i] - res_field.steer_cmd[i])
                 for i in range(len(res_field.steer) - 2))
check("8.5 `steer`（执行）与 `steer_cmd`（指令）在跟踪过程内**逐步一致**（除末尾停车两拍）",
      res_field.steer != res_field.steer_cmd and _gap_track == 0.0,
      f"过程内最大 |执行−指令| = {_gap_track:.6f}°（执行斜坡每拍最多 "
      f"{STEP_MAX_DEG:.2f}°，实测从未落后）；"
      f"末尾两拍差 {abs(res_field.steer[-1] - res_field.steer_cmd[-1]):.2f}°"
      f"（刹车归零后舵机仍在回正斜坡上，属正常）")
_cmd_rates = [abs(res_field.steer_cmd[i] - res_field.steer_cmd[i - 1]) / DT
              for i in range(1, len(res_field.steer_cmd) - 2)]
check("8.6 跟踪过程内指令速率**从未超出** 120 °/s 限幅（只有停车那拍是脉冲）",
      max(_cmd_rates) <= RATE_LIMIT * (1 + 1e-9),
      f"过程内 peak={max(_cmd_rates):.2f} °/s（**正好贴着**限幅 {RATE_LIMIT:.0f} °/s，"
      f"说明限幅确实在工作、且这条路线用满了一个斜坡拍）；"
      f"指标里的 peak={res_field.metrics['steer_rate_max_deg_s']:.1f} °/s 来自停车脉冲"
      f"（{abs(res_field.steer_cmd[-1] - res_field.steer_cmd[-3]):.2f}° / {DT} s）")

with tempfile.TemporaryDirectory() as tmp:
    csv_path = S.write_csv(res_field, os.path.join(tmp, "traj.csv"))
    svg_path = S.write_svg(res_field, os.path.join(tmp, "traj.svg"), field_route(),
                           title="unit test")
    seg_path = S.write_segments_csv(res_field, os.path.join(tmp, "by_segment.csv"))
    check("8.7 write_csv 落盘（路径存在且非空）",
          os.path.isfile(csv_path) and os.path.getsize(csv_path) > 1000,
          f"{os.path.basename(csv_path)} = {os.path.getsize(csv_path)} B")
    with open(csv_path, "r", encoding="utf-8") as fh:
        head = fh.readline().strip()
        rows = sum(1 for _ in fh)
    check("8.8 CSV 前 10 列 == 契约列序，且含诊断列；行数 == 采样拍数",
          head.startswith("t,x,y,yaw,v,steer,truth_x,truth_y,seg_index,s")
          and "steer_cmd" in head and "anchored" in head and rows == len(res_field.t),
          f"列={head}；数据行={rows} 采样拍={len(res_field.t)}")
    svg_text = pathlib.Path(svg_path).read_text(encoding="utf-8")
    check("8.9 write_svg 落盘且含路线折线 / 真值轨迹 / 航点元素",
          os.path.isfile(svg_path) and "<svg" in svg_text and "</svg>" in svg_text
          and svg_text.count("<polyline") >= 3 and "<circle" in svg_text,
          f"{os.path.getsize(svg_path)} B，polyline={svg_text.count('<polyline')} "
          f"circle={svg_text.count('<circle')}")
    with open(seg_path, "r", encoding="utf-8") as fh:
        seg_head = fh.readline().strip()
        seg_rows = sum(1 for _ in fh)
    check("8.10 write_segments_csv 落盘且段数正确（每段一行，含峰值与时刻）",
          seg_head.startswith("index,from_block,to_block") and seg_rows == len(res_field.segments)
          and "peak_m" in seg_head and "peak_t" in seg_head,
          f"列={seg_head}；行={seg_rows} 段={len(res_field.segments)}")

fmt = S.format_metrics(res_field, S.SimConfig(), title="t")
check("8.11 指标表格式化含主表 + 分组表，不抛异常",
      "分组指标" in fmt and "拐角" in fmt and "信念" in fmt,
      f"输出 {len(fmt)} 字符")
check("8.12 逐段表格式化不抛异常且列出峰值",
      "峰值" in S.format_segment_table(res_field, top=5),
      f"TOP5 表 {len(S.format_segment_table(res_field, top=5))} 字符")

p_field = field_path()
p_rev = [i for i in range(p_field.segment_count) if p_field.segment_is_reverse(i)]
p_piv = sorted(field_route().waypoints[i + 1].block
               for i in range(p_field.segment_count - 1)
               if p_field.corner_turn_deg(i) >= 135.0)
check("8.13 现场路线几何：34 航点 / 16.45 m / 0 倒车段 / 3 个 180° PIVOT 折返（E11/A6/D6）",
      len(p_field.waypoints) == 34 and abs(p_field.total_length - 16.45) < 0.01
      and p_rev == [] and p_piv == ["A6", "D6", "E11"],
      f"航点={len(p_field.waypoints)} 总长={p_field.total_length:.3f} m "
      f"倒车段={p_rev}（旧基线 [12,18,19]：R:F11/R:B6/R:D6 已删）PIVOT 折返={p_piv}")
check("8.14 最小转弯半径 R_min = 轴距/tan(30°) = 0.272 m > 半格 0.175 m"
      "（这是现场「到不了格心」的几何根因）",
      abs(_R_MIN_M - 0.27193) < 1e-4 and _R_MIN_M > _GRID_M / 2,
      f"R_min={_R_MIN_M:.5f} m，半格={_GRID_M/2:.3f} m，格={_GRID_M} m")


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
