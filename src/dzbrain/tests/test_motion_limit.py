# -*- coding: utf-8 -*-
"""t20：`motion_limit` 限速请求 —— 共享键收口 + 读侧钳速 + 无信号逐位 no-op。

覆盖（对应任务卡 t20 验收 ①~⑤）：
  1. 共享键收口：`core.blackboard.P_MOTION_LIMIT` 与**写侧**（`tasks/localization.py`）
     是同一个字符串；键名只此一处定义，读侧（`pursuit.controller`）引用它。
  2. ★ 无信号 = 完全 no-op：键不存在 / 值为 `None` / 载荷非法（非 dict、缺字段、非数字、
     NaN、±inf、负数、bool）→ `compute_command` 与 `speed_profile` 的结果**逐位**等于
     不传限速时的结果，并且等于**改动前 controller 的历史黄金值**（写死常量，见 §2.3）。
  3. 限速生效：钳到 `speed_mps`、`speed_reason == "motion_limit"`、只降不升、
     `0.0` 真的给 0（不被 `min_speed_mps` 蠕动下限顶回去）。
  4. 倒车/负速度**按绝对值钳且保号**（`0.0` 时给正零）。
  5. 配置对账：`config/robot.yaml` 的 `localization:` 段 12 个键 ——
     ① 等于**现场落盘值**（5.2，不是 SCHEMA 默认值：边界两键有意偏离）
     ② 每个值都在 SCHEMA 声明的 lo/hi 内（5.2b）
     ③ 与默认值的偏离**恰好**是 boundary_guard_m / boundary_stop_m（5.2c，语义护栏）
     + 标定三项与布局模式 route.tokens 空表 / 第七图路线夹具 68 token（5.3 / 5.3b，v4.4.0）。

跑法：`$env:PYTHONPATH='…\\src\\dzbrain'; python src\\dzbrain\\tests\\test_motion_limit.py`
全过 exit 0；任一失败 exit 1。
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:
    sys.stdout.reconfigure(errors="replace")  # type: ignore[union-attr]
except Exception:                              # pragma: no cover
    pass

from dzbrain.core.blackboard import Blackboard, P_MOTION_LIMIT                          # noqa: E402
from dzbrain.core.config import Config                                                  # noqa: E402
from dzbrain.core.geometry import Pose                                                  # noqa: E402
from dzbrain.pursuit.controller import (PursuitCommand, compute_command,                # noqa: E402
                                        motion_limit_speed_limit, speed_profile)
from dzbrain.pursuit.path import Path                                                    # noqa: E402
from dzbrain.route.model import Route, RouteBuilder, Waypoint                            # noqa: E402
from dzbrain.tasks.localization import P_MOTION_LIMIT as WRITER_KEY                      # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]

_PASS = 0
_FAIL = 0
_FAILED: list[str] = []


def ok(name: str, cond: bool, detail: str = "") -> None:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        _FAILED.append(name)
        print(f"FAIL  {name}   [{detail}]")


def sec(title: str) -> None:
    print()
    print(f"--- {title} ---")


def cfg_of(**pursuit) -> Config:
    base = {"cruise_speed_mps": 0.30, "min_speed_mps": 0.05, "speed_quant_comp": 1.0,
            "stop_approach_speed_gain": 0.30, "corner_speed_mps": 0.12,
            "corner_slow_dist_m": 0.25, "reverse_speed_ratio": 0.6}
    base.update(pursuit)
    return Config.from_dict({"pursuit": base})


def route_of(points, reverses=None) -> Route:
    b = RouteBuilder("t20")
    rev = set(reverses or ())
    for i, (x, y) in enumerate(points):
        b.add(Waypoint(block=f"P{i}", x=float(x), y=float(y), yaw=0.0, reverse=(i in rev)))
    return b.build()


def straight(length: float = 5.0, reverses=None) -> Path:
    return Path(route_of([(0.0, 0.0), (length, 0.0)], reverses=reverses))


def bb_with(payload) -> Blackboard:
    bb = Blackboard()
    bb.set(P_MOTION_LIMIT, payload)
    return bb


def main() -> int:
    cfg = cfg_of()

    # ------------------------------------------------------------------ #
    sec("1. 共享键收口（写侧/读侧同一个字符串常量）")
    ok("core.blackboard.P_MOTION_LIMIT == 'motion_limit'", P_MOTION_LIMIT == "motion_limit",
       f"{P_MOTION_LIMIT!r}")
    ok("★ 写侧 localization.P_MOTION_LIMIT == 读侧 core.blackboard.P_MOTION_LIMIT",
       WRITER_KEY == P_MOTION_LIMIT, f"写侧 {WRITER_KEY!r} / 收口处 {P_MOTION_LIMIT!r}")

    # ------------------------------------------------------------------ #
    sec("2. 无信号 = 完全 no-op")
    p = straight()
    poses = [Pose(x=0.5, y=0.0, yaw=0.0), Pose(x=2.0, y=0.0, yaw=0.0),
             Pose(x=2.0, y=0.15, yaw=0.2), Pose(x=4.9, y=0.0, yaw=0.0)]
    base_cmds = [compute_command(ps, p, p.project(ps.x, ps.y), cfg) for ps in poses]

    # 2.1 空黑板 / 显式 None / 缺键 → 逐位相同
    same_empty = all(compute_command(ps, p, p.project(ps.x, ps.y), cfg, bb=Blackboard()) == b
                     for ps, b in zip(poses, base_cmds))
    ok("2.1 ★ 空黑板（键不存在）→ 逐位等于不传限速", same_empty)
    same_none = all(compute_command(ps, p, p.project(ps.x, ps.y), cfg,
                                    bb=bb_with(None)) == b
                    for ps, b in zip(poses, base_cmds))
    ok("2.2 键存在但值为 None → 逐位 no-op", same_none)

    # 2.3 ★ 历史黄金值：由**改动前**的 controller（backup/t20-before 快照）实跑同一批
    #     位姿得到，写死为字面量 —— 这就是"无信号时行为逐位不变"的回归钉，不依赖外部文件。
    golden = [PursuitCommand(linear_x=0.3, steer_deg=0.0, curvature=0.0,
                             speed_reason="cruise", reverse=False),
              PursuitCommand(linear_x=0.3, steer_deg=0.0, curvature=0.0,
                             speed_reason="cruise", reverse=False),
              PursuitCommand(linear_x=0.3, steer_deg=-26.046484966389414,
                             curvature=-3.112976435361941, speed_reason="cruise", reverse=False),
              PursuitCommand(linear_x=0.05, steer_deg=0.0, curvature=0.0,
                             speed_reason="stop_approach", reverse=False)]
    ok("2.3 ★ 与改动前黄金值逐位一致（4 姿态含贴站减速/带偏航）", base_cmds == golden,
       "; ".join(f"{c.linear_x!r}/{c.steer_deg!r}/{c.speed_reason}" for c in base_cmds))

    # 2.4 非法载荷 → 一律 no-op 且 helper 返回 None
    bad_payloads = [1.5, "x", [], {}, {"speed_mps": "x"}, {"speed_mps": None},
                    {"speed_mps": float("nan")}, {"speed_mps": float("inf")},
                    {"speed_mps": float("-inf")}, {"speed_mps": -1.0},
                    {"speed_mps": True}, {"reason": "drift"}]
    bad_ok = True
    bad_detail = ""
    for bad in bad_payloads:
        bb = bb_with(bad)
        helper_none = motion_limit_speed_limit(bb) is None
        same = all(compute_command(ps, p, p.project(ps.x, ps.y), cfg, bb=bb) == b
                   for ps, b in zip(poses, base_cmds))
        if not (helper_none and same):
            bad_ok = False
            bad_detail = f"首个反例 {bad!r}: helper_none={helper_none} same={same}"
            break
    ok("2.4 ★ 12 种非法载荷 → helper 返回 None 且指令逐位 no-op", bad_ok, bad_detail or
       f"{len(bad_payloads)} 种全过")

    ok("2.5 speed_profile(motion_limit_mps=None) == 不传该参",
       all(speed_profile(curvature=c, dist_to_stop=d, corner_dist=k, speed_scale=1.0, cfg=cfg,
                         motion_limit_mps=None)
           == speed_profile(curvature=c, dist_to_stop=d, corner_dist=k, speed_scale=1.0,
                            cfg=cfg)
           for c in (0.0, 0.5, -0.3) for d in (None, 0.1, 1.0) for k in (None, 0.1, 1.0)))

    # ------------------------------------------------------------------ #
    sec("3. 限速生效（只降不升；0.0 = 真停）")
    v0, r0 = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None,
                           speed_scale=1.0, cfg=cfg)
    v1, r1 = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None, speed_scale=1.0,
                           cfg=cfg, motion_limit_mps=0.08)
    ok("3.1 巡航 0.30 → 限速 0.08", abs(v1 - 0.08) < 1e-12 and r1 == "motion_limit",
       f"{v0:.3f} → {v1:.3f}，reason={r1!r}")
    v2, r2 = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None, speed_scale=1.0,
                           cfg=cfg, motion_limit_mps=0.90)
    ok("3.2 限速高于巡航 → 不变（只降不升）", (v2, r2) == (v0, r0),
       f"{v2:.3f}，reason={r2!r}")
    v3, r3 = speed_profile(curvature=0.0, dist_to_stop=None, corner_dist=None, speed_scale=1.0,
                           cfg=cfg, motion_limit_mps=0.0)
    ok("3.3 ★ 限速 0.0 = 真给 0（不被 min_speed 蠕动下限顶回）", v3 == 0.0 and r3 == "motion_limit",
       f"v={v3!r}（min_speed={cfg.get('pursuit.min_speed_mps')}）reason={r3!r}")

    # ------------------------------------------------------------------ #
    sec("4. compute_command 走黑板 + 倒车按绝对值钳")
    ps = Pose(x=2.0, y=0.0, yaw=0.0)
    cmd_bb = compute_command(ps, p, p.project(ps.x, ps.y), cfg, bb=bb_with({"speed_mps": 0.08,
                                                                           "reason": "drift"}))
    cmd_arg = compute_command(ps, p, p.project(ps.x, ps.y), cfg, motion_limit_mps=0.08)
    ok("4.1 走黑板 == 显式传参（同一读侧路径）", cmd_bb == cmd_arg,
       f"bb={cmd_bb.linear_x:.4f}/{cmd_bb.speed_reason} arg={cmd_arg.linear_x:.4f}/{cmd_arg.speed_reason}")
    ok("4.2 巡航 0.30 被钳到 0.08 且 reason=motion_limit",
       abs(cmd_bb.linear_x - 0.08) < 1e-12 and cmd_bb.speed_reason == "motion_limit",
       f"{cmd_bb.linear_x:.4f} m/s / {cmd_bb.speed_reason}")

    prev = straight(reverses=(0,))
    ps_r = Pose(x=2.0, y=0.0, yaw=0.0)
    # 显式 `reverse=True`（与黄金值脚本同一调用式）：段标记与显式参数是"取或"关系，
    # 这里两者都给，避免依赖 `segment_is_reverse` 的索引口径。
    r_no = compute_command(ps_r, prev, prev.project(ps_r.x, ps_r.y), cfg, reverse=True)
    r_lim = compute_command(ps_r, prev, prev.project(ps_r.x, ps_r.y), cfg, reverse=True,
                            motion_limit_mps=0.08)
    ok("4.3 倒车：|v| ≤ 限速 且保号（负）",
       abs(r_lim.linear_x) <= 0.08 + 1e-12 and r_lim.linear_x < 0.0,
       f"无信号 {r_no.linear_x:.4f} → 限速 {r_lim.linear_x:.4f}")
    r_stop = compute_command(ps_r, prev, prev.project(ps_r.x, ps_r.y), cfg, reverse=True,
                             motion_limit_mps=0.0)
    ok("4.4 倒车 + 限速 0.0 → 0.0（正零，不被 max(min_speed,…) 顶回）",
       r_stop.linear_x == 0.0 and str(r_stop.linear_x) == "0.0",
       f"linear_x={r_stop.linear_x!r} reason={r_stop.speed_reason!r}")

    # ------------------------------------------------------------------ #
    sec("5. 配置对账：config/robot.yaml 的 localization: 段 == 现场落盘值（t12 §T12-B）")
    real = Config.load(str(WS / "config" / "robot.yaml"))
    from dzbrain.core.config import SCHEMA
    # ★★ 2026-10-31 重钉（v4.4.0 fc3aba29）：现场值从「快速采集档」回到**常规安全档** ——
    #   `drift_slow_after / drift_stop_after` 999 → 3 / 8（= SCHEMA 默认，漂移降速/停车门禁
    #   重新生效）；`boundary_guard_m` 0.10 → 0.25、`boundary_stop_m` 0.03 → 0.10（边界限速/
    #   停车提前，停车阈值须 > 0.044 滑行距离）。旧断言钉的是 2026-10-11 采集档，
    #   **不能据旧断言倒改现场值**（`config/robot.yaml` 的 `localization:` 段头注释已明示）。
    #   因此本节契约拆成三条，分别钉「现场值」「SCHEMA 区间」「与默认值的偏离清单」：
    #     5.2  现场落盘值（改值必须同时改这里 + 说明理由）
    #     5.2b 每个值都在 SCHEMA 声明的 lo/hi 内（越界=写错，与 5.2 独立）
    #     5.2c 与 SCHEMA 默认值的偏离**恰好**是 boundary_guard_m / boundary_stop_m 两键
    #          （防「顺手改值 + 顺手改 expect」而没人意识到语义变了）
    expect = {
        "localization.anchor_max_pull_m": 0.12,
        "localization.anchor_reject_eps_m": 0.005,
        "localization.anchor_scale_max": 0.08,
        "localization.implied_scale_window": 4,
        "localization.reanchor_pull_max_m": 0.35,
        "localization.drift_slow_after": 3,
        "localization.drift_slow_speed_mps": 0.08,
        "localization.drift_stop_after": 8,
        "localization.drift_clear_ticks": 5,
        "localization.boundary_guard_m": 0.25,
        "localization.boundary_guard_speed_mps": 0.08,
        "localization.boundary_stop_m": 0.10,
    }
    missing = [k for k in expect if real.get(k) is None]
    ok("5.1 12 个键都在 robot.yaml 里", not missing, f"缺 {missing}" if missing else "12/12")
    mismatch = {k: (real.get(k), v) for k, v in expect.items() if real.get(k) != v}
    ok("5.2 ★ 逐条等于现场落盘值", not mismatch, f"{mismatch}" if mismatch else "12/12 一致")
    out_of_range = {k: SCHEMA[k].check(real.get(k)) for k in expect
                    if SCHEMA[k].check(real.get(k)) is not None}
    ok("5.2b 12 个键的值都在 SCHEMA 声明区间内（越界=写错）",
       not out_of_range, f"{out_of_range}" if out_of_range else "12/12 合规")

    def _same(a, b):
        return abs(a - b) < 1e-12 if isinstance(b, float) else a == b

    deviations = {k: (real.get(k), SCHEMA[k].default) for k in expect
                  if not _same(real.get(k), SCHEMA[k].default)}
    ok("5.2c ★ 与 SCHEMA 默认值的偏离 = 仅 boundary_guard_m / boundary_stop_m 两键",
       sorted(deviations) == ["localization.boundary_guard_m", "localization.boundary_stop_m"],
       f"{deviations}")
    # ★ 2026-10-11 重钉（t22）：第三次换图落盘的标定值里 ld_max 0.40 / 62 点路线 /
    #   ticksPerMeter 3280 不变，**巡航 0.30 → 0.35**（用户现场口径）。
    # ★★ t31 重钉（第五张图收口）：**巡航 0.35 → 0.15**（t29 首跑安全档）、
    #   路线 **62 → 60 → 54 点**（官方图 tracker-safe 自规划产物）；ld_max 0.40 与
    #   ticksPerMeter 3280 不变。
    #   ⚠️ 行内保留历史值原文（`ticksPerMeter 3280）` / 旧断言本体 `- 0.30` / `- 0.35`）作为
    #      `tools/calib_backfill.py` 的钉锚点（内容式定位；该映射在 t2 写域，本卡不改）。
    #   ★ t3-A（2026-10-16）：停站退到 `D3`（板心 0.875 m）⇒ 路线回到 **44 点**（t3 一版的
    #     50 点是 `F2*` 绕行产物，随 1.00 档一起作废）。
    # ★★ 2026-10-31 重钉（第七张图 / 布局模式，v4.4.0）：手写路线随旧核删除，`route.tokens`
    #   现为**占位空表**（SCHEMA 默认 `[]`；`planner.auto_plan_enabled=true` ⇒ 路线由无人机
    #   field_layout 运行期规划）；现场路线 68 token 由夹具 `route_field7_t24.txt` 承载
    #   （见 test_route §10 / test_path_overlap §5）。标定三项不变。
    ok("5.3 落盘值（cruise 0.35 / ld_max 0.40 / 布局模式 route.tokens 空表 / ticksPerMeter 3280）"
       "（旧 cruise 0.30 / 0.35）",
       abs(real.get("pursuit.cruise_speed_mps") - 0.35) < 1e-12
       and abs(real.get("pursuit.lookahead_max_m") - 0.40) < 1e-12
       and (real.get("route.tokens") or []) == []
       and abs(real.get("dzactuator.ticksPerMeter") - 3280.0) < 1e-9,
       f"cruise={real.get('pursuit.cruise_speed_mps')} ld_max={real.get('pursuit.lookahead_max_m')} "
       f"tokens={len(real.get('route.tokens') or [])} ticks={real.get('dzactuator.ticksPerMeter')}")
    fixture = WS / "src" / "dzbrain" / "tests" / "fixtures" / "route_field7_t24.txt"
    ftokens = fixture.read_text(encoding="utf-8").split()
    ok("5.3b 现场路线夹具 route_field7_t24.txt = 68 token（第七图，与删除前 robot.yaml 逐字一致）",
       len(ftokens) == 68, f"{len(ftokens)} 个")

    # ------------------------------------------------------------------ #
    print()
    print("=" * 72)
    print(f"结果：{_PASS}/{_PASS + _FAIL} 通过" + ("" if not _FAIL else f"，{_FAIL} 失败"))
    if _FAIL:
        print("失败项：" + "; ".join(_FAILED))
        return 1
    print("全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
