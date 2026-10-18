#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
路线全链路校验   route_check.py
================================================================================
一句话：把「规划器输出 → 路线下发 → 几何校验」这条链从头到尾跑一遍，
        在【上车之前】把会撞挡板、会踩障碍、会漏掉雷、会卡死的路线拦下来。

--------------------------------------------------------------------------------
为什么需要它
--------------------------------------------------------------------------------
改一次路线要经过三套代码、四道转换，任何一环出错都要到车上才发现：

    global_planner.py            算出访问顺序与阿克曼路径
            |  （输出纯文本路线串）
            v
    config/robot.yaml            人工填进 mission_blocks
            |
            v
    path_executor 校验器         语法层：token 合不合法
            |
            v
    path_executor 解析器         解析层：转成航点 + 三个标记数组
            |                    （shoot / scan / reverse）
            v
    实际行驶                     几何层：真的能开过去吗？

本工具把后三步全部自动化，并且**直接复用 path_executor 里的校验与解析函数**
（不是重写一份 —— 重写就会两处逻辑各说各话，那正是要避免的）。

--------------------------------------------------------------------------------
五层检查（⑥ 折返层为新增，共六层）
--------------------------------------------------------------------------------
    ① 语法层    token 合法性（复用 path_executor.validate_mission_blocks）
    ② 解析层    模拟 follow_route_command，产出航点与标记数组
    ③ 几何层    挡板穿越列/余量、障碍最近距离、地图边界、转弯角度
    ④ 任务层    雷/靶/桥洞覆盖、射击点距离是否在 0.20~0.45m
    ⑤ 安全层    越界余量、桥洞保护点、倒车段长度
    ⑥ 折返层    `:PIVOT` 声明与实际折返几何是否一致（虚报 = ❌，漏报 = ⚠️）

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
    # 校验配置中心里当前的路线
    ./route_check.py

    # 校验一条手工路线
    ./route_check.py --route "M10 K7 D3* R:K9 N5"

    # 一条命令跑完「规划器 -> 校验」全链路
    ./route_check.py --from-planner \
        --start M10 --end N5 \
        --mines J9 D10 B9 E7 C5 A1 F4 J5 \
        --obstacles B2 D3 A5 G6 E9 B10 H10 F11 \
        --targets F1 G1 --bridges K7 K1

退出码：0 = 全部通过（可能有警告）  1 = 存在❌致命问题  2 = 参数/文件问题
================================================================================
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import os
import subprocess
import sys
from typing import List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)

# =============================================================================
#  地图常量 —— ★ 唯一真源 = `src/dzbrain/dzbrain/core/geometry.py`
# =============================================================================
# 不再在本文件抄数字（四份副本漂移曾让"规划器 14 列 / 校验器另一套"成为隐患）：
# `_field_const` 从 `__file__` 推出 `src/dzbrain` 插进 sys.path，再从 `core.geometry`
# 取常量。因此 `python tools/route_check.py` 仍可**直接跑**：不设 PYTHONPATH、
# 不 pip 装 dzbrain 包（只要求仓库源码树在场）。
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from _field_const import (                                    # noqa: E402
    BARRIER_ROW_HI, BARRIER_ROW_LO,
    BLOCK_SIZE, COL_LETTERS, MAP_COLS, MAP_ROWS,
    MAP_X_MAX, MAP_X_MIN, MAP_Y_MAX, MAP_Y_MIN,
)

# 判定阈值
BARRIER_MIN_MARGIN = 0.10        # 过挡板缺口时离挡板边缘的最小余量(m)
OBSTACLE_WARN_M = 0.30           # 航段离障碍的告警距离(m)

#: ★★物理几何（用户 2026-10-11 现场口径）—— 与 `config/robot.yaml` 的
#: `vehicle.*` / `field.*` / `planner.obstacle_margin_m` 逐项对应，`load_cfg()` 会用配置值
#: **覆盖**这里的默认值（默认值与 `core/config.py` SCHEMA 一致）。
#: 公式与 `dzbrain/route/physics.py` 是**同一套**——本工具**不复用那份实现**（物理判据要能
#: 脱离 dzbrain 包独立演进），所以由 `src/dzbrain/tests/test_physics.py` 的**对拍断言**钉住
#: 两份实现不许分叉。★ 场地常量（格边/行列数/挡板行列）**不在此列**：它们已统一由
#: `core/geometry.py` 单点供源（见上方 `_field_const` import）。
VEHICLE_LENGTH_M = 0.28          # 车体长
VEHICLE_WIDTH_M = 0.135          # 车体宽（半宽 0.0675 进挡板/障碍判据）
COVERAGE_RADIUS_M = 0.035        # 车心假设圆半径（压雷判据）
OBSTACLE_RADIUS_M = 0.1275       # 障碍圆柱半径（柱心在格心）
MINE_PAD_M = 0.12                # 雷垫片边长（0.12×0.12 正方形）
BARRIER_WIDTH_M = 0.02           # 挡板厚度（2 cm 细条；半厚 0.01 进触线判据）
BARRIER_COLS = "D..N"            # 挡板覆盖的列（含两端 = 11 列 = 3.85 m）
OBSTACLE_MARGIN_M = 0.02         # 障碍净空余量
SHOOT_DIST_MIN = 0.20            # 射击点距离下限(m)，会被配置中心覆盖
SHOOT_DIST_MAX = 0.45            # 射击点距离上限(m)，会被配置中心覆盖
SHARP_CORNER_DEG = 45.0          # 超过此角度算"急转弯"（提示级；★ PIVOT 的真正门限见下行）
# ★ PIVOT 真正触发门限（度）：循迹层用 `pursuit.path.corner_turn_deg`（**车头**要转的角度）
#   与 `pursuit.pivot_angle_deg`（`core/config.py` 默认 135°、robot.yaml 未覆盖）比较。
#   本工具里它只用来校验 `:PIVOT` 声明的一致性（第 ⑥ 层），
#   **不改变** `pursuit/**` 与 `tasks/tracker.py` 的任何行为。
PIVOT_DECL_DEG = 135.0

MAP_SAFETY_MARGIN = 0.60

FAIL, WARN, OK, INFO = [], [], [], []


def ok(msg, detail=''):
    OK.append(msg)
    print("  \033[32m✅\033[0m %-30s %s" % (msg, detail))


def warn(msg, detail=''):
    WARN.append(msg)
    print("  \033[33m⚠️ \033[0m %-30s %s" % (msg, detail))


def bad(msg, detail=''):
    FAIL.append(msg)
    print("  \033[31m❌\033[0m %-30s %s" % (msg, detail))


def info(msg, detail=''):
    INFO.append(msg)
    print("     %-30s %s" % (msg, detail))


def sec(name):
    print("\n\033[1m【%s】\033[0m" % name)


# =============================================================================
#  坐标工具
# =============================================================================
def token_block(tok) -> str:
    """token -> **纯格名**（剥掉 `R:` / `SHOOT@` 前缀与 `*` / `:PIVOT` / `,朝向` 后缀）。

    ★ 与 `block_to_meters()` 的清洗规则**同一套**（这里抽出来只为复用，不另立口径）：
      第 ④ 任务层的 `visited`、以及「航点压靶板」判据都走这一个函数。
    """
    s = str(tok).strip().upper()
    for p in ('R:', 'SHOOT@'):
        if s.startswith(p):
            s = s[len(p):]
    if s.endswith('*'):
        s = s[:-1]
    if ':' in s:
        s = s.split(':', 1)[0]
    if ',' in s:
        s = s.split(',', 1)[0]
    return s.strip()


def block_to_meters(name: str) -> Tuple[float, float]:
    s = token_block(name)
    if len(s) < 2 or s[0] not in COL_LETTERS or not s[1:].isdigit():
        raise ValueError("非法坐标: %r" % name)
    row = int(s[1:])
    col = COL_LETTERS.index(s[0])
    return (10.5 - (row - 1)) * BLOCK_SIZE, (13.5 - col) * BLOCK_SIZE


def parse_block(name: str) -> Tuple[int, int]:
    x, y = block_to_meters(name)
    return int(round(10.5 - x / BLOCK_SIZE)) + 1, int(round(13.5 - y / BLOCK_SIZE))


def zone_of(row: int) -> int:
    return 0 if row <= BARRIER_ROW_HI else 1


# =============================================================================
#  复用 path_executor 的校验/解析逻辑（不重写，避免两处逻辑分叉）
# =============================================================================
def load_path_executor():
    p = os.path.join(WS, 'src', 'car_localization', 'car_localization',
                     'path_executor.py')
    if not os.path.exists(p):
        return None
    try:
        spec = importlib.util.spec_from_file_location("pe_for_check", p)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception as exc:
        print("  \033[33m⚠️ \033[0m 无法导入 path_executor（%s），改用内置副本校验"
              % type(exc).__name__)
        return None


def fallback_validate(tokens):
    """path_executor 不可用时的内置等价校验（保持同样的规则）。"""
    problems, n_rev, n_shoot, n_norm = [], 0, 0, 0
    n_pivot = 0
    for i, tok in enumerate(tokens):
        t = str(tok).strip()
        if not t:
            problems.append("#%d 空 token" % i)
            continue
        if t.upper().startswith('SHOOT@'):
            tgt = t[6:].strip().upper()
            if len(tgt) >= 2 and tgt[0] in COL_LETTERS and tgt[1:].isdigit():
                n_shoot += 1
            else:
                problems.append("#%d '%s' 靶标名非法" % (i, t))
            continue
        body = t
        if ',' in body and i != len(tokens) - 1:
            problems.append("#%d '%s' 朝向后缀不在末尾" % (i, t))
        if ',' in body:
            body = body.rsplit(',', 1)[0].strip()
        if body.upper().startswith('R:'):
            body = body[2:].strip()
            n_rev += 1
        if ':' in body:
            h, tl = body.split(':', 1)
            tu = tl.strip().upper()
            if tu in ('SHOOT_STOP', 'SHOOT'):
                body, n_shoot = h.strip(), n_shoot + 1
            elif tu in ('SCAN_STOP', 'SCAN'):
                body = h.strip()
            elif tu == 'PIVOT':
                # ★ 新增后缀：折返点**声明**（该顶点车头需转过 ≥ PIVOT_DECL_DEG）。
                #   不产生停站、不改变循迹层行为，第 ⑥ 层再校验它与几何是否一致。
                body, n_pivot = h.strip(), n_pivot + 1
        if body.endswith('*'):
            body, n_shoot = body[:-1].strip(), n_shoot + 1
        bu = body.upper()
        if len(bu) < 2 or bu[0] not in COL_LETTERS or not bu[1:].isdigit():
            problems.append("#%d '%s' 不是合法坐标" % (i, t))
        elif not (1 <= int(bu[1:]) <= MAP_ROWS):
            problems.append("#%d '%s' 行号越界" % (i, t))
        else:
            n_norm += 1
    return (not problems), problems, {'total': len(tokens), 'reverse': n_rev,
                                      'shoot': n_shoot, 'normal': n_norm,
                                      'pivot': n_pivot}


def parse_route(tokens):
    """模拟 follow_route_command 的 token 循环，产出 (航点, shoot, scan, reverse, pivot)。

    这一步刻意与 path_executor 的解析顺序保持完全一致：
        R: 前缀  ->  :LABEL 标签  ->  * 后缀  ->  ,朝向

    ★ `pivot`（第 5 个返回值，新增）与 `pts` **逐点对齐**：
      `:PIVOT` 只是声明，不产生新航点、不影响去重（同一个格 + 同样的 reverse 才合并）。
    """
    pts, shoot_f, scan_f, rev_f, pivot_f = [], [], [], [], []
    for tok in tokens:
        t = str(tok).strip()
        if not t:
            continue
        rev = False
        if t.upper().startswith('R:'):
            rev = True
            t = t[2:].strip()
        pivot = False
        if ':' in t:
            head, tail = t.split(':', 1)
            tu = tail.strip().upper()
            if tu in ('SHOOT_STOP', 'SHOOT'):
                t = head.strip() + '*'
            elif tu in ('SCAN_STOP', 'SCAN'):
                t = head.strip()
            elif tu == 'PIVOT':
                t, pivot = head.strip(), True
        shoot = t.endswith('*')
        b = (t[:-1] if shoot else t).upper()
        if ',' in b:
            b = b.rsplit(',', 1)[0]
        if len(b) < 2 or b[0] not in COL_LETTERS or not b[1:].isdigit():
            continue
        px, py = block_to_meters(b)
        if pts and abs(pts[-1][0] - px) < 1e-9 and abs(pts[-1][1] - py) < 1e-9 \
                and rev_f[-1] == rev:
            if shoot:
                shoot_f[-1] = True
            pivot_f[-1] = pivot_f[-1] or pivot
            continue
        pts.append((px, py))
        shoot_f.append(shoot)
        scan_f.append(True)
        rev_f.append(rev)
        pivot_f.append(pivot)
    return pts, shoot_f, scan_f, rev_f, pivot_f


# =============================================================================
#  几何计算
# =============================================================================
def barrier_crossings(pts):
    """找出所有跨越挡板（6<->7 行之间的 x 分界）的航段。

    挡板在"行 6 与行 7 之间"，换算成米制就是 x 的一个分界面：
        x(行6) = 5.5*BLOCK, x(行7) = 4.5*BLOCK  ->  分界 x = 5.0*BLOCK
    车从半场到半场必然穿过这条线，检查穿越点的 y（即所在列）。
    """
    BX = 5.0 * BLOCK_SIZE
    GAP_Y = 13.5 * BLOCK_SIZE - 3 * BLOCK_SIZE   # C 列右缘 = y 上限（含 C 列）
    out = []
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        if (x1 - BX) * (x2 - BX) < 0:
            t = (BX - x1) / (x2 - x1)
            yc = y1 + t * (y2 - y1)
            col_idx = int(round(13.5 - yc / BLOCK_SIZE))
            col_idx = max(0, min(MAP_COLS - 1, col_idx))
            out.append({
                'seg': i, 'y': yc, 'col': COL_LETTERS[col_idx],
                'col_idx': col_idx,
                'margin': yc - GAP_Y,      # 离挡板西缘(缺口边)的距离
                'ok': yc > GAP_Y,
            })
    return out


def seg_obstacle_min(pts, obstacles):
    """所有航段到所有障碍的最近距离。"""
    worst = (9e9, None)
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        for ob in obstacles:
            try:
                ox, oy = block_to_meters(ob)
            except ValueError:
                continue
            if L2 > 1e-12:
                t = max(0.0, min(1.0, ((ox - x1) * dx + (oy - y1) * dy) / L2))
            else:
                t = 0.0
            d = math.hypot(ox - (x1 + t * dx), oy - (y1 + t * dy))
            if d < worst[0]:
                worst = (d, (i, ob))
    return worst




#: 「进洞角」上限（度）。真源是 `src/dzbrain/dzbrain/route/validate.py` 的
#: `MAX_CROSS_ANGLE_DEG`（R009）—— 本工具**不 import `route.validate`**（那是校验规则的实现，
#: 与本工具的六层判据各管一段），所以在这里复写一份常量，值必须与真源一致。
#: （场地几何常量不在此列：已由 `core/geometry.py` 单点供源，见文件头 `_field_const`。）
R009_MAX_ANGLE_DEG = 10.0

#: ★「车心压过靶板」的几何判据半径（米）= **半格**（0.35/2 = 0.175）。
#: 语义：`route_covers(pts, cell, tol)` 算的是"路线折线到**格心**的最短距离"，
#: 取半格即"车心进到格心 0.175 m 以内" ⇒ 必然压在这块 0.35 m 见方的靶板上
#: （= 该格的**内切圆**口径，偏保守：擦角不入圆的极端划角可能漏判，但**不会误报**）。
#: 与「雷覆盖」同一套算法，只是 tol 从 `BLOCK_SIZE*0.6` 收到半格。
TARGET_RUNOVER_TOL_M = BLOCK_SIZE / 2.0


def _bridge_group_check(bridges, pts):
    """桥洞穿越检查（支持"整列都是桥洞"）。

    ★ 三层口径（2026-10-17 硬化，与 dzbrain.route.validate R007/R008/R009 同口径）：

    ① **水平直穿**（段严格同行 `x1 == x2`）：唯一合法形态。**把连续相接的穿越段并成
       一次"过洞"**（`L9->K9` 与 `K9->J9` 是同一次穿越的两段，旧版算成 2 段计数翻倍）；
       **不按 map.bridges 行区间过滤**（区间外的穿越正该报错，滤掉就再也看不见）；
       下半场（行 7~11）恰 1 次且方向**向西**（列号减小 `L*->K*->J*`，= y 增大），
       上半场（行 1~6）恰 1 次且方向**向东**（列号增大 `J*->K*->L*`，= y 减小）。
    ② **斜切进洞**（段与桥洞列**相交**、但行号在变，例如 `L8 -> K7`）：进洞角
       `atan2(|Δ列|, |Δ行|)`（口径与 R009 相同），> `R009_MAX_ANGLE_DEG` 即"斜切" ——
       桥洞只有单格宽，斜着切进去**会蹭侧壁**；
    ③ **列内行驶**（`y` 恒等于桥洞列、`x` 在变，例如 `K9 -> K10`）：格与格之间有侧壁，
       在洞里沿列挪格同样必撞（与 R009 的「列内」判据同义）。

    ★ 判级：① 的次数/方向违规报 ❌（改退出码）；② ③ 只报 ⚠️ 警告（硬拦由真源
      `dzbrain.route.validate` 的 R009 负责，本工具不抢判决权）。
      —— 硬化背景：实测旧版对「区间外穿越」「只穿 1 次」rc=0 假绿
        （`_tmp_session/t7_review/p14_bridge_count.py`），而 dzbrain 复合门禁能拦。
    """
    if not bridges:
        info("桥洞覆盖", "未提供，跳过")
        return
    cols = sorted({str(b)[0].upper() for b in bridges})
    if not cols:
        info("桥洞覆盖", "未提供，跳过")
        return
    tol = BLOCK_SIZE * 0.45
    tol_plane = 1e-6                       # "正好落在桥洞列上"的容差（同一表达式算出的坐标是逐位相等的）
    runs = []                              # 每次物理过洞：{row, dy(+1 向西 / -1 向东), segs:[段号], index, end}
    diag = []                              # ② 斜切进洞：(段号, 行号, 角度)
    along = []                             # ③ 列内行驶：(段号, 起行, 止行)
    for col in cols:
        cx = (13.5 - COL_LETTERS.index(col)) * BLOCK_SIZE
        prev = None
        for i in range(len(pts) - 1):
            (x1, y1), (x2, y2) = pts[i], pts[i + 1]
            # ★注意坐标轴：x 对应"行"，y 对应"列"。
            #   穿 K 列 = 在同一行内跨越 y 方向 -> 应筛"x 不变"的段。
            if abs(x1 - x2) <= 1e-6:                       # ① 水平直穿
                if min(y1, y2) - tol <= cx <= max(y1, y2) + tol:
                    r = round(10.5 - x1 / BLOCK_SIZE) + 1
                    dy = 1 if y2 > y1 else -1              # y 增大 = 列号减小 = 向西
                    # 与前一段**相接且同行同向** → 合并成同一次过洞（L->K 与 K->J 并 1 次）
                    if (prev is not None and prev["row"] == r and prev["dy"] == dy
                            and prev["end"] == (x1, y1)):
                        prev["segs"].append(i)
                        prev["end"] = (x2, y2)
                    else:
                        run = {"row": r, "dy": dy, "segs": [i], "index": i, "end": (x2, y2)}
                        runs.append(run)
                        prev = run
                else:
                    prev = None
                continue
            prev = None
            # ②/③ 的入口判据：这一段到底有没有"碰到桥洞列"（含正好落在列上）
            if not (min(y1, y2) - tol_plane <= cx <= max(y1, y2) + tol_plane):
                continue
            if abs(y1 - y2) <= tol_plane:                  # ③ 整段都躺在桥洞列里
                r1 = round(10.5 - x1 / BLOCK_SIZE) + 1
                r2 = round(10.5 - x2 / BLOCK_SIZE) + 1
                along.append("段#%d(行%d->%d)" % (i, r1, r2))
                continue
            t = (cx - y1) / (y2 - y1)                      # ② 与桥洞列的交点
            xc = x1 + t * (x2 - x1)
            r = round(10.5 - xc / BLOCK_SIZE) + 1
            ang = math.degrees(math.atan2(abs(y2 - y1), abs(x2 - x1)))
            diag.append((i, r, ang))

    lower = [r for r in runs if 7 <= r["row"] <= 11]
    upper = [r for r in runs if 1 <= r["row"] <= 6]
    other = [r for r in runs if r["row"] < 1 or r["row"] > 11]

    def _fmt(runs, sym):
        return " ".join("段#%d(行%d%s)" % (r["index"], r["row"], sym) for r in runs[:4])

    n_ok = 0
    if len(lower) != 1:
        bad("下半场桥洞穿越 %d 次" % len(lower),
            "行 7~11 应恰 1 次（%s）" % (_fmt(lower, "") or "无"))
    elif lower[0]["dy"] <= 0:
        bad("下半场桥洞穿越方向错", "必须向西（L* -> K* -> J*，列号减小）")
    else:
        n_ok += 1
    if len(upper) != 1:
        bad("上半场桥洞穿越 %d 次" % len(upper),
            "行 1~6 应恰 1 次（%s）" % (_fmt(upper, "") or "无"))
    elif upper[0]["dy"] >= 0:
        bad("上半场桥洞穿越方向错", "必须向东（J* -> K* -> L*，列号增大）")
    else:
        n_ok += 1
    if other:
        bad("桥洞穿越行号异常 %d 次" % len(other),
            "%s 列  %s" % ("/".join(cols), _fmt(other, "")))
    if n_ok == 2 and not other:
        ok("桥洞穿越（下半场向西 / 上半场向东 各 1 次）",
           "%s 列  下%s · 上%s"
           % ("/".join(cols),
              _fmt(lower, "向西"), _fmt(upper, "向东")))

    steep = [d for d in diag if d[2] > R009_MAX_ANGLE_DEG + 1e-9]
    if steep:
        warn("桥洞斜切进洞 %d 段" % len(steep),
             "%s 列  %s（与穿洞方向夹角 > %.0f°，R009 级 —— 会蹭侧壁）"
             % ("/".join(cols),
                " ".join("段#%d(行%d)%.1f°" % d for d in steep[:4]),
                R009_MAX_ANGLE_DEG))
    elif diag:
        info("桥洞轻微斜切 %d 段" % len(diag),
             "%s 列  %s（≤ %.0f°，R009 放行）"
             % ("/".join(cols), " ".join("段#%d(行%d)%.1f°" % d for d in diag[:4]),
                R009_MAX_ANGLE_DEG))
    if along:
        warn("桥洞列内行驶 %d 段" % len(along),
             "%s 列  %s（格与格之间有侧壁，必须严格直穿）"
             % ("/".join(cols), " ".join(along[:4])))


def _is_rev_vertex(i, rev_f):
    """航点 i 的前后两段里只要有一段是倒车，就不算"转弯"。"""
    a = (i + 1 < len(rev_f)) and rev_f[i + 1]      # 进入段
    b = (i < len(rev_f)) and rev_f[i]              # 离开段
    return a or b


def turn_angles(pts):
    """每个中间航点的转折角(度)。>45° 会触发 PIVOT 原地转向。"""
    out = []
    for i in range(1, len(pts) - 1):
        d1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        d2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        n1 = math.hypot(*d1)
        n2 = math.hypot(*d2)
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        cosv = max(-1.0, min(1.0, (d1[0] * d2[0] + d1[1] * d2[1]) / (n1 * n2)))
        out.append((i, math.degrees(math.acos(cosv))))
    return out


def seg_min_dist_to_cell(pts, cell_xy):
    """路线所有航段到某个格中心的最短距离(m)。"""
    cx, cy = cell_xy
    best = 9e9
    for i in range(len(pts) - 1):
        (x1, y1), (x2, y2) = pts[i], pts[i + 1]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        if L2 > 1e-12:
            t = max(0.0, min(1.0, ((cx - x1) * dx + (cy - y1) * dy) / L2))
        else:
            t = 0.0
        d = math.hypot(cx - (x1 + t * dx), cy - (y1 + t * dy))
        if d < best:
            best = d
    return best


def route_covers(pts, cell_name, tol_m=BLOCK_SIZE * 0.6):
    """路线是否【几何上经过】该格。

    ★ 为什么不能只数 token：
      规划器输出的航点是被压缩过的（只保留拐点），直线段中间的格子
      （比如 K7->D7 之间的 E7）不会出现在 token 列表里 ——
      但车确确实实从它上面压过去了，雷就算踩到了。
      早期版本只查 token，把这类"穿过但没写"的点全判成"遗漏"，
      是纯粹的误报。
    """
    try:
        x, y = block_to_meters(cell_name)
    except ValueError:
        return False, 9e9
    d = seg_min_dist_to_cell(pts, (x, y))
    return d <= tol_m, d


def path_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


# =============================================================================
#  ★物理几何判据—— 公式与 `dzbrain/route/physics.py` 同一套（对拍见 test_physics.py）
# =============================================================================
def _phys_point_seg_dist(px, py, ax, ay, bx, by):
    """点 P 到线段 AB 的最近距离。"""
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-18:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _phys_point_rect_dist(px, py, cx, cy, half):
    """点到轴对齐矩形（心 (cx,cy)、半边长 half）的最近距离；在矩形内 = 0。

    ★ 压雷必须用**点到矩形**的距离：垫片是 0.12 m 见方的方块，"贴边压过"时
      车心离雷心可以到 0.06+0.035 = 0.095 m 仍算压到（差一个垫片半宽 0.06 m 量级）。
    """
    ox, oy = abs(px - cx) - half, abs(py - cy) - half
    if ox <= 0.0 and oy <= 0.0:
        return 0.0
    return math.hypot(max(ox, 0.0), max(oy, 0.0))


def _phys_seg_rect_intersects(ax, ay, bx, by, cx, cy, half):
    """线段与轴对齐矩形是否相交（含相切）—— Liang–Barsky 平板裁剪。"""
    lo_x, hi_x = cx - half, cx + half
    lo_y, hi_y = cy - half, cy + half
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, ax - lo_x), (dx, hi_x - ax), (-dy, ay - lo_y), (dy, hi_y - ay)):
        if abs(p) <= 1e-18:
            if q < 0.0:
                return False
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return False
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return False
            if r < t1:
                t1 = r
    return t0 <= t1


def _phys_seg_rect_dist(ax, ay, bx, by, cx, cy, half):
    """线段到轴对齐矩形的最近距离（相交 = 0）。

    两步：① 平板裁剪判相交（少了它，"从矩形中间穿过去"会被算成"离角还有 half 那么远"）；
          ② 不相交时取 `min(4 个角到线段, 两端点到矩形)` —— 凸集最近点对必在某一方的顶点上。
    """
    if _phys_seg_rect_intersects(ax, ay, bx, by, cx, cy, half):
        return 0.0
    corners = ((cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half))
    return min(min(_phys_point_seg_dist(qx, qy, ax, ay, bx, by) for qx, qy in corners),
               _phys_point_rect_dist(ax, ay, cx, cy, half),
               _phys_point_rect_dist(bx, by, cx, cy, half))


def _phys_seg_seg_dist(ax, ay, bx, by, cx, cy, dx, dy):
    """两条线段的最近距离（相交 = 0）。"""
    def _cross(ox, oy, px, py, qx, qy):
        return (px - ox) * (qy - oy) - (py - oy) * (qx - ox)
    d1 = _cross(cx, cy, dx, dy, ax, ay)
    d2 = _cross(cx, cy, dx, dy, bx, by)
    d3 = _cross(ax, ay, bx, by, cx, cy)
    d4 = _cross(ax, ay, bx, by, dx, dy)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return 0.0
    return min(_phys_point_seg_dist(ax, ay, cx, cy, dx, dy),
               _phys_point_seg_dist(bx, by, cx, cy, dx, dy),
               _phys_point_seg_dist(cx, cy, ax, ay, bx, by),
               _phys_point_seg_dist(dx, dy, ax, ay, bx, by))


def vehicle_half_diagonal_m(length_m=None, width_m=None):
    """车体半对角线（= 拐角扫掠半径）= ``√((L/2)² + (W/2)²)``（现场 0.15542 m）。"""
    L = VEHICLE_LENGTH_M if length_m is None else length_m
    W = VEHICLE_WIDTH_M if width_m is None else width_m
    return math.hypot(L / 2.0, W / 2.0)


def bridge_column(bridges, default="K"):
    """从 `bridges` 列表推出桥洞所在**列字母**（现场 K1/K11 → `"K"`）；空则用 default。

    ★ 只取第一个字母：本工具的桥洞豁免/统计都按"一个桥洞列"建模（现场就是 K 列），
      多列时按字母序取首个，并在调用处如实打印，避免"没豁免"或"乱豁免"。
    """
    letters = sorted({str(b).strip().upper()[:1] for b in (bridges or []) if str(b).strip()})
    return letters[0] if letters else default


def mine_clearance_by_route(pts, mine_block):
    """路线（折线）到**雷垫片**（`MINE_PAD_M` 见方）的最近距离；格名非法返回 9e9。

    ⇒ **压雷判据** = 该值 ≤ `COVERAGE_RADIUS_M`（车心 3.5 cm 圆与垫片产生过重叠）。
    """
    try:
        mx, my = block_to_meters(mine_block)
    except ValueError:
        return 9e9
    half = MINE_PAD_M / 2.0
    return min((_phys_seg_rect_dist(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1],
                                    mx, my, half)
                for i in range(len(pts) - 1)), default=9e9)


def obstacle_clearances(pts, obstacles, rev_f=None):
    """障碍净空：``(直段最小距离, 段号, 障碍名, 拐角最小距离, 航点号, 障碍名)``。

    * **直段口径**：折线到柱心 ≥ `柱半径 + 车半宽 + margin` = 0.1275+0.0675+0.02 = **0.215 m**；
    * **拐角口径**：转向顶点到柱心 ≥ `柱半径 + 扫掠半径 + margin` = 0.1275+0.1554+0.02 = **0.3029 m**
      （车绕顶点转向时车身外包圆 = 半对角线；用"车半宽"会低估 0.088 m）。
      倒车顶点（`_is_rev_vertex`）不算转向 —— 那只是"开到死格再退出来"，车没掉头。
    """
    d_seg, seg_i, seg_name = 9e9, -1, ""
    for i in range(len(pts) - 1):
        for ob in obstacles:
            try:
                ox, oy = block_to_meters(ob)
            except ValueError:
                continue
            d = _phys_point_seg_dist(ox, oy, pts[i][0], pts[i][1],
                                     pts[i + 1][0], pts[i + 1][1])
            if d < d_seg:
                d_seg, seg_i, seg_name = d, i, ob
    d_turn, turn_i, turn_name = 9e9, -1, ""
    for i, ang in turn_angles(pts):
        if ang <= 10.0:                                  # 基本直线 ⇒ 车身横向占位只有半宽
            continue
        if rev_f is not None and _is_rev_vertex(i, rev_f):
            continue
        for ob in obstacles:
            try:
                ox, oy = block_to_meters(ob)
            except ValueError:
                continue
            d = math.hypot(pts[i][0] - ox, pts[i][1] - oy)
            if d < d_turn:
                d_turn, turn_i, turn_name = d, i, ob
    return d_seg, seg_i, seg_name, d_turn, turn_i, turn_name


def barrier_centerline():
    """挡板**中心线** ``(x, y0, y1)``：第 6/7 行中点、`BARRIER_COLS` 覆盖列的外缘。

    现场 = `(1.75, 0.0, 3.85)`（长 3.85 m = 11 列 × 0.35 m）。
    """
    x_hi = (10.5 - (BARRIER_ROW_HI - 1)) * BLOCK_SIZE
    x_lo = (10.5 - (BARRIER_ROW_LO - 1)) * BLOCK_SIZE
    spec = str(BARRIER_COLS).upper().replace("..", "-").replace("—", "-")
    a, _, b = spec.partition("-")
    a = (a.strip() or "D")[0]
    b = (b.strip() or a)[0]
    c0, c1 = COL_LETTERS.index(a), COL_LETTERS.index(b)
    y_a = (13.5 - c0) * BLOCK_SIZE + BLOCK_SIZE / 2.0
    y_b = (13.5 - c1) * BLOCK_SIZE - BLOCK_SIZE / 2.0
    return (x_hi + x_lo) / 2.0, min(y_a, y_b), max(y_a, y_b)


def seg_outside_band(ax, ay, bx, by, band_y, band_half):
    """把线段抠掉"落在 ``|y − band_y| ≤ band_half`` 带内"的部分，返回剩余子线段。

    ★ 用途：**桥洞豁免** —— 挡板在 K 列有通口，带内穿行合法；带外照样不许贴线。
      必须精确抠段（"一端点在带内就整段豁免"会把"斜着蹭过洞口外侧"放过去）。
    """
    out = []
    lo, hi = band_y - band_half, band_y + band_half
    dy = by - ay
    if abs(dy) <= 1e-15:
        if ay < lo - 1e-12 or ay > hi + 1e-12:
            out.append((ax, ay, bx, by))
        return out
    ta, tb = (lo - ay) / dy, (hi - ay) / dy
    c0, c1 = max(0.0, min(ta, tb)), min(1.0, max(ta, tb))
    if c0 >= c1:
        out.append((ax, ay, bx, by))
        return out
    for (u, v) in ((0.0, c0), (c1, 1.0)):
        if v - u > 1e-12:
            out.append((ax + u * (bx - ax), ay + u * dy,
                        ax + v * (bx - ax), ay + v * dy))
    return out


def barrier_touch_min(pts, bridge_col="K"):
    """车心到挡板**中心线**的最近距离，返回 ``(距离, 段号)``。

    ★ 判据：该值 ≥ `挡板半厚 + 车半宽` = 0.01 + 0.0675 = **0.0775 m**（2 cm 细条口径），
      **不得跨越**。
    ★★ 2026-10-11 用户现场澄清（以澄清为准）：挡板是 385 cm × 2 cm 细条、第 6/7 行之间、
      **D..N 列（含 K 段）** ⇒ 挡板在 K 列处**没有通口**（"通格 K1/K11 横向贯通"说的是
      K 列**格与格之间**的纵向封死关系，不是挡板上的洞）⇒ 本判据**不做桥洞豁免**。
      （`seg_outside_band()` 作为通用工具保留，本判据不再调用。）
    :param bridge_col: 仅为兼容旧签名保留；判据本身已与桥洞无关。
    """
    x, y0, y1 = barrier_centerline()
    best, best_i = 9e9, -1
    for i in range(len(pts) - 1):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        d = _phys_seg_seg_dist(ax, ay, bx, by, x, y0, x, y1)
        if d < best:
            best, best_i = d, i
    return best, best_i


# =============================================================================
#  主流程
# =============================================================================
def run_check(route: str, cfg: dict) -> int:
    tokens = route.split()
    obstacles = cfg.get('obstacles') or []
    mines = cfg.get('mines') or []
    targets = cfg.get('targets') or []
    bridges = cfg.get('bridges') or []
    covered = {str(x).strip().upper() for x in (cfg.get('already_covered') or [])}

    print("=" * 78)
    print("  路线全链路校验   planner -> dispatch -> geometry")
    print("=" * 78)
    if cfg.get('map_name'):
        print("  地图: %s   (%s)" % (cfg['map_name'], cfg.get('map_file', '')))
    print("  路线: %s" % route)
    print("  共 %d 个 token" % len(tokens))

    # ---------------- ① 语法层 ----------------
    sec("1/6 语法层")
    pe = load_path_executor()
    if pe is not None and hasattr(pe, 'normalize_mission_token'):
        toks = [x for x in (pe.normalize_mission_token(t) for t in tokens) if x]
        valid, problems, stats = pe.validate_mission_blocks(toks)
        info("校验来源", "path_executor.validate_mission_blocks（权威实现）")
    else:
        toks = tokens
        valid, problems, stats = fallback_validate(tokens)
        info("校验来源", "内置等价副本")

    if valid:
        ok("全部 token 合法", "普通 %d | 倒车 %d | 射击 %d | 折返 %d"
           % (stats['normal'], stats['reverse'], stats['shoot'], stats.get('pivot', 0)))
    else:
        bad("%d 处 token 非法" % len(problems))
        for p in problems[:8]:
            print("        " + p)

    # ---------------- ② 解析层 ----------------
    sec("2/6 解析层")
    pts, shoot_f, _, rev_f, pivot_f = parse_route(toks)
    if len(pts) < 2:
        bad("解析出的航点不足 2 个，无法构成路线")
        return summarise()
    ok("解析出 %d 个航点" % len(pts))
    info("射击点索引", str([i for i, f in enumerate(shoot_f) if f]) or "(无)")
    info("倒车段索引", str([i for i, f in enumerate(rev_f) if f]) or "(无)")
    info("折返点索引", str([i for i, f in enumerate(pivot_f) if f]) or "(无)")

    # ---------------- ③ 几何层 ----------------
    sec("3/6 几何层")

    # 边界
    oob = [(i, x, y) for i, (x, y) in enumerate(pts)
           if not (MAP_X_MIN - MAP_SAFETY_MARGIN <= x <= MAP_X_MAX + MAP_SAFETY_MARGIN
                   and MAP_Y_MIN - MAP_SAFETY_MARGIN <= y <= MAP_Y_MAX + MAP_SAFETY_MARGIN)]
    if oob:
        bad("%d 个航点越界" % len(oob),
            "首个: #%d (%.2f,%.2f)" % oob[0])
    else:
        ok("地图边界", "全部 %d 点在 [%.2f,%.2f]x[%.2f,%.2f] 内"
           % (len(pts), MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX))

    # 挡板穿越
    crossings = barrier_crossings(pts)
    if not crossings:
        info("挡板穿越", "0 次（路线全在半场之内）")
    else:
        bad_n = 0
        for c in crossings:
            if not c['ok'] or c['margin'] < BARRIER_MIN_MARGIN:
                bad_n += 1
                bad("挡板穿越违规", "航段#%d 在 %s 列(y=%.3f, 余量 %.3f m)"
                    % (c['seg'], c['col'], c['y'], c['margin']))
        if bad_n == 0:
            m = min(c['margin'] for c in crossings)
            ok("挡板穿越 %d 次" % len(crossings),
               "均在 %s 列，最小余量 %.3f m"
               % ("/".join(sorted({c['col'] for c in crossings})), m))

    # ★挡板**触线**（细条口径替换旧的格带口径）
    #   旧口径：只看"过缺口时的余量 ≥ 0.10 m"（`BARRIER_MIN_MARGIN`），挡板被当成一整格宽。
    #   新口径：挡板是 2 cm 细条 ⇒ 车心到**中心线** ≥ 半厚 + 车半宽 = 0.0775 m；
    #          桥洞 K 处豁免（`seg_outside_band` 精确抠段，见其 docstring）。
    _bx, _by0, _by1 = barrier_centerline()
    _bd, _bi = barrier_touch_min(pts, bridge_col=bridge_column(bridges))
    _bd_req = BARRIER_WIDTH_M / 2.0 + VEHICLE_WIDTH_M / 2.0
    if _bd < _bd_req - 1e-9:
        bad("挡板触线 %.4f m < %.4f m" % (_bd, _bd_req),
            "航段#%d 距挡板中心线 x=%.3f（y %.3f~%.3f）不足；"
            "判据 = 挡板半厚 %.3f + 车半宽 %.4f"
            % (_bi, _bx, _by0, _by1, BARRIER_WIDTH_M / 2.0, VEHICLE_WIDTH_M / 2.0))
    else:
        ok("挡板不触线 %.4f m ≥ %.4f m" % (_bd, _bd_req),
           "最近在航段#%d（挡板 D..N 含 K 段连续，无豁免）" % _bi)

    # 障碍距离
    if obstacles:
        d, where = seg_obstacle_min(pts, obstacles)
        if d < OBSTACLE_WARN_M:
            warn("障碍最近距离 %.3f m" % d,
                 "航段#%d 距 障%s（<%.2fm）" % (where[0], where[1], OBSTACLE_WARN_M))
        else:
            ok("障碍最近距离", "%.3f m（航段#%d 距 障%s）> %.2f m"
               % (d, where[0], where[1], OBSTACLE_WARN_M))
    else:
        info("障碍距离", "未提供障碍表，跳过")

    # ★障碍**净空**（新增，**error 级**）：直段 0.215 m / 拐角按扫掠圆 0.3029 m
    if obstacles:
        _ds, _si, _sn, _dt, _ti, _tn = obstacle_clearances(pts, obstacles, rev_f)
        _req_straight = OBSTACLE_RADIUS_M + VEHICLE_WIDTH_M / 2.0 + OBSTACLE_MARGIN_M
        _req_corner = (OBSTACLE_RADIUS_M
                       + vehicle_half_diagonal_m(VEHICLE_LENGTH_M, VEHICLE_WIDTH_M)
                       + OBSTACLE_MARGIN_M)
        _villain = []
        if _ds < _req_straight - 1e-9:
            _villain.append("直段 %.3f < %.3f m（航段#%d 距 障%s）" % (_ds, _req_straight, _si, _sn))
        if _dt < _req_corner - 1e-9:
            _villain.append("拐角 %.3f < %.3f m（航点#%d 距 障%s，按扫掠半径 %.4f）"
                            % (_dt, _req_corner, _ti, _tn, vehicle_half_diagonal_m()))
        if _villain:
            bad("障碍净空不足", "；".join(_villain))
        else:
            ok("障碍净空", "直段 %.3f ≥ %.3f m；拐角 %.3f ≥ %.3f m（扫掠半径 %.4f）"
               % (_ds, _req_straight, _dt, _req_corner, vehicle_half_diagonal_m()))
    else:
        info("障碍净空", "未提供障碍表，跳过")

    # 转弯角度
    # ★必须排除倒车段：'R:K9' 这种是"往前开、然后倒着退一格"，
    #   在几何上表现为 180° 折返，但车根本没有掉头 ——
    #   不排除的话每一段倒车都会被误报成"急转弯触发 PIVOT"。
    turns = [(i, a) for (i, a) in turn_angles(pts)
             if not _is_rev_vertex(i, rev_f)]
    sharp = [t for t in turns if t[1] > SHARP_CORNER_DEG]
    declared_in_sharp = [i for i, _a in sharp if i < len(pivot_f) and pivot_f[i]]
    if sharp:
        warn("急转弯 %d 处" % len(sharp),
             "最大 %.0f° @航点#%d（其中 %d 处已声明 :PIVOT）"
             % (max(sharp, key=lambda x: x[1])[1], max(sharp, key=lambda x: x[1])[0],
                len(declared_in_sharp)))
    else:
        ok("转弯平顺", "最大 %.0f° < %.0f°，全程无需 PIVOT"
           % (max([t[1] for t in turns] or [0]), SHARP_CORNER_DEG))

    # ---------------- ④ 任务层 ----------------
    sec("4/6 任务层")
    visited = set()
    for t in toks:
        b = token_block(t)
        if b:
            visited.add(b)

    # ★雷/桥洞按【几何覆盖】判定：路线压过该格中心 tol 以内就算覆盖。
    #   （规划器会把直线段中间的格压缩掉，只查 token 会大面积误报）
    # 桥洞：K 列整列都是桥洞（每格被侧壁隔开），判据从"每个通格都要经过"
    # 改为"桥洞列被水平直穿足够多次"—— 把整列 11 格都算进去没有意义。
    _bridge_group_check(bridges, pts)

    # ★★★ 雷覆盖（换成**物理判据**）：车心 3.5 cm 圆与 0.12×0.12 垫片是否产生过重叠
    #   ⇔ 路线（折线）到**垫片**的最近距离 ≤ `COVERAGE_RADIUS_M`（0.035 m）。
    #   ⚠️ 旧口径是"路线到**雷格心** ≤ BLOCK_SIZE*0.6 = 0.21 m"——它是个"差不多踩到"的宽松代理，
    #      既会把"擦着格心 0.2 m 过"判成压到（垫片只有 0.12 m 见方，实际差 0.1 m 就没压上），
    #      又会把"贴边压过"（车心离雷心 0.095 m、圆与垫片确实重叠）判成压到 ——**恰好相反方向**。
    #      现场口径（用户给定）：判据必须落在**点到正方形**的距离上，不能用"到格心 ≤ 3.5 cm"。
    for name, lst in (("雷", mines),):
        if not lst:
            info("%s覆盖" % name, "未提供，跳过")
            continue
        req = [x for x in lst if str(x).strip().upper() not in covered]
        skipped = [x for x in lst if str(x).strip().upper() in covered]
        if skipped:
            info("%s覆盖（增量口径）" % name, "已压不再要求重压：%s" % " ".join(sorted(skipped)))
        miss, worst = [], []
        for x in req:
            d = mine_clearance_by_route(pts, x)
            if d > COVERAGE_RADIUS_M + 1e-12:
                miss.append("%s(离垫片 %.3fm)" % (x, d))
            else:
                worst.append(d)
        if miss:
            bad("%s覆盖 %d/%d" % (name, len(req) - len(miss), len(req)),
                "未压到（车心 %.3fm 圆 ∩ %.2fm 垫片）: %s"
                % (COVERAGE_RADIUS_M, MINE_PAD_M, " ".join(miss)))
        else:
            ok("%s覆盖 %d/%d（车心 %.3fm 圆 ∩ %.2fm 垫片）" % (name, len(req), len(req),
                                                              COVERAGE_RADIUS_M, MINE_PAD_M),
               "最差 %.4f m（阈值 %.3f）" % (max(worst), COVERAGE_RADIUS_M))

    # ★★【航点压过靶板 = ❌ error】(2026-10-11)
    #   用户口径：「所有靶子都是一个占用两格的跨格靶板」⇒ 靶板是**实体**，车不能开上去。
    #   规划器侧已把靶格并进 `Grid.blocked`，但本工具此前只查"打不打得到"、
    #   不查"有没有从靶板上开过去"：手工 `route.txt` 压过 `C1/D1` 仍拿 `0 error`
    #   （取证报告 §5.4 记的盲区，`probe_hand_check.txt` 独立复现）。
    #   ★ 判据只看**航点 token 自身**（`token_block()` 剥后缀后的格名）∈ 本图 `targets`：
    #     不做坐标换算、不做曼哈顿展开 ⇒ 天然绕开 0-based/1-based 口径坑
    #     （`geo.block_to_index` 是 0-based、规划器 `Grid`/`_cell` 是 1-based，这里两边都不碰）。
    #   ★ 覆盖度判据（下面的「靶覆盖 N/N」）**保持不变**：打得到 ≠ 压得起，两条各管各的。
    if targets:
        tset = {str(x).strip().upper() for x in targets}
        hits = [(i, token_block(t)) for i, t in enumerate(toks) if token_block(t) in tset]
        if hits:
            names: List[str] = []
            for _, b in hits:
                if b not in names:
                    names.append(b)
            bad("航点压过靶板 %d 处" % len(hits),
                "token#%s = %s（本图靶板 %s 是占两格的实体，车不可压过）"
                % ("/#".join(str(i) for i, _ in hits), " ".join(names),
                   " ".join(sorted(tset))))
        else:
            ok("航点不压靶板", "航点里没有 %s（靶板是实体，只在旁边停站射击）"
               % " ".join(sorted(tset)))

    # ★★【航段几何横穿靶格 = ❌ error】(2026-10-11)
    #   为什么在 token 级判据之外还要这一条：规划器/手工路线会把直线段**中间的格压缩掉**
    #   （`F8 -> B8` 这种 4 格直段只留两端），于是 `B1 -> E1` 这类直段可以**几何上横穿
    #   `C1`+`D1` 两块靶板**而航点里一个靶格 token 都没有 —— 取证报告 §4 的
    #   `probe_residual_gap` 实测这种路线仍拿 `exit 0`（采样 34/101 点落在靶格内）。
    #   ★ 判据与「雷覆盖」**完全同一套**（`route_covers(pts, cell, tol)`：路线折线到格心的
    #     最短距离 ≤ tol，解析式、非采样），只是 tol 取**半格**（见 `TARGET_RUNOVER_TOL_M`）。
    #     不另写几何、不手写采样展开 ⇒ 不给自己造第二套口径。
    #   ★ 纵深防御：token 级（航点自身）与几何级（航段经过）两条**都要**，任一命中即 error。
    if targets:
        worst = 9e9
        runover: List[str] = []
        for t in targets:
            covered, dist = route_covers(pts, t, TARGET_RUNOVER_TOL_M)
            worst = min(worst, dist)
            if covered:
                runover.append("%s(车心最近 %.3f m)" % (t, dist))
        if runover:
            bad("航段横穿靶板 %d 块" % len(runover),
                "%s —— 判据 = 车心进到格心 %.3f m 以内（半格，靶板 0.35 m 见方）"
                % (" ".join(runover), TARGET_RUNOVER_TOL_M))
        else:
            ok("航段不横穿靶板", "全路线（含直段中间的格）离最近靶格心 %.3f m > %.3f m"
               % (worst, TARGET_RUNOVER_TOL_M))

    # ★★【车体压靶 = ❌ error】(追加：用户 2026-10-11 现场澄清④)
    #   靶板 **35×35 cm/格、中心在格心** ⇒ 格级挡块（planner 侧）"恰好物理精确"（澄清③，那边不动）；
    #   路线几何这一侧的判据是：**车体 28×13.5 不许与靶板 overlap** ⇔
    #     直段：折线到靶板**矩形**的距离 ≥ 车半宽 0.0675；
    #     拐角：转向顶点到靶板矩形的距离 ≥ 车体半对角线 0.1554（车绕顶点转时车身外包圆）。
    #   ⚠️ 与上一条「航段横穿靶板」（车心半格口径）是**两级纵深**：这条是物理精确口径，
    #      两条都是 error 级；常量全部来自 config（`vehicle.*`），本工具不写第二份。
    if targets:
        _bhalf = BLOCK_SIZE / 2.0
        _req_str = VEHICLE_WIDTH_M / 2.0
        _req_cnr = vehicle_half_diagonal_m(VEHICLE_LENGTH_M, VEHICLE_WIDTH_M)
        _bs, _bt = 9e9, 9e9
        for _t in targets:
            try:
                _tx, _ty = block_to_meters(_t)
            except ValueError:
                continue
            for _i in range(len(pts) - 1):
                _bs = min(_bs, _phys_seg_rect_dist(pts[_i][0], pts[_i][1],
                                                   pts[_i + 1][0], pts[_i + 1][1],
                                                   _tx, _ty, _bhalf))
            for _i, _ang in turn_angles(pts):
                if _ang <= 10.0 or (rev_f is not None and _is_rev_vertex(_i, rev_f)):
                    continue
                _bt = min(_bt, _phys_point_rect_dist(pts[_i][0], pts[_i][1], _tx, _ty, _bhalf))
        _bv = []
        if _bs < _req_str - 1e-9:
            _bv.append("直段 %.4f < %.4f m（车半宽）" % (_bs, _req_str))
        if _bt < _req_cnr - 1e-9:
            _bv.append("拐角 %.4f < %.4f m（车体半对角线 %.4f）" % (_bt, _req_cnr,
                                                                  vehicle_half_diagonal_m()))
        if _bv:
            bad("车体压靶板", "；".join(_bv))
        else:
            ok("车体不压靶板", "直段 %.4f ≥ %.4f m；拐角 %.4f ≥ %.4f m（靶板 %.2f m 见方，中心在格心）"
               % (_bs, _req_str, _bt, _req_cnr, BLOCK_SIZE))

    # ★靶标按【射击覆盖】判定：靶不需要被踩到，只要路线上有某个
    #   "射击点"落在 0.20~0.45m 且视线通畅即可开火。
    if targets:
        shoot_idx = [i for i, f in enumerate(shoot_f) if f]
        if not shoot_idx:
            bad("靶覆盖 0/%d" % len(targets), "路线上没有任何射击点(*)")
        else:
            miss_t, detail = [], []
            for t in targets:
                try:
                    tx, ty = block_to_meters(t)
                except ValueError:
                    continue
                # ★必须是"最小的【有效】距离"，不能直接取 min：
                #   射击点 #16 就是 F1 格本身（它是为靶 G1 准备的），
                #   直接取 min 会得到 d(F1,F1)=0，把合法的点误判成超范围。
                #   正确判据 = 存在某个射击点到该靶的距离落在有效区间内。
                cands = [(math.hypot(pts[i][0] - tx, pts[i][1] - ty), i)
                         for i in shoot_idx]
                inrange = [c for c in cands
                           if SHOOT_DIST_MIN - 0.03 <= c[0] <= SHOOT_DIST_MAX + 0.06]
                if inrange:
                    d, i = min(inrange)
                    detail.append("%s<-射击点#%d(%.2fm)" % (t, i, d))
                else:
                    d, i = min(cands)
                    miss_t.append("%s(最近射击点#%d 仅 %.2fm，不在 %.2f~%.2f)"
                                  % (t, i, d, SHOOT_DIST_MIN, SHOOT_DIST_MAX))
            if miss_t:
                bad("靶覆盖 %d/%d" % (len(targets) - len(miss_t), len(targets)),
                    "; ".join(miss_t))
            else:
                ok("靶覆盖 %d/%d（射击点可达）" % (len(targets), len(targets)),
                   " ".join(detail))

    # 射击点到靶的距离
    tgt_cells = set()
    for t in targets:
        try:
            tgt_cells.add(parse_block(t))
        except ValueError:
            pass
    if tgt_cells and any(shoot_f):
        checked = 0
        prob = []
        for i, f in enumerate(shoot_f):
            if not f:
                continue
            sx, sy = pts[i]
            checked += 1
            # ★关键：一个射击点是"为某个靶准备的"，所以判断依据是
            #   "存在【某个】靶，与本点距离落在 0.20~0.45 内"，
            #   而不是"最近的靶有多远"。
            #   早期版本取 min(ds) 会把"站在靶旁边却离另一个靶更近"的点误判 ——
            #   更糟的是当射击点恰好就在某个靶格上时 d=0，被判成"超范围"。
            good_tgt = [t for t in targets
                        if SHOOT_DIST_MIN - 0.03
                        <= math.hypot(sx - block_to_meters(t)[0],
                                      sy - block_to_meters(t)[1])
                        <= SHOOT_DIST_MAX + 0.06]
            if not good_tgt:
                dmin = min(math.hypot(sx - block_to_meters(t)[0],
                                      sy - block_to_meters(t)[1]) for t in targets)
                prob.append((i, dmin))
        if prob:
            bad("射击点距离 %d 处超范围" % len(prob),
                "如 #%d 距最近靶 %.2fm（应有一个靶落在 %.2f~%.2f m）"
                % (prob[0][0], prob[0][1], SHOOT_DIST_MIN, SHOOT_DIST_MAX))
        else:
            ok("射击点距离", "%d 个射击点都能服务于某个靶（%.2f~%.2f m 内）"
               % (checked, SHOOT_DIST_MIN, SHOOT_DIST_MAX))
    elif any(shoot_f):
        info("射击点距离", "未提供靶标位置，跳过")

    # ---------------- ⑤ 安全层 ----------------
    sec("5/6 安全层")
    total = path_length(pts)
    info("总里程", "%.2f m（%d 航段）" % (total, len(pts) - 1))
    info("越界保护", "MAP_SAFETY_MARGIN = %.2f m（保持不变）" % MAP_SAFETY_MARGIN)

    # 倒车段合理性
    # ★判据要区分两种倒车，风险差得很远：
    #     · 直线倒车（单段、方向不变）：车头不用动，零横向摆动，
    #       在已知畅通的直线走廊上倒 1~2m 是安全的。
    #       —— 这正是"180° 死角掉头"的正确替代方案（掉头会横扫 0.54m 出界）。
    #     · 倒车 + 转向（多段连续倒车、方向在变）：盲区里打舵，风险高得多。
    #   早期版本只看单段长度，把前者也一并告警，属于误报。
    rev_segs = [(i, pts[i], pts[i + 1])
                for i in range(len(pts) - 1)
                if i + 1 < len(rev_f) and rev_f[i + 1]]
    if rev_segs:
        # 连续相邻的倒车段归为一组（组内有多段 = 倒车中转了向）
        groups, cur = [], [rev_segs[0]]
        for seg in rev_segs[1:]:
            if seg[0] == cur[-1][0] + 1:
                cur.append(seg)
            else:
                groups.append(cur)
                cur = [seg]
        groups.append(cur)

        longest = max(groups, key=lambda g: sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for _, a, b in g))
        L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for _, a, b in longest)

        turned = [g for g in groups if len(g) > 1]
        if turned:
            warn("倒车+转向 %d 组" % len(turned),
                 "最长 %.2f m / %d 段 —— 盲区里打舵风险高，建议缩短或拆开"
                 % (L, len(longest)))
        elif L > 2.5:
            warn("直线倒车偏长", "%.2f m（>2.5m）—— 无后方传感器，建议复核该走廊" % L)
        else:
            ok("倒车 %d 段" % len(rev_segs),
               "均为直线倒车，最长 %.2f m（零横向摆动，安全）" % L)

    # 桥洞保护点是否在路线上
    brg = cfg.get('bridges') or []
    if brg:
        onroute = [b for b in brg if str(b).upper() in visited]
        if onroute:
            ok("桥洞保护点覆盖", " ".join(onroute))
        else:
            warn("桥洞保护点未覆盖",
                 "路线不经过 %s —— 强制直行保护不会触发" % " ".join(brg))

    # ---------------- ⑥ 折返层（:PIVOT 声明一致性） ----------------
    sec("6/6 折返层")
    _pivot_layer(pts, rev_f, pivot_f)

    return summarise()


def _pivot_layer(pts, rev_f, pivot_f):
    """`:PIVOT` 声明与实际折返几何的一致性 —— **只报不改，不改循迹层行为**。

    判据（与 `planner.PIVOT_MARK_DEG` / `pursuit.pivot_angle_deg` 同一套）：
      * "折返顶点" = 顶点处**行进方向**夹角 ≥ `PIVOT_DECL_DEG`（135°），
        且相邻两段**都不是倒车段**（`_is_rev_vertex`：倒车段车头一度都不用转）；
      * 虚报（声明了 `:PIVOT` 但不是折返顶点）= ❌ 真缺陷（会让现场误判该处要掉头）；
      * 漏报（是折返顶点但没声明）= ⚠️ 提示（循迹层仍会按 `corner_turn_deg` 自动 PIVOT，
        车不会撞 —— 但下发的路线就"没写明"，与「折返点显式 :PIVOT」的目标不符）。
    """
    thick = []
    for i, angle in turn_angles(pts):
        if angle < PIVOT_DECL_DEG - 1e-9:
            continue
        if _is_rev_vertex(i, rev_f):            # 相邻段有倒车 → 车头不用掉头
            continue
        thick.append((i, angle))

    declared = [i for i, f in enumerate(pivot_f) if f]
    extra = [i for i in declared if not any(i == j for j, _a in thick)]
    missing = [(i, a) for i, a in thick if not (i < len(pivot_f) and pivot_f[i])]

    def name_of(idx):
        row, col = parse_block_xy(pts[idx])
        return "%s%d" % (COL_LETTERS[col], row)

    if extra:
        bad("虚报 :PIVOT %d 处" % len(extra),
            " ".join("#%d %s" % (i, name_of(i)) for i in extra[:4]))
    if missing:
        warn("折返点缺 :PIVOT %d 处" % len(missing),
             " ".join("#%d %.0f° @%s" % (i, a, name_of(i)) for i, a in missing[:4]))
    if not extra and not missing:
        if thick:
            ok("折返点声明一致 %d 处" % len(thick),
               " ".join("#%d %.0f° @%s" % (i, a, name_of(i)) for i, a in thick[:5]))
        else:
            info("折返点", "0 处 —— 无 ≥%.0f° 的正向折返顶点，无需 :PIVOT"
                 % PIVOT_DECL_DEG)


def parse_block_xy(xy):
    return parse_block_from_meters(xy[0], xy[1])


def parse_block_from_meters(x, y):
    row = int(round(10.5 - x / BLOCK_SIZE)) + 1
    col = int(round(13.5 - y / BLOCK_SIZE))
    return max(1, min(MAP_ROWS, row)), max(0, min(MAP_COLS - 1, col))


def summarise() -> int:
    print("\n" + "=" * 78)
    if FAIL:
        print("  \033[31m\033[1m❌ %d 项失败 / %d 项警告 —— 请先处理再上车\033[0m"
              % (len(FAIL), len(WARN)))
        for f in FAIL:
            print("     · %s" % f)
        rc = 1
    elif WARN:
        print("  \033[33m\033[1m⚠️  全部通过，但有 %d 项提醒\033[0m" % len(WARN))
        for w in WARN:
            print("     · %s" % w)
        rc = 0
    else:
        print("  \033[32m\033[1m🎉 全链路校验通过，路线可以下发\033[0m")
        rc = 0
    print("=" * 78 + "\n")
    return rc


def _find_map_file(cfg_path):
    """找地图文件。

    ★ 地图数据【不能】放在 robot.yaml 里：ROS 2 的 params 文件顶层只允许
      <节点名>: ros__parameters: 结构，多一个 map: 段就会让 --params-file
      解析失败，相机/视觉/云台三个节点会同时 abort。所以它单独一个文件。
    """
    cands = []
    if os.environ.get('ROBOT_MAP'):
        cands.append(os.environ['ROBOT_MAP'])
    if cfg_path:
        cands.append(os.path.join(os.path.dirname(os.path.abspath(cfg_path)),
                                  'map.yaml'))
    cands.append(os.path.join(WS, 'config', 'map.yaml'))
    for c in cands:
        if c and os.path.exists(c):
            return c
    return None


def load_cfg(path):
    """从配置中心 + map.yaml 读取地图要素。"""
    cfg = {'obstacles': [], 'mines': [], 'targets': [], 'bridges': []}
    try:
        import yaml
    except ImportError:
        return cfg

    # 参数（路径/桥洞/当前路线）来自 robot.yaml
    if path and os.path.exists(path):
        try:
            with open(path, encoding='utf-8') as fp:
                d = yaml.safe_load(fp) or {}
            p = (d.get('path_executor') or {}).get('ros__parameters', {}) or {}
            cfg['mission_blocks'] = list(p.get('mission_blocks') or [])
            cfg['obstacles'] = list(p.get('obstacle_blocks') or [])
            cfg['bridges'] = list(p.get('bridge_blocks') or [])
            # ★射程随配置走：射程一变，合法射击点就完全不同，
            #   校验器硬编码旧值会把正确的路线判成错的（已经踩过一次）。
            for _k, _g in (('shoot_distance_min', 'SHOOT_DIST_MIN'),
                           ('shoot_distance_max', 'SHOOT_DIST_MAX')):
                if _k in p:
                    globals()[_g] = float(p[_k])
            # ★ 折返声明门限也随配置走（`/dzbrain: ros__parameters: pursuit:` 段；
            #   robot.yaml 目前没写这个键，所以取 `core/config.py` 的默认 135°）。
            dz = (d.get('/dzbrain') or {}).get('ros__parameters', {}) or {}
            pu = dz.get('pursuit') if isinstance(dz.get('pursuit'), dict) else {}
            if pu and 'pivot_angle_deg' in pu:
                globals()['PIVOT_DECL_DEG'] = float(pu['pivot_angle_deg'])
            # ★★物理几何：尺寸全部随配置走，判据里**不许**再出现硬编码数字。
            #   现场一量尺寸只改 `config/robot.yaml`，本工具的判据自动跟上。
            veh = dz.get('vehicle') if isinstance(dz.get('vehicle'), dict) else {}
            fld = dz.get('field') if isinstance(dz.get('field'), dict) else {}
            pln = dz.get('planner') if isinstance(dz.get('planner'), dict) else {}
            for _src, _k, _g in ((veh, 'length_m', 'VEHICLE_LENGTH_M'),
                                 (veh, 'width_m', 'VEHICLE_WIDTH_M'),
                                 (veh, 'coverage_radius_m', 'COVERAGE_RADIUS_M'),
                                 (fld, 'obstacle_radius_m', 'OBSTACLE_RADIUS_M'),
                                 (fld, 'mine_pad_m', 'MINE_PAD_M'),
                                 (fld, 'barrier_width_m', 'BARRIER_WIDTH_M'),
                                 (pln, 'obstacle_margin_m', 'OBSTACLE_MARGIN_M')):
                if _k in _src:
                    globals()[_g] = float(_src[_k])
            if fld.get('barrier_cols'):
                globals()['BARRIER_COLS'] = str(fld['barrier_cols'])
            cfg['physics'] = {'vehicle': dict(veh), 'field': dict(fld), 'planner': dict(pln)}
        except Exception as exc:
            print("  ⚠️  读配置中心失败: %s" % exc, file=sys.stderr)

    # 地图要素来自独立的 map.yaml（优先，覆盖上面的兜底值）
    mf = _find_map_file(path)
    if mf:
        try:
            with open(mf, encoding='utf-8') as fp:
                m = (yaml.safe_load(fp) or {}).get('map') or {}
            for k in ('obstacles', 'mines', 'targets', 'bridges'):
                if m.get(k):
                    cfg[k] = list(m[k])
            cfg['map_name'] = m.get('name', '')
            cfg['map_file'] = mf
        except Exception as exc:
            print("  ⚠️  读地图文件失败: %s" % exc, file=sys.stderr)
    return cfg


def run_planner(args, cfg) -> Optional[str]:
    """调用 global_planner.py 生成路线串。"""
    planner = os.path.join(HERE, 'global_planner.py')
    if not os.path.exists(planner):
        print("  ❌ 找不到 %s" % planner, file=sys.stderr)
        return None
    cmd = [sys.executable, planner,
           '--start', args.start, '--end', args.end,
           '--emit-route', '--quiet']
    if args.mines:
        cmd += ['--mines'] + args.mines
    if args.obstacles or cfg['obstacles']:
        cmd += ['--obstacles'] + (args.obstacles or cfg['obstacles'])
    if args.targets or cfg['targets']:
        cmd += ['--targets'] + (args.targets or cfg['targets'])
    if args.bridges or cfg['bridges']:
        cmd += ['--bridges'] + (args.bridges or cfg['bridges'])
    print("  → 调用规划器: %s" % " ".join(
        "'%s'" % c if ' ' in c else c for c in cmd[2:]))
    try:
        # ★2026-10-18 修（同类）：Windows 默认 GBK 解码，规划器输出含中文横幅时会 UnicodeDecodeError
        #   并让 r.stdout 变 None（后续 .strip() 直接崩）。与 tools/wf/check.py 同一处坑。
        r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8",
                           errors="replace", timeout=900)
        if r.returncode != 0:
            print("  ❌ 规划器失败:\n%s" % r.stderr[-800:], file=sys.stderr)
            return None
        return r.stdout.strip().splitlines()[-1].strip()
    except subprocess.TimeoutExpired:
        print("  ❌ 规划器超时", file=sys.stderr)
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="路线全链路校验（规划器输出 -> 路线下发 -> 几何校验）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config',
                    default=os.path.join(WS, 'config', 'robot.yaml'))
    ap.add_argument('--route', default=None, help='直接给一条路线串')
    ap.add_argument('--from-planner', action='store_true',
                    help='先跑 global_planner.py 拿路线，再校验')
    ap.add_argument('--start', default=None)
    ap.add_argument('--end', default=None)
    ap.add_argument('--mines', nargs='*', default=[])
    ap.add_argument('--obstacles', nargs='*', default=[])
    ap.add_argument('--targets', nargs='*', default=[])
    ap.add_argument('--bridges', nargs='*', default=[])
    ap.add_argument('--already-covered', nargs='*', default=[],
                    help='已压雷格（增量续算口径）：这些雷不再要求本路线重压，'
                         '仅从覆盖判据里剔除，其余校验不变')
    args = ap.parse_args(argv)

    cfg = load_cfg(args.config)

    if args.from_planner:
        if not args.start or not args.end:
            print("❌ --from-planner 需要 --start 与 --end", file=sys.stderr)
            return 2
        if not args.mines and not cfg['mines']:
            print("❌ --from-planner 需要 --mines 或配置中心 map.mines",
                  file=sys.stderr)
            return 2
        print("=" * 78)
        print("  阶段一：规划器生成路线")
        print("=" * 78)
        route = run_planner(args, cfg)
        if not route:
            return 2
        print("  ✅ 规划器输出:\n     %s" % route)
        if args.targets or cfg['targets']:
            cfg['targets'] = args.targets or cfg['targets']
        if args.bridges or cfg['bridges']:
            cfg['bridges'] = args.bridges or cfg['bridges']
        if args.obstacles or cfg['obstacles']:
            cfg['obstacles'] = args.obstacles or cfg['obstacles']
        if args.mines:
            cfg['mines'] = args.mines
        print()
    elif args.route:
        route = args.route
        if args.mines:
            cfg['mines'] = args.mines
        if args.obstacles:
            cfg['obstacles'] = args.obstacles
        if args.targets:
            cfg['targets'] = args.targets
        if args.bridges:
            cfg['bridges'] = args.bridges
        if args.already_covered:
            cfg['already_covered'] = args.already_covered
    else:
        mb = cfg.get('mission_blocks') or []
        if not mb:
            print("❌ 未指定路线：用 --route / --from-planner，"
                  "或确保配置中心有 mission_blocks", file=sys.stderr)
            return 2
        route = ' '.join(str(x) for x in mb)
        print("=" * 78)
        print("  校验配置中心当前路线")
        print("=" * 78)
        print("  来源: %s" % args.config)
        print()

    return run_check(route, cfg)


if __name__ == '__main__':
    sys.exit(main())
