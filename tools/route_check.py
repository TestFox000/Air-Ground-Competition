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
五层检查（⑥ 折返层为卡 C 新增，共六层）
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
    ./route_check.py --route "M10 K7 F2* R:K9 N5"

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
from typing import Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)

# =============================================================================
#  地图常量（与 path_executor.py 保持一致）
# =============================================================================
COL_LETTERS = "ABCDEFGHIJKLMN"
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35

BARRIER_ROW_HI = 6
BARRIER_ROW_LO = 7
BARRIER_COL_FROM = "D"

# 判定阈值
BARRIER_MIN_MARGIN = 0.10        # 过挡板缺口时离挡板边缘的最小余量(m)
OBSTACLE_WARN_M = 0.30           # 航段离障碍的告警距离(m)
SHOOT_DIST_MIN = 0.20            # 射击点距离下限(m)，会被配置中心覆盖
SHOOT_DIST_MAX = 0.45            # 射击点距离上限(m)，会被配置中心覆盖
SHARP_CORNER_DEG = 45.0          # 超过此角度算"急转弯"（提示级；★ PIVOT 的真正门限见下行）
# ★ PIVOT 真正触发门限（度）：循迹层用 `pursuit.path.corner_turn_deg`（**车头**要转的角度）
#   与 `pursuit.pivot_angle_deg`（`core/config.py` 默认 135°、robot.yaml 未覆盖）比较。
#   本工具里它只用来校验 `:PIVOT` 声明的一致性（第 ⑥ 层），
#   **不改变** `pursuit/**` 与 `tasks/tracker.py` 的任何行为。
PIVOT_DECL_DEG = 135.0

MAP_X_MIN = 0.5 * BLOCK_SIZE
MAP_X_MAX = 10.5 * BLOCK_SIZE
MAP_Y_MIN = 0.5 * BLOCK_SIZE
MAP_Y_MAX = 13.5 * BLOCK_SIZE
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
def block_to_meters(name: str) -> Tuple[float, float]:
    s = str(name).strip().upper()
    for p in ('R:', 'SHOOT@'):
        if s.startswith(p):
            s = s[len(p):]
    if s.endswith('*'):
        s = s[:-1]
    if ':' in s:
        s = s.split(':', 1)[0]
    if ',' in s:
        s = s.split(',', 1)[0]
    s = s.strip()
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
                # ★ 卡 C 新增后缀：折返点**声明**（该顶点车头需转过 ≥ PIVOT_DECL_DEG）。
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

    ★ `pivot`（第 5 个返回值，卡 C 新增）与 `pts` **逐点对齐**：
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




def _bridge_group_check(bridges, pts):
    """桥洞穿越检查（支持"整列都是桥洞"）。"""
    if not bridges:
        info("桥洞覆盖", "未提供，跳过")
        return
    cols = sorted({str(b)[0].upper() for b in bridges})
    rows = [int(str(b)[1:]) for b in bridges if str(b)[1:].isdigit()]
    if not cols:
        info("桥洞覆盖", "未提供，跳过")
        return
    tol = BLOCK_SIZE * 0.45
    n_cross = 0
    detail = []
    for col in cols:
        cx = (13.5 - COL_LETTERS.index(col)) * BLOCK_SIZE
        for i in range(len(pts) - 1):
            (x1, y1), (x2, y2) = pts[i], pts[i + 1]
            # ★注意坐标轴：x 对应"行"，y 对应"列"。
            #   穿 K 列 = 在同一行内跨越 y 方向 -> 应筛"x 不变"的段。
            #   （之前写成筛 y 不变，方向正好反了，导致明明穿了却判"未直穿"。）
            if abs(x1 - x2) > 1e-6:
                continue                       # 只看同一行内的段
            if min(y1, y2) - tol <= cx <= max(y1, y2) + tol:
                r = round(10.5 - x1 / BLOCK_SIZE) + 1
                if not rows or min(rows) <= r <= max(rows):
                    n_cross += 1
                    detail.append("段#%d(行%d)" % (i, r))
    if n_cross >= 2:
        ok("桥洞水平直穿 %d 次" % n_cross,
           "%s 列  %s" % ("/".join(cols), " ".join(detail[:4])))
    elif n_cross == 1:
        warn("桥洞只直穿 1 次", "本场有 2 个通口，应各穿一次")
    else:
        bad("桥洞未直穿", "%s 列上没有水平穿越段" % "/".join(cols))


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
#  主流程
# =============================================================================
def run_check(route: str, cfg: dict) -> int:
    tokens = route.split()
    obstacles = cfg.get('obstacles') or []
    mines = cfg.get('mines') or []
    targets = cfg.get('targets') or []
    bridges = cfg.get('bridges') or []

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
    pts, shoot_f, scan_f, rev_f, pivot_f = parse_route(toks)
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
        b = t.upper()
        for p in ('R:', 'SHOOT@'):
            if b.startswith(p):
                b = b[len(p):]
        if b.endswith('*'):
            b = b[:-1]
        if ':' in b:
            b = b.split(':', 1)[0]
        if ',' in b:
            b = b.split(',', 1)[0]
        b = b.strip()
        if b:
            visited.add(b)

    # ★雷/桥洞按【几何覆盖】判定：路线压过该格中心 tol 以内就算覆盖。
    #   （规划器会把直线段中间的格压缩掉，只查 token 会大面积误报）
    # 桥洞：K 列整列都是桥洞（每格被侧壁隔开），判据从"每个通格都要经过"
    # 改为"桥洞列被水平直穿足够多次"—— 把整列 11 格都算进去没有意义。
    _bridge_group_check(bridges, pts)

    for name, lst in (("雷", mines),):
        if not lst:
            info("%s覆盖" % name, "未提供，跳过")
            continue
        miss, far = [], []
        for x in lst:
            cov, d = route_covers(pts, x)
            if not cov:
                miss.append("%s(%.2fm)" % (x, d))
            elif d > BLOCK_SIZE * 0.25:
                far.append("%s(%.2fm)" % (x, d))
        if miss:
            bad("%s覆盖 %d/%d" % (name, len(lst) - len(miss), len(lst)),
                "未经过: %s" % " ".join(miss))
        else:
            extra = ("（其中 %s 为途经而非航点，已确认真实压过）"
                     % " ".join(far)) if far else ""
            ok("%s覆盖 %d/%d" % (name, len(lst), len(lst)), extra)

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
        车不会撞 —— 但下发的路线就"没写明"，与本卡「折返点显式 :PIVOT」的目标不符）。
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
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
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
