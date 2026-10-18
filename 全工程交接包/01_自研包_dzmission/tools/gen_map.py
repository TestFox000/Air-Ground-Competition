#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 gen_map.py —— 随机地图生成器（同时输出「数组形式」与「YAML」）
================================================================================

用途
----
1. 生成一份**随机但物理自洽**的场地地图，供离线测试使用
2. 以**数组形式**打印 `cells`（154 个整数），便于你核对地图模型
3. 同时给出对应的 YAML 片段（可直接粘进 tools/maps/*.yaml）

场地规格（与 C++ FieldGrid::Spec / air_protocol.py 完全一致）
------------------------------------------------------------
    网格        14 列 × 11 行 = 154 格
    格边长      35 cm
    坐标        列 col: 0=A .. 10=K(桥洞列) .. 13=N
                行 row: 0=最北 .. 5=隔离线上方 .. 6=隔离线下方 .. 10=最南
    隔离线      在 row6/row7 之间（= barrier_up_row 5），覆盖 col 3..13 (D..N)
    桥洞列      col 10 (K)，整列是隔板，只有两个门能通过
    区段一      row 0..5   （K1..K6），随机 1 格可通行
    区段二      row 6..10  （K7..K11），随机 1 格可通行
    起点        (col 12, row 10) = M11，起始航向 -90°（朝北）
    终点        (col 13, row 5)  = N6

★ 关键约束（生成器会保证）
--------------------------
    · 隔离线（row5 的 col 3..13）是 OCC_BLOCKED，不可通行
    · 起点/终点本身不能是阻挡或雷
    · 桥洞列除了两个门，其余都是 OCC_GATE_UNK（未知），门是 OCC_GATE_OPEN
    · 地雷不放在起点/终点/隔离线/门位上

cells 的解码方式
----------------
    occ = v & 0x07          # 0未知 1自由 2阻挡 3地雷 4靶标 5洞未知 6洞开
    conf = (v >> 3) & 0x0F  # 置信度 0..15
    age  = (v >> 7) & 0x0F  # 陈旧度 0..15

    常用值: 0x0078 = 未知格(conf15,age0)   0x007D = 洞开
            OCC_BLOCKED(2)|conf15 = 2 + 15*8 = 122 = 0x7A

用法
----
    python3 tools/gen_map.py                      # 固定种子 1
    python3 tools/gen_map.py --seed 42
    python3 tools/gen_map.py --seed 42 --mines 8 --obstacles 10 --out r.yaml
================================================================================
"""

from __future__ import annotations

import argparse
import os
import random
import sys

# ------------------------------------------------------------------ 场地常量（权威值）
COLS, ROWS = 14, 11
CELL_CM = 35
BARRIER_UP_ROW = 5            # 隔离线在 row5 与 row6 之间
BARRIER_COL_LO, BARRIER_COL_HI = 3, 13     # D..N
BRIDGE_COL = 10               # K
BRIDGE_A = (0, 5)             # K1..K6
BRIDGE_B = (6, 10)            # K7..K11
START = (12, 10)              # M11 —— 智能车起点
GOAL = (13, 5)                # N6  —— 智能车终点
START_HEADING_DEG = -90.0
# ★ 无人机起降点（现场图固有标注，随 FIELD_FIXED v2.1 下发）
#   无人机起飞前与降落后停在这两格，地面车必须**避开** ——
#   撞机等价于"撞固定场地"，持续 >10s 裁判切手动。
DRONE_START = (12, 8)         # M9
DRONE_END = (13, 1)           # N2

# 占用码
OCC_UNKNOWN, OCC_FREE, OCC_BLOCKED = 0, 1, 2
OCC_MINE, OCC_TARGET, OCC_GATE_UNK, OCC_GATE_OPEN = 3, 4, 5, 6

# ★ 未侦察格的默认取值（用户已确认：**乐观 = FREE**）
#   依据 field_grid.cpp::cellPassable()：只有 kFree / kMine 可通行。
#   所以默认给 FREE 表示"未发现障碍即视为可通行"，否则未侦察区全都不能走。
DEFAULT_OCC = OCC_FREE

OCC_NAMES = {
    OCC_UNKNOWN: '未知', OCC_FREE: '自由', OCC_BLOCKED: '阻挡',
    OCC_MINE: '地雷', OCC_TARGET: '靶标', OCC_GATE_UNK: '洞未知',
    OCC_GATE_OPEN: '洞开',
}


def encode(occ: int, conf: int = 15, age: int = 0) -> int:
    """与 air_protocol.cell_encode 完全一致：低3位 occ，bit3-6 conf，bit7-10 age"""
    return (occ & 0x07) | ((conf & 0x0F) << 3) | ((age & 0x0F) << 7)


def col_letter(c: int) -> str:
    return chr(ord('A') + c) if 0 <= c < 26 else '?'


def cell_name(c: int, r: int) -> str:
    return '%s%d' % (col_letter(c), r + 1)


# ==============================================================================
def gen_map(seed: int, n_mines: int, n_obstacles: int, n_targets: int,
            gates: tuple, verbose: bool = True) -> dict:
    """生成随机地图

    ★★ 本函数用的地图模型（依据 C++ FieldGrid 源码，不是猜的）★★
    ------------------------------------------------------------------
    `FieldGrid::reset()` 里写得很明确：
        cells_.assign(n, kFree);      // 所有格默认【自由】
        buildFixedEdges();            // 隔离线是【边】，不是格
        K 列整列 kBlocked             // 桥洞列是隔板，只有门能过
    `buildFixedEdges()` 的注释：
        "row(barrier_up_row) 与 row(+1) 之间的**纵向边**…格子本身可通行，
         只是这条边不可跨越。"

    所以：
      · 绝大多数格 = OCC_FREE（自由），**不是"未知"**
      · K 列非门位 = OCC_GATE_UNK；两个门 = OCC_GATE_OPEN
      · **隔离线不出现在 154 格数组里** —— 它是固定的边，
        车端 FieldGrid 已经内置，不需要（也不应该）用"整排阻挡格"表达。
        ★ 早先把它编成整排 OCC_BLOCKED 是**错的**：barrier_col_hi=13
          覆盖到 N 列，会把终点 N6=(13,5) 也变成阻挡 -> 终点不可达。
    ------------------------------------------------------------------
    """
    rng = random.Random(seed)

    # ---- 1) 底图：未侦察格 = 默认值（用户确认：FREE 乐观）----
    cells = [encode(DEFAULT_OCC, 15, 0) for _ in range(COLS * ROWS)]

    def set_cell(c, r, occ, conf=15, age=0):
        cells[r * COLS + c] = encode(occ, conf, age)

    # ---- 2) 桥洞列 K：整列隔板，只有两个门可通行 ----
    gate_rows = {1: gates[0], 2: gates[1]}      # 区段 -> 门所在 row
    for r in range(ROWS):
        if r in (gate_rows[1], gate_rows[2]):
            set_cell(BRIDGE_COL, r, OCC_GATE_OPEN, 15, 0)
        else:
            set_cell(BRIDGE_COL, r, OCC_GATE_UNK, 15, 0)

    # ---- 3) ★ 无人机起降点：禁入格（用 OCC_BLOCKED 表示）----
    #   车端 FieldGrid 也会在自己 reset() 里默认标它们为 kBlocked，
    #   这里显式写上是为了让下发的全图与车端一致（conf 给满，确保被采纳）。
    for (c, r) in (DRONE_START, DRONE_END):
        set_cell(c, r, OCC_BLOCKED, 15, 0)

    # ---- 4) 禁止放置地雷/障碍的格 ----
    reserved = {(BRIDGE_COL, r) for r in range(ROWS)}   # 整个 K 列
    reserved.add(START)
    reserved.add(GOAL)
    reserved.add(DRONE_START)      # ★ 无人机位不放东西
    reserved.add(DRONE_END)
    # 起点/终点/无人机位周围留一格空隙，避免生成"出门就被堵"的图
    for (c, r) in (START, GOAL, DRONE_START, DRONE_END):
        for dc in (-1, 0, 1):
            for dr in (-1, 0, 1):
                cc, rr = c + dc, r + dr
                if 0 <= cc < COLS and 0 <= rr < ROWS:
                    reserved.add((cc, rr))

    free_cells = [(c, r) for c in range(COLS) for r in range(ROWS)
                  if (c, r) not in reserved]

    # ---- 5) 放地雷 ----
    n_mines = min(n_mines, len(free_cells))
    mine_cells = rng.sample(free_cells, n_mines)
    for (c, r) in mine_cells:
        set_cell(c, r, OCC_MINE, rng.randint(70, 99), 0)

    # ---- 6) 放障碍 ----
    rest = [p for p in free_cells if p not in set(mine_cells)]
    n_obstacles = min(n_obstacles, len(rest))
    obs_cells = rng.sample(rest, n_obstacles)
    for (c, r) in obs_cells:
        set_cell(c, r, OCC_BLOCKED, 15, 0)

    # ---- 7) 放打击区（靶标）----
    rest2 = [p for p in rest if p not in set(obs_cells)]
    n_targets = min(n_targets, len(rest2))
    tgt_cells = rng.sample(rest2, n_targets)
    for (c, r) in tgt_cells:
        set_cell(c, r, OCC_TARGET, 15, 0)

    # 隔离线（仅用于打印/自检，**不进数组** —— 它是边）
    barrier_edge_cols = [c for c in range(BARRIER_COL_LO, BARRIER_COL_HI + 1)]

    return {
        'seed': seed,
        'cells': cells,
        'mines': [(c, r) for (c, r) in mine_cells],
        'mine_confs': {p: (cells[p[1] * COLS + p[0]] >> 3) & 0x0F
                       for p in mine_cells},
        'obstacles': obs_cells,
        'targets': tgt_cells,
        'gates': gate_rows,
        'barrier_edge_cols': barrier_edge_cols,
    }


# ==============================================================================
def ascii_map(m: dict) -> str:
    """把地图画成 ASCII，便于一眼核对

    ★ 对齐要点（这里踩过坑）：
      行号前缀必须是**固定 5 字符宽**（`'%2d   '`），与列标头的前缀
      （5 个空格）严格等宽。否则每行整体偏移一格，
      看起来像"所有坐标都错了一列" —— 本工具最初就是这样自我误导的。
    """
    cells = m['cells']
    lines = []
    # 列标头：5 空格前缀 + 每列 2 字符
    lines.append('     ' + ''.join('%2s' % col_letter(c) for c in range(COLS)))
    lines.append('     ' + '-' * (COLS * 2))
    gate_rows = m['gates']
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            v = cells[r * COLS + c]
            occ = v & 0x07
            ch = {OCC_UNKNOWN: '.', OCC_FREE: ' ', OCC_BLOCKED: '#',
                  OCC_MINE: '*', OCC_TARGET: 'T', OCC_GATE_UNK: 'x',
                  OCC_GATE_OPEN: 'O'}.get(occ, '?')
            if (c, r) == START:
                ch = 'S'
            elif (c, r) == GOAL:
                ch = 'G'
            row.append('%2s' % ch)
        lines.append('%2d   ' % (r + 1) + ''.join(row))
        # 隔离线：画在 row 与下一行之间（它是"边"）
        if r == BARRIER_UP_ROW:
            seg = []
            for c in range(COLS):
                seg.append('==' if BARRIER_COL_LO <= c <= BARRIER_COL_HI else '  ')
            lines.append('     ' + ''.join(seg)
                         + '  <== 隔离线(边) %s..%s'
                         % (col_letter(BARRIER_COL_LO), col_letter(BARRIER_COL_HI)))
    lines.append('')
    lines.append('    图例: S起点 G终点 ==隔离线(边) #障碍/隔板 *地雷 T靶标')
    lines.append('          x洞未知 O洞开 (空格)自由')
    lines.append('    门位: 区段一 K%d (row%d) / 区段二 K%d (row%d)'
                 % (gate_rows[1] + 1, gate_rows[1],
                    gate_rows[2] + 1, gate_rows[2]))
    return '\n'.join(lines)


def to_yaml(m: dict, name: str) -> str:
    """输出可直接使用的 YAML"""
    cells = m['cells']
    out = []
    out.append('# ============================================================')
    out.append('# 由 tools/gen_map.py --seed %d 生成（确定性，可复现）' % m['seed'])
    out.append('# ============================================================')
    out.append('name: "%s"' % name)
    out.append('')
    out.append('field:')
    out.append('  cols: %d' % COLS)
    out.append('  rows: %d' % ROWS)
    out.append('  cell_cm: %d' % CELL_CM)
    out.append('  barrier_up_row: %d' % BARRIER_UP_ROW)
    out.append('  barrier_col_lo: %d' % BARRIER_COL_LO)
    out.append('  barrier_col_hi: %d' % BARRIER_COL_HI)
    out.append('  bridge_col: %d' % BRIDGE_COL)
    out.append('  start: [%d, %d]' % START)
    out.append('  goal: [%d, %d]' % GOAL)
    out.append('')
    out.append('mines:')
    for (c, r) in m['mines']:
        conf = m['mine_confs'][(c, r)]
        out.append('  - [%d, %d, %d]    # %s' % (c, r, conf, cell_name(c, r)))
    out.append('t_mines: 0.5')
    out.append('')
    out.append('obstacles:')
    for (c, r) in m['obstacles']:
        out.append('  - [%d, %d, 1]    # %s' % (c, r, cell_name(c, r)))
    out.append('t_obstacles: 0.8')
    out.append('')
    out.append('targets:')
    for (c, r) in m['targets']:
        out.append('  - [%d, %d, 2, 95]    # %s  face=2 蓝/敌'
                   % (c, r, cell_name(c, r)))
    out.append('t_targets: 1.0')
    out.append('')
    out.append('gates:')
    out.append('  - [1, %d, 100, 1]    # 区段一 K%d'
               % (m['gates'][1], m['gates'][1] + 1))
    out.append('  - [2, %d, 100, 1]    # 区段二 K%d'
               % (m['gates'][2], m['gates'][2] + 1))
    out.append('t_gates: 1.2')
    out.append('')
    out.append('commands:')
    out.append('  - {t: 2.0,  cmd: nav_to, col: %d, row: %d}' % m['mines'][0]
               if m['mines'] else '  - {t: 2.0,  cmd: nav_to, col: 0, row: 0}')
    out.append('  - {t: 20.0, cmd: hold}')
    out.append('')
    out.append('# ---- 全图数组（154 个格，行优先：index = row*14 + col）----')
    out.append('#   解码: occ = v & 7 ; conf = (v>>3) & 15 ; age = (v>>7) & 15')
    out.append('map_full:')
    out.append('  map_version: 1')
    out.append('  cells:')
    for r in range(ROWS):
        chunk = cells[r * COLS:(r + 1) * COLS]
        out.append('    # row %d (%s%d..%s%d)  index %d..%d'
                   % (r, col_letter(0), r + 1, col_letter(COLS - 1), r + 1,
                      r * COLS, (r + 1) * COLS - 1))
        out.append('    ' + ', '.join('%d' % v for v in chunk) + ',')
    return '\n'.join(out)


# ==============================================================================
def main():
    p = argparse.ArgumentParser(
        description='随机地图生成器（输出数组 + YAML，可直接用于离线测试）')
    p.add_argument('--seed', type=int, default=1, help='随机种子（同种子=同地图）')
    p.add_argument('--mines', type=int, default=8, help='地雷数量（默认 8，赛题满分）')
    p.add_argument('--obstacles', type=int, default=8, help='障碍数量')
    p.add_argument('--targets', type=int, default=2, help='打击区格数')
    p.add_argument('--out', default='', help='把 YAML 写到该文件')
    p.add_argument('--no-ascii', action='store_true', help='不打印 ASCII 图')
    args = p.parse_args()

    # 门位：区段一/二各随机一格
    rng = random.Random(args.seed * 7919)
    gate_a = rng.randint(BRIDGE_A[0], BRIDGE_A[1])
    gate_b = rng.randint(BRIDGE_B[0], BRIDGE_B[1])

    m = gen_map(args.seed, args.mines, args.obstacles, args.targets,
                (gate_a, gate_b))

    print('=' * 78)
    print(' 随机地图（seed=%d）' % args.seed)
    print('=' * 78)
    print('  网格      : %d 列 × %d 行 = %d 格' % (COLS, ROWS, COLS * ROWS))
    print('  格边长    : %d cm' % CELL_CM)
    print('  隔离线    : row%d 与 row%d 之间，覆盖 col %d..%d (%s..%s)'
          % (BARRIER_UP_ROW + 1, BARRIER_UP_ROW + 2, BARRIER_COL_LO,
             BARRIER_COL_HI, col_letter(BARRIER_COL_LO),
             col_letter(BARRIER_COL_HI)))
    print('  桥洞列    : col %d (%s)' % (BRIDGE_COL, col_letter(BRIDGE_COL)))
    print('  区段一门  : row %d (K%d)' % (gate_a, gate_a + 1))
    print('  区段二门  : row %d (K%d)' % (gate_b, gate_b + 1))
    print('  起点/终点 : %s / %s' % (cell_name(*START), cell_name(*GOAL)))
    print('  地雷 %d / 障碍 %d / 打击区 %d'
          % (len(m['mines']), len(m['obstacles']), len(m['targets'])))
    print()

    if not args.no_ascii:
        print(ascii_map(m))
        print()

    # ---------------------------------------------------------------- 数组形式
    print('=' * 78)
    print(' cells 数组（%d 个整数，行优先：index = row * %d + col）' % (len(m['cells']), COLS))
    print('=' * 78)
    cells = m['cells']
    for r in range(ROWS):
        chunk = cells[r * COLS:(r + 1) * COLS]
        print('  # row %2d (%s%d..%s%d)' % (r, col_letter(0), r + 1,
                                            col_letter(COLS - 1), r + 1))
        print('  ' + ', '.join('%3d' % v for v in chunk) + ',')
    print()

    # ---------------------------------------------------------------- 检查
    print('=' * 78)
    print(' 自洽性检查')
    print('=' * 78)
    ok_all = True

    def chk(cond, what):
        nonlocal ok_all
        print('  %s %s' % ('[OK]' if cond else '[XX]', what))
        if not cond:
            ok_all = False

    chk(len(cells) == COLS * ROWS, 'cells 数量 == %d' % (COLS * ROWS))

    # ★ 隔离线是"边"，不在数组里；数组里 row5 的格除以下情况外应为【自由】：
    #     · 桥洞列 K（隧道格 = 洞未知/洞开）
    #     · 随机障碍格（OCC_BLOCKED）
    #     · 起终点 / 无人机起降点
    #   —— 所以这里只校验"不是未知(kUnknown)"，而不是"必须自由"。
    row5_unknown = [col_letter(c) for c in range(BARRIER_COL_LO, BARRIER_COL_HI + 1)
                    if (cells[BARRIER_UP_ROW * COLS + c] & 7) == OCC_UNKNOWN]
    chk(not row5_unknown,
        '隔离线所在 row%d 没有"未知"格（未知=不可通行，会误挡路）%s'
        % (BARRIER_UP_ROW + 1,
           '' if not row5_unknown else '  ★ ' + ','.join(row5_unknown)))
    # 桥洞列
    gates_open = sum(1 for r in range(ROWS)
                     if (cells[r * COLS + BRIDGE_COL] & 7) == OCC_GATE_OPEN)
    chk(gates_open == 2, '桥洞列恰好 2 个门（实际 %d）' % gates_open)
    gate_unk = sum(1 for r in range(ROWS)
                   if (cells[r * COLS + BRIDGE_COL] & 7) == OCC_GATE_UNK)
    chk(gate_unk == ROWS - 2, '桥洞列其余 %d 格是洞未知（实际 %d）'
        % (ROWS - 2, gate_unk))
    # 起终点
    chk((cells[START[1] * COLS + START[0]] & 7) == OCC_FREE,
        '%s 起点是自由格' % cell_name(*START))
    chk((cells[GOAL[1] * COLS + GOAL[0]] & 7) == OCC_FREE,
        '%s 终点是自由格（★ 隔离线不得把它挡住）' % cell_name(*GOAL))
    # 编码范围
    chk(all(0 <= v <= 0x7FF for v in cells), '所有编码在 11 位内（<= 0x7FF）')
    # 编码自洽：解码回来与设定一致
    bad_dec = 0
    for v in cells:
        occ = v & 7
        if occ not in OCC_NAMES:
            bad_dec += 1
    chk(bad_dec == 0, '所有 occ 码都在 0..6 范围内')
    # ★ 无人机起降点必须是禁入（阻挡）
    for nm, (c, r) in (('无人机起点 ' + cell_name(*DRONE_START), DRONE_START),
                       ('无人机终点 ' + cell_name(*DRONE_END), DRONE_END)):
        occ = cells[r * COLS + c] & 7
        chk(occ == OCC_BLOCKED,
            '%s 是禁入格（occ=%d %s）' % (nm, occ, OCC_NAMES.get(occ, '?')))
    # 无人机位不得与起终点重合
    chk(DRONE_START not in (START, GOAL) and DRONE_END not in (START, GOAL),
        '无人机起降点不与智能车起终点重合')
    # 地雷/障碍不得落在无人机位上
    reserved_pts = {DRONE_START, DRONE_END, START, GOAL}
    chk(not (set(m['mines']) & reserved_pts),
        '地雷不落在起终点/无人机位上')
    chk(not (set(m['obstacles']) & reserved_pts),
        '障碍不落在起终点/无人机位上')

    print()
    print('  结论: %s' % ('地图自洽 ✓' if ok_all else '★ 有问题'))

    # ---------------------------------------------------------------- YAML
    yaml_text = to_yaml(m, '随机局 seed=%d' % args.seed)
    print()
    print('=' * 78)
    print(' YAML（可存成 tools/maps/random_%d.yaml 直接用）' % args.seed)
    print('=' * 78)
    print(yaml_text)

    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(yaml_text + '\n')
        print()
        print('  已写入: %s' % args.out)

    return 0 if ok_all else 1


if __name__ == '__main__':
    sys.exit(main())
