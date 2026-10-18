# -*- coding: utf-8 -*-
"""
无人机全局感知 + 地面小车精确执行 —— 分块建图 / 路径规划 / 航点下发
====================================================================
整体目标(裁判系统随机生成动态地图, 3分钟内完成):
  区域侦察、地雷排除、敌我识别与打击、障碍穿越、桥洞穿梭。
  无人机负责全场态势感知(起飞巡航/建图/识别), 小车依据信息完成地面执行。

无人机 · 全局感知:
  1. 自主起飞(飞行限高 1.5m), 起飞完成后开始场地巡航; 扫描路线为 U 形左右扫描
     (左右扫描先于向前推进: 每一行带左右扫完后再前进):
     先扫 N11~D7 右侧区域(从 N11 出发左右蛇形扫描、自下而上推进)
     -> 再扫 C11~A1 顶部区域(每行带从 11 向 1 逐步扫描)
     -> 最后扫 D1~N6 左侧区域(左右蛇形扫描、向底部推进, 止于 N6);
     起飞后单次可扫描 2行x3列 范围, 按此范围分块扫描建图(全场抽象为
     14x11 二维矩阵, 扫到哪块填充哪块), 每块扫描后立即重规划;
     无人机扫描始终快于路径规划、保持在小车前方: 扫描按 U 形 S 型逐块衔接
     (扫描一块连接一块, 相邻衔接、不重复路线, 不做全局跳转规划), 未扫描区域
      小车不得驶入; 路径规划优先把已扫描区域规划进去
      (未知格附加代价 UNKNOWN_PEN), 每块扫描后立即重规划、不锁定死路径;
  2. 识别地雷准确位置(8 个方格, 生成在 J~A 行之间, 尽量全踩排雷; 死路地雷可放弃);
  3. 识别敌我打击区: 相连的两个横向或纵向单元内放 1 个双面移动电子靶标
     (蓝面=敌方/红面=友方, 朝向随机摆放); 蓝面必须激光打击, 红面不打、
     且小车不得碾压靶体(视为实体障碍绕行);
  4. 识别障碍物(8 个装满水的彩色带盖水桶, 直径26cm/高25cm, 占一格禁入,
     生成在 J~A 行之间);
  5. 桥洞 K1~K11(高35cm, 隔板隔离): 官方为上下两个桥洞通道, 下段 K1~K5 5选1、
     上段 K6~K11 6选1, 裁判系统在每处通道随机指定 1 个可通行通道, 桥梁顶部
     箭头标明通行状态(地面车看不见, 须无人机高空侦察); 每通过一个通道 +10 分、
     每个通道只计一次; 其余桥体格禁止穿行;
  6. 动态地图: 任务中途新障碍出现, 无人机发现后立即动态重规划;
  7. 中场横向挡板(官方: D7~N7, 第 6/7 行边界; 黄色挡板+白色单面胶长条,
     "用于阻挡小车穿行"): 覆盖 D~N 列、严禁跨越; A/B/C 列无挡板自由穿越;
     挡板只拦穿越、不占格。E 版坐标系为官方转置
     (行=字母/列=数字), 即第6/7列之间 D~N 行禁跨;
  8. 时序: 无人机先在 N8 起飞 -> 开始巡航扫描 -> 扫描 4 块后小车发车(与 E 版原版
     时序一致) -> 边扫边走: 规划出一部分就跑一部分, 路径规划无需敌打击区/完整路径
     -> 每块扫描后重规划, 小车从当前格继续跑新计划 -> 小车先行到达终点
     N5(到达即结束, 不要求全部完成; 到达前尽量完整: 边扫边重规划尽力补全
     排雷/打击/避障) -> 无人机随后返航降落于终点 M2(晚于小车); 全程总耗时 3 分钟。

地面小车 · 精确执行:
  0.3m/s 直行(0.35/0.30≈1.17s/格); 无法原地拐 90°(三点调头: 减速+倒车重进,
  90°拐弯≈5.6s, 180°掉头×3.0), 路径规划加大转弯惩罚、自动减少拐弯并尽量避免掉头;
  路径沿格心到格心行驶, 爆炸物位于格子正中央,
  小车必须压过正中心完成排雷; 自主导航按航点(含格心坐标)行驶; 全场全踩排雷
  (死路地雷可放弃); 对敌打击区停靠蓝面对面、车头对准蓝面激光打击
  (官方 10 发, 仿真按 3 发×10s 标定); 自主避障(动态重规划); 桥洞可通行通道穿梭。

技术: 核心仅标准库(heapq/json/socket/time/math/collections);
     cv2/numpy 仅用于可视化与相机视觉, 未安装时自动降级 ASCII 仿真。

运行:
  python mission_planner.py                     # 纯标准库 ASCII 仿真
  python mission_planner.py --gui               # cv2 可视化(需 opencv-python)
  python mission_planner.py --link tcp          # TCP 下发航点(小车端另跑 car_client_demo.py)
  python mission_planner.py --cam 0             # 真实相机 + 霍夫圆/蓝色识别
"""

import argparse
import heapq
import json
import math
import os
import random
import socket
import sys
import time

try:
    import cv2
    import numpy as np
    HAS_CV = True
except Exception:
    HAS_CV = False

# ============================ 场地常量 ============================
ROW_LETTERS = "ABCDEFGHIJKLMN"
ROWS, COLS = 14, 11                 # 14 行(A~N) x 11 列(1~11)
TILE_CM = 35.0                      # 每格 35cm
START_TILE, RETURN_TILE = "N10", "N5"   # 小车起点 / 终点
DRONE_START_TILE = "N8"                 # 无人机起飞点
DRONE_END_TILE = "M2"                   # 无人机终点(降落点)
# ---- 中场横向挡板(官方: 黄色挡板+白色单面胶长条, 03 §6"用于阻挡小车穿行") ----
# 官方口径(2026-10-17 终裁, 与桌面包现场核实一致): 横向挡板 D7~N7, 位于第 6/7 行
# 边界(行=数字轴 1~11), 覆盖 D~N 列(列=字母轴 A~N), 严禁跨越; A/B/C 走廊可穿线
# (无挡板); 只拦穿越、不占格。
# E 版坐标系为官方场地图的转置(行=字母 A~N、列=数字 1~11):
#   官方"第 6/7 行边界" == E 版"第 6/7 列(数字轴)之间"
#   官方"覆盖 D~N 列"   == E 版"覆盖 D~N 行(字母轴)"
BARRIER_COLS = (6, 7)               # 挡板线在第6/7行边界(官方行=数字轴; E版=第6/7列之间, 索引5↔6), 严禁跨越
BARRIER_LO_ROW, BARRIER_HI_ROW = "D", "N"   # 挡板覆盖 D7~N7(官方列=字母轴; E版=字母行 D~N)
BARRIER_FREE_ROWS = ("A", "B", "C")         # A/B/C 走廊可穿线(无挡板)

UNKNOWN, EMPTY, BUCKET, EXPLOSIVE, TARGET, FRIEND, LINE = 0, 1, 2, 3, 4, 5, 6
SYM = {UNKNOWN: ".", EMPTY: " ", BUCKET: "#", EXPLOSIVE: "x",
       TARGET: "T", FRIEND: "F", LINE: "|"}

DIR4 = ((-1, 0, "N"), (1, 0, "S"), (0, 1, "E"), (0, -1, "W"))
DIR_NAME = {(-1, 0): "N", (1, 0): "S", (0, 1): "E", (0, -1): "W"}
NAME_DIR = {"N": (-1, 0), "S": (1, 0), "E": (0, 1), "W": (0, -1)}
ALL_DIRS = ((0, 0), (-1, 0), (1, 0), (0, 1), (0, -1))

# -------------------- 任务与时间参数(按比赛规则标定) --------------------
TOTAL_BUDGET_S = 180.0              # 整体目标: 3 分钟内完成
SOFT_BUDGET_S = 160.0               # 软预算(留 20s 余量: 重规划/动态障碍/预估误差);
                                    # 预估总耗时超软预算时按"分/秒"效率逐项放弃任务
TAKEOFF_S = 2.0                     # 起飞耗时; 扫描在起飞完成后才开始
DRONE_MAX_ALT_M = 1.5               # 无人机飞行限高 1.5m
DRONE_SPEED_MS = 0.8                # 无人机飞行速度 0.8 m/s(块间转场/返航按此速度计时)
DEPART_AFTER_BLOCKS = 4             # 发车条件: 扫描 4 块后发车(与 E 版原版时序一致)
BLOCK_SCAN_S = 0.6                  # 每块悬停扫描耗时(仿真)
CAR_SPEED_MS = 0.30                 # 小车速度 0.3 m/s
CELL_TIME_S = (TILE_CM / 100.0) / CAR_SPEED_MS      # ≈1.17s/格
TURN_MAX_DEG = 35.0                 # 行进中单次转向最大转角 35°(无法原地拐90°)
TURN_RATE_DEG_S = 35.0              # 转向速率 35°/s (90°≈2.57s, 相当于 3 次≤35°转向动作)
TURN_DECEL_S = 0.5                  # 拐弯减速: 入弯刹车+出弯提速附加耗时
TURN_THREE_POINT_S = 2.5            # 无法原地拐90°: 三点调头(倒车+重进)附加耗时
TURN_TIME_S = 90.0 / TURN_RATE_DEG_S + TURN_DECEL_S + TURN_THREE_POINT_S  # 90°拐弯≈5.6s
U_TURN_FACTOR = 3.0                 # 180°掉头≈3次三点调头, 规划大幅加重惩罚以避免掉头
FACING_PAUSE_S = 1.0                # 蓝面停靠确认(识别敌我)
FALL_TIMES = 3                      # 仿真打击发数(官方: 10 发激光、每2次有效命中计数+1、计数3=击毁; 本仿真按 3 发×10s 简化标定)
STRIKE_TIME_S = 30.0                # 打靶总耗时 30 秒
FALL_INTERVAL = STRIKE_TIME_S / FALL_TIMES   # 每发间隔(30s 内打 3 发)
# ---- 计分模型(2026-10-17 确认口径: 规定时间内得分最高) ----
SCORE_BRIDGE = 10.0                  # 正确通过桥洞一次 +10(每处只计一次, 两处最多 20)
SCORE_MINE = 5.0                     # 踩过一次爆炸物 +5(共 8 颗, 最多 40)
SCORE_HIT = 5.0                      # 打靶命中一次 +5(每发命中计一次, 3 发最多 15)
TURN_PEN = TURN_TIME_S / CELL_TIME_S   # A* 转弯惩罚(折算成格数)≈4.8
UNKNOWN_PEN = 2.0                     # 未知格附加代价: 规划优先把已扫描区域规划进去
BUCKETS_N = 8                       # 官方: 8 个障碍物区域
EXPLOSIVES_N = 8                    # 官方: 8 个地雷
OBJECT_ROWS = ("J", "A")            # 障碍物/爆炸物生成在 J~A 行之间(字母轴, 用户口径)
TARGET_CELLS = 2                    # 官方 02 §三(二)2: 打击区域=相连的两个横向或纵向单元
TARGET_ROWS = ("J", "A")            # 标靶(打击区)位于 J~A 行之间(字母轴, 用户口径)
BRIDGE_HEIGHT_CM = 35.0             # 桥洞高35cm(无人机过桥注意高度)
# 官方 02 §三(二)4: 上下两个桥洞通道, 各通道之间隔板隔离; 下面的桥洞通道占
# 5 个格子(K1~K5)、上面的占 6 个格子(K6~K11); 裁判系统在每处通道随机指定
# 1 个可通行通道, 桥梁顶部箭头标明通行状态(地面车看不见, 须无人机高空侦察);
# 每通过一个桥洞通道 +10 分, 每个通道只计一次分。其余桥体格禁止穿行。
BRIDGE_CELLS = ["K" + str(i) for i in range(1, 12)]   # 桥洞结构 K1~K11(仅可通行通道可穿)
BRIDGE_LO_RANGE = ("K1", "K5")      # 下段桥洞通道 5 选 1(挡板线下方)
BRIDGE_HI_RANGE = ("K6", "K11")     # 上段桥洞通道 6 选 1(挡板线上方)
BRIDGE_ARROWS = True                # 桥洞顶部通行状态标识(箭头标明哪格可通行, 无方向语义)
DYNAMIC_OBSTACLE = True             # 动态地图: 任务中途出现新障碍
OBSTACLE_D_CM, OBSTACLE_H_CM = 26.0, 25.0   # 8 个装满水的彩色带盖水桶(直径26cm/高25cm, 占一格; 官方03布置说明书+现场物料口径)
BLOCK_ROWS, BLOCK_COLS = 2, 3       # 起飞后单次扫描范围: 2行x3列 = 28 块
EARLY_START = True                  # 路径规划无需敌打击区; 第一次规划完整后即可发车(边扫边走)


# ============================ 基础工具 ============================
def name2rc(name):
    return ROW_LETTERS.index(name[0]), int(name[1:]) - 1


def rc2name(rc):
    return ROW_LETTERS[rc[0]] + str(rc[1] + 1)


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def man(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def turn_pen_factor(d_in, d_out):
    """拐弯代价系数: 起步/直行 0; 90°拐弯 1; 180°掉头 U_TURN_FACTOR(无法原地拐弯, 掉头最慢)"""
    if d_in == (0, 0) or d_in == d_out:
        return 0.0
    if d_out == (-d_in[0], -d_in[1]):
        return U_TURN_FACTOR
    return 1.0


def barrier_blocks_cross(a, b):
    """官方横向挡板 D7~N7(第 6/7 行边界, 黄色挡板+白色单面胶长条): "用于阻挡
    小车穿行"(03 §6)。E 版坐标系(转置): 挡板线在数字轴 5↔6 之间, 字母 D~N 严禁跨越;
    A/B/C 走廊可穿线。只拦穿越、不占格。返回 a→b 是否跨越挡板。"""
    lo = ROW_LETTERS.index(BARRIER_LO_ROW)
    hi = ROW_LETTERS.index(BARRIER_HI_ROW)
    if not (min(lo, hi) <= a[0] <= max(lo, hi)
            and min(lo, hi) <= b[0] <= max(lo, hi)):
        return False
    c_lo = min(BARRIER_COLS) - 1    # 第6列 = 索引5
    c_hi = max(BARRIER_COLS) - 1    # 第7列 = 索引6
    return {a[1], b[1]} == {c_lo, c_hi}


def green_line_cells():
    """兼容接口: 旧版"绿线整格禁入"已废弃(官方挡板只拦穿越), 返回空集。"""
    return []


def make_blocks():
    """全场划分为 BLOCK_ROWS x BLOCK_COLS 的小块(无人机分块扫描单元)"""
    blocks = []
    r = 0
    while r < ROWS:
        r1 = min(r + BLOCK_ROWS, ROWS)
        c = 0
        while c < COLS:
            c1 = min(c + BLOCK_COLS, COLS)
            blocks.append([(rr, cc) for rr in range(r, r1) for cc in range(c, c1)])
            c = c1
        r = r1
    return blocks


def scan_block_order(blocks):
    """U 形单条连续 S 曲线(每块区域以各自底边为起点, 一条连续曲线扫完):
    1) 右侧区域: 以 N7~N11 为底边, 从 N11 出发蛇形自下而上(左右扫描先于推进);
    2) 顶部区域: 以 A11~C11 为底边, 从 C11 出发每行带 11->1 连续扫到 A1;
    3) 左侧区域: 以 D1~D6 为底边, 从 D1 出发蛇形向底部推进, 止于 N6 附近"""
    def key(b):
        rr = sum(rc[0] for rc in b) / len(b)
        cc = sum(rc[1] for rc in b) / len(b)
        if rr >= 4.5:                        # E~N 行
            if cc >= 6:                      # 右侧区域(N7~N11 底边)
                rank = int(round((12.5 - rr) / 2.0))
                return (0, rank, -cc if rank % 2 == 0 else cc)
            rank = int(round((rr - 4.5) / 2.0))   # 左侧区域(蛇形向底部)
            return (2, rank + 1, cc if rank % 2 == 0 else -cc)
        if rr >= 2.0:                        # C~D 行带
            if cc >= 6:                      # 顶部区域先扫(C11 底边锚点)
                return (1, 0, -cc)
            return (2, 0, cc)                # 左侧区域先扫(D1~D6 底边)
        return (1, 1, -cc)                   # A~B 行: 顶部区域后扫, 每行带 11->1
    return sorted(blocks, key=key)


# ============================ 二维矩阵 ============================
class Field:
    """整个场地抽象成的二维矩阵; 无人机扫描到哪块, 就填充哪块"""

    def __init__(self):
        self.grid = [[{"state": UNKNOWN, "known": False, "blue": None}
                      for _ in range(COLS)] for _ in range(ROWS)]
        # 官方挡板只拦穿越不占格: 不再预置禁入格(旧版绿线整格禁入已废弃)
        self.bridge = {name2rc(n) for n in BRIDGE_CELLS}   # 桥洞结构格(K1~K11)
        self.open_tunnels = set()      # 已识别的可通行通道(顶部箭头标明, 须无人机侦察)

    def is_bridge(self, rc):
        return rc in self.bridge

    def set(self, rc, state, blue=None):
        cell = self.grid[rc[0]][rc[1]]
        cell["state"] = state
        cell["known"] = True
        cell["blue"] = blue

    def state(self, rc):
        return self.grid[rc[0]][rc[1]]["state"]

    def is_known(self, rc):
        return self.grid[rc[0]][rc[1]]["known"]

    def blue(self, rc):
        return self.grid[rc[0]][rc[1]]["blue"]

    def target_rc(self):
        """第一个敌方打击区(带蓝面)"""
        for r in range(ROWS):
            for c in range(COLS):
                if self.grid[r][c]["state"] == TARGET:
                    return (r, c)
        return None

    def known_of(self, state):
        return [(r, c) for r in range(ROWS) for c in range(COLS)
                if self.grid[r][c]["state"] == state]


# ============================ 地面真值(仿真用) ============================
def _object_rows():
    """水桶/爆炸物所在行范围(A~J)"""
    r0 = ROW_LETTERS.index(OBJECT_ROWS[0])
    r1 = ROW_LETTERS.index(OBJECT_ROWS[1])
    return min(r0, r1), max(r0, r1)


def _truth_ok(truth, tunnels):
    """校验: 起点能到达所有敌方/地雷/返程点; 敌方蓝面停靠位可用"""
    bridge = {name2rc(n) for n in BRIDGE_CELLS}
    blocked = {rc for rc, info in truth.items() if info[0] == BUCKET} \
        | (bridge - set(tunnels))
    from collections import deque
    start = name2rc(START_TILE)
    seen = {start}
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc, _h in DIR4:
            nb = (r + dr, c + dc)
            if not (0 <= nb[0] < ROWS and 0 <= nb[1] < COLS):
                continue
            if nb in blocked or nb in seen:
                continue
            if barrier_blocks_cross((r, c), nb):
                continue
            seen.add(nb)
            q.append(nb)
    if name2rc(RETURN_TILE) not in seen:
        return False
    for rc, info in truth.items():
        if info[0] in (TARGET, EXPLOSIVE) and rc not in seen:
            return False
        if info[0] == TARGET:            # 敌方蓝面停靠位必须可用
            blue = info[1]
            if blue is None:
                return False
            s = (rc[0] + blue[0], rc[1] + blue[1])
            if not (0 <= s[0] < ROWS and 0 <= s[1] < COLS):
                return False
            if s in truth or s in blocked or s in (start, name2rc(RETURN_TILE)):
                return False
    return True


def build_truth(rng):
    """随机动态地图(裁判系统风格, 官方 02/03 规则):
    8 雷 + 8 障碍全场随机(起终点/桥体除外); 打击区域=相连两个单元内的
    1 个双面移动电子靶标(蓝面=敌方/红面=友方, 朝向随机); 桥洞上下通道
    各随机指定 1 个可通行通道(下段 K1~K5 5选1 / 上段 K6~K11 6选1),
    顶部箭头标明通行状态(地面车看不见, 须无人机侦察)"""
    for _attempt in range(300):
        start_rc = name2rc(START_TILE)
        ret_rc = name2rc(RETURN_TILE)
        kr = ROW_LETTERS.index("K")
        bridge_set = {name2rc(n) for n in BRIDGE_CELLS}
        used = {start_rc, ret_rc} | bridge_set
        truth = {}
        # 1) 打击区域: 相连的两个横向或纵向单元, 单局仅 1 个; 双面靶当前占
        #    其中 1 格, 蓝面朝向随机(4 方向), 停靠位可用; 另一格为空地
        t_r0 = ROW_LETTERS.index(TARGET_ROWS[0])
        t_r1 = ROW_LETTERS.index(TARGET_ROWS[1])
        free = [(r, c) for r in range(ROWS) for c in range(COLS) if (r, c) not in used]
        free_set = set(free)
        rng.shuffle(free)
        placed = False
        for t in free:
            if not (min(t_r0, t_r1) <= t[0] <= max(t_r0, t_r1)):
                continue                        # 打击区域在目标行范围内
            dirs = [d[:2] for d in DIR4]
            rng.shuffle(dirs)
            for dd in dirs:
                t2 = (t[0] + dd[0], t[1] + dd[1])
                if t2 not in free_set:
                    continue
                if not (min(t_r0, t_r1) <= t2[0] <= max(t_r0, t_r1)):
                    continue                        # 打击区域整体(两格)都在 J~A 行之间
                blues = [d[:2] for d in DIR4]
                rng.shuffle(blues)
                blue = None
                for b in blues:
                    s = (t[0] + b[0], t[1] + b[1])
                    if 0 <= s[0] < ROWS and 0 <= s[1] < COLS \
                            and s in free_set and s != t2:
                        blue = b
                        break
                if blue is None:
                    continue
                truth[t] = (TARGET, blue)          # 双面靶当前格(蓝面朝向随机)
                used.add(t)
                used.add(t2)
                placed = True
                break
            if placed:
                break
        if not placed:
            continue                               # 重新生成
        # 2) 爆炸物: 8 个, 生成在 J~A 行之间(用户口径; K 列桥体与起终点除外)
        or0, or1 = _object_rows()
        obj_free = [(r, c) for r in range(ROWS) for c in range(COLS)
                    if (r, c) not in used and or0 <= r <= or1]
        rng.shuffle(obj_free)
        for rc in obj_free[:EXPLOSIVES_N]:
            truth[rc] = (EXPLOSIVE, None)
            used.add(rc)
        # 3) 障碍物: 8 个水桶, 生成在 J~A 行之间(用户口径)
        obj_free2 = [(r, c) for r in range(ROWS) for c in range(COLS)
                     if (r, c) not in used and or0 <= r <= or1]
        rng.shuffle(obj_free2)
        for rc in obj_free2[:BUCKETS_N]:
            truth[rc] = (BUCKET, None)
            used.add(rc)
        # 4) 桥洞可通行通道: 下段 K1~K5 5选1、上段 K6~K11 6选1(官方正文);
        #    通道及左右出口(J/K/L)避开障碍物, 保证小车能穿行
        tunnels = set()
        if BRIDGE_ARROWS:
            lo0, lo1 = name2rc(BRIDGE_LO_RANGE[0])[1], name2rc(BRIDGE_LO_RANGE[1])[1]
            hi0, hi1 = name2rc(BRIDGE_HI_RANGE[0])[1], name2rc(BRIDGE_HI_RANGE[1])[1]

            def ok_col(c):
                for r in (kr - 1, kr, kr + 1):       # 整条穿行通道(J/K/L)避开障碍
                    info = truth.get((r, c))
                    if info is not None and info[0] in (BUCKET, TARGET):
                        return False
                return True

            los = [c for c in range(min(lo0, lo1), max(lo0, lo1) + 1) if ok_col(c)]
            his = [c for c in range(min(hi0, hi1), max(hi0, hi1) + 1) if ok_col(c)]
            if los and his:
                tunnels.add((kr, rng.choice(los)))
                tunnels.add((kr, rng.choice(his)))
        if _truth_ok(truth, tunnels):
            return truth, {rc: True for rc in tunnels}
    return truth, {}

# ============================ 路径规划器 ============================
class Planner:
    def __init__(self, field):
        self.field = field
        self.scan_complete = False   # 全场扫描是否完成(扫描中: 雷只需当前可达即可纳入)

    def blocked_set(self):
        """障碍/我方禁入 + 桥洞结构格(除已识别可通行通道)禁入;
        挡板只拦穿越(见 barrier_blocks_cross), 不占格。"""
        return ({(r, c) for r in range(ROWS) for c in range(COLS)
                 if self.field.grid[r][c]["state"] in (BUCKET, FRIEND, TARGET)}
                | (self.field.bridge - self.field.open_tunnels))

    def a_star(self, start, goal, turn_pen=TURN_PEN, extra_avoid=()):
        """带转弯代价的 A*: 障碍/我方区/桥洞结构禁入; 官方中场横向挡板(D~N 行
        第6/7列之间)禁止跨越; 已扫描区域优先规划进去
        (未知格附加 UNKNOWN_PEN 代价), 每块扫描后重规划、不锁定死路径"""
        if start == goal:
            return [start]
        blocked = self.blocked_set() | set(extra_avoid)
        g = {(start[0], start[1], 0, 0): 0.0}
        came = {}
        heap = [(man(start, goal), 0.0, start[0], start[1], 0, 0)]
        while heap:
            f, _, r, c, dr, dc = heapq.heappop(heap)
            if (r, c) == goal:
                path = [(r, c)]
                node = (r, c, dr, dc)
                while node in came:
                    pr, pc, pdr, pdc = came[node]
                    path.append((pr, pc))
                    node = (pr, pc, pdr, pdc)
                return path[::-1]
            for ndr, ndc, _h in DIR4:
                nr, nc = r + ndr, c + ndc
                if not (0 <= nr < ROWS and 0 <= nc < COLS):
                    continue
                if (nr, nc) in blocked:
                    continue
                if barrier_blocks_cross((r, c), (nr, nc)):
                    continue      # 官方横向挡板: D~N 行严禁跨越第6/7列(A/B/C 走廊可穿线)
                extra = turn_pen * turn_pen_factor((dr, dc), (ndr, ndc))
                unknown_pen = (0.0 if self.field.is_known((nr, nc))
                               else UNKNOWN_PEN)
                ng = g[(r, c, dr, dc)] + 1.0 + extra + unknown_pen
                key = (nr, nc, ndr, ndc)
                if ng < g.get(key, 1e18):
                    g[key] = ng
                    came[key] = (r, c, dr, dc)
                    heapq.heappush(heap, (ng + man((nr, nc), goal), ng, nr, nc, ndr, ndc))
        return None

    def shoot_candidates(self, t, blue):
        """蓝色面对面停靠位: 优先蓝面正对格, 其次其他 3 邻域(被挡时退避)"""
        order = []
        if blue is not None:
            order.append(tuple(blue))
        for d in DIR4:
            if tuple(d[:2]) not in order:
                order.append(d[:2])
        blocked = self.blocked_set()
        out = []
        for dr, dc in order:
            s = (t[0] + dr, t[1] + dc)
            if 0 <= s[0] < ROWS and 0 <= s[1] < COLS and s not in blocked and s != t:
                out.append(s)
        return out

    def strike_cell(self, cur, t):
        """为敌方打击区选择蓝面停靠位(严格优先蓝面正对格)"""
        blue = self.field.blue(t)
        cands = self.shoot_candidates(t, blue)
        chosen = None
        if blue is not None:
            s_blue = (t[0] + blue[0], t[1] + blue[1])
            if s_blue in cands and self.a_star(cur, s_blue) is not None:
                chosen = s_blue
        if chosen is None:
            best = None
            for s in cands:
                p = self.a_star(cur, s)
                if p is None:
                    continue
                cost = route_cost(p)
                if best is None or cost < best[0]:
                    best = (cost, s)
            chosen = best[1] if best else None
        return chosen

    def is_dead_end_mine(self, e):
        """死路地雷: 四周可通邻格 ≤1 个(进出只能原路折返, 必然掉头) -> 可以放弃"""
        blocked = self.blocked_set()
        cnt = 0
        for dr, dc, _h in DIR4:
            nb = (e[0] + dr, e[1] + dc)
            if 0 <= nb[0] < ROWS and 0 <= nb[1] < COLS and nb not in blocked:
                cnt += 1
        return cnt <= 1

    STRIKE_NODE = ("strike",)          # 打靶节点标记: 可穿插在踩爆炸物过程中

    def explosive_order(self, start, explosives, stop_cell, strike_pair=None):
        """地雷排雷 + 打靶穿插: 带转弯代价的精确 TSP(状态压缩 DP)
        状态 = (已访问集合, 最后节点, 进入该节点的方向), 转弯在节点处结算。
        死路地雷(进出必须掉头)直接放弃; 打靶不是最终任务, 作为节点与地雷
        一起参加排序, 可穿插在踩爆炸物过程中。"""
        reachable = []
        for e in explosives:
            if self.a_star(start, e) is None:
                print(f"[planner] 地雷 {rc2name(e)} 被围死不可达, 跳过")
            elif self.is_dead_end_mine(e):
                print(f"[planner] 死路地雷 {rc2name(e)} 放弃(进出需掉头)")
            elif not self.scan_complete:
                # 扫描进行中: 只要当前可达就纳入(终点方向的通路待扫描补全,
                # 边扫边走——能踩的先踩, 不因终点暂不可达而闲着)
                reachable.append(e)
            elif self.a_star(e, stop_cell) is None:
                print(f"[planner] 地雷 {rc2name(e)} 到终点不可达, 跳过")
            else:
                reachable.append(e)
        # 打靶节点: (停靠位s或None, 敌方格t); 标靶格无法通行, 打击在停靠位原地完成,
        # s 与 t 必相邻, s->t 段不再行驶(虚拟连接), 不因 t 禁入而丢失打靶
        s = t = None
        p_st = None
        has_strike = False
        if strike_pair:
            s, t = strike_pair
            if s is not None:
                if man(s, t) == 1:
                    p_st = [s, t]                    # 相邻: 停靠位正对蓝面, 原地打击
                    has_strike = True
                else:
                    p_st = self.a_star(s, t)
                    has_strike = p_st is not None
            else:
                has_strike = self.a_star(t, stop_cell) is not None
        nodes = list(reachable)
        if has_strike:
            if s is not None:
                nodes.append(self.STRIKE_NODE)       # 打靶对(S->T)作为整体节点
            else:
                nodes.append(t)                      # 已停靠过, 只剩打击格
        n = len(nodes)
        if n == 0:
            return []
        keys = {start, stop_cell}
        keys.update(e for e in reachable)
        if has_strike:
            if s is not None:
                keys.add(s)
            keys.add(t)
        paths = {}
        for a in keys:
            for b in keys:
                if a != b:
                    paths[(a, b)] = self.a_star(a, b)

        def seg_info(p):
            if p is None:
                return None
            d_out = sub(p[1], p[0]) if len(p) > 1 else (0, 0)
            d_in = sub(p[-1], p[-2]) if len(p) > 1 else (0, 0)
            return route_cost(p), d_out, d_in

        def info(a, b):
            if b == self.STRIKE_NODE:
                p1 = paths.get((a, s))
                i1 = seg_info(p1)
                ist = seg_info(p_st)
                if i1 is None or ist is None or len(p1) < 2:
                    return None
                c1, dout, din_s = i1
                c_st, d_st_out, d_st_in = ist
                extra = TURN_PEN * turn_pen_factor(din_s, d_st_out)
                return c1 + extra + c_st, dout, d_st_in
            if a == self.STRIKE_NODE:
                return seg_info(paths.get((t, b)))
            return seg_info(paths.get((a, b)))

        INF = float("inf")
        dp = {}
        par = {}
        for i in range(n):
            inf = info(start, nodes[i])
            if inf is None:
                continue
            c, dout, din = inf
            dp[(1 << i, i, din)] = c
        for mask in range(1, 1 << n):
            states = [(k, v) for k, v in dp.items() if k[0] == mask]
            for (m, last, din), c in states:
                for nx in range(n):
                    if (mask >> nx) & 1:
                        continue
                    inf = info(nodes[last], nodes[nx])
                    if inf is None:
                        continue
                    c2, dout, din2 = inf
                    extra = TURN_PEN * turn_pen_factor(din, dout)
                    nc = c + extra + c2
                    key = (mask | (1 << nx), nx, din2)
                    if nc < dp.get(key, INF):
                        dp[key] = nc
                        par[key] = (m, last, din)
        full = (1 << n) - 1
        best = None
        for (m, last, din), c in dp.items():
            if m != full:
                continue
            if self.scan_complete:
                inf = info(nodes[last], stop_cell)
                if inf is None:
                    continue
                c2, dout, din2 = inf
                extra = TURN_PEN * turn_pen_factor(din, dout)
                tot = c + extra + c2
            else:
                # 扫描进行中: 只求已可达雷的最优访问序(终点段待扫描补全后
                # 重规划补齐)——边扫边走, 能踩的先踩
                tot = c
            if best is None or tot < best[0]:
                best = (tot, (m, last, din))
        if best is None:
            return []
        order = []
        st = best[1]
        while st in par:
            mask, last, din = st
            order.append(nodes[last])
            st = par[st]
        mask, last, din = st
        order.append(nodes[last])
        order.reverse()
        return order

    def build_goals(self, cur, collected=None, done=None):
        """任务目标序列: [地雷(全踩TSP序)] + 每敌[蓝面停靠位, 打击] + [返程点]
        done = 已触发的角色格(S/T), 重规划时不把车拉回已完成的位置"""
        col = collected or set()
        done = done or set()
        ret = name2rc(RETURN_TILE)
        # 敌方打击区(带蓝面), 我方不打; 已打击的与正在击打的不重复
        enemies = [t for t in self.field.known_of(TARGET)
                   if self.field.blue(t) is not None and t not in done and t != cur]
        pairs = []          # (停靠位或None, 敌方格)
        for t in enemies:
            s = self.strike_cell(cur, t)
            if s is None:
                print(f"[planner] 敌方打击区 {rc2name(t)} 不可达, 无法打击")
                continue
            if s in done:
                s = None    # 已停靠过蓝面对面, 直接打击
            pairs.append((s, t))
        # 敌方打击顺序: 贪心近邻
        if len(pairs) > 1:
            ordered = [pairs[0]]
            rest = pairs[1:]
            pos = pairs[0][0] if pairs[0][0] is not None else pairs[0][1]
            while rest:
                nxt = min(range(len(rest)),
                          key=lambda i: man(pos, rest[i][0] if rest[i][0] is not None else rest[i][1]))
                ordered.append(rest.pop(nxt))
                pos = ordered[-1][0] if ordered[-1][0] is not None else ordered[-1][1]
            pairs = ordered
        endpoint = ret                    # 打靶不是最终任务: 地雷+打靶统一排序, 终点是返程
        # 桥洞可通行通道(官方: 上下通道各 1 个, 各穿一次各 +10 分, 无方向语义):
        # **不强制首任务**——小车沿 TSP 排雷/打靶时顺路穿过通道即自动计分
        # (step_car 进入通道格即 +10), 未穿过的在返程前按就近序补穿
        # 地雷 TSP + 打靶穿插(停靠位/打击格上的地雷顺带踩, 不单独排)
        skip = {rc for s, t in pairs for rc in (s, t) if rc is not None}
        exs = [rc for rc in self.field.known_of(EXPLOSIVE)
               if rc not in col and rc not in skip]
        order = self.explosive_order(cur, exs, ret,
                                     strike_pair=(pairs[0] if pairs else None)) if exs or pairs else []
        goals = []
        for e in order:
            if e == self.STRIKE_NODE:      # 打靶穿插在踩爆炸物过程中
                s0, t0 = pairs[0]
                if s0 is not None and s0 != cur:
                    goals.append((s0, "stop_face"))   # 停靠蓝色面对面(识别敌我)
                goals.append((t0, "hit"))             # 激光打击敌方
            elif pairs and e == pairs[0][1]:
                goals.append((e, "hit"))   # 已停靠过(s=None): 打击格直接以节点参加排序
            else:
                goals.append((e, "explosive"))
        for s, t in pairs[1:]:            # 多个打击区(罕见): 剩余按近邻序追加
            if s is not None and s != cur:
                goals.append((s, "stop_face"))
            goals.append((t, "hit"))
        # 未穿过的桥洞通道: 返程前按距当前格就近序补穿(顺路已穿的不重复)
        tun_goals = [rc for rc in self.field.open_tunnels
                     if rc != cur and rc not in done]
        tun_goals.sort(key=lambda rc: man(cur, rc))
        for rc in tun_goals:
            goals.append((rc, "bridge"))
        if cur != ret:
            goals.append((ret, "return"))
        return goals

    def route_through(self, cur, goals, global_avoid=()):
        """把目标序列连成完整格点路径。
        每段只避开"尚未访问的关键目标格"(停靠/打击/返程/桥洞), 防止角色提前触发;
        global_avoid=全局避让格(如标靶, 目标格本身除外); 地雷允许途中顺路踩:
        踩到即标记角色并从未完成清单移除, 不再回头。"""
        route = [cur]
        roles = {}
        remaining = list(goals)
        gavoid = set(global_avoid)
        while remaining:
            rc, role = remaining.pop(0)
            if role == "hit":
                continue      # 标靶所占格子无法通行: 打击在停靠位 S 原地完成, 不驶入标靶
            if rc == route[-1]:
                roles[rc] = role
                continue
            avoid = {g for g, r in remaining
                     if r in ("stop_face", "hit", "return", "bridge")}
            avoid |= (gavoid - {rc})
            leg = self.a_star(route[-1], rc, extra_avoid=avoid)
            if leg is None:
                leg = self.a_star(route[-1], rc)      # 兜底: 允许穿过
            if leg is None:
                return None, None
            route.extend(leg[1:])
            roles[rc] = role
            # 途中顺路踩到的地雷: 标记并从未完成目标中移除(不再回头)
            passed = set(leg[1:-1])
            if passed:
                remaining = [(g, r) for g, r in remaining
                             if not (r == "explosive" and g in passed)]
                for g in passed:
                    if self.field.state(g) == EXPLOSIVE and g not in roles:
                        roles[g] = "explosive"
        return route, roles


def route_cost(route):
    """路径代价 = 格数 + 转弯惩罚(180°掉头×U_TURN_FACTOR)"""
    cost = 0.0
    pd = None
    for i in range(1, len(route)):
        d = sub(route[i], route[i - 1])
        cost += 1.0
        if pd is not None:
            cost += TURN_PEN * turn_pen_factor(pd, d)
        pd = d
    return cost


def seg_turns(cells):
    """一段格点序列内的转弯次数"""
    turns = 0
    pd = None
    for i in range(1, len(cells)):
        d = sub(cells[i], cells[i - 1])
        if pd is not None and d != pd:
            turns += 1
        pd = d
    return turns


def compress(route, roles):
    """航点压缩: 只保留起点/拐点/角色点, 直线段小车全速行驶;
    同一(格,角色)只标注一次, 返程再次路过不重复标角色。
    航点给出格子正中心坐标(cm): 小车沿格心到格心行驶, 保证压过爆炸物正中心"""
    n = len(route)
    keep = {0}
    for i in range(1, n):
        d_in = sub(route[i], route[i - 1])
        d_out = sub(route[i + 1], route[i]) if i + 1 < n else None
        if roles.get(route[i]) or i == n - 1 or d_in != d_out:
            keep.add(i)
    pts = []
    emitted = set()
    for i in sorted(keep):
        role = roles.get(route[i])
        if (route[i], role) in emitted:
            role = None
        elif role:
            emitted.add((route[i], role))
        if i == 0:
            head = DIR_NAME.get(sub(route[1], route[0])) if n > 1 else None
        else:
            head = DIR_NAME.get(sub(route[i], route[i - 1]))
        pts.append({"cell": rc2name(route[i]), "heading": head, "role": role,
                    "x_cm": round((route[i][1] + 0.5) * TILE_CM, 1),
                    "y_cm": round((route[i][0] + 0.5) * TILE_CM, 1)})
    return pts


def estimate_time(route, roles):
    t = 0.0
    pd = None
    for i in range(1, len(route)):
        d = sub(route[i], route[i - 1])
        t += CELL_TIME_S
        if pd is not None:
            t += TURN_TIME_S * turn_pen_factor(pd, d)
        pd = d
    if any(r == "stop_face" for r in roles.values()):
        t += FACING_PAUSE_S + FALL_TIMES * FALL_INTERVAL   # 停靠位原地打击(标靶格禁入)
    elif any(r == "hit" for r in roles.values()):
        t += FALL_TIMES * FALL_INTERVAL
    return t


# ============================ 航点下发链路 ============================
class CarLink:
    """无人机 -> 小车航点通信。sim=控制台演示, file=JSON文件, tcp=实时下发"""

    def __init__(self, mode, host, port, path):
        self.mode = mode
        self.conn = None
        self.buf = b""
        self.file = None
        self.last_plan = None
        if mode == "tcp":
            self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.srv.bind((host, port))
            self.srv.listen(1)
            self.srv.setblocking(False)
            print(f"[link] TCP 航点服务监听 {host}:{port}, 等待小车接入...")
        elif mode == "file":
            self.file = open(path, "w", encoding="utf-8")
            self.file.write("[\n")
            self.file_first = True

    def wait_car(self, timeout=30.0):
        """阻塞等待小车接入(便于真实部署时先握手再起飞)"""
        if self.mode != "tcp":
            return True
        print(f"[link] 等待小车接入({timeout:.0f}s)...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.conn is not None:
                return True
            try:
                c, addr = self.srv.accept()
                self.conn = c
                self.conn.setblocking(False)
                print(f"[link] 小车已接入 {addr}")
                return True
            except OSError:
                time.sleep(0.1)
        print("[link] 超时未接入, 继续任务(最新航点会保留待补发)")
        return False

    def _send_tcp(self, plan):
        if self.conn:
            try:
                self.conn.sendall((json.dumps(plan, ensure_ascii=False) + "\n")
                                  .encode("utf-8"))
                return True
            except OSError:
                self.conn.close()
                self.conn = None
                print("[link] 小车断开, 等待重连...")
        return False

    def send_plan(self, plan):
        self.last_plan = plan
        if self.mode == "sim":
            pts = plan["points"]
            names = " -> ".join(p["cell"] + ("(" + p["role"] + ")" if p["role"] else "")
                                for p in pts)
            print(f"[link] 下发航点 #{plan['seq']} 共{len(pts)}个: {names}")
            print(f"[link]   全路径 {len(plan['path_cells'])} 格 -> 压缩为 {len(pts)} 个拐点, "
                  f"预估 {plan['est_time_s']}s, 已扫描 {plan['scan_pct']}%")
        elif self.mode == "file":
            if not self.file_first:
                self.file.write(",\n")
            self.file.write(json.dumps(plan, ensure_ascii=False))
            self.file_first = False
            self.file.flush()
        elif self.mode == "tcp":
            self._send_tcp(plan)

    def send_done(self, message="任务完成"):
        """任务结束握手"""
        if self.mode == "sim":
            print(f"[link] 发送完成握手: {message}")
        elif self.mode == "tcp":
            self._send_tcp({"type": "done", "message": message})

    def update(self):
        if self.mode != "tcp":
            return
        if self.conn is None:
            try:
                c, addr = self.srv.accept()
                self.conn = c
                self.conn.setblocking(False)
                print(f"[link] 小车已接入 {addr}")
                if self.last_plan:
                    self._send_tcp(self.last_plan)   # 补发最新航点
            except OSError:
                return
        try:
            while True:
                data = self.conn.recv(4096)
                if not data:
                    self.conn.close()
                    self.conn = None
                    print("[link] 小车断开, 等待重连...")
                    return
                self.buf += data
                while b"\n" in self.buf:
                    line, self.buf = self.buf.split(b"\n", 1)
                    try:
                        st = json.loads(line.decode("utf-8"))
                        if st.get("type") == "status":
                            print(f"[link] 小车状态: 在{st.get('cell')} {st.get('event', '')}")
                    except Exception:
                        pass
        except OSError:
            pass

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except OSError:
                pass
        if self.file:
            self.file.write("\n]\n")
            self.file.close()
        if self.mode == "tcp":
            try:
                self.srv.close()
            except OSError:
                pass


# ============================ 任务总控(无人机端) ============================
class Mission:
    def __init__(self, args, vision=False):
        self.args = args
        self.seed = args.seed if args.seed is not None else random.randrange(1, 10 ** 9)
        self.rng = random.Random(self.seed)
        self.field = Field()
        self.vision = vision
        if vision:
            self.truth = None
            self.arrows = {}
        else:
            self.truth, self.arrows = build_truth(self.rng)
        self.arrow_seen = []             # 已识别的桥洞上方箭头
        self.bridge_crossed = set()     # 已穿过的可通行通道格(每个通道只计一次分)
        self.task_queue = []             # 剩余任务队列(按段规划, 规划一段运行一段)
        self.portion_fails = 0           # 该段规划连续失败计数(防卡死)
        # ---- 桌面包口径: 时间预算门控与降级兜底 ----
        self.strike_shed = False         # 超软预算已砍打靶停靠(弃 15 分)
        self.mine_shed = []              # 超软预算已放弃的地雷(弃 5 分/颗)
        self.budget_checked = False      # 本次执行已做过一次软预算检查
        self.return_unreachable = False  # 终点不可达: 降级按已执行部分结算
        self.timeout_hard = False        # 180s 到点自动结束(官方口径)
        self.frame = None
        self.scale = getattr(args, "scale", 8.0)
        self.planner = Planner(self.field)
        self.link = CarLink(args.link, args.host, args.port, args.link_path)
        self.start_rc = name2rc(START_TILE)
        self.blocks = scan_block_order(make_blocks())   # U 形扫描路线(N10→N5)
        self.pending = list(self.blocks)                # 剩余待扫描块
        self.waiting_scan = False                       # 小车等待无人机前方扫描
        self.scan_idx = 0
        self.scan_done = False
        self.scan_done_at = 0.0
        self.state = "takeoff"
        self.vnow = 0.0
        self.last_dt = 0.0
        # ---- 无人机飞行计时(DRONE_SPEED_MS): 起飞点 -> 各块中心 -> 返航终点 ----
        def _center_rc(items):
            return (sum(rc[0] for rc in items) / len(items),
                    sum(rc[1] for rc in items) / len(items))
        _m = TILE_CM / 100.0
        _pts = [_center_rc([name2rc(DRONE_START_TILE)])] + [_center_rc(b) for b in self.blocks]
        self.flight_to_block = [math.hypot((b[0] - a[0]) * _m, (b[1] - a[1]) * _m)
                                / DRONE_SPEED_MS for a, b in zip(_pts, _pts[1:])]
        _end = _center_rc([name2rc(DRONE_END_TILE)])
        self.drone_return_s = math.hypot((_end[0] - _pts[-1][0]) * _m,
                                         (_end[1] - _pts[-1][1]) * _m) / DRONE_SPEED_MS
        self.scan_timer = self.flight_to_block[0]   # 起飞后飞往第一块中心的耗时
        self.route = [self.start_rc]
        self.route_i = 0
        self.roles = {}
        self.full_path = []             # 全流程轨迹: 小车实际走过的每一格(最终地图用)
        self.full_roles = {}            # 全流程角色标记(雷/停靠/打击/桥洞/返程)
        self.strike_map = {}
        self.role_done = set()          # 已触发的角色格(返程再路过不重复触发)
        self.collected = set()
        self.step_timer = 0.0
        self.pause_until = None
        self.pause_kind = None
        self.pause_start = 0.0
        self.last_hits = 0
        self.hits_done = 0
        self.hit_count = 0              # 打靶命中总发数(每发 +5 分)
        self.score = 0.0                # 实时得分 = 5×踩雷 + 10×桥洞穿行 + 5×打靶命中
        self.score_breakdown = {"mine": 0, "bridge": 0, "hit": 0}
        self.seq = 0
        self.last_sent = None
        self.plan_info = None
        self.car_started = False      # 首次规划完成并已发车
        self.car_done = False
        self.car_done_at = 0.0
        self.wall_car_start = 0.0
        self.first_plan_at = None     # 第一次路径规划完整时刻(=发车时机)
        self.finished = False
        self.dynamic_at = None
        self.dynamic_done = False
        self.drone_arrive_at = None
        # 实时地图导出(可选, 需 Pillow)
        self.map_out = getattr(args, "map_out", None)
        self.map_dir = getattr(args, "map_dir", None)
        self.map_seq = 0
        self.gif_out = getattr(args, "gif_out", None)
        self.gif_frames = []
        self.gif_dur = []
        self.map_ok = bool(self.map_out or self.map_dir or self.gif_out)

    # ---------------- 扫描(起飞完成后才开始) ----------------
    def _roi(self, rc):
        img_h, img_w = self.frame.shape[:2]
        x0, y0 = rc[1] * TILE_CM, rc[0] * TILE_CM
        px0, py0 = cm_to_pixel(x0, y0, self.scale, img_w)
        px1, py1 = cm_to_pixel(x0 + TILE_CM, y0 + TILE_CM, self.scale, img_w)
        px0, py0 = max(0, px0), max(0, py0)
        px1, py1 = min(img_w, px1), min(img_h, py1)
        if px1 <= px0 or py1 <= py0:
            return None
        return self.frame[py0:py1, px0:px1]

    def vision_detect(self, rc):
        roi = self._roi(rc)
        if roi is None:
            return None
        return detect_marker_in_roi(roi, self.scale)

    def vision_blue(self, rc):
        roi = self._roi(rc)
        if roi is None:
            return None
        return detect_blue_dir(roi)

    def reveal_block(self):
        """扫描下一块: 严格按 U 形 S 型顺序逐块衔接(扫描一块连接一块,
        每块与前一块相邻、不重复路线, 不做全局跳转规划); 扫到哪块填充哪块"""
        if not self.pending:
            return
        block = self.pending.pop(0)
        self.scan_idx += 1
        newly = []
        for rc in block:
            if self.truth is not None:
                info = self.truth.get(rc)
                if info:
                    self.field.set(rc, info[0], info[1])
                else:
                    self.field.set(rc, EMPTY)
            else:
                kind = self.vision_detect(rc)
                if kind is None:
                    kind = EMPTY
                blue = None
                if kind == TARGET:
                    blue = self.vision_blue(rc)
                    if blue is None:
                        kind = FRIEND      # 无蓝面 -> 显示红面(友方), 不打
                self.field.set(rc, kind, blue)
            newly.append(rc)
        # 桥洞顶部通行状态标识识别(箭头标明哪格可通行, 地面车看不见须无人机侦察);
        # 新识别可通行通道 -> 立即重规划, 把"穿行该通道"插入路径
        new_arrow = False
        for rc in block:
            if rc in self.arrows and rc not in self.arrow_seen:
                self.arrow_seen.append(rc)
                self.field.open_tunnels.add(rc)
                new_arrow = True
                print(f"[{self.vnow:6.1f}s] 识别桥洞可通行通道 {rc2name(rc)} "
                      f"(顶部通行状态标识, 穿行 +{SCORE_BRIDGE:.0f}分)")
        if new_arrow and self.state == "executing" and not self.car_done:
            print(f"[{self.vnow:6.1f}s] 新识别桥洞可通行通道, 立即重规划(必须穿行)")
            self.replan_from_current()
        if not self.pending:
            self.scan_done = True
            self.scan_done_at = self.vnow
            self.planner.scan_complete = True
            if DYNAMIC_OBSTACLE:
                self.dynamic_at = self.vnow + self.rng.uniform(8.0, 20.0)
        filled = sum(1 for r in range(ROWS) for c in range(COLS)
                     if self.field.is_known((r, c)))
        print(f"[{self.vnow:6.1f}s] 扫描块 {self.scan_idx}/{len(self.blocks)}, "
              f"已填充矩阵 {filled}/{ROWS * COLS} 格: "
              f"障{len(self.field.known_of(BUCKET))} "
              f"雷{len(self.field.known_of(EXPLOSIVE))} "
              f"敌{len(self.field.known_of(TARGET))} "
              f"我{len(self.field.known_of(FRIEND))}"
              + (" [全场扫描完成]" if self.scan_done else ""))
        if self.args.verbose or self.args.step_map:
            print(render_map(self.field, self.route, self.roles, self.current_cell(),
                             f"扫描块 {self.scan_idx}/{len(self.blocks)} 填充完成",
                             arrows={rc: self.arrows[rc] for rc in self.arrow_seen}))
        self.export_map(f"实时地图 扫描{self.scan_idx}/{len(self.blocks)} "
                        f"障{len(self.field.known_of(BUCKET))} "
                        f"雷{len(self.field.known_of(EXPLOSIVE))} "
                        f"敌{len(self.field.known_of(TARGET))} "
                        f"我{len(self.field.known_of(FRIEND))}",
                        highlight=block, duration_ms=350)
        self.on_reveal(newly)

    def on_reveal(self, newly):
        new_buckets = [rc for rc in newly if self.field.state(rc) == BUCKET]
        new_friends = [rc for rc in newly if self.field.state(rc) == FRIEND]
        new_exs = [rc for rc in newly if self.field.state(rc) == EXPLOSIVE]
        new_enemies = [rc for rc in newly if self.field.state(rc) == TARGET]
        if self.state == "executing" and not self.car_done:
            future = set(self.route[self.route_i:])
            block_ahead = [rc for rc in new_buckets + new_friends if rc in future]
            if block_ahead or new_exs or new_enemies:
                reason = ""
                if block_ahead:
                    reason += f"前方新障碍/我方区 {[rc2name(x) for x in block_ahead]}, "
                if new_exs:
                    reason += f"新发现地雷 {[rc2name(x) for x in new_exs]}(全踩), "
                if new_enemies:
                    reason += f"新发现敌打击区 {[rc2name(x) for x in new_enemies]}, "
                print(f"[{self.vnow:6.1f}s] {reason}立即重规划")
                self.replan_from_current()

    def _still_connected(self):
        """动态障碍落点校验: 落桶后 当前格→终点 与全部可通行通道仍连通(否则换格)。
        官方裁判系统随机生成不会封死通路, 仿真落点同样要保证不切断全场。"""
        from collections import deque
        blocked = self.planner.blocked_set()
        start = self.route[self.route_i]
        seen = {start}
        q = deque([start])
        while q:
            r, c = q.popleft()
            for dr, dc, _h in DIR4:
                nb = (r + dr, c + dc)
                if not (0 <= nb[0] < ROWS and 0 <= nb[1] < COLS):
                    continue
                if nb in blocked or nb in seen:
                    continue
                if barrier_blocks_cross((r, c), nb):
                    continue
                seen.add(nb)
                q.append(nb)
        if name2rc(RETURN_TILE) not in seen:
            return False
        for rc in self.field.open_tunnels:
            if rc not in seen:
                return False
        return True

    def spawn_dynamic_obstacle(self):
        """动态地图: 任务中途出现新障碍(官方"动态地图"字样, 数量/时机为仿真标定),
        无人机发现后立即动态重规划; 落点保护: 不落在起终点/可通行通道/
        A/B/C 走廊穿越口, 且落桶后全场仍连通(否则换落点)"""
        past = set(self.route[:self.route_i + 1])
        or0, or1 = _object_rows()
        protect = (set(self.field.open_tunnels)
                   | {name2rc(RETURN_TILE), name2rc(START_TILE)}
                   | {(r, c) for r in range(3) for c in (5, 6)})  # A/B/C 挡板穿越口

        def ok(rc):
            return (self.field.state(rc) == EMPTY
                    and rc not in past and rc not in self.roles
                    and rc not in protect)

        cands = [(r, c) for r in range(ROWS) for c in range(COLS)
                 if ok((r, c)) and or0 <= r <= or1]
        self.rng.shuffle(cands)
        for rc in cands:
            self.field.set(rc, BUCKET)
            if self._still_connected():
                print(f"[{self.vnow:6.1f}s] 动态地图变化: 新障碍出现在 {rc2name(rc)}")
                if rc in set(self.route[self.route_i:]):
                    self.replan_from_current()
                return
            self.field.set(rc, EMPTY)      # 落点会切断通路: 撤销并换一个
        print(f"[{self.vnow:6.1f}s] 动态地图变化: 无安全落点, 本局跳过新障碍")

    # ---------------- 规划与下发 ----------------
    @staticmethod
    def _derive_strike_map(goals):
        m = {}
        for i, (rc, role) in enumerate(goals):
            if role == "stop_face" and i + 1 < len(goals) and goals[i + 1][1] == "hit":
                m[rc] = goals[i + 1][0]
        return m

    def adopt(self, route, roles):
        """采纳新路径; 路径上已扫描出的地雷自动标记为顺路踩"""
        for rc in route:
            if self.field.state(rc) == EXPLOSIVE and rc not in roles and rc != route[0]:
                roles[rc] = "explosive"
        self.route = route
        self.roles = roles
        if self.last_sent != (list(route), dict(roles)):
            self.send_plan()

    def send_plan(self):
        pts = compress(self.route, self.roles)
        enemies = [rc2name(x) for x in self.field.known_of(TARGET)]
        first_t = self.field.target_rc()
        for q in pts:
            rc = name2rc(q["cell"])
            if q["role"] == "stop_face" and rc in self.strike_map:
                q["heading"] = DIR_NAME.get(sub(self.strike_map[rc], rc))
            elif q["role"] == "hit":
                s = next((k for k, v in self.strike_map.items() if v == rc), None)
                if s is not None:
                    q["heading"] = DIR_NAME.get(sub(rc, s))
        plan = {
            "type": "plan",
            "seq": self.seq,
            "points": pts,
            "path_cells": [rc2name(rc) for rc in self.route],
            "blocked": [rc2name(rc) for rc in self.planner.blocked_set()],
            "target": enemies[0] if enemies else None,
            "enemies": enemies,
            "friends": [rc2name(x) for x in self.field.known_of(FRIEND)],
            "blue": DIR_NAME.get(self.field.blue(first_t)) if first_t else None,
            "est_time_s": round(estimate_time(self.route, self.roles), 1),
            "scan_pct": int(self.scan_idx * 100 / len(self.blocks)),
            "score": round(self.score, 1),       # 实时得分(5×雷 + 10×桥洞 + 10×命中)
        }
        self.seq += 1
        self.link.send_plan(plan)
        if not (self.args.gui and HAS_CV) or self.args.verbose:
            print(render_map(self.field, self.route, self.roles,
                             self.current_cell(),
                             f"规划 #{plan['seq']} (敌={plan['enemies']} 我={plan['friends']})",
                             arrows={rc: self.arrows[rc] for rc in self.arrow_seen}))
        self.export_map(f"实时地图 规划#{plan['seq']} "
                        f"敌={plan['enemies']} 我={plan['friends']} "
                        f"蓝面={plan['blue']}", duration_ms=400)
        self.last_sent = (list(self.route), dict(self.roles))

    def done_bridge_cells(self):
        """已穿行过的可通行通道格, 重规划时不再拉回"""
        return set(self.bridge_crossed)

    def plan_next_portion(self, why=""):
        """规划出部分路径就立即运行, 不等待完整路径:
        任务队列按段切分(排雷/打靶段 -> 桥洞补穿+返程段; 桥洞不强制首任务,
        顺路穿过即自动计分), 每段规划完成立即采纳运行"""
        if not self.task_queue:
            return False
        portion = []
        if self.task_queue[0][1] == "bridge":
            while self.task_queue and self.task_queue[0][1] == "bridge":
                portion.append(self.task_queue.pop(0))
        elif self.task_queue[0][1] in ("stop_face", "hit", "explosive"):
            while self.task_queue and self.task_queue[0][1] in ("stop_face", "hit", "explosive"):
                portion.append(self.task_queue.pop(0))
        else:                                # 桥洞下行穿行 + 返程: 剩余整段
            portion = self.task_queue
            self.task_queue = []
        if not portion:
            return False
        cur = self.route[self.route_i] if self.route and self.route_i < len(self.route) \
            else self.start_rc
        # 全局避让: 标靶格(敌/我)不允许提前压过
        global_avoid = set(self.field.known_of(TARGET)) | set(self.field.known_of(FRIEND))
        route, roles = self.planner.route_through(cur, portion,
                                                  global_avoid=global_avoid)
        if route is None or len(route) < 2:
            self.portion_fails += 1
            if self.portion_fails >= 3:
                dropped, rl = portion[0]
                self.portion_fails = 0
                if not self.scan_done and rl == "return":
                    # 官方桥洞: 可通行通道未识别全前 K 列全封、场地断连,
                    # 终点"暂不可达"是常态——等待扫描补全, 不算永久失败
                    # (永久降级只在全场扫完后仍不可达时才结算)
                    self.task_queue = portion + self.task_queue
                    print(f"[{self.vnow:6.1f}s] 终点 {rc2name(dropped)} 暂不可达"
                          f"(场地图未扫完/桥洞通道未识别全), 等待无人机扫描")
                    return False
                if rl == "return":
                    # 全场扫完后终点仍不可达: 降级按已执行部分结算(不崩、不卡死——
                    # 超时整局作废, 已踩雷/已穿桥洞照常计分; 无人机扫完后正常返航结束)
                    self.car_done = True
                    self.car_done_at = self.vnow
                    self.return_unreachable = True
                    self.task_queue = []
                    print(f"[{self.vnow:6.1f}s] 终点 {rc2name(dropped)} 不可达, "
                          f"降级: 按已执行部分结算(保完赛)")
                    return True
                print(f"[{self.vnow:6.1f}s] 该段 {rc2name(dropped)}({rl}) 不可达, 丢弃后继续")
                self.task_queue = portion[1:] + self.task_queue
            else:
                print(f"[{self.vnow:6.1f}s] 该段规划失败, 稍后重试")
                self.task_queue = portion + self.task_queue
            return False
        self.portion_fails = 0
        self.strike_map = self._derive_strike_map(portion)
        self.adopt(route, roles)
        self.route_i = 0
        self.step_timer = 0.0
        names = " -> ".join(rc2name(g[0]) + f"({g[1]})" for g in portion)
        print(f"[{self.vnow:6.1f}s] 规划出部分路径, 小车立即运行{why}: {names}")
        return True

    def start_mission(self):
        goals = self.planner.build_goals(self.start_rc, set(), set())
        if goals is None:
            return False
        goals = self.budget_shed(goals, "(启动段)")
        self.task_queue = goals
        if not self.full_path:
            self.full_path = [self.start_rc]   # 发车起点进入全流程轨迹
        return self.plan_next_portion("(启动段)")

    def replan_from_current(self):
        cur = self.route[self.route_i]
        goals = self.planner.build_goals(cur, self.collected,
                                         self.role_done | self.done_bridge_cells())
        if goals is None:
            return False
        goals = self.budget_shed(goals, "(重规划段)")
        self.task_queue = goals
        return self.plan_next_portion("(重规划段)")

    # ---------------- 时间预算门控与降级兜底(桌面包口径) ----------------
    def _est_total_with(self, route, roles):
        """预估总耗时 = 当前时刻 + max(剩余行驶, 剩余扫描) + 无人机返航耗时。
        与 finish() 的"实际总耗时"同口径(小车先到/扫描先完取较大者)。"""
        rem_scan = 0.0
        if not self.scan_done and self.pending:
            rem_scan = len(self.pending) * BLOCK_SCAN_S \
                       + sum(self.flight_to_block[self.scan_idx:])
        return self.vnow + max(estimate_time(route, roles), rem_scan) + self.drone_return_s

    def budget_shed(self, goals, why=""):
        """时间预算门控(口径: 规定时间内得分最高, 超时整局作废):
        预估总耗时超 165s 软预算时, 按"分数/省时"效率逐项放弃任务——
        打靶整块(15 分)与单颗地雷(5 分)都是可弃单元, 桥洞通道(各 10 分,
        必经主路)保留; 每轮放弃效率最低的单元并重估, 直到预估进软预算。
        放弃的雷只从"必经清单"移除, 路径顺路踩到仍计分。"""
        if self.strike_shed:
            goals = [(g, r) for g, r in goals if r not in ("stop_face", "hit")]
        if self.mine_shed:
            goals = [(g, r) for g, r in goals
                     if not (r == "explosive" and rc2name(g) in self.mine_shed)]
        cur = self.route[self.route_i] if self.route and self.route_i < len(self.route) \
            else self.start_rc
        global_avoid = set(self.field.known_of(TARGET)) | set(self.field.known_of(FRIEND))

        def est_route(gl):
            rr, rl = self.planner.route_through(cur, gl, global_avoid=global_avoid)
            if rr is None:
                return float("inf")
            return self._est_total_with(rr, rl)

        est = est_route(goals)
        if est == float("inf") or est <= SOFT_BUDGET_S:
            # 规划不出完整路线(扫描中信息不全/真不可达): 这不是时间问题,
            # 不做取舍——交给分段规划(能跑的先跑)与等待扫描逻辑
            return goals
        while True:
            units = []
            si = [i for i, (g, r) in enumerate(goals) if r in ("stop_face", "hit")]
            if si:
                units.append(("strike", si, SCORE_HIT * FALL_TIMES))
            for i, (g, r) in enumerate(goals):
                if r == "explosive":
                    units.append(("mine", [i], SCORE_MINE))
            if not units:
                break
            est = est_route(goals)
            if est == float("inf"):
                break
            best = None
            for kind, idxs, score in units:
                trial = [x for j, x in enumerate(goals) if j not in idxs]
                t = est_route(trial)
                saved = est - t
                if saved <= 1e-6:
                    continue
                ratio = score / saved          # 分/秒 效率: 越低越先弃
                if best is None or ratio < best[0]:
                    best = (ratio, kind, idxs, score, t)
            if best is None:
                break
            _ratio, kind, idxs, score, t = best
            names = ", ".join(rc2name(goals[i][0]) for i in idxs
                              if goals[i][1] == "explosive")
            label = "打靶停靠" if kind == "strike" else f"地雷 {names}"
            print(f"[{self.vnow:6.1f}s] 时间预算门控{why}: 预估 {est:.0f}s "
                  f"> 软预算{SOFT_BUDGET_S:.0f}s, 放弃{label}(弃 {score:.0f} 分) "
                  f"-> 预估 {t:.0f}s (顺路踩到仍计分)")
            if kind == "strike":
                self.strike_shed = True
            else:
                if names not in self.mine_shed:
                    self.mine_shed.append(names)
            goals = [x for j, x in enumerate(goals) if j not in idxs]
            est = t
            if est <= SOFT_BUDGET_S:
                break
        return goals

    # ---------------- 小车推进 ----------------
    def current_cell(self):
        if self.route and self.route_i < len(self.route):
            return self.route[self.route_i]
        return None

    def export_map(self, title, highlight=(), duration_ms=300, png=True,
                   route=None, roles=None):
        """实时导出当前矩阵+路径为 PNG/GIF 帧(可选, 需 Pillow; 未安装自动跳过)。
        --map-out 覆盖写一张实时地图; --map-dir 另存编号快照;
        --gif-out 录制逐块建图全过程动画。highlight=刚填充的扫描块;
        route/roles=None 时用当前规划段; 传全流程轨迹则画整局完整线路;
        png=False 时只录 GIF 帧、不写 PNG(小车每步只录帧, 避免文件爆炸)"""
        if not self.map_ok:
            return
        try:
            import map_render
        except Exception:
            print("提示: 实时地图导出需要 Pillow, 已跳过")
            self.map_ok = False
            return
        seen_arrows = {rc: True for rc in self.arrow_seen}
        route = route if route is not None else self.route
        roles = roles if roles is not None else self.roles
        if self.gif_out:
            img = map_render.render_map_image(self.field, route, roles,
                                              self.current_cell(), title, highlight,
                                              arrows=seen_arrows)
            if img is not None:
                self.gif_frames.append(img)
                self.gif_dur.append(duration_ms)
        if png and self.map_out:
            map_render.render_map_png(self.field, route, roles,
                                      self.current_cell(), title, self.map_out,
                                      highlight, arrows=seen_arrows)
        if png and self.map_dir:
            try:
                os.makedirs(self.map_dir, exist_ok=True)
                self.map_seq += 1
                snap = os.path.join(self.map_dir, f"map_{self.map_seq:03d}.png")
                map_render.render_map_png(self.field, route, roles,
                                          self.current_cell(), title, snap,
                                          highlight, arrows=seen_arrows)
            except OSError:
                pass

    def save_gif_now(self):
        """把录制的帧保存为 GIF(任务异常结束时兜底)"""
        if self.gif_out and self.gif_frames:
            try:
                import map_render
                if map_render.save_gif(self.gif_frames, self.gif_out,
                                       self.gif_dur):
                    print(f"[gif] 过程动画已保存: {self.gif_out} "
                          f"({len(self.gif_frames)} 帧)")
            except Exception:
                pass
            self.gif_frames = []
            self.gif_dur = []

    def step_car(self):
        vnow = self.vnow
        if self.pause_until is not None:
            if vnow >= self.pause_until:
                self.pause_until = None
                self.pause_kind = None
            else:
                if self.pause_kind == "hit":
                    hits = min(FALL_TIMES, int((vnow - self.pause_start) / FALL_INTERVAL) + 1)
                    if hits != self.last_hits:
                        delta = hits - self.last_hits
                        self.last_hits = hits
                        self.hit_count += delta
                        self.score += SCORE_HIT * delta
                        self.score_breakdown["hit"] += SCORE_HIT * delta
                        print(f"[{vnow:6.1f}s] 激光打击 {hits}/{FALL_TIMES} 次 "
                              f"(命中 +{SCORE_HIT:.0f}×{delta}, 累计得分 {self.score:.0f})")
                return
        if self.route_i >= len(self.route) - 1:
            if self.task_queue and not self.car_done:
                self.plan_next_portion("(下一段)")   # 跑完一段立即规划下一段
            return
        self.step_timer += self.last_dt
        while self.route_i < len(self.route) - 1:
            cur = self.route[self.route_i]
            nxt = self.route[self.route_i + 1]
            # 无人机始终保持在小车前方: 未扫描区域小车不得驶入
            if not self.field.is_known(nxt):
                if not self.waiting_scan:
                    self.waiting_scan = True
                    print(f"[{vnow:6.1f}s] 前方 {rc2name(nxt)} 尚未扫描, "
                          f"小车等待无人机前方扫描")
                break
            self.waiting_scan = False
            d = sub(nxt, cur)
            prev_d = sub(cur, self.route[self.route_i - 1]) if self.route_i >= 1 else d
            # 无法原地拐90°: 拐弯=减速+三点调头(行进中完成), 180°掉头更慢;
            # 直行 1 格 CELL_TIME_S, 拐弯额外 TURN_TIME_S×系数
            need = CELL_TIME_S + TURN_TIME_S * turn_pen_factor(prev_d, d)
            if self.step_timer < need:
                break
            self.step_timer -= need
            self.route_i += 1
            rc = nxt
            role = self.roles.get(rc)
            if role in ("stop_face", "hit") and rc in self.role_done:
                role = None                     # 已停靠/已打击过的格: 不再重复触发
            # 全流程轨迹: 记录小车实际走过的每一格(重规划/分段不丢失)
            if self.full_path[-1:] != [rc]:
                self.full_path.append(rc)
            if role and rc not in self.full_roles:
                self.full_roles[rc] = role
            name = rc2name(rc)
            if role == "explosive":
                if rc not in self.collected:
                    self.collected.add(rc)
                    self.score += SCORE_MINE
                    self.score_breakdown["mine"] += SCORE_MINE
                    print(f"[{vnow:6.1f}s] 驶入 A~J 区, 压过爆炸物正中心 {name} 完成排雷 "
                          f"(+{SCORE_MINE:.0f}分, 累计 {self.score:.0f}) "
                          f"({len(self.collected)}/{len(self.field.known_of(EXPLOSIVE))})")
            elif role == "stop_face":
                self.role_done.add(rc)
                t = self.strike_map.get(rc)
                if t is not None:
                    self.role_done.add(t)          # 标靶格已处理(不驶入)
                head = DIR_NAME.get(sub(t, rc)) if t else None
                print(f"[{vnow:6.1f}s] 识别敌我: 敌方 {rc2name(t) if t else rc2name(rc)}, "
                      f"停靠蓝色面对面 {name}, 车头朝向 {head} 对准蓝面")
                # 标靶所占格子无法通行: 停靠位原地完成对蓝面打击(不驶入标靶格)
                self.hits_done += 1
                print(f"[{vnow:6.1f}s] 标靶格禁入: 在停靠位 {name} 原地激光打击"
                      f"(朝向蓝面, 打靶{FALL_TIMES}发/{STRIKE_TIME_S:.0f}s)")
                self.pause_start = vnow + FACING_PAUSE_S
                self.pause_until = vnow + FACING_PAUSE_S + FALL_TIMES * FALL_INTERVAL
                self.pause_kind = "hit"
                self.last_hits = 0
            elif role == "hit":
                self.role_done.add(rc)
                self.hits_done += 1
                print(f"[{vnow:6.1f}s] 到达敌打击区 {name}, 开始激光打击蓝面")
                self.pause_start = vnow
                self.pause_until = vnow + FALL_TIMES * FALL_INTERVAL
                self.pause_kind = "hit"
                self.last_hits = 0
            elif role == "return":
                self.car_done = True
                self.car_done_at = vnow
                print(f"[{vnow:6.1f}s] 小车已返回 {name}, 任务结束 "
                      f"(到达前已尽量完成: 排雷 {len(self.collected)}/"
                      f"{len(self.field.known_of(EXPLOSIVE))}, "
                      f"打击 {self.hits_done}/{len(self.field.known_of(TARGET))})")
            # 停靠位/途经格上正好有地雷 -> 顺带排雷
            if self.field.state(rc) == EXPLOSIVE and rc not in self.collected:
                self.collected.add(rc)
                self.score += SCORE_MINE
                self.score_breakdown["mine"] += SCORE_MINE
                if role != "explosive":
                    print(f"[{vnow:6.1f}s] 顺带排雷 {name} (+{SCORE_MINE:.0f}分, "
                          f"累计 {self.score:.0f}) "
                          f"({len(self.collected)}/{len(self.field.known_of(EXPLOSIVE))})")
            # 桥洞可通行通道穿行记录(官方: 每通过一个桥洞通道 +10 分, 每个通道只计一次)
            if rc in self.field.open_tunnels and rc not in self.bridge_crossed:
                self.bridge_crossed.add(rc)
                self.score += SCORE_BRIDGE
                self.score_breakdown["bridge"] += SCORE_BRIDGE
                print(f"[{vnow:6.1f}s] 小车通过桥洞可通行通道 {name} ✓ "
                      f"(+{SCORE_BRIDGE:.0f}分, 累计 {self.score:.0f}, "
                      f"{len(self.bridge_crossed)}/2 通道)")
            # 小车每走一格录一帧(充分展示过程)
            self.export_map(f"实时地图 小车@{name} "
                            f"雷{len(self.collected)}/{len(self.field.known_of(EXPLOSIVE))} "
                            f"打{self.hits_done}/{len(self.field.known_of(TARGET))}",
                            duration_ms=150, png=False)
            if self.pause_until is not None:
                break

    # ---------------- 主循环 ----------------
    def tick(self, vnow, dt):
        self.vnow = vnow
        self.last_dt = dt
        self.link.update()
        # 扫描严格在起飞完成后开始
        if not self.scan_done and vnow >= TAKEOFF_S:
            self.scan_timer -= dt
            while self.scan_timer <= 0 and not self.scan_done:
                self.reveal_block()
                if not self.scan_done:
                    # 下一块: 本块悬停扫描 0.6s + 按 0.8m/s 飞到下一块中心
                    self.scan_timer += BLOCK_SCAN_S + self.flight_to_block[self.scan_idx]
        if self.state == "takeoff":
            if vnow >= TAKEOFF_S:
                self.state = "scanning"
                print(f"[{vnow:6.1f}s] 无人机自主起飞完成(起飞点{DRONE_START_TILE}, "
                      f"限高 {DRONE_MAX_ALT_M:.1f}m, 速度 {DRONE_SPEED_MS:.1f}m/s), "
                      f"开始场地巡航扫描 (单次扫描 {BLOCK_ROWS}x{BLOCK_COLS} 范围, "
                      f"共 {len(self.blocks)} 块; 扫描 {DEPART_AFTER_BLOCKS} 块后小车启动)")
        elif self.state == "scanning":
            # 无人机扫描 N 块区域后小车启动; 路径规划无需敌打击区
            # (发车时刻与 E 版原版一致: 扫描 4 块后发车)
            if self.args.early and not self.car_started \
                    and self.scan_idx >= DEPART_AFTER_BLOCKS:
                if self.start_mission():
                    self.state = "executing"
                    self.wall_car_start = vnow
                    self.first_plan_at = vnow
                    self.car_started = True
                    print(f"[{vnow:6.1f}s] 无人机已扫描 {DEPART_AFTER_BLOCKS} 块区域, "
                          f"规划出部分路径(无需敌打击区/完整路径), 小车立即运行")
                elif getattr(self, "_start_fail_idx", -1) != self.scan_idx:
                    self._start_fail_idx = self.scan_idx
                    print(f"[{vnow:6.1f}s] 规划失败(桥洞可通行通道未识别全/场地未扫完), 等待扫描")
            if self.scan_done and not self.car_started:
                if self.field.target_rc() is None:
                    print("警告: 未识别到敌打击区, 仅完成排雷与返程")
                if self.start_mission():
                    self.state = "executing"
                    self.wall_car_start = vnow
                    self.first_plan_at = vnow
                    self.car_started = True
                    print(f"[{vnow:6.1f}s] 路径规划完整, 小车发车")
                else:
                    print("规划失败: 无可行路径")
                    self.state = "abort"
        elif self.state == "executing":
            self.step_car()
            # 软预算门控: 到 165s 做一次得分/时间权衡重规划(砍低效率任务);
            # 180s 硬顶到点: 官方"比赛自动结束, 用时计 180s", 按已得分结算
            if not self.budget_checked and vnow >= SOFT_BUDGET_S \
                    and not self.car_done:
                self.budget_checked = True
                print(f"[{vnow:6.1f}s] 已达软预算 {SOFT_BUDGET_S:.0f}s, "
                      f"重规划做得分/时间权衡")
                self.replan_from_current()
            if vnow >= TOTAL_BUDGET_S and not self.car_done and not self.finished:
                self.timeout_hard = True
                self.car_done = True
                self.car_done_at = TOTAL_BUDGET_S
                self.drone_arrive_at = TOTAL_BUDGET_S
                print(f"[{vnow:6.1f}s] 180s 到点, 比赛自动结束"
                      f"(官方: 用时计 180s, 按已得分结算)")
                self.finish()
                return
            if not self.dynamic_done and self.dynamic_at is not None \
                    and vnow >= self.dynamic_at and not self.car_done:
                self.dynamic_done = True
                self.spawn_dynamic_obstacle()
            if self.car_done and self.scan_done and not self.finished:
                self.state = "drone_return"
                # 小车先行到达终点后, 无人机才开始返航飞往终点(晚于小车到达)
                self.drone_arrive_at = max(self.car_done_at, self.scan_done_at) + self.drone_return_s
                print(f"[{vnow:6.1f}s] 小车先行到达终点 {RETURN_TILE}, "
                      f"无人机返航飞往终点 {DRONE_END_TILE}")
        elif self.state == "drone_return":
            if vnow >= self.drone_arrive_at:
                print(f"[{self.drone_arrive_at:6.1f}s] 无人机到达终点 {DRONE_END_TILE}"
                      f"(晚于小车 {self.drone_arrive_at - self.car_done_at:.1f}s), 任务完成")
                self.finish()

    def finish(self):
        self.finished = True
        self.state = "done"
        vnow = self.vnow
        ret = name2rc(RETURN_TILE)
        enemies = self.field.known_of(TARGET)
        friends = self.field.known_of(FRIEND)
        # 规则内基线: 地雷/敌方均贪心近邻顺序, 不做转弯优化
        base_cells = base_turns = 0
        seq = []
        cur = self.start_rc
        remaining = [e for e in self.field.known_of(EXPLOSIVE)
                     if e not in {s for s, _t in
                                  [(t[0] + self.field.blue(t)[0], t[1] + self.field.blue(t)[1])
                                   for t in enemies if self.field.blue(t)]}]
        while remaining:
            nxt = min(remaining, key=lambda e: man(cur, e))
            seq.append(nxt)
            remaining.remove(nxt)
            cur = nxt
        pairs = []
        for t in enemies:
            blue = self.field.blue(t)
            if blue:
                pairs.append(((t[0] + blue[0], t[1] + blue[1]), t))
        while pairs:
            nxt = min(range(len(pairs)), key=lambda i: man(cur, pairs[i][0]))
            s, t = pairs.pop(nxt)
            seq.append(s)
            seq.append(t)
            cur = t
        seq.append(ret)
        base_route = [self.start_rc]
        ok = True
        for g in seq:
            leg = self.planner.a_star(base_route[-1], g, turn_pen=0.0)
            if leg is None:
                ok = False
                break
            base_route.extend(leg[1:])
        if ok:
            base_cells = len(base_route) - 1
            base_turns = seg_turns(base_route)
        base_time = (base_cells * CELL_TIME_S + base_turns * TURN_TIME_S
                     + FACING_PAUSE_S + FALL_TIMES * FALL_INTERVAL)
        # 全流程线路: 小车实际走过的完整轨迹(含重规划/分段, 最终地图用)
        full = self.full_path if len(self.full_path) >= 2 else self.route
        our_cells = len(full) - 1
        our_time = estimate_time(full, self.full_roles)
        turns = seg_turns(full)
        bad = [f"{rc2name(a)}↔{rc2name(b)}" for a, b in zip(full, full[1:])
               if barrier_blocks_cross(a, b)]
        bad_obj = [rc2name(x) for x in full
                   if self.field.state(x) in (BUCKET, FRIEND)]
        scan_time = len(self.blocks) * BLOCK_SCAN_S + sum(self.flight_to_block)
        car_wall = self.car_done_at - self.wall_car_start
        n_ex = len(self.field.known_of(EXPLOSIVE))
        unstepped = [rc2name(x) for x in self.field.known_of(EXPLOSIVE)
                     if x not in self.collected]
        # 死路地雷(进出需掉头)可放弃, 单独统计
        dead_end = [rc2name(x) for x in self.field.known_of(EXPLOSIVE)
                    if x not in self.collected and self.planner.is_dead_end_mine(x)]
        unstepped = [n for n in unstepped if n not in dead_end]
        waited = None
        goals_f = self.planner.build_goals(self.start_rc, set(), set())
        if goals_f:
            fr, froles = self.planner.route_through(self.start_rc, goals_f)
            if fr:
                waited = TAKEOFF_S + scan_time + estimate_time(fr, froles) + self.drone_return_s
        print(render_map(self.field, full, self.full_roles, self.current_cell(),
                         "最终地图(全流程线路)", arrows={rc: True for rc in self.arrow_seen}))
        # 全流程线路航点(压缩为拐点/角色点)
        _ROLE_TAG = {"explosive": "踩雷", "stop_face": "停靠", "hit": "打击",
                     "return": "终点", "bridge": "桥洞"}
        wpts = compress(full, self.full_roles)
        wpt_str = " -> ".join(
            f"{p['cell']}({_ROLE_TAG[p['role']]})" if p.get("role") in _ROLE_TAG
            else p["cell"] for p in wpts)
        print(f"全流程线路({our_cells} 格, {len(wpts)} 个航点): {wpt_str}")
        print("=" * 56)
        print(f"===== 任务完成 (种子 {self.seed}) =====")
        print(f"起飞 {TAKEOFF_S:.1f}s | 巡航扫描 {scan_time:.1f}s | "
              f"小车行驶 {our_cells} 格, 拐弯 {turns} 次, 预估 {our_time:.1f}s")
        # 掉头(180°折返)统计: 规划已大幅加重惩罚, 尽量避免
        u_turns = 0
        pd = None
        for i in range(1, len(full)):
            d = sub(full[i], full[i - 1])
            if pd is not None and d == (-pd[0], -pd[1]):
                u_turns += 1
            pd = d
        print(f"掉头(180°折返): {u_turns} 次  {'✓ 已尽量避免' if u_turns == 0 else '(仅在必经死路时出现)'}")
        print(f"敌我识别: 敌 {len(enemies)} 个已打击 {self.hits_done} 轮"
              f"(命中 {self.hit_count} 发, 打靶{STRIKE_TIME_S:.0f}s)"
              + (" [超软预算已砍打靶]" if self.strike_shed else "")
              + f", 我方 {len(friends)} 个已避让(未碾压)")
        for t in enemies:
            blue = self.field.blue(t)
            s = (t[0] + blue[0], t[1] + blue[1]) if blue else None
            print(f"    敌 {rc2name(t)} 蓝面{DIR_NAME.get(blue)}, "
                  f"停靠 {rc2name(s) if s else '?'} 正对蓝面")
        print(f"挡板规避: {'通过(未跨越 D7~N7 横向挡板, 第 6/7 行边界)' if not bad else '违规! ' + str(bad)}")
        print(f"路径安全: {'通过(未压障碍物/我方)' if not bad_obj else '违规! ' + str(bad_obj)}")
        print(f"排雷: 已踩 {len(self.collected)}/{n_ex}"
              + ("" if not unstepped else f", 未踩: {unstepped}")
              + ("" if not dead_end else f", 死路放弃: {dead_end}"))
        if BRIDGE_ARROWS:
            print(f"桥洞可通行通道识别: {len(self.arrow_seen)}/2 "
                  f"(下段 {BRIDGE_LO_RANGE[0]}~{BRIDGE_LO_RANGE[1]} 5选1, "
                  f"上段 {BRIDGE_HI_RANGE[0]}~{BRIDGE_HI_RANGE[1]} 6选1"
                  f"{', 已识别: ' + ', '.join(rc2name(x) for x in self.arrow_seen) if self.arrow_seen else ''})")
            for rc in sorted(self.field.open_tunnels):
                print(f"    通道 {rc2name(rc)} "
                      f"{'✓穿过' if rc in self.bridge_crossed else '✗未穿'}")
        print(f"时间压缩: 基线(贪心序+不优化拐弯) {base_cells} 格/{base_turns} 转"
              f"({base_time:.1f}s) -> 本方案 {our_cells} 格/{turns} 转({our_time:.1f}s)")
        if base_time > 0:
            print(f"    路径优化省 {(1 - our_time / base_time) * 100:.0f}%")
        if self.args.early and waited is not None:
            actual = self.drone_arrive_at
            print(f"边扫边走: 实际总耗时 {actual:.1f}s vs 扫完再走 {waited:.1f}s, "
                  f"{'省 ' + format(waited - actual, '.1f') + 's' if waited > actual else '慢 ' + format(actual - waited, '.1f') + 's'}")
        total_actual = self.drone_arrive_at
        if total_actual > TOTAL_BUDGET_S:
            # 官方: 比赛时间达到 3 分钟自动结束, 用时计 180s
            total_actual = TOTAL_BUDGET_S
            self.drone_arrive_at = TOTAL_BUDGET_S
            self.timeout_hard = True
        fp = self.first_plan_at if self.first_plan_at is not None else self.wall_car_start
        print(f"时序: 无人机先起飞({TAKEOFF_S:.1f}s, 起飞点{DRONE_START_TILE}) -> "
              f"第一次路径规划完整、小车发车({fp:.1f}s) "
              f"-> 小车先到终点 {RETURN_TILE}({self.car_done_at:.1f}s) "
              f"-> 无人机后到终点 {DRONE_END_TILE}({self.drone_arrive_at:.1f}s)")
        if self.strike_shed:
            print(f"时间预算门控: 超{SOFT_BUDGET_S:.0f}s软预算已砍打靶停靠"
                  f"(放弃打靶 {SCORE_HIT * FALL_TIMES:.0f} 分, 敌方格避让未打)")
        if self.mine_shed:
            print(f"时间预算门控: 已放弃地雷 {', '.join(self.mine_shed)}"
                  f"(各弃 {SCORE_MINE:.0f} 分) 保完赛——路径顺路踩到仍计分")
        if self.timeout_hard:
            print(f"180s 硬顶: 官方比赛自动结束, 用时计 180s, 按已得分结算")
        if self.return_unreachable:
            print("降级兜底: 终点不可达, 按已执行部分结算(不崩)")
        full_score = SCORE_MINE * EXPLOSIVES_N + SCORE_BRIDGE * 2 + SCORE_HIT * FALL_TIMES
        print(f"得分: 排雷 {SCORE_MINE:.0f}×{len(self.collected)} + "
              f"桥洞 {SCORE_BRIDGE:.0f}×{len(self.bridge_crossed)} + "
              f"打靶 {SCORE_HIT:.0f}×{self.hit_count} = {self.score:.0f} 分 "
              f"(满分 {full_score:.0f} 分)"
              + ("" if self.score >= full_score - 1e-9 else " [未满分: 打靶已砍/未打满]"))
        remain = TOTAL_BUDGET_S - total_actual
        print(f"时限: 总耗时 {total_actual:.1f}s / 3分钟({TOTAL_BUDGET_S:.0f}s), "
              f"{'剩余 ' + format(remain, '.1f') + 's' if remain >= 0 else '超时 ' + format(-remain, '.1f') + 's !'}")
        print("=" * 56)
        self.export_map(f"最终地图(全流程线路) 总耗时{total_actual:.1f}s "
                        f"排雷{len(self.collected)}/{n_ex} 打击{self.hits_done}/{len(enemies)}",
                        duration_ms=1200, route=full, roles=self.full_roles)
        self.save_gif_now()
        self.link.send_done("任务完成")


# ============================ 可视化 ============================
def render_map(field, route=None, roles=None, car=None, title="", arrows=None):
    """ASCII 矩阵渲染(纯标准库, 无 cv2 时使用); arrows=桥洞上方箭头(已识别)"""
    route_set = set(route or [])
    roles = roles or {}
    arrows = arrows or {}
    lines = [title]
    lines.append("     " + "".join(f"{c:2d}" for c in range(1, COLS + 1)))
    lo_row = ROW_LETTERS.index(BARRIER_LO_ROW)
    hi_row = ROW_LETTERS.index(BARRIER_HI_ROW)
    b_col = max(BARRIER_COLS) - 1      # 挡板线画在第 max(BARRIER_COLS) 列左侧, 与禁跨约束同一条线
    for r in range(ROWS):
        parts = []
        for c in range(COLS):
            rc = (r, c)
            role = roles.get(rc)
            if rc in arrows:
                ch = "通"              # 桥洞可通行通道(顶部标识)
            elif car == rc:
                ch = "@"
            elif role == "stop_face":
                ch = "S"
            elif role == "hit":
                ch = "T"
            elif role == "return":
                ch = "R"
            elif role == "explosive":
                ch = "x"
            elif rc in route_set and field.state(rc) not in (BUCKET, FRIEND):
                ch = "+"
            elif field.is_bridge(rc) and field.state(rc) == EMPTY:
                ch = "="              # 桥洞结构(隔板隔离, 仅可通行通道可穿)
            else:
                ch = SYM[field.state(rc)]
            parts.append(ch)
        if lo_row <= r <= hi_row:
            parts.insert(b_col, "|")   # 官方横向挡板线(D~N 行, 第6/7列之间, 严禁跨越)
        lines.append(f" {ROW_LETTERS[r]}  " + " ".join(parts))
    lines.append("图例: .未知 #障碍 x地雷 T敌方(蓝面) =桥洞结构 通=可通行通道 "
                 "|挡板线(禁跨) +路径 S蓝面停靠位 R返程 @小车")
    return "\n".join(lines)


def cm_to_pixel(cm_x, cm_y, scale, img_w):
    """场地坐标 -> 像素(与原系统一致: A1 在右上, x 镜像)"""
    px = int((COLS * TILE_CM - cm_x) * scale)
    py = int(cm_y * scale)
    return px, py


def detect_marker_in_roi(roi, scale=8.0):
    """轻量霍夫圆识别(半径按 cm 折算):
    障碍物(水桶)直径26cm -> 半径13cm; 更小的圆按地雷/打击区区分。
    可整体替换为 YOLOv8n 等轻量模型。"""
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=15,
                               param1=50, param2=30, minRadius=8, maxRadius=40)
    if circles is None:
        return None
    circles = np.uint16(np.around(circles))
    r_cm = float(np.mean(circles[0][:, 2])) / max(scale, 1e-6)
    if r_cm >= OBSTACLE_D_CM / 2.0 - 3.0:     # 半径>=10cm 视为障碍物
        return BUCKET
    if r_cm >= 3.0:                           # 3~10cm 视为地雷
        return EXPLOSIVE
    return TARGET                              # 更小视为打击区(敌我由蓝面判定)


def detect_blue_dir(roi):
    """识别打击区蓝色面朝向(有蓝面=敌方, 无蓝面=我方):
    蓝块质心在 ROI 中偏向哪侧, 蓝面就朝哪侧"""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (90, 50, 50), (140, 255, 255))
    if int(mask.sum()) < 500:
        return None
    m = cv2.moments(mask)
    if m["m00"] == 0:
        return None
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    h, w = roi.shape[:2]
    dx, dy = cx - w / 2, cy - h / 2
    # 像素 -> 场地方向: x 镜像(dx>0 即场地左/W), y 同向(dy>0 即场地下/S)
    if abs(dx) >= abs(dy):
        return (0, -1) if dx > 0 else (0, 1)
    return (1, 0) if dy > 0 else (-1, 0)


def draw_sim_gui(m):
    """cv2 仿真可视化"""
    ps, mx, my = 52, 40, 96
    w, h = mx * 2 + COLS * ps, my * 2 + ROWS * ps + 30
    img = np.full((h, w, 3), 235, np.uint8)
    colors = {UNKNOWN: (185, 185, 185), EMPTY: (255, 255, 255),
              BUCKET: (70, 70, 230), EXPLOSIVE: (0, 220, 220),
              TARGET: (90, 90, 245), FRIEND: (200, 120, 220), LINE: (80, 200, 80)}
    for r in range(ROWS):
        for c in range(COLS):
            rc = (r, c)
            x0, y0 = mx + c * ps, my + r * ps
            color = colors[m.field.state(rc)]
            if m.field.is_bridge(rc) and m.field.state(rc) == EMPTY:
                color = (215, 215, 215)       # 桥洞可通行通道
            cv2.rectangle(img, (x0, y0), (x0 + ps - 2, y0 + ps - 2), color, -1)
            if m.field.state(rc) == TARGET and m.field.blue(rc):
                dr, dc = m.field.blue(rc)
                if dc == 1:
                    cv2.line(img, (x0 + ps - 3, y0), (x0 + ps - 3, y0 + ps - 2), (255, 120, 0), 5)
                elif dc == -1:
                    cv2.line(img, (x0, y0), (x0, y0 + ps - 2), (255, 120, 0), 5)
                elif dr == 1:
                    cv2.line(img, (x0, y0 + ps - 3), (x0 + ps - 2, y0 + ps - 3), (255, 120, 0), 5)
                else:
                    cv2.line(img, (x0, y0), (x0 + ps - 2, y0), (255, 120, 0), 5)
            cv2.putText(img, rc2name(rc), (x0 + 2, y0 + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 0, 0), 1)
    for i in range(len(m.route) - 1):
        a, b = m.route[i], m.route[i + 1]
        pa = (mx + a[1] * ps + ps // 2, my + a[0] * ps + ps // 2)
        pb = (mx + b[1] * ps + ps // 2, my + b[0] * ps + ps // 2)
        cv2.line(img, pa, pb, (180, 0, 180), 2)
    if m.state in ("executing", "done") and m.route:
        rc = m.route[m.route_i]
        pt = (mx + rc[1] * ps + ps // 2, my + rc[0] * ps + ps // 2)
        cv2.circle(img, pt, 12, (0, 180, 0), -1)
    if not m.scan_done and m.scan_idx < len(m.blocks):
        blk = m.blocks[m.scan_idx]
        rr = sum(x[0] for x in blk) // len(blk)
        cc = sum(x[1] for x in blk) // len(blk)
        dp = (mx + cc * ps + ps // 2, my + rr * ps + ps // 2 - 24)
    else:
        rc = m.route[m.route_i] if m.route else m.start_rc
        dp = (mx + rc[1] * ps + ps // 2, my + rc[0] * ps + ps // 2 - 24)
    cv2.circle(img, dp, 9, (0, 0, 255), -1)
    info = (f"seed={m.seed}  {m.state}  规划#{max(m.seq - 1, 0)}  "
            f"雷{len(m.collected)}/{len(m.field.known_of(EXPLOSIVE))}  "
            f"敌{len(m.field.known_of(TARGET))} 我{len(m.field.known_of(FRIEND))}")
    cv2.putText(img, info, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
    cv2.putText(img, "黄带=挡板线(禁跨) 灰格=桥洞 红=障碍 黄=地雷 橙边=敌蓝面 绿圆=小车 红圆=无人机",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
    return img


def draw_cam(frame, m):
    img_h, img_w = frame.shape[:2]
    colors = {UNKNOWN: (190, 190, 190), EMPTY: (255, 255, 255),
              BUCKET: (0, 0, 255), EXPLOSIVE: (0, 220, 220),
              TARGET: (0, 128, 255), FRIEND: (200, 120, 220), LINE: (0, 200, 0)}
    for r in range(ROWS):
        for c in range(COLS):
            rc = (r, c)
            x0, y0 = c * TILE_CM, r * TILE_CM
            px0, py0 = cm_to_pixel(x0, y0, m.scale, img_w)
            px1, py1 = cm_to_pixel(x0 + TILE_CM, y0 + TILE_CM, m.scale, img_w)
            cv2.rectangle(frame, (px0, py0), (px1, py1),
                          colors[m.field.state(rc)], 2)
            cx, cy = cm_to_pixel(x0 + TILE_CM / 2, y0 + TILE_CM / 2, m.scale, img_w)
            cv2.putText(frame, rc2name(rc), (cx - 10, cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
    for i in range(len(m.route) - 1):
        a, b = m.route[i], m.route[i + 1]
        p1 = cm_to_pixel((a[1] + 0.5) * TILE_CM, (a[0] + 0.5) * TILE_CM, m.scale, img_w)
        p2 = cm_to_pixel((b[1] + 0.5) * TILE_CM, (b[0] + 0.5) * TILE_CM, m.scale, img_w)
        cv2.line(frame, p1, p2, (200, 0, 255), 2)
    if m.state in ("executing", "done") and m.route:
        rc = m.route[m.route_i]
        pt = cm_to_pixel((rc[1] + 0.5) * TILE_CM, (rc[0] + 0.5) * TILE_CM, m.scale, img_w)
        cv2.circle(frame, pt, 10, (0, 200, 0), -1)
    cv2.putText(frame, f"{m.state} seed={m.seed} plan#{max(m.seq - 1, 0)}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame


# ============================ 运行入口 ============================
def run_sim(args):
    m = Mission(args)
    print(f"===== 无人机全局感知 + 小车精确执行 仿真 (seed={m.seed}) =====")
    print(f"场地 {ROWS}x{COLS} 格 ({TILE_CM:.0f}cm/格) | "
          f"小车 {START_TILE}->{RETURN_TILE}, 无人机 {DRONE_START_TILE}->{DRONE_END_TILE}")
    print(f"中场横向挡板(官方): D7~N7, 第 6/7 行边界, A/B/C 列自由穿越"
          f"(E 版坐标系转置: 第 {BARRIER_COLS[0]}/{BARRIER_COLS[1]} 列之间 "
          f"{BARRIER_LO_ROW}~{BARRIER_HI_ROW} 行禁跨; 只拦穿越不占格)")
    print(f"小车 {CAR_SPEED_MS:.1f}m/s ({CELL_TIME_S:.2f}s/格), "
          f"无法原地拐90°(三点调头), 90°拐弯≈{TURN_TIME_S:.2f}s, "
          f"180°掉头×{U_TURN_FACTOR}(规划避免掉头)")
    print(f"障碍物 {BUCKETS_N} 个(水桶, 直径{OBSTACLE_D_CM:.0f}cm/高{OBSTACLE_H_CM:.0f}cm, 占一格) | "
          f"桥洞{BRIDGE_CELLS[0]}~{BRIDGE_CELLS[-1]} 高{BRIDGE_HEIGHT_CM:.0f}cm, "
          f"上下通道各随机指定 1 个可通行通道: "
          f"下段 {BRIDGE_LO_RANGE[0]}~{BRIDGE_LO_RANGE[1]} 5选1, "
          f"上段 {BRIDGE_HI_RANGE[0]}~{BRIDGE_HI_RANGE[1]} 6选1, 顶部箭头标明")
    print(f"地雷 {EXPLOSIVES_N} 个(生成在 J~A 行之间, 尽量全踩; 死路地雷可放弃) | "
          f"打击区域: 相连两单元内 1 个双面移动电子靶标(蓝面=敌/红面=友, 朝向随机, 位于 "
          f"{TARGET_ROWS[1]}~{TARGET_ROWS[0]} 行之间), "
          f"打靶{STRIKE_TIME_S:.0f}s(官方 10 发, 仿真 3 发标定) | "
          f"动态障碍={'开' if DYNAMIC_OBSTACLE else '关'} | 时限 {TOTAL_BUDGET_S:.0f}s")
    print(f"时序: 无人机先起飞 -> U形左右扫描(N11~D7右侧 -> C11~A1顶部 -> D1~N6左侧)"
          f" -> 扫描 {DEPART_AFTER_BLOCKS} 块后小车启动"
          f" -> 小车先到终点 -> 无人机后到终点(返航{m.drone_return_s:.1f}s)")
    print(f"无人机飞行限高 {DRONE_MAX_ALT_M:.1f}m, 速度 {DRONE_SPEED_MS:.1f}m/s, "
          f"起飞后单次扫描 {BLOCK_ROWS}x{BLOCK_COLS} 范围"
          f" -> 分 {len(m.blocks)} 块扫描建图; 边扫边走={'开' if args.early else '关'}; "
          f"航点链路={args.link}")
    if args.wait_car:
        m.link.wait_car(args.car_timeout)
    gui = args.gui and HAS_CV
    if args.gui and not HAS_CV:
        print("提示: 未安装 cv2, 已退回 ASCII 模式")
    win = "UAV-Car Mission"
    if gui:
        cv2.namedWindow(win)
    last_real = time.time()
    vnow = 0.0
    try:
        while True:
            now_real = time.time()
            dt = min(0.1, now_real - last_real)
            last_real = now_real
            vnow += dt * args.speed
            m.tick(vnow, dt * args.speed)
            if vnow > TOTAL_BUDGET_S * 2.0 and m.state == "executing":
                print(f"[{vnow:6.1f}s] 超时限2倍仍未完成, 强制结束")
                m.state = "abort"
            if gui:
                cv2.imshow(win, draw_sim_gui(m))
                key = cv2.waitKey(10) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("p") and m.state == "executing":
                    m.replan_from_current()
            else:
                time.sleep(0.02)
            if m.state in ("done", "abort"):
                if gui:
                    print("任务结束, 按 q 关闭窗口")
                    while True:
                        cv2.imshow(win, draw_sim_gui(m))
                        if (cv2.waitKey(10) & 0xFF) == ord("q"):
                            break
                break
    finally:
        m.save_gif_now()               # 兜底保存过程动画(任务中途退出时)
        if args.link == "tcp":
            time.sleep(1.5)          # 给小车留出收尾时间
        m.link.close()
        if gui:
            cv2.destroyAllWindows()
    print("程序退出")


def run_camera(args):
    if not HAS_CV:
        print("相机模式需要 opencv-python; 无相机请用默认仿真模式")
        return
    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    m = Mission(args, vision=True)
    if args.wait_car:
        m.link.wait_car(args.car_timeout)
    win = "UAV-Car Mission"
    cv2.namedWindow(win)
    last_real = time.time()
    vnow = 0.0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            m.frame = frame
            now_real = time.time()
            dt = min(0.1, now_real - last_real)
            last_real = now_real
            vnow += dt * args.speed
            m.tick(vnow, dt * args.speed)
            cv2.imshow(win, draw_cam(frame, m))
            key = cv2.waitKey(10) & 0xFF
            if key == ord("q"):
                break
            if key == ord("p") and m.state == "executing":
                m.replan_from_current()
            if m.state in ("done", "abort"):
                print("任务结束, 按 q 关闭窗口")
                while True:
                    cv2.imshow(win, draw_cam(m.frame or frame, m))
                    if (cv2.waitKey(10) & 0xFF) == ord("q"):
                        break
                break
    finally:
        m.save_gif_now()               # 兜底保存过程动画
        if args.link == "tcp":
            time.sleep(1.5)
        m.link.close()
        cap.release()
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="无人机全局感知+小车精确执行: 分块建图+路径规划+航点下发")
    ap.add_argument("--seed", type=int, default=None, help="随机种子(复现)")
    ap.add_argument("--gui", action="store_true", help="cv2 可视化(默认 ASCII)")
    ap.add_argument("--cam", type=int, default=None, help="相机编号, 进入真实视觉模式")
    ap.add_argument("--scale", type=float, default=8.0, help="相机模式标定 1cm=像素")
    ap.add_argument("--link", choices=["sim", "file", "tcp"], default="sim",
                    help="航点下发方式")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9600)
    ap.add_argument("--link-path", default="waypoints.json")
    ap.add_argument("--wait-car", action="store_true",
                    help="TCP 模式: 起飞前先等小车接入握手")
    ap.add_argument("--car-timeout", type=float, default=30.0,
                    help="等待小车接入超时(秒)")
    ap.add_argument("--early", action="store_true", default=EARLY_START,
                    help="路径规划无需敌打击区, 首次规划完整即可发车(边扫边走)")
    ap.add_argument("--no-early", action="store_false", dest="early")
    ap.add_argument("--speed", type=float, default=1.0, help="仿真倍速(不影响评估值)")
    ap.add_argument("--verbose", action="store_true", help="每块扫描都打印地图")
    ap.add_argument("--map-out", default=None,
                    help="实时地图导出路径: 每扫一块/每次规划都覆盖更新该 PNG(需 Pillow)")
    ap.add_argument("--map-dir", default=None,
                    help="实时地图快照目录: 每次更新另存 map_xxx.png 编号快照(需 Pillow)")
    ap.add_argument("--gif-out", default=None,
                    help="过程动画导出: 逐块填充+小车行驶全过程录制为 GIF(需 Pillow)")
    ap.add_argument("--step-map", action="store_true",
                    help="控制台逐步展示: 每填充一块打印一次完整地图")
    args = ap.parse_args()
    if args.cam is not None:
        run_camera(args)
    else:
        run_sim(args)


if __name__ == "__main__":
    main()
