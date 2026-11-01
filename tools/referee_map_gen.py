# -*- coding: utf-8 -*-
"""裁判规则地图生成器（WB-23）—— 与 `tools/map_synth.py` **并行、不混用**。

═══════════════════════════════════════════════════════════════════════════════
一、这个文件为什么存在，以及它**不是**什么
═══════════════════════════════════════════════════════════════════════════════
现役 `tools/map_synth.py` 是 **14 列（A–N）× 11 行**的场地模型，服务「tier-1 台架
合法地图」用途。裁判程序的生成规则是 **另一套格网**：

| 项 | map_synth.py | 本文件（裁判规则） |
|---|---|---|
| 列数 | 14 列（A–N） | **9 列（u=0..8）** |
| 候选池 | 除障碍/靶外每格 | **固定 99 格**（x%=4.5+7u, y%=6.25+8.75v）|
| 雷规则 | 不进 K 列（桥洞物理） | **同类同列最多 1**，无桥洞概念 |
| 选优 | 无（构造参考解） | **3000 轮 S 评价选优** |

⇒ **两套格网根本不同，不许合成一个混合体。** 本文件是**新建的独立生成器**；
`map_synth.py` 的规则**零改动**（本文件不 import 它、不改它、不调用它的私有函数）。

═══════════════════════════════════════════════════════════════════════════════
二、证据来源（静态分析，**未运行裁判 exe**）
═══════════════════════════════════════════════════════════════════════════════
- `map_data_findings.txt`（87 行）：99 格坐标表、雷/障约束伪代码、3000 轮循环、
  S 公式、通点/靶点算法，**每条都带反汇编地址**（.text 内偏移 + 行号）。
- `map_rng_notes.txt`（37 行）：`std::mt19937` 播种（random_device ⊕ 毫秒低 32 位）、
  生命周期（同一窗口多次点击不重 seed；重开窗口新建管理器并重 seed）、
  洗牌/重试语义、**失败无外层重试**。
- `map_code_match.txt`：`materialpoint.o` 与 `qt_referee.exe` 逐字节对照
  nonrelocation **16588/16588, mismatches=0**。

**证据边界（决定本实现能声称什么，见 §八）**：
1. **未运行裁判 exe、未调用目标函数** ⇒ 输出分布/随机性**未实测**。
2. 与 exe 的字节级一致**只证明「exe 含这些代码」**，**不证明运行时行为**。

═══════════════════════════════════════════════════════════════════════════════
三、坐标三层（**不许混**）
═══════════════════════════════════════════════════════════════════════════════
    格索引 (u,v)  →  百分比 (4.5+7u, 6.25+8.75v)  →  场景坐标（仿真台已标定换算）

- **邻接判定只用格索引**（本文件全部约束都在 (u,v) 上判，从不拿百分比做距离）。
- **百分比不是厘米**：`x%=4.5+7u` 是**地图宽高百分比**（UI 把坐标乘 sceneRect 宽高再除 100）。
- **不许假设图像纵轴 = 世界纵轴**：本文件**不输出**世界坐标，只给格索引与百分比；
  换算留给调用方用仿真台**已标定**的换算（本卡不做那个换算，见 §九 未知项）。
- 底图只作背景，**不参与**候选生成/道路识别/障碍检测（本文件不读任何图像）。

═══════════════════════════════════════════════════════════════════════════════
三-b、**物理尺度：尚未校准（这是"未校准"不是"已对齐"）**
═══════════════════════════════════════════════════════════════════════════════
卡面要求：米制坐标/原点/轴向/裁剪/缩放**一律采用仿真台现有已验证的场地标定**
（`src/dzbrain/dzbrain/core/geometry.py`）。**但实测该标定与本生成器不相容**：

| 项 | `core/geometry.py`（仿真台现役标定） | 本生成器（裁判规则） |
|---|---|---|
| 列数 | `MAP_COLS = 14`（`COL_LETTERS = "ABCDEFGHIJKLMN"`） | **9 列（u=0..8）** |
| 格边 | `BLOCK_SIZE = 0.35` m | **无米制定义**（裁判只给百分比） |
| 范围 | `MAP_X_MIN/MAX = 0.175/3.675`、`MAP_Y_MIN/MAX = 0.175/4.725` | 同上，不适用 |

⇒ **本文件不输出米制坐标**，理由是：把 9 列裁判格网硬套到 14 列标定会得到**错的米数**，
而那比"不输出"更危险（错的数会被下游当真）。
⇒ 本文件只输出**格索引 / 百分比 / 显示像素**三层（各自可算可对账）。
⇒ **物理尺度明确标记为「尚未校准」**：需要在 `dzrobot_v2` 侧新增一份**裁判 9 列专用标定**
（谁做、何时做不在本卡范围）。**不得**把本文件的百分比输出当成厘米或米。

═══════════════════════════════════════════════════════════════════════════════
四、两种模式（**必须显式分开**，见 acceptance「模式」条）
═══════════════════════════════════════════════════════════════════════════════
- **`referee`（默认）**：忠实保留约束 / 选优顺序 / 失败行为。唯一「裁判规则」口径。
- **`enhanced`（显式 `--mode enhanced`）**：在裁判规则基础上**额外**加
  可达性筛选 / 车体碰撞 / 失败重试。**每一项都会在输出里标 `non_referee_rule`**，
  绝不允许把新增规则称为「裁判规则」。

★ 原生成器**没有**道路连通性/车体尺寸/实体碰撞检查 ⇒ **不要默认所有裁判地图都可达**；
本文件**不悄悄过滤困难图**（`referee` 模式下 100% 忠实，含不可达图）。

═══════════════════════════════════════════════════════════════════════════════
五、失败语义（**必须真失败**）
═══════════════════════════════════════════════════════════════════════════════
- 3000 轮无完整雷障布局 ⇒ 生成**失败**（`ok=False, reason="random_selector_exhausted"`）。
- 最佳雷障布局容不下合法靶对 ⇒ 生成**失败**（`ok=False, reason="target_pair_failed"`）。
- **没有**「退回次优布局」的外层重试 —— 裁判没有，本文件**也不加**。
  （`enhanced` 模式允许重试，但会标 `non_referee_rule` 并写明次数。）

═══════════════════════════════════════════════════════════════════════════════
六、跨语言一致性边界（**这条决定你能声称什么**）
═══════════════════════════════════════════════════════════════════════════════
仅用同一个 `mt19937` + 同一种子**不足以**保证与裁判逐点一致：
候选枚举顺序 / 洗牌实现 / 有界抽样 / 随机数消耗顺序**都会影响结果**。
⇒ **未经与裁判实测对照，不得宣称「与裁判逐种子一致」。**
本文件能声称的只有：**自身实现版本内、同种子逐位可复现**（有断言证明）。

═══════════════════════════════════════════════════════════════════════════════
七、已知的「合法布局中列项恒 −20」（**不是新约束**）
═══════════════════════════════════════════════════════════════════════════════
证据 §3 原文：「由于每类同列最多 1，合并每列最多 2，列项在 16 点的合法布局中总惩罚固定 20」。
⇒ 在**合法**布局里 `sum(abs(col[u]-2))` **恒等于 2**（8 列各 2 点、1 列为 0 点 ⇒ `|0-2|=2`），
  故该项 **恒为 −20**。（★ 我第一版 docstring 误写成「恒等于 20 ⇒ −200」，
  是把「固定 20」这个惩罚值当成了求和值；实测 `column_term=-20` 才对上证据。）
本文件**照公式实现**（不硬编码 −20），并在输出里**如实注明**这个性质；
**绝不许**把它当成"可以省掉该项"或"可以加一条新约束"的理由。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
from pathlib import Path

# ───────────────────────────────────────────────────────────────────────────
# 一、常量：全部来自证据，每条带出处
# ───────────────────────────────────────────────────────────────────────────

#: 横坐标百分比表（证据 §1：`X=[4.5,11.5,...,60.5]`，`.rdata+120..160`）
X_PCT = [4.5, 11.5, 18.5, 25.5, 32.5, 39.5, 46.5, 53.5, 60.5]
#: 纵坐标百分比表（证据 §1：`Y=[6.25,15,...,93.75]`，`.rdata+C0..110`）
Y_PCT = [6.25, 15.0, 23.75, 32.5, 41.25, 50.0, 58.75, 67.5, 76.25, 85.0, 93.75]

U_MAX = 8          # u ∈ [0,8]
V_MAX = 10         # v ∈ [0,10]
N_CELLS = (U_MAX + 1) * (V_MAX + 1)   # 99

#: 小区域 Z：证据 §2「`Z={(u,v): u<=2 且 v 为 5 或 6}`，内雷障合计最多 1 点；**不是完全禁区**」
Z_ZONE = frozenset((u, v) for u in range(0, 3) for v in (5, 6))
ZONE_MAX = 1       # 合计最多 1（不是 0！）

#: 20 点类别（证据 §2：ID1..8 雷 A、ID9..16 障 B、ID17..18 通、ID19..20 靶）
N_MINE = 8
N_OBST = 8
N_PASS = 2         # 编号 17/18；**不赋予任何业务含义**（见下）
N_TARGET = 2

#: 3000 轮（证据 §2：`0xBB8=3000`，`.text+2012`）
N_ROUNDS = 3000

#: 评价函数系数（证据 §3：`.rdata+98=30, +A0=20, +A8=8, +B0=10, +C8=15`）
W_LOWHIGH = 30
W_LEFTRIGHT = 20
W_CENTER = 8
W_COLUMN = 10
W_ROWCROWD = 15

#: 通点 x 固定 74.5%（证据 §4：`通17=(74.5,Y[rand 0..5])`、`通18=(74.5,Y[rand 6..10])`）
PASS_X_PCT = 74.5
PASS17_V_MAX = 5   # v ∈ [0,5]（6 个候选）
PASS18_V_MIN = 6   # v ∈ [6,10]（5 个候选）

CENTER_U = 4       # 证据 §3：`nCenter = count(u==4)`
CENTER_TARGET = 2  # 证据 §3：`-8*abs(nCenter-2)`

GEN_VERSION = "wb23-referee/1"

# ───────────────────────────────────────────────────────────────────────────
# 二、基础几何（**只用格索引**）
# ───────────────────────────────────────────────────────────────────────────


def cell_pct(cell):
    """格索引 → (x%, y%)。**百分比不是厘米**，见 §三。"""
    u, v = cell
    return (X_PCT[u], Y_PCT[v])


def cell_pixels(cell, img_w, img_h):
    """格索引 → 显示像素。**卡面要求的第三层**（kb ``x%=4.5+7u``、``y%=6.25+8.75v``）。

    证据 §1：「UI 代理独立确认预览和主地图均将坐标乘 sceneRect 宽高再除 100」：
        px = img_w * x% / 100 ,  py = img_h * y% / 100

    ★ 三层各自可算、可对账：``(u,v)`` → ``(x%, y%)`` → ``(px, py)``。
    ★ **底图只作背景**：本函数只做「百分比 → 像素」的算术，**不读任何图像**，
      不参与候选生成、不做道路识别、不做障碍检测。
    ★ **不许假设图像纵轴 = 世界纵轴**：本函数返回的是**图像像素**，
      纵轴方向取决于图片本身的约定；本文件**不把它当世界 y**。
    """
    x_pct, y_pct = cell_pct(cell)
    return (img_w * x_pct / 100.0, img_h * y_pct / 100.0)


def l1(a, b):
    """曼哈顿距离。证据 §2：同类内禁止 `|Δu|+|Δv|==1`。"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def all_cells():
    """99 格候选池（按 u 外层 v 内层枚举；★ 顺序在此固定，跨语言一致性见 §六）。"""
    return [(u, v) for u in range(U_MAX + 1) for v in range(V_MAX + 1)]


def target_pairs():
    """靶对候选（证据 §5 伪代码逐行照抄）。

    ★ 端点规则：
      - 跳过起点在 Z 的格；
      - 水平右邻若在 Z 也跳过；
      - 竖直上邻若在 Z 也跳过；
      - **额外排除跨 v=5→6 的竖边**（`v != 5`）。
    """
    pairs = []
    for v in range(V_MAX + 1):
        for u in range(U_MAX + 1):
            if (u, v) in Z_ZONE:
                continue
            if u < U_MAX and (u + 1, v) not in Z_ZONE:
                pairs.append(((u, v), (u + 1, v)))
            if v < V_MAX and v != 5 and (u, v + 1) not in Z_ZONE:
                pairs.append(((u, v), (u, v + 1)))
    return pairs


# ───────────────────────────────────────────────────────────────────────────
# 三、评价函数 S（证据 §3 逐项照抄，**不硬编码常数项**）
# ───────────────────────────────────────────────────────────────────────────


def layout_score(mines, obsts):
    """16 点布局评价（生成器内部量，**不是比赛得分**）。

    ★ 不硬编码"列项恒 −20"：虽然合法布局里它恒等于 −20（证据 §3「固定 20」），
      但照公式算才能保证**非法布局也得到正确的分**（测试会用非法布局反向验证）。
    """
    pts = list(mines) + list(obsts)
    n_low = sum(1 for (_u, v) in pts if v <= 5)
    n_high = sum(1 for (_u, v) in pts if v > 5)
    n_left = sum(1 for (u, _v) in pts if u <= 3)
    n_right = sum(1 for (u, _v) in pts if u >= 5)
    n_center = sum(1 for (u, _v) in pts if u == CENTER_U)
    col = [0] * (U_MAX + 1)
    row = [0] * (V_MAX + 1)
    for (u, v) in pts:
        col[u] += 1
        row[v] += 1
    return (float(-W_LOWHIGH * abs(n_low - n_high))
            + float(-W_LEFTRIGHT * abs(n_left - n_right))
            + float(-W_CENTER * abs(n_center - CENTER_TARGET))
            + float(-W_COLUMN * sum(abs(c - 2) for c in col))
            + float(-W_ROWCROWD * sum(max(r - 2, 0) for r in row)))


def column_term(mines, obsts):
    """单独返回列项 `-10*Σ|col-2|`，供测试验证「合法布局恒 −20」这个性质。"""
    pts = list(mines) + list(obsts)
    col = [0] * (U_MAX + 1)
    for (u, _v) in pts:
        col[u] += 1
    return float(-W_COLUMN * sum(abs(c - 2) for c in col))


# ───────────────────────────────────────────────────────────────────────────
# 四、单轮雷/障选择（证据 §2 伪代码逐行照抄）
# ───────────────────────────────────────────────────────────────────────────


def _pick_class(rng, order, used, cls, want):
    """在一个已打乱的候选序列上贪心选 `want` 个同类别点。

    约束（证据 §2：「同上，但类内约束针对 B」）：
      ① 不在 `used`（跨类只禁重叠 —— **允许正交相邻**）
      ② 列约束：同类内 `first`(=u) 不得重复（即**同类同列最多 1**）
      ③ 距离约束：不得与同类已有点 `L1 == 1`（**对角一律允许**）
      ④ Z 区：`p in Z and count(used ∩ Z) > 0` ⇒ 跳过（**合计最多 1**，不是禁区）

    ⚠️ `used` 在跨类时**同时含** A 与 B —— 因此雷的 Z 计数已经算上障，反之亦然。
       证据原文是 `count(used∩Z)`，这里**严格照抄**，不做"只算本类"的改写。

    ⚠️ **轮内不回溯**：选不满就 `return None`，由调用方作废整轮。
    """
    picked = []
    for p in order:
        if p in used:                       # ① 重叠
            continue
        if p in Z_ZONE and sum(1 for q in used if q in Z_ZONE) >= ZONE_MAX:
            continue                        # ④ Z 区合计上限
        if any(q[0] == p[0] for q in cls):  # ② 同类同列最多 1
            continue
        if any(l1(p, q) == 1 for q in cls): # ③ 同类内禁 L1==1
            continue
        cls.append(p)
        used.append(p)
        picked.append(p)
        if len(cls) == want:
            break
    return picked if len(picked) == want else None


def random_select_mines_obsts(rng, rounds=N_ROUNDS, trace=None):
    """3000 轮选优（证据 §2）。

    ★ 五条必须照做的顺序语义：
      ① **严格优于才替换**（`score > best_score`），同分保留先前；
      ② **不在找到首个合法布局时提前结束**（必须跑满 `rounds` 轮）；
      ③ 雷：`shuffle` 后贪心选 8，**不足则该轮作废**；
      ④ 障：**再洗牌**（不是复用雷那一轮的打乱结果）后贪心选 8；
      ⑤ 失败：一轮没选满就 `continue`，不消耗额外随机数以外的任何东西。

    `trace`（可选 dict）：回填统计（成功轮数等），供报告用。
    """
    best_score = -1e9
    best_mines = best_obsts = None
    rounds_ok = 0            # 雷+障都选满 8 的轮数
    rounds_mine_fail = 0     # 雷阶段就作废的轮数
    rounds_obst_fail = 0     # 雷成功、障作废的轮数

    cells = all_cells()
    for _r in range(rounds):
        order = list(cells)
        rng.shuffle(order)                       # ③ 雷：洗牌
        mines, obsts, used = [], [], []
        got = _pick_class(rng, order, used, mines, N_MINE)
        if got is None:
            rounds_mine_fail += 1
            continue                             # ③ 不足 ⇒ 该轮作废

        rng.shuffle(order)                       # ④ 障：**再洗牌**（关键！）
        got_b = _pick_class(rng, order, used, obsts, N_OBST)
        if got_b is None:
            rounds_obst_fail += 1
            continue                             # ④ 不足 ⇒ 该轮作废

        rounds_ok += 1
        score = layout_score(mines, obsts)
        if score > best_score:                   # ① 严格优于才替换
            best_score, best_mines, best_obsts = score, list(mines), list(obsts)
        # ② 不 break：继续跑满 rounds

    if trace is not None:
        trace["rounds_ok"] = rounds_ok
        trace["rounds_mine_fail"] = rounds_mine_fail
        trace["rounds_obst_fail"] = rounds_obst_fail
        trace["rounds_total"] = rounds
        trace["best_score"] = None if best_mines is None else best_score

    if best_mines is None:
        return None, None, best_score
    return best_mines, best_obsts, best_score


def pick_pass_points(rng):
    """通点 17/18（证据 §4）。

    ★ **不许赋予业务含义**：它们只是两个 `x=74.5%` 的固定 x、y 分别随机取的点。
      名字叫「通」是裁判 UI 的编号，**不等于补给/通信/通行**。本文件不解释它的用途。
    ★ **不参与 3000 轮评价、不属于 99 格池**（证据 §4：候选来自 v 值域，不是格池）。
    """
    v17 = rng.randint(0, PASS17_V_MAX)       # v ∈ [0,5]
    v18 = rng.randint(PASS18_V_MIN, V_MAX)   # v ∈ [6,10]
    return [(PASS_X_PCT, Y_PCT[v17], v17), (PASS_X_PCT, Y_PCT[v18], v18)]


def pick_targets(rng, mines, obsts):
    """靶点 19/20（证据 §5）：洗牌 pair 列表 → 取第一个满足条件的。

    条件：
      ① 两端都未被雷/障占用；
      ② 两端**都不与任意雷或障正交相邻**（`L1==1`）。
    ★ 失败 ⇒ 返回 `(None, None)`；**不做外层重试**（证据 §5 末句明写没有）。
    """
    pairs = list(target_pairs())
    rng.shuffle(pairs)
    occupied = set(mines) | set(obsts)
    for (p, q) in pairs:
        if p in occupied or q in occupied:
            continue
        if any(l1(p, r) == 1 or l1(q, r) == 1 for r in occupied):
            continue
        return p, q
    return None, None


# ───────────────────────────────────────────────────────────────────────────
# 五、单张生成（含失败语义）
# ───────────────────────────────────────────────────────────────────────────


def pixel_layer(pts, img_size):
    """把 20 点的百分比换成像素。``img_size=None`` ⇒ 返回 ``None``（**不造假值**）。"""
    if not img_size:
        return None
    w, h = img_size
    out = {"img_w": w, "img_h": h, "cells": {}}
    for grp in ("mines", "obsts", "targets"):
        for p in pts[grp]:
            px, py = cell_pixels((p["u"], p["v"]), w, h)
            out["cells"]["%s:%d" % (grp, p["id"])] = [px, py]
    for p in pts["passes"]:
        px = w * p["x_pct"] / 100.0
        py = h * p["y_pct"] / 100.0
        out["cells"]["passes:%d" % p["id"]] = [px, py]
    return out


def generate_one(seed, rounds=N_ROUNDS, mode="referee", img_size=None):
    """生成一张。返回 dict（`ok` 决定成败）。

    `referee` 模式**完全不加重试**；`enhanced` 模式允许重试但每步都标 `non_referee_rule`。
    """
    if mode not in ("referee", "enhanced"):
        raise ValueError("mode 必须是 referee / enhanced，收到 %r" % (mode,))

    flags = []
    rng = random.Random(seed)
    trace = {}
    mines, obsts, score = random_select_mines_obsts(rng, rounds=rounds, trace=trace)
    if mines is None:
        return {"ok": False, "reason": "random_selector_exhausted", "seed": seed,
                "mode": mode, "trace": trace, "version": GEN_VERSION}

    tgt = pick_targets(rng, mines, obsts)
    if tgt[0] is None:
        if mode == "referee":
            # 证据 §5：靶对筛选失败后**直接返回空列表**，未见重启 3000 轮
            return {"ok": False, "reason": "target_pair_failed", "seed": seed,
                    "mode": mode, "trace": trace, "version": GEN_VERSION}
        # enhanced：允许补跑（明标非裁判规则）
        flags.append({"rule": "target_retry", "non_referee_rule": True,
                      "detail": "裁判规则模式下靶对失败即失败；enhanced 允许补跑"})
        retries = 0
        while tgt[0] is None and retries < 5:
            retries += 1
            mines, obsts, score = random_select_mines_obsts(rng, rounds=rounds)
            if mines is None:
                break
            tgt = pick_targets(rng, mines, obsts)
        if tgt[0] is None:
            return {"ok": False, "reason": "target_pair_failed", "seed": seed,
                    "mode": mode, "trace": trace, "version": GEN_VERSION,
                    "non_referee_rules": flags}

    m, o = tgt
    passes = pick_pass_points(rng)
    pts = {
        "mines": [{"id": i + 1, "u": c[0], "v": c[1], "pct": cell_pct(c)}
                  for i, c in enumerate(mines)],
        "obsts": [{"id": N_MINE + i + 1, "u": c[0], "v": c[1], "pct": cell_pct(c)}
                  for i, c in enumerate(obsts)],
        "passes": [{"id": 17 + i, "x_pct": x, "y_pct": y, "v": v}
                   for i, (x, y, v) in enumerate(passes)],
        "targets": [{"id": 19, "u": m[0], "v": m[1], "pct": cell_pct(m)},
                    {"id": 20, "u": o[0], "v": o[1], "pct": cell_pct(o)}],
    }

    return {
        "ok": True, "reason": "", "seed": seed, "mode": mode,
        "version": GEN_VERSION,
        "points": pts,
        # ★ 显示像素层只在**显式给了图尺寸时**才算（不瞎猜）：
        #   没给 img_size 就不输出像素，而不是默认一个假尺寸。
        "pixels": pixel_layer(pts, img_size),
        # 便于逐位比对的紧凑串（**只用格索引**，不含浮点，避免格式化差异）
        "signature": signature_of(pts),
        "score": score,
        "column_term": column_term(mines, obsts),
        "z_zone_points": sorted([list(c) for c in (set(mines) | set(obsts)) & Z_ZONE]),
        "trace": trace,
        "non_referee_rules": flags,
    }


def signature_of(pts):
    """紧凑签名字符串：只由格索引与通点 v 组成（**无浮点**，便于稳定 sha256）。"""
    ms = "|".join("%d:%d,%d" % (p["id"], p["u"], p["v"]) for p in pts["mines"])
    os_ = "|".join("%d:%d,%d" % (p["id"], p["u"], p["v"]) for p in pts["obsts"])
    ps = "|".join("%d:%d" % (p["id"], p["v"]) for p in pts["passes"])
    ts = "|".join("%d:%d,%d" % (p["id"], p["u"], p["v"]) for p in pts["targets"])
    return "M[%s]O[%s]P[%s]T[%s]" % (ms, os_, ps, ts)


def sha256_of(result):
    """整张结果的 sha256（对**规范化 JSON** 求，不含 trace 里的计时等易变量）。"""
    payload = {k: result.get(k) for k in
               ("seed", "mode", "version", "points", "signature", "score")}
    blob = json.dumps(payload, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


# ───────────────────────────────────────────────────────────────────────────
# 六、约束自检（**给测试与 --report 用；不是生成逻辑**）
# ───────────────────────────────────────────────────────────────────────────


def validate(result):
    """对一张结果逐条查违规，返回违规列表（空 = 合法）。

    ★ 这条函数**故意不复用 `_pick_class`**：它是"独立复核"，复用生成器的判断逻辑
      等于自己给自己打分。所有约束在这里**重写一遍**。
    """
    bad = []
    if not result.get("ok"):
        return bad                     # 失败结果没有布局可查
    pts = result["points"]
    mines = [(p["u"], p["v"]) for p in pts["mines"]]
    obsts = [(p["u"], p["v"]) for p in pts["obsts"]]
    passes = pts["passes"]
    targets = [(p["u"], p["v"]) for p in pts["targets"]]
    all16 = mines + obsts

    # ① 各 8 个
    if len(mines) != N_MINE:
        bad.append("雷数 %d != %d" % (len(mines), N_MINE))
    if len(obsts) != N_OBST:
        bad.append("障数 %d != %d" % (len(obsts), N_OBST))
    # ID 固定
    if [p["id"] for p in pts["mines"]] != list(range(1, 9)):
        bad.append("雷 ID 不是 1..8")
    if [p["id"] for p in pts["obsts"]] != list(range(9, 17)):
        bad.append("障 ID 不是 9..16")
    if [p["id"] for p in pts["passes"]] != [17, 18]:
        bad.append("通 ID 不是 17,18")
    if [p["id"] for p in pts["targets"]] != [19, 20]:
        bad.append("靶 ID 不是 19,20")

    # ② 同类同列最多 1（各占 9 列中 8 列）
    for name, group in (("雷", mines), ("障", obsts)):
        us = [u for (u, _v) in group]
        if len(set(us)) != len(us):
            bad.append("%s 同类同列出现重复 u（%s）" % (name, us))
        if len(set(us)) != N_MINE:
            bad.append("%s 只占了 %d 列（应 %d 列）" % (name, len(set(us)), N_MINE))

    # ③ 同类内禁 L1==1（对角允许）
    for name, group in (("雷", mines), ("障", obsts)):
        for i in range(len(group)):
            for j in range(i + 1, len(group)):
                if l1(group[i], group[j]) == 1:
                    bad.append("%s 同类正交相邻: %s ~ %s" % (name, group[i], group[j]))

    # ④ 跨类仅禁重叠（**允许正交相邻** —— 这里只查重叠）
    if set(mines) & set(obsts):
        bad.append("雷障重叠: %s" % sorted(set(mines) & set(obsts)))

    # ⑤ Z 区合计 <=1（**非完全禁区**）
    z_in = [c for c in all16 if c in Z_ZONE]
    if len(z_in) > ZONE_MAX:
        bad.append("Z 区雷障合计 %d > %d（%s）" % (len(z_in), ZONE_MAX, z_in))

    # ⑥ 通点：x 固定 74.5，v 分别落在两侧
    if len(passes) != N_PASS:
        bad.append("通点数 %d != %d" % (len(passes), N_PASS))
    else:
        for p in passes:
            if abs(p["x_pct"] - PASS_X_PCT) > 1e-9:
                bad.append("通 %d 的 x_pct=%s != 74.5" % (p["id"], p["x_pct"]))
        if not (0 <= passes[0]["v"] <= PASS17_V_MAX):
            bad.append("通 17 的 v=%d 不在 [0,5]" % passes[0]["v"])
        if not (PASS18_V_MIN <= passes[1]["v"] <= V_MAX):
            bad.append("通 18 的 v=%d 不在 [6,10]" % passes[1]["v"])

    # ⑦ 靶：正交相邻、在 157 对候选里、不占雷障、不与雷障正交相邻、端点不在 Z
    if len(targets) != N_TARGET:
        bad.append("靶数 %d != %d" % (len(targets), N_TARGET))
    else:
        if l1(targets[0], targets[1]) != 1:
            bad.append("靶 19/20 不是正交相邻（L1=%d）" % l1(targets[0], targets[1]))
        if (targets[0], targets[1]) not in target_pairs() and \
           (targets[1], targets[0]) not in target_pairs():
            bad.append("靶对 (%s,%s) 不在预筛候选里" % (targets[0], targets[1]))
        if set(targets) & set(all16):
            bad.append("靶与雷障重叠")
        for t in targets:
            if t in Z_ZONE:
                bad.append("靶端点 %s 落在 Z 区" % (t,))
            if any(l1(t, r) == 1 for r in all16):
                bad.append("靶端点 %s 与雷/障正交相邻" % (t,))

    # ⑧ 不许出现多余约束的痕迹：本文件不该产生欧氏距离/对角隔离的判断
    #    （这里只查"对角相邻是否被误判为违规"——合法结果里对角相邻应当**允许存在**）
    #    该性质由测试用人造对角布局正向验证，此处不做断言。
    return bad


# ───────────────────────────────────────────────────────────────────────────
# 七、批量与 CLI
# ───────────────────────────────────────────────────────────────────────────


def generate_batch(seed0, count, rounds=N_ROUNDS, mode="referee", img_size=None):
    """按 `seed0 + i` 逐张生成；返回 (results, summary)。"""
    results = []
    t_start = None
    import time
    t_start = time.perf_counter()
    for i in range(count):
        results.append(generate_one(seed0 + i, rounds=rounds, mode=mode, img_size=img_size))
    elapsed = time.perf_counter() - t_start

    ok_n = sum(1 for r in results if r["ok"])
    fail_n = len(results) - ok_n
    reasons = {}
    for r in results:
        if not r["ok"]:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
    viol = sum(len(validate(r)) for r in results)
    rounds_ok = sum((r.get("trace") or {}).get("rounds_ok", 0) for r in results if r["ok"])
    summary = {
        "count": len(results), "ok": ok_n, "failed": fail_n,
        "failed_reasons": reasons,
        "violations": viol,
        "elapsed_s": round(elapsed, 4),
        "mean_rounds_ok_per_success": (round(rounds_ok / ok_n, 2) if ok_n else None),
        "rounds_per_attempt": rounds,
        "mode": mode, "version": GEN_VERSION,
    }
    return results, summary


def write_csv(results, path):
    """落 CSV：一行一张，含 ok/reason/signature/sha256/违规数（便于对账）。"""
    import csv
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cols = ["i", "seed", "ok", "reason", "mode", "score", "column_term",
            "z_points", "rounds_ok", "signature", "sha256", "violations"]
    with path.open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for i, r in enumerate(results):
            v = validate(r)
            w.writerow({
                "i": i, "seed": r["seed"], "ok": int(bool(r["ok"])),
                "reason": r.get("reason", ""), "mode": r.get("mode", ""),
                "score": r.get("score"), "column_term": r.get("column_term"),
                "z_points": len(r.get("z_zone_points") or []),
                "rounds_ok": (r.get("trace") or {}).get("rounds_ok"),
                "signature": r.get("signature", ""), "sha256": sha256_of(r),
                "violations": len(v),
            })
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="裁判规则地图生成器（WB-23）。与 tools/map_synth.py 并行、不混用。")
    ap.add_argument("--seed", type=int, default=20261031, help="起始种子（后续 +i）")
    ap.add_argument("--count", type=int, default=1, help="生成张数（seed+i）")
    ap.add_argument("--rounds", type=int, default=N_ROUNDS,
                    help="每张的选优轮数（裁判为 3000；改小仅供测试，会在报告里标注）")
    ap.add_argument("--mode", choices=["referee", "enhanced"], default="referee",
                    help="referee=裁判规则（默认）；enhanced=训练增强（标 non_referee_rule）")
    ap.add_argument("--img-size", default=None, metavar="WxH",
                    help="显示图尺寸（例 1400x1100）；给了才算像素层，"
                         "不给则 pixels=None（**不造假值**）")
    ap.add_argument("--report", default=None, help="批量 CSV 落盘路径")
    ap.add_argument("--json", action="store_true", help="把结果 JSON 打到 stdout")
    ap.add_argument("--out", default=None, help="把结果 JSON 写到文件")
    a = ap.parse_args(argv)

    img_size = None
    if a.img_size:
        try:
            _w, _h = a.img_size.lower().split("x")
            img_size = (float(_w), float(_h))
        except Exception:
            raise SystemExit("--img-size 格式应为 WxH，例 1400x1100")

    if a.count == 1 and not a.report:
        res = generate_one(a.seed, rounds=a.rounds, mode=a.mode, img_size=img_size)
        if a.json or not a.out:
            print(json.dumps(res, ensure_ascii=False, indent=2))
        if a.out:
            Path(a.out).parent.mkdir(parents=True, exist_ok=True)
            Path(a.out).write_text(json.dumps(res, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
        return 0 if res["ok"] else 2

    results, summary = generate_batch(a.seed, a.count, rounds=a.rounds, mode=a.mode,
                                      img_size=img_size)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if a.report:
        p = write_csv(results, a.report)
        print("批量明细已落盘: %s" % p)
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps(results, ensure_ascii=False, indent=2),
                               encoding="utf-8")
    return 0 if summary["violations"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
