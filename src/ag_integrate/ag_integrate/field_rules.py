# -*- coding: utf-8 -*-
"""现场规则校验（只读，不改规划）—— 2026-10-19。

移植自 V1019 路径规划交付包（原作者 yubinghuan，2026-10-20 入仓；
原包 _main 依赖本机硬编码路径未随迁，库函数完整保留）；
方案：docs/路径规划优化-转角物理分级-20261020.md。

规则来源（用户确认）：
    R1. **地雷只会出现在 A1–I11 区域**（列 A..I）→ 情报里 J~N 出现的"雷"必为误检。
    R1b. 雷总数 ≤ 8（地图上不可能超过 8 颗）→ 超出即有误检。
    R2. **隧道（K 列桥洞）是单向的**，且方向由"解存在"反推固定 →
        车必须**直穿**隧道格（进入侧 ≠ 离开侧）；
        若"进出同侧"（洞内掉头/原路退回）→ 必然有一次逆行。
    R3. 两段隧道（下段 K1–K6、上段 K7–K11）各为计分点，每个解里各直穿一次最优。

用法：
    from ag_integrate.field_rules import validate  # 规划后只读校验（pipeline 已接线）
"""
from __future__ import annotations


# ---- 规则常量 ----
MINE_COLS = "ABCDEFGHI"          # 雷只在 A..I
MAX_MINES = 8                    # ★ 雷总数上限：地图不可能超过 8 颗（2026-10-19 用户确认）
K_COL = 10                       # K 列（隧道所在）
TUNNEL_SEGMENTS = (range(0, 10), range(10, 14))   # 下段 K0..K5(1-6行) / 上段 K6..K10 —— 仅用于说明


def _col(cell: str) -> int:
    return ord(cell[0]) - ord("A")


def _row(cell: str) -> int:
    return int(cell[1:]) - 1


def tunnels_straight(route, tunnel_cells) -> dict:
    """R2：每条隧道必须以直穿方式通过（进入侧 ≠ 离开侧）。

    返回 {"ok": bool, "visits": {cell: [side_in, side_out]}, "issues": [...]}
    """
    tun = set(tunnel_cells)
    issues: list[str] = []
    visits: dict[str, list[str]] = {}
    for i in range(1, len(route) - 1):
        cell = route[i]
        if cell not in tun:
            continue
        a, b = route[i - 1], route[i + 1]
        side_in = "W" if _col(a) < K_COL else "E"
        side_out = "W" if _col(b) < K_COL else "E"
        visits.setdefault(cell, []).append(f"{side_in}->{side_out}")
        if side_in == side_out:
            issues.append(f"{cell}: 进出同侧（{side_in}->{side_out}）= 洞内掉头 → 必有一次逆行 ✗")
    for cell, v in visits.items():
        if len(v) > 1:
            issues.append(f"{cell}: 被通过 {len(v)} 次（{v}）→ 单向规则下多次通过有逆行风险 ✗")
    return {"ok": not issues, "visits": visits, "issues": issues}


def mines_in_allowed_region(mines) -> dict:
    """R1：雷只允许出现在 A..I 列。"""
    bad = [m for m in mines if m and _col(m) > _col("I")]
    return {"ok": not bad, "out_of_region": bad,
            "note": "J~N 出现的雷必为误检（规则：雷只在 A1–I11）" if bad else "全部在 A~I ✓"}


def mine_count_ok(mines) -> dict:
    """R1b：雷总数 ≤ 8（地图上不可能出现超过 8 颗）→ 超出即有误检。"""
    n = len(set(mines))
    return {"ok": n <= MAX_MINES, "count": n, "max": MAX_MINES,
            "note": (f"雷 {n} 颗 ≤ {MAX_MINES} ✓" if n <= MAX_MINES
                     else f"雷 {n} 颗 > {MAX_MINES} ✗ → 必有 {n - MAX_MINES} 颗误检（建议人工复核）")}


def validate(route, mines, tunnel_cells) -> dict:
    t = tunnels_straight(route, tunnel_cells)
    m = mines_in_allowed_region(mines)
    n = mine_count_ok(mines)
    return {"ok": t["ok"] and m["ok"] and n["ok"], "tunnel": t, "mines": m, "mines_count": n}
