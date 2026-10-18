#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 test/air_protocol_golden.txt —— 车端 C++ 自检的**期望输出**

背景
----
`test/air_protocol_selftest.cpp` 每算完一个向量就 `hexOut("KEY", bytes)`，
输出形如：

    KEY <小写十六进制>

它的文件头注释说"由 test/air_protocol_cpp_check.py 读入并与 air_protocol.py
逐字节比对"，但该脚本目前**不存在**。本脚本补上"期望值"这一半：
用参考实现（air_protocol.py）算出同一批 KEY 的正确字节，写成 golden 文件。

随后用 `test/air_protocol_golden_check.py` 把 C++ 自检的实际输出与它比对，
即可在**没有 ROS、没有网络**的情况下完成 C++ 与参考实现的逐字节验收。

★ 重要：本文件里的用例与 C++ 自检**一一对应**，键名与参数都照抄自
  test/air_protocol_selftest.cpp。任何一方改了用例，都要同步改本文件。

运行:
  python3 test/air_protocol_golden_gen.py            # 写 golden 文件
  python3 test/air_protocol_golden_gen.py --stdout   # 只打印
"""

from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import air_protocol as A  # noqa: E402

OUT = os.path.join(HERE, "air_protocol_golden.txt")

# 与 air_protocol.hpp 对齐
COLS, ROWS = 14, 11
BRIDGE_COL = 10

K_ST_MINE_CLEAR = 2
K_ST_IDLE = 0
K_FAULT_NONE = 0
K_FAULT_BLOCKED = 4

K_EV_GATE_CONFIRMED = 11
K_EV_MINE_CLEARED = 1
K_EV_GATE_INVALID = 10

K_CMD_NOP = 0
K_CMD_STOP = 8
K_CMD_MINE_CLEAR_ALL = 3
K_CMD_RETURN_BASE = 9


def fr_ack() -> bytes:
    """encodeFrame(kAck, 7, enc::ack(7, 0, 0))"""
    return A.encode(A.T_ACK, 7, A.hb_ack(7, 0, 0))


def fr_empty() -> bytes:
    """encodeFrame(kHeartbeat, 0, 空载荷)

    ★ 修复记录：原来这里写的是 `A.encode(A.T_ACK, 1, b"")`，
      但 C++ 自测用的是 `encodeFrame(kHeartbeat, 0, nullptr, 0)`：
          · 类型码：HEARTBEAT(0x01) vs ACK(0x7F)
          · 序列号：0 vs 1
      两者不一致，导致 golden 对撞报：
          fr_empty 期望 a5c3027f... 实际 a5c30201...
      **C++ 是对的**：这一项测的是"空载荷帧能否正确编解码"，
      用 HEARTBEAT + seq=0 最自然（ACK 另有用例覆盖）。
      所以改生成器去对齐 C++，而不是反过来。
    """
    return A.encode(A.T_HEARTBEAT, 0, b"")


def m_map_full() -> bytes:
    """照抄 selftest 第 250-268 行的构造（注意写入顺序决定覆盖关系）"""
    ce = A.cell_encode
    cells = [A.OCC_UNKNOWN] * (COLS * ROWS)
    # K 列（col 10）整列 GATE_UNK
    for r in range(ROWS):
        cells[r * COLS + BRIDGE_COL] = ce(A.OCC_GATE_UNK, 15, 0)
    # 门 K4(row3) / K9(row8) 置 GATE_OPEN
    cells[3 * COLS + BRIDGE_COL] = ce(A.OCC_GATE_OPEN, 15, 0)
    cells[8 * COLS + BRIDGE_COL] = ce(A.OCC_GATE_OPEN, 15, 0)
    # 水平隔离线 D..N 在 row5 —— ★ 会把 (K,row5) 从 GATE_UNK 覆盖成 BLOCKED
    for c in range(3, 14):
        cells[5 * COLS + c] = ce(A.OCC_BLOCKED, 15, 0)
    # 两颗雷 + 一个标靶
    cells[2 * COLS + 3] = ce(A.OCC_MINE, 15, 0)
    cells[4 * COLS + 6] = ce(A.OCC_MINE, 15, 0)
    cells[6 * COLS + 6] = ce(A.OCC_TARGET, 15, 0)
    return A.hb_map_full(5, 1000, cells)


def build() -> list:
    """返回 [(KEY, 期望字节), ...]，顺序与 selftest 的 hexOut 顺序一致"""
    out = []

    out.append(("fr_ack", fr_ack()))
    out.append(("fr_empty", fr_empty()))

    out.append(("m_heartbeat", A.hb_heartbeat(1000, 2, 95, 0x05)))
    out.append(("m_team_info", A.hb_team_info(1, 14, 11, 35)))
    out.append(("m_field_fixed", A.hb_field_fixed()))
    out.append(("m_field_fixed2",
                A.hb_field_fixed(5, 3, 13, 10, (2, 4), (7, 9))))

    out.append(("m_map_full", m_map_full()))

    out.append(("m_mine_list",
                A.hb_mine_list(1000, [(3, 2, 90), (6, 4, 88), (11, 1, 85),
                                      (4, 7, 92), (9, 8, 80), (0, 3, 95),
                                      (8, 9, 70), (5, 2, 88)])))
    out.append(("m_mine_list_empty", A.hb_mine_list(1000, [])))

    out.append(("m_gate_a", A.hb_bridge_gate(1, 3, 100, 0x01)))
    out.append(("m_gate_b", A.hb_bridge_gate(2, 8, 100, 0x01)))
    out.append(("m_gate_lo", A.hb_bridge_gate(1, 0, 0, 0x00)))

    out.append(("m_target_list", A.hb_target_list(1000, [(6, 4, 2, 90), (6, 5, 2, 90)])))
    out.append(("m_target_list_noface",
                A.hb_target_list(1000, [(6, 4, 0, 0), (6, 5, 0, 0)])))

    out.append(("m_obstacle_list",
                A.hb_obstacle_list([(1, 1, 1), (7, 5, 2), (13, 10, 1), (2, 6, 2),
                                    (10, 3, 1), (6, 8, 2), (0, 9, 1), (9, 4, 2)])))
    out.append(("m_obstacle_list_empty", A.hb_obstacle_list([])))

    out.append(("m_cmd_nop", A.hb_task_cmd(K_CMD_NOP, 0)))
    out.append(("m_cmd_stop", A.hb_task_cmd(K_CMD_STOP, 7)))
    out.append(("m_cmd_nav", A.hb_task_nav_to_cell(3, 12, 10, 60)))
    out.append(("m_cmd_bridge", A.hb_task_bridge_cross(4, 1)))
    out.append(("m_cmd_engage", A.hb_task_engage(5, 6, 4, 2)))
    out.append(("m_cmd_mine_all", A.hb_task_cmd(K_CMD_MINE_CLEAR_ALL, 6)))
    out.append(("m_cmd_return", A.hb_task_cmd(K_CMD_RETURN_BASE, 8)))

    out.append(("m_route", A.hb_route(7, 2, [(12, 10, 0), (10, 8, 1), (13, 5, 0)])))

    out.append(("m_veh_status",
                A.hb_veh_status(90000, 245, 175, -450, 30, -12, K_ST_MINE_CLEAR,
                                K_FAULT_NONE, 3, 1, 100, 7, 82, 95, 8, 0x01,
                                7, 5, 12345)))
    out.append(("m_veh_status_neg",
                A.hb_veh_status(0, -1, -1, -1800, -30, 0, K_ST_IDLE,
                                K_FAULT_BLOCKED, 0, 0, 0, 0, 0, 0, 0, 0,
                                -1, -1, 0)))

    out.append(("m_event", A.hb_event(1000, K_EV_GATE_CONFIRMED, 0, 1, 0)))
    out.append(("m_event_mine", A.hb_event(2000, K_EV_MINE_CLEARED, 3, 2, 0)))
    out.append(("m_event_invalid", A.hb_event(3000, K_EV_GATE_INVALID, 0, 0, 0)))

    out.append(("m_ack", A.hb_ack(7, 0, 0)))
    out.append(("m_ack_err", A.hb_ack(0xFFFFFFFF, 3, 9)))

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="生成 air_protocol_golden.txt")
    ap.add_argument("--stdout", action="store_true", help="只打印，不写文件")
    args = ap.parse_args()

    vecs = build()

    lines = [
        "# air_protocol C++ 自检的期望输出（golden vectors）",
        "#",
        "# 由 test/air_protocol_golden_gen.py 用参考实现 air_protocol.py 生成。",
        "# 用途: 与 test/air_protocol_selftest.cpp 的实际输出逐字节比对。",
        "#",
        "# 用法:",
        "#   g++ -std=c++17 -I include src/air_protocol.cpp \\",
        "#       test/air_protocol_selftest.cpp -o /tmp/air_selftest",
        "#   /tmp/air_selftest | python3 test/air_protocol_golden_check.py",
        "# 或:",
        "#   python3 test/air_protocol_golden_check.py --run",
        "#",
        "# 格式: KEY <小写十六进制>    （与 selftest 的 hexOut 输出格式一致）",
        "# 注意: selftest 还会打印 [ OK ]/[FAIL] 行，比对时会被忽略。",
        "#",
        "# ★ 键名与参数照抄自 test/air_protocol_selftest.cpp；",
        "#   任何一方改了用例都要同步改 test/air_protocol_golden_gen.py。",
        "",
    ]
    for key, payload in vecs:
        lines.append("%s %s" % (key, payload.hex()))

    text = "\n".join(lines) + "\n"

    if args.stdout:
        sys.stdout.write(text)
        return 0

    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)

    total = sum(len(p) for _, p in vecs)
    print("已写出 %s" % OUT)
    print("  向量 %d 条，共 %d 字节" % (len(vecs), total))
    return 0


if __name__ == "__main__":
    sys.exit(main())
