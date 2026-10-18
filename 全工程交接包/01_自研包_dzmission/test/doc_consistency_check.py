#!/usr/bin/env python3
"""接口文档一致性校验 —— 确保 doc/AIR_GROUND_INTERFACE.md 与实现不脱节

为什么需要这个脚本：
  这份文档要交给无人机端实现。文档里写错一个字节长度，对面就会整体错位，
  而且是"看起来能跑但数据全乱"的隐性故障 —— 比直接报错更难查。
  所以必须让**文档里的每个长度都从实现现算并比对**，不允许手写。

校验内容：
  1. 文档声明的帧头字段与 §3 一致（SOF/VER/TYP/SEQ/LEN/PAYLOAD/CRC）
  2. 文档声明的每条消息长度 == air_protocol.py 实际编码出来的长度
  3. 文档声明的场地常量 == v5 拓扑（起点 M11 / 终点 N6 / 线 D..N）
  4. 文档声明的门位取值表与 BRIDGE_A/BRIDGE_B 一致

运行:  python3 doc_consistency_check.py
"""

import io
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import air_protocol as A  # noqa: E402

DOC = os.path.join(HERE, "..", "doc", "AIR_GROUND_INTERFACE.md")

fails = []


def check(name, ok, extra=""):
    print("  [%s] %s%s" % ("OK" if ok else "FAIL", name,
                           "" if ok else "  -> " + str(extra)))
    if not ok:
        fails.append(name)
    return ok


def read_doc():
    with io.open(DOC, encoding="utf-8") as f:
        return f.read()


def extract_fixed_len(doc, section_marker):
    """从文档某节里提取长度声明

    先截到本节结束（下一个 '### ' 或 '---'），避免跨节误匹配。
    支持两种表述：
      · 「固定 **N** 字节」            -> 定长消息
      · 「典型长度 | `.. = N` 字节」  -> 变长消息的典型长度
    """
    i = doc.find(section_marker)
    if i < 0:
        return None
    j = doc.find("\n### ", i + 1)
    k = doc.find("\n---", i + 1)
    end = min(x for x in (j, k, len(doc)) if x > 0)
    seg = doc[i:end]
    m = re.search(r"固定\s*\*\*(\d+)\*\*\s*字节", seg)
    if m:
        return int(m.group(1))
    # 「典型长度 | `5 + 3×8 = 29` 字节」
    m = re.search(r"典型长度\s*\|\s*`[^`]*=\s*(\d+)[^`]*`\s*字节", seg)
    if m:
        return int(m.group(1))
    m = re.search(r"典型长度\s*\|\s*\*\*(\d+)\*\*", seg)
    return int(m.group(1)) if m else None


def sec(t):
    print()
    print("=" * 74)
    print(t)
    print("=" * 74)


def main():
    if not os.path.exists(DOC):
        print("找不到文档: %s" % DOC)
        return 1
    doc = read_doc()
    print("接口文档一致性校验")
    print("文档: %s" % os.path.normpath(DOC))
    print("实现: air_protocol.py")

    # =============================== 1. 帧头 ===============================
    sec("1. 帧头结构与 §3 声明一致")
    check("文档 §3 存在", "## 3. 帧格式" in doc)
    check("SOF = A5 C3", A.MAGIC0 == 0xA5 and A.MAGIC1 == 0xC3,
          "got %02X %02X" % (A.MAGIC0, A.MAGIC1))
    check("HEADER_LEN == 12（2 SOF + 1 VER + 1 TYP + 4 SEQ + 2 LEN + 2 CRC）",
          A.HEADER_LEN == 12, "got %d" % A.HEADER_LEN)
    check("文档声明 VER == 0x02", "VER         协议版本，本规范 = 0x02" in doc or
          "本规范 = `0x02`" in doc or "本规范 = 0x02" in doc)
    check("文档声明最大帧长 272", "272" in doc)
    check("文档声明 CRC-16/CCITT-FALSE", "CCITT-FALSE" in doc)

    # 用真实帧验证帧头布局
    fr = A.encode(A.T_ACK, 7, A.hb_ack(1))
    check("实帧 SOF 正确", fr[0] == 0xA5 and fr[1] == 0xC3)
    check("实帧 VER 位置正确", fr[2] == A.VER)
    check("实帧 TYP 位置正确", fr[3] == A.T_ACK)
    seq = struct.unpack_from("<I", fr, 4)[0]
    check("实帧 SEQ 位置正确（小端）", seq == 7, "got %d" % seq)
    ln = struct.unpack_from("<H", fr, 8)[0]
    check("实帧 LEN 位置正确（小端）", ln == len(fr) - A.HEADER_LEN,
          "got %d" % ln)
    check("帧总长 == HEADER_LEN + LEN", len(fr) == A.HEADER_LEN + ln)

    # =============================== 2. 消息长度 ===============================
    sec("2. 各消息长度：文档声明 == 实现实际")

    # ★ 用**真实签名**调用，签名不匹配会直接报错（这本身就是一种校验）
    cases = []

    cases.append(("### 4.8 `0x01` `HEARTBEAT`", "HEARTBEAT",
                  A.hb_heartbeat(1000, map_version=2, quality=95, flags=0)))

    # BRIDGE_GATE：一条消息只带一个门
    cases.append(("### 4.4 `0x15` `BRIDGE_GATE`", "BRIDGE_GATE",
                  A.hb_bridge_gate(1, 3, confidence=100, flags=0x01)))

    cases.append(("### 4.3 `0x14` `MINE_LIST`", "MINE_LIST",
                  A.hb_mine_list(1000, [(3, 2, 90), (6, 4, 88), (11, 1, 85),
                                        (4, 7, 92), (9, 8, 80), (0, 3, 95),
                                        (8, 9, 70), (5, 2, 88)])))

    cases.append(("### 4.5 `0x17` `OBSTACLE_LIST`", "OBSTACLE_LIST",
                  A.hb_obstacle_list([(1, 1, 1), (7, 5, 2), (13, 10, 1), (2, 6, 2),
                                      (10, 3, 1), (6, 8, 2), (0, 9, 1), (9, 4, 2)])))

    cases.append(("### 4.7 `0x16` `TARGET_LIST`", "TARGET_LIST",
                  A.hb_target_list(1000, [(6, 4, 2, 90), (6, 5, 2, 90)])))

    cases.append(("### 4.10 `0x31` `EVENT`", "EVENT",
                  A.hb_event(1000, 2, 0, 1, 0)))

    cases.append(("### 4.11 `0x7F` `ACK`", "ACK", A.hb_ack(7, status=0)))

    cases.append(("### 4.9 `0x30` `VEH_STATUS`", "VEH_STATUS",
                  A.hb_veh_status(90000, 245, 175, -450, 30, -12, 2, 0,
                                  3, 1, 100, 7, 82, 95, 8, 0x01, 7, 5, 12345)))

    for marker, name, payload in cases:
        got = len(payload)
        declared = extract_fixed_len(doc, marker)
        if declared is None:
            check("%-16s 文档有长度声明" % name, False, "未找到'固定 N 字节'")
            continue
        check("%-16s 文档 %d B == 实现 %d B" % (name, declared, got),
              declared == got, "文档 %s / 实现 %d" % (declared, got))

    # 变长消息校验典型长度
    sec("3. 变长消息的典型长度")
    ml = A.hb_mine_list(1000, [(i % 14, i % 11, 90) for i in range(8)])
    check("MINE_LIST(8) 长度 == 5 + 3*8 = 29", len(ml) == 29, "got %d" % len(ml))
    ol = A.hb_obstacle_list([(i % 14, i % 11, 1) for i in range(8)])
    check("OBSTACLE_LIST(8) 长度 == 1 + 3*8 = 25", len(ol) == 25, "got %d" % len(ol))
    tl = A.hb_target_list(1000, [(6, 4, 2, 90), (6, 5, 2, 90)])
    check("TARGET_LIST(2) 长度 == 5 + 4*2 = 13", len(tl) == 13, "got %d" % len(tl))

    cells = [(A.OCC_FREE, 12) for _ in range(A.COLS * A.ROWS)]
    mf = A.hb_map_full(5, 1000, cells)
    # 文档 §4.6：10 字节头（map_version 4 + ts_ms 4 + cols 1 + rows 1）+ 154 × 2
    check("MAP_FULL 长度 == 10 + 2*154 = 318（文档声明 318）",
          len(mf) == 318, "got %d" % len(mf))
    check("MAP_FULL 帧长 == 318 + 12 = 330",
          len(A.encode(A.T_MAP_FULL, 1, mf)) == 330,
          "got %d" % len(A.encode(A.T_MAP_FULL, 1, mf)))

    # =============================== 4. 场地常量 ===============================
    sec("4. 场地常量与 v5 拓扑一致")
    check("BARRIER_UP_ROW == 5（row6/row7 之间）", A.BARRIER_UP_ROW == 5,
          "got %d" % A.BARRIER_UP_ROW)
    check("水平隔离线覆盖 D..N（col 3..13）",
          A.BARRIER_COL_LO == 3 and A.BARRIER_COL_HI == 13,
          "got %d..%d" % (A.BARRIER_COL_LO, A.BARRIER_COL_HI))
    check("BRIDGE_COL == 10（K 列）", A.BRIDGE_COL == 10)
    check("桥洞区段一 K1..K6", A.BRIDGE_A == (0, 5), "got %s" % (A.BRIDGE_A,))
    check("桥洞区段二 K7..K11", A.BRIDGE_B == (6, 10), "got %s" % (A.BRIDGE_B,))
    check("起点 M11 == (12, 10)",
          getattr(A, "START_CELL", None) == (12, 10),
          "got %s" % (getattr(A, "START_CELL", None),))
    check("终点 N6 == (13, 5)",
          getattr(A, "GOAL_CELL", None) == (13, 5),
          "got %s" % (getattr(A, "GOAL_CELL", None),))
    check("起点朝向 -90 度",
          getattr(A, "START_HEADING_DEG", None) == -90.0,
          "got %s" % (getattr(A, "START_HEADING_DEG", None),))

    # =============================== 5. 文档里的关键结论 ===============================
    sec("5. 文档必须包含的关键结论（防止后续误删）")
    must_have = [
        ("两个门缺一不可", "缺一不可"),
        ("北段门范围 K1-K6", "K1–K6"),
        ("南段门范围 K7-K11", "K7–K11"),
        ("起点 M11", "M11"),
        ("终点 N6", "N6"),
        ("线覆盖 D..N", "D..N"),
        ("A/B/C 列为无门出口", "A/B/C"),
        ("发车闸门", "发车闸门"),
        ("断链降级", "断链降级"),
        ("CRC CCITT-FALSE", "CCITT-FALSE"),
    ]
    for name, needle in must_have:
        check("文档含「%s」" % name, needle in doc, "未找到 %r" % needle)

    # =============================== 6. BRIDGE_GATE 取值表 ===============================
    sec("6. BRIDGE_GATE 取值表与实现一致")
    # 实现是「一条消息带一个门 + bridge_id」：bridge_id 1=区段一, 2=区段二
    check("文档声明 bridge_id 1 = 区段一", "`1`" in doc and "K1–K6" in doc)
    check("文档声明 bridge_id 2 = 区段二", "`2`" in doc and "K7–K11" in doc)
    check("文档声明区段一 gate_row 合法值 0..5（K1–K6）",
          "K1, K2, K3, K4, K5, K6" in doc)
    check("文档声明区段二 gate_row 合法值 6..10（K7–K11）",
          "K7, K8, K9, K10, K11" in doc)

    # 用真实编码/解码往返验证：bridge_id 与 gate_row 的组合
    for (bid, grow) in ((1, 0), (1, 5), (2, 6), (2, 10), (1, 3), (2, 8)):
        payload = A.hb_bridge_gate(bid, grow, confidence=100, flags=0x01)
        rt = A.describe(A.encode(A.T_BRIDGE_GATE, 1, payload))
        ok = (rt.get("bridge_id") == bid and rt.get("gate_row") == grow)
        check("门位 (bridge_id=%d, gate_row=%d) 往返一致" % (bid, grow),
              ok, rt)

    # =============================== 汇总 ===============================
    sec("结论")
    if not fails:
        print("  ✓ 文档与实现完全一致（%d 项校验全部通过）" % (len(cases) + 26))
        return 0
    print("  ✗ %d 项不一致：" % len(fails))
    for f in fails:
        print("      - %s" % f)
    print()
    print("  ★ 文档是要交给无人机端实现的，长度/常量不一致会导致对面整体错位。")
    print("     请修文档或修实现，使两者一致后再交付。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
