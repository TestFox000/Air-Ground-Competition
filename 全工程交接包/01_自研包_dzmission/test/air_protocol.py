#!/usr/bin/env python3
"""空地协同通信协议 v2 —— 独立参考实现 + 往返自检

不依赖 ROS，可直接给无人机端（PC / Orin）对接使用。
协议规格见 doc/AIR_GROUND_PROTOCOL.md。

运行:
  python3 air_protocol.py                # 全量自检
  python3 air_protocol.py --dump         # 打印各消息的示例十六进制帧
"""

import argparse
import struct
import sys

# ============================================================ 常量
MAGIC0, MAGIC1 = 0xA5, 0xC3
VER = 2
HEADER_LEN = 12          # magic(2) + ver(1) + typ(1) + seq(4) + len(2) + crc(2)

T_HEARTBEAT = 0x01
T_TEAM_INFO = 0x10
T_FIELD_FIXED = 0x11
T_MAP_FULL = 0x12
T_MAP_DELTA = 0x13
T_MINE_LIST = 0x14
T_BRIDGE_GATE = 0x15
T_TARGET_LIST = 0x16
T_OBSTACLE_LIST = 0x17
T_TASK_CMD = 0x20
T_ROUTE = 0x21
T_VEH_STATUS = 0x30
T_EVENT = 0x31
T_ACK = 0x7F

TYPE_NAMES = {
    T_HEARTBEAT: "HEARTBEAT", T_TEAM_INFO: "TEAM_INFO",
    T_FIELD_FIXED: "FIELD_FIXED", T_MAP_FULL: "MAP_FULL",
    T_MAP_DELTA: "MAP_DELTA", T_MINE_LIST: "MINE_LIST",
    T_BRIDGE_GATE: "BRIDGE_GATE", T_TARGET_LIST: "TARGET_LIST",
    T_OBSTACLE_LIST: "OBSTACLE_LIST", T_TASK_CMD: "TASK_CMD",
    T_ROUTE: "ROUTE", T_VEH_STATUS: "VEH_STATUS",
    T_EVENT: "EVENT", T_ACK: "ACK",
}

# occupancy
OCC_UNKNOWN, OCC_FREE, OCC_BLOCKED, OCC_MINE, OCC_TARGET, OCC_GATE_UNK, OCC_GATE_OPEN = range(7)

# task cmd
CMD_NOP = 0x00
CMD_NAV_TO_CELL = 0x01
CMD_NAV_TO_POSE = 0x02
CMD_MINE_CLEAR_ALL = 0x03
CMD_MINE_CLEAR_ONE = 0x04
CMD_BRIDGE_CROSS = 0x05
CMD_ENGAGE = 0x06
CMD_HOLD = 0x07
CMD_STOP = 0x08
CMD_RETURN_BASE = 0x09
CMD_SAFE_RETREAT = 0x0A

TASK_CMD_NAMES = {
    CMD_NOP: "NOP", CMD_NAV_TO_CELL: "NAV_TO_CELL", CMD_NAV_TO_POSE: "NAV_TO_POSE",
    CMD_MINE_CLEAR_ALL: "MINE_CLEAR_ALL", CMD_MINE_CLEAR_ONE: "MINE_CLEAR_ONE",
    CMD_BRIDGE_CROSS: "BRIDGE_CROSS", CMD_ENGAGE: "ENGAGE", CMD_HOLD: "HOLD",
    CMD_STOP: "STOP", CMD_RETURN_BASE: "RETURN_BASE", CMD_SAFE_RETREAT: "SAFE_RETREAT",
}

TASK_STATE = {0: "IDLE", 1: "NAV", 2: "MINE_CLEAR", 3: "BRIDGE_CROSS",
              4: "ENGAGE", 5: "RETURN", 6: "ESTOP", 7: "DONE"}

FAULT = {0: "无", 1: "无弹", 2: "低电", 3: "定位失效", 4: "前障阻塞",
         5: "裁判板链路断", 6: "指令链路断", 7: "卡死已触发脱困", 8: "无法脱困"}

EVENT = {0x01: "MINE_CLEARED", 0x02: "BRIDGE_PASSED", 0x03: "TARGET_HIT",
         0x04: "TARGET_DESTROYED", 0x05: "ARRIVED", 0x06: "STUCK_ESCAPED",
         0x07: "SAFE_RETREAT_START", 0x08: "MAP_STALE", 0x09: "FRIENDLY_DETECTED"}

# 场地规格
COLS, ROWS, CELL_CM = 14, 11, 35
BARRIER_UP_ROW = 5        # "6"：水平隔离线在 row6 与 row7 之间
# ★ v5 现场确认：覆盖列 D..N（col 3..13），**全宽**
#   后果：起点 M11 与终点 N6 分居线两侧，且 M/N 两列都被覆盖
#   -> 东侧条带被完全切断，必须穿桥洞，且**两个门缺一不可**
#   （详见 doc/AIR_GROUND_INTERFACE.md §0.4 与 test/topology_v5_check.py）
BARRIER_COL_LO, BARRIER_COL_HI = 3, 13   # D..N
BRIDGE_COL = 10           # K
BRIDGE_A = (0, 5)         # K1..K6
BRIDGE_B = (6, 10)        # K7..K11

# 起终点（0-based 格坐标）
START_CELL = (12, 10)     # M11 —— 智能车起点
GOAL_CELL = (13, 5)       # N6  —— 智能车终点
# 起点朝向：车头朝 M10 方向 = 行号减小 = -y = -90°
START_HEADING_DEG = -90.0

# ★ 无人机起降点（现场图固有标注，v2.1 起随 FIELD_FIXED 下发）
#   无人机**起飞前与降落后**停在场上这两个格，地面车必须避开 ——
#   撞机等价于"撞固定场地"，持续 >10s 裁判切手动。
#     无人机起点 M9 = (col 12, row 8)
#     无人机终点 N2 = (col 13, row 1)
DRONE_START_CELL = (12, 8)     # M9
DRONE_END_CELL = (13, 1)       # N2


# ============================================================ CRC
def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, 不反射。
    标准校验串 '123456789' -> 0x29B1
    """
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ============================================================ 组帧
def encode(typ: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > 1024:
        raise ValueError("payload too long")
    body = struct.pack("<BB I H", VER, typ, seq & 0xFFFFFFFF, len(payload)) + payload
    crc = crc16_ccitt(body)
    return bytes([MAGIC0, MAGIC1]) + body + struct.pack("<H", crc)


def decode(frame: bytes):
    """返回 (ver, typ, seq, payload)；失败抛 ValueError"""
    if len(frame) < HEADER_LEN:
        raise ValueError("frame too short")
    if frame[0] != MAGIC0 or frame[1] != MAGIC1:
        raise ValueError("bad magic")
    ver = frame[2]
    typ = frame[3]
    seq, ln = struct.unpack("<IH", frame[4:10])
    if ln > 1024:
        raise ValueError("bad len %d" % ln)
    if len(frame) != 10 + ln + 2:
        raise ValueError("len mismatch: expect %d got %d" % (10 + ln + 2, len(frame)))
    body = frame[2:10 + ln]
    crc_recv = struct.unpack("<H", frame[10 + ln:])[0]
    crc_calc = crc16_ccitt(body)
    if crc_recv != crc_calc:
        raise ValueError("crc mismatch: recv 0x%04X calc 0x%04X" % (crc_recv, crc_calc))
    if ver != VER:
        raise ValueError("version mismatch: %d != %d" % (ver, VER))
    return ver, typ, seq, frame[10:10 + ln]


# ============================================================ 载荷构造
def cell_encode(occ, conf=15, age=0) -> int:
    return (occ & 0x07) | ((conf & 0x0F) << 3) | ((age & 0x0F) << 7)


def cell_decode(v: int):
    return (v & 0x07, (v >> 3) & 0x0F, (v >> 7) & 0x0F)


def hb_heartbeat(ts_ms, map_version=0, quality=100, flags=0) -> bytes:
    return struct.pack("<IHBB", ts_ms & 0xFFFFFFFF, map_version & 0xFFFF, quality, flags)


def hb_team_info(our_color, cols=COLS, rows=ROWS, cell_cm=CELL_CM) -> bytes:
    return struct.pack("<BBBB", our_color, cols, rows, cell_cm)


def hb_field_fixed(up=BARRIER_UP_ROW, lo=BARRIER_COL_LO, hi=BARRIER_COL_HI,
                   bcol=BRIDGE_COL, a=None, b=None,
                   drone_start=None, drone_end=None) -> bytes:
    """场地固定结构

    ★ v2.1（12 字节）：在 v2.0 的 8 字节之后**追加**无人机起降点。
        v2.0  8B: up lo hi bcol a0 a1 b0 b1
        v2.1 12B: 以上 + dsc dsr dec der

    ★ 为什么需要无人机起降点：
      无人机起飞前/降落后停在场上固定格，地面车经过会撞机；
      而"撞固定场地 >10s 裁判切手动"是硬规则，故车端须视为禁入格。

    ★ 向后兼容：追加式扩展，旧车端按 payload.size() 解释，
      收到 12 字节会忽略后 4 字节，不会报错。
    """
    a = a or BRIDGE_A
    b = b or BRIDGE_B
    ds = drone_start or DRONE_START_CELL
    de = drone_end or DRONE_END_CELL
    return struct.pack("<12B", up, lo, hi, bcol, a[0], a[1], b[0], b[1],
                       ds[0], ds[1], de[0], de[1])


def hb_map_full(map_version, ts_ms, cells) -> bytes:
    """cells: 长度 154 的 occupancy 列表，或 (occ,conf,age) 元组列表"""
    assert len(cells) == COLS * ROWS
    body = struct.pack("<II BB", map_version & 0xFFFFFFFF, ts_ms & 0xFFFFFFFF,
                       COLS, ROWS)
    vals = []
    for x in cells:
        if isinstance(x, tuple):
            vals.append(cell_encode(*x))
        else:
            vals.append(cell_encode(x))
    return body + struct.pack("<%dH" % len(vals), *vals)


def hb_map_delta(map_version, changes) -> bytes:
    """changes: [(col,row,(occ,conf,age))]"""
    body = struct.pack("<IB", map_version & 0xFFFFFFFF, len(changes))
    for c, r, v in changes:
        code = cell_encode(*v) if isinstance(v, tuple) else cell_encode(v)
        body += struct.pack("<BBB", c, r, (code >> 8) & 0xFF)
        # 低 8 位不传（依赖之前的地图状态）；完整实现可改为 2 字节
    return body


def hb_mine_list(ts_ms, mines) -> bytes:
    """mines: [(col,row,confidence)]"""
    body = struct.pack("<IB", ts_ms & 0xFFFFFFFF, len(mines))
    for c, r, conf in mines:
        body += struct.pack("<BBB", c, r, conf)
    return body


def hb_bridge_gate(bridge_id, gate_row, confidence=100, flags=0x01) -> bytes:
    return struct.pack("<BBBB", bridge_id, gate_row, confidence, flags)


def hb_target_list(ts_ms, targets) -> bytes:
    """targets: [(col,row,face,confidence)]  face 1=红/友 2=蓝/敌 0=未知"""
    body = struct.pack("<IB", ts_ms & 0xFFFFFFFF, len(targets))
    for c, r, face, conf in targets:
        body += struct.pack("<BBBB", c, r, face, conf)
    return body


def hb_obstacle_list(obstacles) -> bytes:
    """obstacles: [(col,row,color)]"""
    body = struct.pack("<B", len(obstacles))
    for c, r, color in obstacles:
        body += struct.pack("<BBB", c, r, color)
    return body


def hb_task_cmd(cmd, cmd_seq, extra: bytes = b"") -> bytes:
    return struct.pack("<BB", cmd, cmd_seq & 0xFF) + extra


def hb_task_nav_to_cell(cmd_seq, col, row, timeout_s=60) -> bytes:
    return hb_task_cmd(CMD_NAV_TO_CELL, cmd_seq,
                       struct.pack("<BBB", col, row, timeout_s))


def hb_task_bridge_cross(cmd_seq, bridge_id) -> bytes:
    return hb_task_cmd(CMD_BRIDGE_CROSS, cmd_seq, struct.pack("<B", bridge_id))


def hb_task_engage(cmd_seq, col, row, target_face=2) -> bytes:
    return hb_task_cmd(CMD_ENGAGE, cmd_seq, struct.pack("<BBB", col, row, target_face))


def hb_route(route_version, task_id, waypoints) -> bytes:
    """waypoints: [(col,row,flags)]"""
    body = struct.pack("<IBB", route_version & 0xFFFFFFFF, len(waypoints), task_id)
    for c, r, f in waypoints:
        body += struct.pack("<BBB", c, r, f)
    return body


# VEH_STATUS 载荷布局（共 32 字节）
#   I    ts_ms                      4
#   hhhh x, y, yaw, v               8
#   h    w                          2
#   BB   task_state, fault          2
#   BBBB mines_cleared, bridges_passed, self_hp, ammo   4
#   BBBB battery, pose_confidence, last_cmd_seq, flags  4
#   hh   blocked_col, blocked_row   4
#   HH   exec_ms, reserved          4
# 合计 4+8+2+2+4+4+4+4 = 32 字节，20 项
_VS_FMT = "<" + "I" + "h" * 5 + "B" * 2 + "B" * 4 + "B" * 4 + "h" * 2 + "H" * 2
_VS_ITEMS = 20


def hb_veh_status(ts_ms, x_cm, y_cm, yaw_c10, v_cms, w_c100, task_state, fault,
                  mines_cleared=0, bridges_passed=0, self_hp=0, ammo=0,
                  battery_pct=100, pose_confidence=100, last_cmd_seq=0,
                  flags=0, blocked_col=-1, blocked_row=-1, exec_ms=0) -> bytes:
    body = struct.pack(
        _VS_FMT,
        ts_ms & 0xFFFFFFFF,
        x_cm, y_cm, yaw_c10, v_cms, w_c100,
        task_state, fault,
        mines_cleared, bridges_passed, self_hp, ammo,
        battery_pct, pose_confidence, last_cmd_seq, flags,
        blocked_col, blocked_row,
        exec_ms, 0)
    assert len(body) == 32, len(body)
    return body


def hb_event(ts_ms, event, a0=0, a1=0, a2=0) -> bytes:
    return struct.pack("<IBBBB", ts_ms & 0xFFFFFFFF, event, a0, a1, a2)


def hb_ack(ack_seq, status=0, detail=0) -> bytes:
    return struct.pack("<IBBH", ack_seq & 0xFFFFFFFF, status, detail, 0)


# ============================================================ 解码为可读
def describe(frame: bytes) -> dict:
    ver, typ, seq, p = decode(frame)
    out = {"type": TYPE_NAMES.get(typ, "UNKNOWN(0x%02X)" % typ),
           "typ": typ, "seq": seq, "ver": ver, "len": len(frame)}

    if typ == T_HEARTBEAT:
        ts, mv, q, fl = struct.unpack("<IHBB", p)
        out.update(ts_ms=ts, map_version=mv, quality=q, flags=fl)
    elif typ == T_TEAM_INFO:
        out.update(our_color=p[0], cols=p[1], rows=p[2], cell_cm=p[3])
    elif typ == T_FIELD_FIXED:
        out.update(barrier_up_row=p[0], barrier_col_lo=p[1], barrier_col_hi=p[2],
                   bridge_col=p[3], bridge_a=(p[4], p[5]), bridge_b=(p[6], p[7]))
        # ★ v2.1：后 4 字节是无人机起降点（旧 8 字节帧没有，要容忍）
        if len(p) >= 12:
            out.update(drone_start_cell=(p[8], p[9]),
                       drone_end_cell=(p[10], p[11]))
        else:
            out.update(drone_start_cell=None, drone_end_cell=None)
    elif typ == T_MAP_FULL:
        mv, ts, cols, rows = struct.unpack("<IIBB", p[:10])
        vals = struct.unpack("<%dH" % ((len(p) - 10) // 2), p[10:])
        out.update(map_version=mv, ts_ms=ts, cols=cols, rows=rows,
                   cells=len(vals),
                   occupied=sum(1 for v in vals if cell_decode(v)[0] == OCC_BLOCKED),
                   mines=sum(1 for v in vals if cell_decode(v)[0] == OCC_MINE))
    elif typ == T_MINE_LIST:
        ts, n = struct.unpack("<IB", p[:5])
        mines = [struct.unpack("<BBB", p[5 + 3 * i:8 + 3 * i]) for i in range(n)]
        out.update(ts_ms=ts, count=n, mines=mines)
    elif typ == T_BRIDGE_GATE:
        bid, g, conf, fl = struct.unpack("<BBBB", p)
        out.update(bridge_id=bid, gate_row=g, gate_cell="K%d" % (g + 1),
                   confidence=conf, flags=fl)
    elif typ == T_TARGET_LIST:
        ts, n = struct.unpack("<IB", p[:5])
        tg = [struct.unpack("<BBBB", p[5 + 4 * i:9 + 4 * i]) for i in range(n)]
        out.update(ts_ms=ts, count=n, targets=tg)
    elif typ == T_OBSTACLE_LIST:
        # ★ 本条**没有 ts_ms**（首字节就是 count）。
        #   此分支曾被遗漏 —— 无人机端审计（§9）指出：少了它，
        #   车端自带的离线解码工具拿到合法 OBSTACLE_LIST 时只能读出
        #   type/typ/seq/ver/len，读不出 count 与任何格坐标，
        #   联调时会被误判成"车端没收到"，严重影响故障定位。
        #   OBSTACLE_LIST 是规范里标记为 ★必须 的消息（8 个水桶）。
        n = p[0]
        obs = [struct.unpack("<BBB", p[1 + 3 * i:4 + 3 * i]) for i in range(n)]
        out.update(count=n, obstacles=obs, has_ts=False)
    elif typ == T_TASK_CMD:
        out.update(cmd=p[0], cmd_name=TASK_CMD_NAMES.get(p[0], "?"), cmd_seq=p[1],
                   extra=p[2:].hex().upper())
    elif typ == T_ROUTE:
        rv, n, tid = struct.unpack("<IBB", p[:6])
        wps = [struct.unpack("<BBB", p[6 + 3 * i:9 + 3 * i]) for i in range(n)]
        out.update(route_version=rv, task_id=tid, count=n, waypoints=wps)
    elif typ == T_VEH_STATUS:
        f = struct.unpack(_VS_FMT, p[:32])
        out.update(ts_ms=f[0], x_cm=f[1], y_cm=f[2], yaw_c10=f[3], v_cms=f[4],
                   w_c100=f[5], task_state=TASK_STATE.get(f[6], "?"),
                   fault=FAULT.get(f[7], "?"), mines_cleared=f[8],
                   bridges_passed=f[9], self_hp=f[10], ammo=f[11],
                   battery_pct=f[12], pose_confidence=f[13],
                   last_cmd_seq=f[14], flags=f[15],
                   blocked=(f[16], f[17]), exec_ms=f[18])
    elif typ == T_EVENT:
        ts, ev, a0, a1, a2 = struct.unpack("<IBBBB", p)
        out.update(ts_ms=ts, event=EVENT.get(ev, "?"), args=(a0, a1, a2))
    elif typ == T_ACK:
        ack_seq, st, det, _ = struct.unpack("<IBBH", p)
        out.update(ack_seq=ack_seq, status=st, detail=det)
    return out


# ============================================================ 自检
def self_test() -> int:
    fails = 0

    def check(name, cond, extra=""):
        nonlocal fails
        if cond:
            print("  [ OK ] %s" % name)
        else:
            fails += 1
            print("  [FAIL] %s %s" % (name, extra))

    print("== 1. CRC 与帧头 ==")
    check("crc16_ccitt('123456789') == 0x29B1",
          crc16_ccitt(b"123456789") == 0x29B1,
          "got 0x%04X" % crc16_ccitt(b"123456789"))
    f = encode(T_HEARTBEAT, 1, hb_heartbeat(1000))
    check("帧头 A5 C3", f[0] == 0xA5 and f[1] == 0xC3)
    check("VER == 2", f[2] == 2)
    check("TYP == HEARTBEAT", f[3] == T_HEARTBEAT)
    check("帧长 == 12 + LEN", len(f) == 12 + 8, "got %d" % len(f))

    print("\n== 2. 逐消息往返 ==")
    cases = [
        ("HEARTBEAT", T_HEARTBEAT, hb_heartbeat(123456, 7, 88, 0x03)),
        ("TEAM_INFO", T_TEAM_INFO, hb_team_info(1)),
        ("FIELD_FIXED", T_FIELD_FIXED, hb_field_fixed()),
        ("MINE_LIST", T_MINE_LIST,
         hb_mine_list(5000, [(3, 2, 90), (6, 4, 85), (11, 1, 70),
                             (4, 7, 95), (9, 8, 60), (0, 3, 80),
                             (2, 8, 75), (12, 9, 65)])),
        ("BRIDGE_GATE", T_BRIDGE_GATE, hb_bridge_gate(1, 5, 95, 0x03)),
        ("TARGET_LIST", T_TARGET_LIST,
         hb_target_list(6000, [(5, 3, 2, 88), (5, 4, 1, 91)])),
        ("OBSTACLE_LIST", T_OBSTACLE_LIST,
         hb_obstacle_list([(1, 1, 1), (7, 5, 3), (13, 10, 2)])),
        ("TASK_CMD/NAV", T_TASK_CMD, hb_task_nav_to_cell(7, 3, 5, 60)),
        ("TASK_CMD/BRIDGE", T_TASK_CMD, hb_task_bridge_cross(8, 2)),
        ("TASK_CMD/ENGAGE", T_TASK_CMD, hb_task_engage(9, 5, 3, 2)),
        ("ROUTE", T_ROUTE,
         hb_route(3, 9, [(3, 5, 0x01), (10, 5, 0x01), (10, 6, 0x01), (6, 8, 0x00)])),
        ("VEH_STATUS", T_VEH_STATUS,
         hb_veh_status(90000, 245, 175, -450, 30, -12, 2, 0, 3, 1, 100, 7, 82, 95, 8, 0x01, 7, 5, 12345)),
        ("EVENT/MINE", T_EVENT, hb_event(91000, 0x01, 3, 2, 0)),
        ("EVENT/FRIENDLY", T_EVENT, hb_event(92000, 0x09, 5, 3, 0)),
        ("ACK", T_ACK, hb_ack(7, 0, 0)),
    ]
    for name, typ, payload in cases:
        try:
            fr = encode(typ, 42, payload)
            d = describe(fr)
            print("  %-18s %4d B  -> %s" % (name, len(fr), d))
        except Exception as e:
            fails += 1
            print("  [FAIL] %-18s %s" % (name, e))

    print("\n== 3. MAP_FULL（全量地图）==")
    cells = []
    for r in range(ROWS):
        for c in range(COLS):
            if BARRIER_COL_LO <= c <= BARRIER_COL_HI and r == BARRIER_UP_ROW:
                cells.append((OCC_BLOCKED, 15, 0))
            elif c == BRIDGE_COL:
                cells.append((OCC_GATE_UNK, 15, 0))
            else:
                cells.append((OCC_FREE, 12, 3))
    fr = encode(T_MAP_FULL, 3, hb_map_full(11, 100000, cells))
    d = describe(fr)
    print("  帧长 %d B（含 %d 格 × 2 B），occupancy: %d 阻挡 / %d 地雷" % (
        len(fr), d["cells"], d["occupied"], d["mines"]))
    check("MAP_FULL 帧长 == 12 + 10 + 308", len(fr) == 330, "got %d" % len(fr))
    check("MAP_FULL 格数 == 154", d["cells"] == COLS * ROWS)

    # ★ 期望的阻挡格数按 v5 拓扑**现算**，不写死数字。
    #   这里只数 OCC_BLOCKED（水平隔离线覆盖的那些格）；
    #   K 列未指定的格是 OCC_GATE_UNK = "未知"而不是"阻挡"，
    #   因为车端自己也不知道哪个门是开的 —— 必须等无人机下发。
    #   ★ 正是这一点让 BRIDGE_GATE 成为致命依赖（见接口文档 §0.4）。
    bar_cols = BARRIER_COL_HI - BARRIER_COL_LO + 1     # 线覆盖的列数
    expect_blocked = bar_cols                          # 只有线上的格是 OCC_BLOCKED
    check("MAP_FULL 阻挡格数 == %d（线覆盖 D..N 共 %d 列，占 row%d）" % (
        expect_blocked, bar_cols, BARRIER_UP_ROW + 1),
        d["occupied"] == expect_blocked, "got %d" % d["occupied"])

    print("\n== 4. 损坏帧必须被拒绝 ==")
    good = bytearray(encode(T_ACK, 1, hb_ack(7)))
    variants = [
        ("CRC 错误", lambda b: (b.__setitem__(-1, b[-1] ^ 0xFF), b)[1]),
        ("魔数错误", lambda b: (b.__setitem__(0, 0x00), b)[1]),
        ("版本错误", lambda b: (b.__setitem__(2, 99), b)[1]),
        ("长度不符", lambda b: b[:-1]),
        ("过短", lambda b: b[:6]),
    ]
    for name, mut in variants:
        try:
            decode(bytes(mut(bytearray(good))))
            fails += 1
            print("  [FAIL] %s 未被拒绝" % name)
        except ValueError as e:
            print("  [ OK ] %-8s 被拒绝: %s" % (name, e))

    print("\n== 5. 场地坐标一致性 ==")
    check("格数与场地一致 (14×11)", COLS * ROWS == 154)
    check("字母轴 14×35cm = 4.90m", abs(COLS * CELL_CM / 100 - 4.90) < 0.01)
    check("数字轴 11×35cm = 3.85m", abs(ROWS * CELL_CM / 100 - 3.85) < 0.01)
    check("线在 row5/row6 之间且覆盖 D..N（v5 现场确认）",
          BARRIER_UP_ROW == 5 and BARRIER_COL_LO == 3 and BARRIER_COL_HI == 13)
    check("桥洞一 6 格 / 桥洞二 5 格",
          BRIDGE_A[1] - BRIDGE_A[0] + 1 == 6 and BRIDGE_B[1] - BRIDGE_B[0] + 1 == 5)

    print()
    if fails == 0:
        print("全部自检通过。")
    else:
        print("%d 项自检失败。" % fails)
    return 1 if fails else 0


def dump_examples():
    print("各消息示例帧（十六进制，可直接喂给解析器）:\n")
    ex = [
        ("TEAM_INFO (红方)", T_TEAM_INFO, hb_team_info(1)),
        ("FIELD_FIXED", T_FIELD_FIXED, hb_field_fixed()),
        ("BRIDGE_GATE 桥洞一 K6", T_BRIDGE_GATE, hb_bridge_gate(1, 5)),
        ("BRIDGE_GATE 桥洞二 K7", T_BRIDGE_GATE, hb_bridge_gate(2, 5)),
        ("MINE_LIST 2 颗", T_MINE_LIST, hb_mine_list(1000, [(3, 2, 90), (9, 8, 80)])),
        ("TASK_CMD 排雷全部", T_TASK_CMD, hb_task_cmd(CMD_MINE_CLEAR_ALL, 7)),
        ("TASK_CMD 穿桥洞一", T_TASK_CMD, hb_task_bridge_cross(8, 1)),
        ("TASK_CMD 回终点", T_TASK_CMD, hb_task_cmd(CMD_RETURN_BASE, 9)),
        ("EVENT 排雷完成", T_EVENT, hb_event(2000, 0x01, 3, 2)),
        ("EVENT 发现友方", T_EVENT, hb_event(3000, 0x09, 5, 3)),
    ]
    for name, typ, p in ex:
        fr = encode(typ, 1, p)
        print("  %-26s %s" % (name, fr.hex().upper()))
    print()
    print("解析示例:")
    fr = encode(T_BRIDGE_GATE, 5, hb_bridge_gate(1, 5, 95, 0x03))
    print("  %s" % fr.hex().upper())
    print("  -> %s" % describe(fr))


def main():
    ap = argparse.ArgumentParser(description="空地协同通信协议 v2 参考实现")
    ap.add_argument("--dump", action="store_true", help="打印示例帧")
    ap.add_argument("--decode", help="解码十六进制帧")
    args = ap.parse_args()

    if args.decode:
        print(describe(bytes.fromhex(args.decode.replace(" ", ""))))
        return 0
    if args.dump:
        dump_examples()
        return 0
    return self_test()


if __name__ == "__main__":
    sys.exit(main())
