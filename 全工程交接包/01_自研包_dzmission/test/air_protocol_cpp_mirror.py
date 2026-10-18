#!/usr/bin/env python3
"""C++ `air_protocol.cpp` 的**逐行 Python 镜像**，用于在不编译的情况下验证字节逻辑

为什么需要这个：
  本机没有 C++ 编译器（g++/clang/cl 全部缺失，WSL 被沙箱拒绝），
  所以我**无法在本地运行 `test/air_protocol_selftest.cpp`**。
  但"看着对"不足以交付 —— 与无人机端的接口必须逐字节一致。

  做法：把 air_protocol.cpp 的字节发射逻辑**逐行转写**成 Python
  （不是复用 air_protocol.py 的 encode，而是独立复刻 C++ 的写法），
  然后逐键与参考实现 air_protocol.py 对撞。
  这样验证了**字节逻辑**；「语法/类型错误」留给用户在 Ubuntu 上的
  colcon build 或直接 g++ 编译 air_protocol_selftest.cpp 时暴露。

运行:  python3 air_protocol_cpp_mirror.py
"""

import io
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import air_protocol as REF  # noqa: E402

# =============================================================================
# 以下常量与 air_protocol.hpp 一一对应
# =============================================================================
K_MAGIC0 = 0xA5
K_MAGIC1 = 0xC3
K_VER = 0x02
K_HEADER_LEN = 10
K_CRC_LEN = 2
K_MAX_PAYLOAD = 1024
K_MAX_FRAME_LEN = 1036

K_HEARTBEAT = 0x01
K_TEAM_INFO = 0x10
K_FIELD_FIXED = 0x11
K_MAP_FULL = 0x12
K_MAP_DELTA = 0x13
K_MINE_LIST = 0x14
K_BRIDGE_GATE = 0x15
K_TARGET_LIST = 0x16
K_OBSTACLE_LIST = 0x17
K_TASK_CMD = 0x20
K_ROUTE = 0x21
K_VEH_STATUS = 0x30
K_EVENT = 0x31
K_ACK = 0x7F

K_OCC_UNKNOWN, K_OCC_FREE, K_OCC_BLOCKED = 0, 1, 2
K_OCC_MINE, K_OCC_TARGET, K_OCC_GATE_UNK, K_OCC_GATE_OPEN = 3, 4, 5, 6

K_CMD_NOP, K_CMD_NAV_TO_CELL, K_CMD_NAV_TO_POSE = 0, 1, 2
K_CMD_MINE_CLEAR_ALL, K_CMD_MINE_CLEAR_ONE, K_CMD_BRIDGE_CROSS = 3, 4, 5
K_CMD_ENGAGE, K_CMD_HOLD, K_CMD_STOP, K_CMD_RETURN_BASE, K_CMD_SAFE_RETREAT = 6, 7, 8, 9, 10

K_EV_NOP, K_EV_MINE_CLEARED, K_EV_BRIDGE_PASSED = 0, 1, 2
K_EV_TARGET_HIT, K_EV_TARGET_DESTROYED, K_EV_ARRIVED = 3, 4, 5
K_EV_STUCK_ESCAPED, K_EV_SAFE_RETREAT_START, K_EV_MAP_STALE = 6, 7, 8
K_EV_FRIENDLY_DETECTED, K_EV_GATE_INVALID = 9, 10
K_EV_GATE_CONFIRMED, K_EV_NEED_GATE = 11, 12

K_ST_IDLE, K_ST_NAV, K_ST_MINE_CLEAR, K_ST_BRIDGE_CROSS = 0, 1, 2, 3
K_ST_ENGAGE, K_ST_RETURN, K_ST_ESTOP, K_ST_DONE = 4, 5, 6, 7

K_FAULT_NONE, K_FAULT_NO_AMMO, K_FAULT_LOW_BATTERY = 0, 1, 2
K_FAULT_NO_LOC, K_FAULT_BLOCKED, K_FAULT_REFEREE = 3, 4, 5
K_FAULT_STUCK_ESC, K_FAULT_STUCK_UNREC = 7, 8


# =============================================================================
# 逐行镜像 air_protocol.cpp
# =============================================================================
def crc16_ccitt_false(data):
    """镜像 C++ crc16CcittFalse（逐位实现，与参考实现的写法独立）"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(typ, seq, payload):
    """镜像 C++ encodeFrame：手工按字节 push_back，不用 struct"""
    if len(payload) > K_MAX_PAYLOAD:
        return b""          # 空 = 失败
    out = bytearray()
    out.append(K_MAGIC0)
    out.append(K_MAGIC1)
    body_at = len(out)
    out.append(K_VER)
    out.append(typ)
    out.append(seq & 0xFF)
    out.append((seq >> 8) & 0xFF)
    out.append((seq >> 16) & 0xFF)
    out.append((seq >> 24) & 0xFF)
    ln = len(payload) & 0xFFFF
    out.append(ln & 0xFF)
    out.append((ln >> 8) & 0xFF)
    out.extend(payload)
    crc = crc16_ccitt_false(bytes(out[body_at:]))
    out.append(crc & 0xFF)
    out.append((crc >> 8) & 0xFF)
    return bytes(out)


def decode_frame(frame):
    """镜像 C++ decodeFrame，返回 (ok, typ, seq, payload, error)"""
    if len(frame) < K_HEADER_LEN + K_CRC_LEN:
        return False, 0, 0, b"", "frame too short"
    if frame[0] != K_MAGIC0 or frame[1] != K_MAGIC1:
        return False, 0, 0, b"", "bad magic"
    ver = frame[2]
    typ = frame[3]
    seq = frame[4] | (frame[5] << 8) | (frame[6] << 16) | (frame[7] << 24)
    ln = frame[8] | (frame[9] << 8)
    if ln > K_MAX_PAYLOAD:
        return False, typ, seq, b"", "bad len %d" % ln
    expect = K_HEADER_LEN + ln + K_CRC_LEN
    if len(frame) != expect:
        return False, typ, seq, b"", "len mismatch: expect %d got %d" % (expect, len(frame))
    crc_recv = frame[10 + ln] | (frame[11 + ln] << 8)
    crc_calc = crc16_ccitt_false(frame[2:10 + ln])
    if crc_recv != crc_calc:
        return False, typ, seq, b"", "crc mismatch: recv 0x%04X calc 0x%04X" % (crc_recv, crc_calc)
    if ver != K_VER:
        return False, typ, seq, b"", "version mismatch: %d != %d" % (ver, K_VER)
    return True, typ, seq, frame[10:10 + ln], ""


class FrameSync:
    """镜像 C++ FrameSync::feed（状态机 + A5 A5 C3 容错）"""

    def __init__(self):
        self.state = 0
        self.buf = bytearray()

    def _reset_to(self, b):
        self.state = 1 if b == K_MAGIC0 else 0
        self.buf = bytearray()

    def feed(self, data):
        frames = []
        for b in data:
            if self.state == 0:
                if b == K_MAGIC0:
                    self.state = 1
            elif self.state == 1:
                if b == K_MAGIC1:
                    self.buf = bytearray()
                    self.state = 2
                elif b == K_MAGIC0:
                    # ★ 第二个 A5 是新起点，留在状态 1
                    self.state = 1
                else:
                    self.state = 0
            else:
                self.buf.append(b)
                if len(self.buf) < 8:
                    continue
                ln = self.buf[6] | (self.buf[7] << 8)
                if ln > K_MAX_PAYLOAD:
                    self._reset_to(b)
                    continue
                want = 8 + ln + 2
                if len(self.buf) < want:
                    continue
                frame = bytes([K_MAGIC0, K_MAGIC1]) + bytes(self.buf)
                ok, typ, seq, payload, err = decode_frame(frame)
                if ok:
                    frames.append((typ, seq, payload))
                self.state = 0
                self.buf = bytearray()
        return frames


# ---------------- 载荷编码（镜像 enc::*） ----------------
def cell_encode(occ, conf=15, age=0):
    return (occ & 0x07) | ((conf & 0x0F) << 3) | ((age & 0x0F) << 7)


def _put32(v, x):
    v.append(x & 0xFF)
    v.append((x >> 8) & 0xFF)
    v.append((x >> 16) & 0xFF)
    v.append((x >> 24) & 0xFF)


def _put16(v, x):
    v.append(x & 0xFFFF & 0xFF)
    v.append((x & 0xFFFF) >> 8)


def e_heartbeat(ts_ms, map_version=0, quality=100, flags=0):
    v = bytearray()
    _put32(v, ts_ms)
    _put16(v, map_version)
    v.append(quality & 0xFF)
    v.append(flags & 0xFF)
    return bytes(v)


def e_team_info(our_color, cols=14, rows=11, cell_cm=35):
    return bytes([our_color, cols, rows, cell_cm])


def e_field_fixed(up=5, lo=3, hi=13, bcol=10, a=(0, 5), b=(6, 10)):
    return bytes([up, lo, hi, bcol, a[0], a[1], b[0], b[1]])


def e_map_full(map_version, ts_ms, cells):
    if len(cells) != 14 * 11:
        return b""
    v = bytearray()
    _put32(v, map_version)
    _put32(v, ts_ms)
    v.append(14)
    v.append(11)
    for c in cells:
        _put16(v, c)
    return bytes(v)


def e_mine_list(ts_ms, mines):
    v = bytearray()
    _put32(v, ts_ms)
    v.append(len(mines))
    for m in mines:
        v.extend(bytes([m[0], m[1], m[2]]))
    return bytes(v)


def e_bridge_gate(bridge_id, gate_row, confidence=100, flags=0x01):
    return bytes([bridge_id, gate_row, confidence, flags])


def e_target_list(ts_ms, targets):
    v = bytearray()
    _put32(v, ts_ms)
    v.append(len(targets))
    for t in targets:
        v.extend(bytes([t[0], t[1], t[2], t[3]]))
    return bytes(v)


def e_obstacle_list(obstacles):
    """★ 本条没有 ts_ms"""
    v = bytearray()
    v.append(len(obstacles))
    for o in obstacles:
        v.extend(bytes([o[0], o[1], o[2]]))
    return bytes(v)


def e_task_cmd(cmd, cmd_seq, extra=b""):
    v = bytearray([cmd, cmd_seq & 0xFF])
    v.extend(extra)
    return bytes(v)


def e_nav_to_cell(cmd_seq, col, row, timeout_s=60):
    return e_task_cmd(K_CMD_NAV_TO_CELL, cmd_seq, bytes([col, row, timeout_s]))


def e_bridge_cross(cmd_seq, bridge_id):
    return e_task_cmd(K_CMD_BRIDGE_CROSS, cmd_seq, bytes([bridge_id]))


def e_engage(cmd_seq, col, row, face=2):
    return e_task_cmd(K_CMD_ENGAGE, cmd_seq, bytes([col, row, face]))


def e_route(route_version, task_id, wps):
    v = bytearray()
    _put32(v, route_version)
    v.append(len(wps))
    v.append(task_id)
    for w in wps:
        v.extend(bytes([w[0], w[1], w[2]]))
    return bytes(v)


def e_veh_status(ts_ms, x_cm, y_cm, yaw_c10, v_cms, w_c100, task_state, fault,
                 mines_cleared=0, bridges_passed=0, self_hp=0, ammo=0,
                 battery_pct=100, pose_confidence=100, last_cmd_seq=0,
                 flags=0, blocked_col=-1, blocked_row=-1, exec_ms=0):
    v = bytearray()
    _put32(v, ts_ms)
    _put16(v, x_cm)
    _put16(v, y_cm)
    _put16(v, yaw_c10)
    _put16(v, v_cms)
    _put16(v, w_c100)
    v.append(task_state)
    v.append(fault)
    v.append(mines_cleared)
    v.append(bridges_passed)
    v.append(self_hp)
    v.append(ammo)
    v.append(battery_pct)
    v.append(pose_confidence)
    v.append(last_cmd_seq)
    v.append(flags)
    _put16(v, blocked_col)
    _put16(v, blocked_row)
    _put16(v, exec_ms)
    _put16(v, 0)
    return bytes(v)


def e_event(ts_ms, ev, a0=0, a1=0, a2=0):
    v = bytearray()
    _put32(v, ts_ms)
    v.extend(bytes([ev, a0, a1, a2]))
    return bytes(v)


def e_ack(ack_seq, status=0, detail=0):
    v = bytearray()
    _put32(v, ack_seq)
    v.append(status)
    v.append(detail)
    _put16(v, 0)
    return bytes(v)


# =============================================================================
# 对撞
# =============================================================================
fails = []


def check(name, cond, extra=""):
    print("  [%s] %s%s" % ("OK" if cond else "FAIL", name,
                           "" if cond else "  -> " + str(extra)))
    if not cond:
        fails.append(name)


def sec(t):
    print()
    print("=" * 76)
    print(t)
    print("=" * 76)


def main():
    print("C++ air_protocol.cpp 的 Python 镜像 —— 与参考实现对撞")
    print("（本机无 C++ 编译器，用逐行转写验证字节逻辑）")

    # ---------------- CRC ----------------
    sec("1. CRC-16/CCITT-FALSE")
    check("镜像 CRC('123456789') == 0x29B1",
          crc16_ccitt_false(b"123456789") == 0x29B1)
    check("镜像 CRC 与参考实现一致",
          crc16_ccitt_false(b"123456789") == REF.crc16_ccitt(b"123456789"))
    check("镜像 CRC(空) == 0xFFFF", crc16_ccitt_false(b"") == 0xFFFF)
    check("镜像 CRC 与参考实现一致(空)",
          crc16_ccitt_false(b"") == REF.crc16_ccitt(b""))
    # 随机对撞
    import random
    rng = random.Random(20260714)
    ok = True
    for _ in range(200):
        n = rng.randint(0, 64)
        d = bytes(rng.randrange(256) for _ in range(n))
        if crc16_ccitt_false(d) != REF.crc16_ccitt(d):
            ok = False
            break
    check("镜像 CRC 与参考实现 200 组随机对撞一致", ok)

    # ---------------- 组帧 ----------------
    sec("2. 组帧 / 解帧")
    payload = e_ack(7, 0, 0)
    f = encode_frame(K_ACK, 7, payload)
    ref = REF.encode(REF.T_ACK, 7, REF.hb_ack(7, 0, 0))
    check("ACK 整帧与参考实现逐字节一致", f == ref,
          "\n      镜像 %s\n      参考 %s" % (f.hex(), ref.hex()))
    check("ACK 帧长 20", len(f) == 20, len(f))
    ok, typ, seq, pl, err = decode_frame(f)
    check("解帧回环", ok and typ == K_ACK and seq == 7 and len(pl) == 8, err)

    # 超长载荷拒绝
    check("载荷 > 1024 被拒绝（返回空）",
          encode_frame(K_ROUTE, 1, bytes(K_MAX_PAYLOAD + 1)) == b"")
    # 最大载荷
    big = bytes([0xA5]) * K_MAX_PAYLOAD
    bf = encode_frame(K_ROUTE, 0xFFFFFFFF, big)
    check("最大载荷帧长 1036", len(bf) == K_MAX_FRAME_LEN, len(bf))

    # ---------------- 帧同步 ----------------
    sec("3. 帧同步状态机容错")
    hb_payload = e_heartbeat(1000, 2, 95, 0x05)
    hb_frame = encode_frame(K_HEARTBEAT, 42, hb_payload)

    fs = FrameSync()
    got = fs.feed(hb_frame)
    check("干净输入解出 1 帧", len(got) == 1 and got[0][1] == 42, len(got))

    fs = FrameSync()
    fs.feed(b"\xA5\xA5")
    got = fs.feed(hb_frame)
    check("★ A5 A5 后接有效帧（第二个 A5 当新起点）", len(got) == 1, len(got))

    fs = FrameSync()
    fs.feed(b"\x00\xFF\x12\xA5\x00")
    got = fs.feed(hb_frame)
    check("垃圾前缀后恢复", len(got) == 1, len(got))

    fs = FrameSync()
    for i in range(len(hb_frame)):
        got = fs.feed(hb_frame[i:i + 1])
    check("逐字节喂入", len(got) == 1, len(got))

    fs = FrameSync()
    second = encode_frame(K_ACK, 43, e_ack(43))
    got = fs.feed(hb_frame + second)
    check("一次喂两帧", len(got) == 2, len(got))

    fs = FrameSync()
    evil = bytes([0xA5, 0xC3, 0x02, 0x01, 0, 0, 0, 0, 0xFF, 0xFF])
    got = fs.feed(evil)
    check("非法 LEN(0xFFFF) 不产出帧", len(got) == 0, len(got))
    got = fs.feed(hb_frame)
    check("非法 LEN 之后仍能恢复", len(got) == 1, len(got))

    # ---------------- cell 位域 ----------------
    sec("4. cell 位域（occupancy 3 / confidence 4 / age 4）")
    check("cellEncode(MINE,15,0) == 0x007B", cell_encode(K_OCC_MINE, 15, 0) == 0x007B,
          hex(cell_encode(K_OCC_MINE, 15, 0)))
    check("与参考实现一致", cell_encode(K_OCC_MINE, 15, 0) == REF.cell_encode(REF.OCC_MINE, 15, 0))
    for (o, c, a) in ((0, 0, 0), (6, 15, 15), (2, 7, 3), (5, 15, 0)):
        v = cell_encode(o, c, a)
        ro, rc, ra = REF.cell_decode(v)
        check("往返 (%d,%d,%d)" % (o, c, a), (ro, rc, ra) == (o, c, a),
              (ro, rc, ra))

    # ---------------- 全部消息 ----------------
    sec("5. 全部消息载荷逐字节对撞")

    pairs = [
        ("HEARTBEAT", e_heartbeat(1000, 2, 95, 0x05),
         REF.hb_heartbeat(1000, 2, 95, 0x05), 8),
        ("TEAM_INFO", e_team_info(1, 14, 11, 35),
         REF.hb_team_info(1, 14, 11, 35), 4),
        ("FIELD_FIXED", e_field_fixed(),
         REF.hb_field_fixed(), 8),
        ("MINE_LIST(8)", e_mine_list(1000, [(3, 2, 90), (6, 4, 88), (11, 1, 85),
                                            (4, 7, 92), (9, 8, 80), (0, 3, 95),
                                            (8, 9, 70), (5, 2, 88)]),
         REF.hb_mine_list(1000, [(3, 2, 90), (6, 4, 88), (11, 1, 85),
                                 (4, 7, 92), (9, 8, 80), (0, 3, 95),
                                 (8, 9, 70), (5, 2, 88)]), 29),
        ("BRIDGE_GATE A", e_bridge_gate(1, 3, 100, 0x01),
         REF.hb_bridge_gate(1, 3, 100, 0x01), 4),
        ("BRIDGE_GATE B", e_bridge_gate(2, 8, 100, 0x01),
         REF.hb_bridge_gate(2, 8, 100, 0x01), 4),
        ("TARGET_LIST", e_target_list(1000, [(6, 4, 2, 90), (6, 5, 2, 90)]),
         REF.hb_target_list(1000, [(6, 4, 2, 90), (6, 5, 2, 90)]), 13),
        ("OBSTACLE_LIST(8)", e_obstacle_list([(1, 1, 1), (7, 5, 2), (13, 10, 1),
                                              (2, 6, 2), (10, 3, 1), (6, 8, 2),
                                              (0, 9, 1), (9, 4, 2)]),
         REF.hb_obstacle_list([(1, 1, 1), (7, 5, 2), (13, 10, 1), (2, 6, 2),
                               (10, 3, 1), (6, 8, 2), (0, 9, 1), (9, 4, 2)]), 25),
        ("TASK_CMD NOP", e_task_cmd(K_CMD_NOP, 0),
         REF.hb_task_cmd(REF.CMD_NOP, 0), 2),
        ("TASK_CMD STOP", e_task_cmd(K_CMD_STOP, 7),
         REF.hb_task_cmd(REF.CMD_STOP, 7), 2),
        ("NAV_TO_CELL", e_nav_to_cell(3, 12, 10, 60),
         REF.hb_task_nav_to_cell(3, 12, 10, 60), 5),
        ("BRIDGE_CROSS", e_bridge_cross(4, 1),
         REF.hb_task_bridge_cross(4, 1), 3),
        ("ENGAGE", e_engage(5, 6, 4, 2),
         REF.hb_task_engage(5, 6, 4, 2), 5),
        ("ROUTE", e_route(7, 2, [(12, 10, 0), (10, 8, 1), (13, 5, 0)]),
         REF.hb_route(7, 2, [(12, 10, 0), (10, 8, 1), (13, 5, 0)]), None),
        ("VEH_STATUS", e_veh_status(90000, 245, 175, -450, 30, -12,
                                    K_ST_MINE_CLEAR, K_FAULT_NONE,
                                    3, 1, 100, 7, 82, 95, 8, 0x01, 7, 5, 12345),
         REF.hb_veh_status(90000, 245, 175, -450, 30, -12, 2, 0,
                           3, 1, 100, 7, 82, 95, 8, 0x01, 7, 5, 12345), 32),
        ("VEH_STATUS 负值", e_veh_status(0, -1, -1, -1800, -30, 0,
                                         K_ST_IDLE, K_FAULT_BLOCKED,
                                         0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0),
         REF.hb_veh_status(0, -1, -1, -1800, -30, 0, 0, 4,
                           0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0), None),
        ("EVENT GATE_CONFIRMED", e_event(1000, K_EV_GATE_CONFIRMED, 0, 1, 0),
         REF.hb_event(1000, 11, 0, 1, 0), 8),
        ("EVENT GATE_INVALID", e_event(3000, K_EV_GATE_INVALID, 0, 0, 0),
         REF.hb_event(3000, 10, 0, 0, 0), 8),
        ("ACK", e_ack(7, 0, 0), REF.hb_ack(7, 0, 0), 8),
        ("ACK 最大值", e_ack(0xFFFFFFFF, 3, 9), REF.hb_ack(0xFFFFFFFF, 3, 9), None),
    ]

    for (name, mine, ref, want_len) in pairs:
        same = mine == ref
        extra = ""
        if not same:
            extra = "\n      镜像 %s\n      参考 %s" % (mine.hex(), ref.hex())
        if want_len is not None and len(mine) != want_len:
            same = False
            extra += "  长度 %d != %d" % (len(mine), want_len)
        check(name, same, extra)

    # MAP_FULL 单独构造（要用带内容的图才验得出位域）
    # ★ 注意：必须给**两边同样的语义**输入。
    #   镜像 e_map_full 收的是「已编码的 uint16」；
    #   参考 hb_map_full 收整数时会自动 cell_encode(occ, conf=15, age=0)。
    #   所以这里构造 occ 列表，镜像侧显式编码，参考侧直接传 occ。
    occ = [K_OCC_UNKNOWN] * (14 * 11)
    for r in range(11):
        occ[r * 14 + 10] = K_OCC_GATE_UNK
    occ[3 * 14 + 10] = K_OCC_GATE_OPEN
    occ[8 * 14 + 10] = K_OCC_GATE_OPEN
    for c in range(3, 14):
        occ[5 * 14 + c] = K_OCC_BLOCKED
    occ[2 * 14 + 3] = K_OCC_MINE
    occ[4 * 14 + 6] = K_OCC_MINE
    occ[6 * 14 + 6] = K_OCC_TARGET

    cells_enc = [cell_encode(o, 15, 0) for o in occ]   # 镜像侧：已编码
    mf_mine = e_map_full(5, 1000, cells_enc)
    mf_ref = REF.hb_map_full(5, 1000, occ)             # 参考侧：原始 occ
    check("MAP_FULL 长度 318", len(mf_mine) == 318, len(mf_mine))
    check("MAP_FULL 逐字节一致", mf_mine == mf_ref,
          "\n      镜像 %s...\n      参考 %s..." % (mf_mine[10:24].hex(), mf_ref[10:24].hex()))

    # ---------------- 整帧对撞 ----------------
    sec("6. 整帧（含帧头与 CRC）逐字节对撞")
    frame_cases = [
        ("ACK", K_ACK, e_ack(7), REF.T_ACK, REF.hb_ack(7)),
        ("HEARTBEAT", K_HEARTBEAT, e_heartbeat(1), REF.T_HEARTBEAT, REF.hb_heartbeat(1)),
        ("BRIDGE_GATE", K_BRIDGE_GATE, e_bridge_gate(1, 3), REF.T_BRIDGE_GATE,
         REF.hb_bridge_gate(1, 3)),
        ("MINE_LIST", K_MINE_LIST, e_mine_list(1, [(3, 2, 90)]), REF.T_MINE_LIST,
         REF.hb_mine_list(1, [(3, 2, 90)])),
        ("OBSTACLE_LIST", K_OBSTACLE_LIST, e_obstacle_list([(1, 1, 1)]),
         REF.T_OBSTACLE_LIST, REF.hb_obstacle_list([(1, 1, 1)])),
        ("MAP_FULL", K_MAP_FULL, mf_mine, REF.T_MAP_FULL, mf_ref),
    ]
    for (name, typ, pl, rtyp, rpl) in frame_cases:
        mine = encode_frame(typ, 9, pl)
        ref = REF.encode(rtyp, 9, rpl)
        check("整帧 %s" % name, mine == ref,
              "\n      镜像 %s\n      参考 %s" % (mine.hex(), ref.hex()))

    # ---------------- 描述器（§9 缺陷） ----------------
    sec("7. 参考实现 describe() 的 OBSTACLE_LIST 分支（无人机端 §9 发现）")
    obs_payload = REF.hb_obstacle_list([(1, 1, 1), (7, 5, 2)])
    obs_frame = REF.encode(REF.T_OBSTACLE_LIST, 1, obs_payload)
    d = REF.describe(obs_frame)
    check("describe() 能读出 count", d.get("count") == 2, d)
    check("describe() 能读出格坐标",
          d.get("obstacles") == [(1, 1, 1), (7, 5, 2)], d.get("obstacles"))
    check("describe() 标明本条无 ts_ms", d.get("has_ts") is False, d)

    # ---------------- 汇总 ----------------
    sec("结论")
    if not fails:
        print("  ✓ 镜像的 C++ 字节逻辑与参考实现完全一致（%d 项校验全过）" % (
            len(pairs) + len(frame_cases) + 20))
        print()
        print("  说明：本脚本验证的是**字节逻辑**（CRC/组帧/状态机/各消息布局）。")
        print("        C++ 的语法与类型正确性需要实际编译验证 —— 请在 Ubuntu 上跑：")
        print("          g++ -std=c++17 -I include src/air_protocol.cpp \\")
        print("              test/air_protocol_selftest.cpp -o /tmp/air_self && /tmp/air_self")
        return 0
    print("  ✗ %d 项不一致：" % len(fails))
    for f in fails:
        print("      - %s" % f)
    return 1


if __name__ == "__main__":
    sys.exit(main())
