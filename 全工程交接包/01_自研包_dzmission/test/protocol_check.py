#!/usr/bin/env python3
"""dzmission 任务指令协议 —— 独立参考校验脚本

不依赖 ROS，纯 Python3 实现协议 v1 的编解码，用于：
  1. 验证 CRC-16/MODBUS 与帧结构是否与 src/mission_command.cpp 一致
  2. 作为无人机/上位机侧的对接样例（可直接拷到 PC 端使用）
  3. 生成测试帧，供 ros2 topic pub / 串口工具注入

运行：
  python3 protocol_check.py            # 自检 + 打印示例帧
  python3 protocol_check.py --decode AA5501006181
"""

import argparse
import struct
import sys

HEAD0 = 0xAA
HEAD1 = 0x55
MAX_PAYLOAD = 64

CMD_NOP = 0x00
CMD_NAV_TO = 0x01
CMD_STOP = 0x02
CMD_ENGAGE = 0x10
CMD_CEASE_FIRE = 0x11
CMD_RETURN_BASE = 0x20
CMD_HOLD = 0x21
CMD_TARGET_REPORT = 0x30
CMD_SET_HOME = 0x40

REPORT_STATUS = 0x81
REPORT_TARGET_ACK = 0x82
REPORT_FAULT = 0x83

CMD_NAMES = {
    CMD_NOP: "NOP", CMD_NAV_TO: "NAV_TO", CMD_STOP: "STOP",
    CMD_ENGAGE: "ENGAGE", CMD_CEASE_FIRE: "CEASE_FIRE",
    CMD_RETURN_BASE: "RETURN_BASE", CMD_HOLD: "HOLD",
    CMD_TARGET_REPORT: "TARGET_REPORT", CMD_SET_HOME: "SET_HOME",
    REPORT_STATUS: "STATUS", REPORT_TARGET_ACK: "TARGET_ACK",
    REPORT_FAULT: "FAULT",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def encode(cmd: int, payload: bytes = b"") -> bytes:
    length = 1 + len(payload)
    assert 1 <= length <= MAX_PAYLOAD, "payload too long"
    body = bytes([length, cmd]) + payload
    crc = crc16_modbus(body)
    return bytes([HEAD0, HEAD1]) + body + struct.pack("<H", crc)


def decode(frame: bytes):
    """返回 (cmd, payload)；校验失败抛 ValueError"""
    if len(frame) < 5:
        raise ValueError("frame too short")
    if frame[0] != HEAD0 or frame[1] != HEAD1:
        raise ValueError("bad header")
    length = frame[2]
    if not 1 <= length <= MAX_PAYLOAD:
        raise ValueError("bad len field %d" % length)
    if len(frame) != 3 + length + 2:
        raise ValueError("len mismatch: expect %d got %d" % (3 + length + 2, len(frame)))
    body = frame[2:3 + length]
    crc_recv = struct.unpack("<H", frame[3 + length:])[0]
    crc_calc = crc16_modbus(body)
    if crc_recv != crc_calc:
        raise ValueError("crc mismatch: recv 0x%04X calc 0x%04X" % (crc_recv, crc_calc))
    return frame[3], frame[4:3 + length]


# ---------------------------------------------------------------------------
# 上行指令构造
# ---------------------------------------------------------------------------
def make_nav_to(x, y, yaw=None, seq=0) -> bytes:
    flags = 0x01 if yaw is not None else 0x00
    payload = struct.pack("<ii", int(round(x * 100)), int(round(y * 100)))
    payload += struct.pack("<h", int(round((yaw or 0.0) * 180.0 / 3.141592653589793 * 10)))
    payload += bytes([flags, seq & 0xFF])
    return encode(CMD_NAV_TO, payload)


def make_stop(stop=1, seq=0) -> bytes:
    return encode(CMD_STOP, bytes([stop, seq & 0xFF]))


def make_engage(target_id, confidence, bearing_deg, range_m, seq=0, solve_offset=True) -> bytes:
    payload = bytes([target_id & 0xFF, confidence & 0xFF,
                     0x01 if solve_offset else 0x00])
    payload += struct.pack("<h", int(round(bearing_deg * 10)))
    payload += struct.pack("<H", int(round(range_m * 100)))
    payload += bytes([seq & 0xFF])
    return encode(CMD_ENGAGE, payload)


def make_cease_fire(seq=0) -> bytes:
    return encode(CMD_CEASE_FIRE, bytes([seq & 0xFF]))


def make_set_home(x, y, seq=0) -> bytes:
    return encode(CMD_SET_HOME, struct.pack("<ii", int(round(x * 100)), int(round(y * 100))) + bytes([seq & 0xFF]))


# ---------------------------------------------------------------------------
def decode_mission(frame: bytes) -> dict:
    cmd, p = decode(frame)
    out = {"cmd": cmd, "name": CMD_NAMES.get(cmd, "UNKNOWN(%02X)" % cmd), "len": len(frame)}

    if cmd == CMD_NAV_TO:
        x = struct.unpack("<i", p[0:4])[0] / 100.0
        y = struct.unpack("<i", p[4:8])[0] / 100.0
        yaw = struct.unpack("<h", p[8:10])[0] / 10.0
        out.update(x=x, y=y, yaw_deg=yaw, has_yaw=bool(p[10] & 1), seq=p[11])
    elif cmd == CMD_STOP:
        out.update(stop=p[0], seq=p[1])
    elif cmd == CMD_ENGAGE:
        out.update(target_id=p[0], confidence=p[1],
                   bearing_deg=struct.unpack("<h", p[3:5])[0] / 10.0,
                   range_m=struct.unpack("<H", p[5:7])[0] / 100.0,
                   seq=p[6])
    elif cmd == CMD_TARGET_REPORT:
        out.update(target_id=p[0], confidence=p[1],
                   bearing_deg=struct.unpack("<h", p[2:4])[0] / 10.0,
                   range_m=struct.unpack("<H", p[4:6])[0] / 100.0,
                   seq=p[5])
    elif cmd == CMD_SET_HOME:
        out.update(x=struct.unpack("<i", p[0:4])[0] / 100.0,
                   y=struct.unpack("<i", p[4:8])[0] / 100.0, seq=p[8])
    elif cmd in (CMD_CEASE_FIRE, CMD_RETURN_BASE, CMD_HOLD):
        out.update(seq=p[0])
    elif cmd == REPORT_STATUS:
        out.update(seq=p[0], mission_state=p[1], fault=p[2], self_hp=p[3], ammo=p[4],
                   hit_dir=p[5], shoot_count=p[6], battery_pct=p[7],
                   x=struct.unpack("<i", p[8:12])[0] / 100.0,
                   y=struct.unpack("<i", p[12:16])[0] / 100.0,
                   yaw_deg=struct.unpack("<h", p[16:18])[0] / 10.0,
                   v=struct.unpack("<h", p[18:20])[0] / 100.0,
                   w=struct.unpack("<h", p[20:22])[0] / 100.0)
    elif cmd == REPORT_TARGET_ACK:
        out.update(seq=p[0], target_id=p[1], result=p[2], ammo_left=p[3],
                   aim_x=struct.unpack("<i", p[4:8])[0] / 100.0,
                   aim_y=struct.unpack("<i", p[8:12])[0] / 100.0)
    elif cmd == REPORT_FAULT:
        out.update(fault=p[0], detail=p[1])
    return out


MISSION_STATE = {
    0: "DISABLED", 1: "IDLE", 2: "NAVIGATING", 3: "ENGAGING",
    4: "RETURNING", 5: "HOLDING", 6: "ESTOP", 7: "ARRIVED",
}


def self_test() -> int:
    failures = 0

    def check(name, cond, extra=""):
        nonlocal failures
        if cond:
            print("  [ OK ] %s" % name)
        else:
            failures += 1
            print("  [FAIL] %s %s" % (name, extra))

    print("== 1. CRC 与最小帧 ==")
    check("CRC('123456789') == 0x4B37", crc16_modbus(b"123456789") == 0x4B37,
          "got 0x%04X" % crc16_modbus(b"123456789"))
    frame = encode(CMD_NOP)
    check("NOP 帧 = AA 55 01 00 00 20",
          frame == bytes.fromhex("AA5501000020"), "got " + frame.hex().upper())

    print("== 2. 帧结构 ==")
    f = make_nav_to(1.25, -3.5, yaw=0.5, seq=7)
    check("NAV_TO 帧长度 = 3 + 13 + 2 = 18", len(f) == 18, "got %d" % len(f))
    check("帧头正确", f[0] == HEAD0 and f[1] == HEAD1)
    check("LEN 字段 = 1 + 12 = 13", f[2] == 13, "got %d" % f[2])

    print("== 3. 上行编解码往返 ==")
    cases = [
        ("NAV_TO", make_nav_to(2.34, -1.07, yaw=-0.785, seq=3)),
        ("NAV_TO(no yaw)", make_nav_to(0.0, 0.0, None, seq=9)),
        ("STOP", make_stop(1, 5)),
        ("ENGAGE", make_engage(1, 90, 30.5, 3.5, seq=11)),
        ("ENGAGE(neg bearing)", make_engage(2, 70, -45.0, 12.0, seq=12)),
        ("CEASE_FIRE", make_cease_fire(13)),
        ("SET_HOME", make_set_home(1.5, 1.5, 1)),
    ]
    for name, fr in cases:
        try:
            d = decode_mission(fr)
            print("  %-20s -> %s" % (name, d))
        except ValueError as e:
            failures += 1
            print("  [FAIL] %-20s %s" % (name, e))

    print("== 4. 边界值 ==")
    try:
        d = decode_mission(make_nav_to(0.0, 0.0, None, 0))
        check("零坐标可解", d["x"] == 0.0 and d["y"] == 0.0)
        d = decode_mission(make_engage(1, 100, 179.9, 655.35, 255))
        check("最大距离 655.35m 可解", abs(d["range_m"] - 655.35) < 0.01,
              "got %s" % d["range_m"])
        d = decode_mission(make_engage(1, 100, -179.9, 1.0, 1))
        check("负方位可解", abs(d["bearing_deg"] + 179.9) < 0.05,
              "got %s" % d["bearing_deg"])
    except ValueError as e:
        failures += 1
        print("  [FAIL] 边界值 %s" % e)

    print("== 5. 损坏帧必须被拒绝 ==")
    good = bytearray(make_nav_to(1.0, 1.0, None, 1))
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0xFF
    bad_hdr = bytearray(good)
    bad_hdr[0] = 0xAB
    bad_len = bytearray(good)
    bad_len[2] = 0x00
    for name, fr in (("CRC 错误", bad_crc), ("帧头错误", bad_hdr), ("LEN=0", bad_len)):
        try:
            decode(bytes(fr))
            failures += 1
            print("  [FAIL] %s 未被拒绝" % name)
        except ValueError as e:
            print("  [ OK ] %s 被拒绝: %s" % (name, e))

    print("== 6. 示例帧（可直接注入） ==")
    print("  NAV_TO (1.25, -3.50) yaw=0.5rad : %s" % make_nav_to(1.25, -3.50, 0.5, 1).hex().upper())
    print("  STOP                            : %s" % make_stop(1, 2).hex().upper())
    print("  ENGAGE id=1 conf=90 brg=30.5 r=3.5: %s" % make_engage(1, 90, 30.5, 3.5, 4).hex().upper())
    print("  RETURN_BASE                     : %s" % encode(CMD_RETURN_BASE, bytes([1])).hex().upper())
    print("  SET_HOME (1.50, 1.50)           : %s" % make_set_home(1.5, 1.5, 5).hex().upper())

    print()
    if failures == 0:
        print("全部自检通过。")
    else:
        print("%d 项自检失败。" % failures)
    return 1 if failures else 0


def main():
    ap = argparse.ArgumentParser(description="dzmission 协议参考实现")
    ap.add_argument("--decode", help="解码十六进制帧，例如 AA5501006181")
    ap.add_argument("--nav-to", nargs=2, type=float, metavar=("X", "Y"),
                    help="生成 NAV_TO 帧")
    ap.add_argument("--yaw-deg", type=float, default=None, help="NAV_TO 的航向（度）")
    ap.add_argument("--engage", nargs=3, metavar=("ID", "BEARING_DEG", "RANGE_M"),
                    help="生成 ENGAGE 帧")
    args = ap.parse_args()

    if args.decode:
        raw = bytes.fromhex(args.decode.replace(" ", ""))
        print(decode_mission(raw))
        return 0

    if args.nav_to:
        yaw = None if args.yaw_deg is None else args.yaw_deg * 3.141592653589793 / 180.0
        print(make_nav_to(args.nav_to[0], args.nav_to[1], yaw, 1).hex().upper())
        return 0

    if args.engage:
        tid = int(args.engage[0])
        brg = float(args.engage[1])
        rng = float(args.engage[2])
        print(make_engage(tid, 100, brg, rng, 1).hex().upper())
        return 0

    return self_test()


if __name__ == "__main__":
    sys.exit(main())
