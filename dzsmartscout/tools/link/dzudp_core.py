# -*- coding: utf-8 -*-
r"""DL1 解析核：教员 udp_bridge.py 的钉死快照（只同步不改）。

来源路径 : D:/User/Desktop/教员交接/udp_bridge.py
取证镜像 : _tmp_session/sqc-20261024/tools/udp_bridge.py（同一 sha256）
SHA-256  : 9ff259a36ce40f70437b713d0911d44a122f678909206ad734f0cc94a85b64e8
字节数   : 11821
抓取日期 : 2026-10-24
同步规则 : 只同步不改。本文件头（含下面那条 BEGIN 分界行）不是教员原件的部分；
           分界行之后是 udp_bridge.py 的**逐字节副本**，因此
           sha256(本文件去掉文件头) == 上面的 SHA-256（provenance 单测钉死）。

改了这个快照，必须**重跑**这两条并更新本文件的 SHA-256 与
tests/test_dzudp_core_provenance.py 的 PIN 常量：

    python -B -X utf8 -m unittest discover -s dzsmartscout/tools/link/tests -p "test_dzudp_*.py"
    python -B -X utf8 -m unittest discover -s _tmp_session/agc-1024/dl1/tests -p "test_*.py"

为什么入仓一份（2026-10-24 Lead 裁决①）：
    教员交接目录会搬走、机上没有这个文件 ⇒ 永久 importlib 外部路径 = 静默漂移
    （今天能跑，下周换个目录就 ImportError 或悄悄换了实现）。入仓一份快照 +
    provenance 单测，才能保证"全队永远用同一份解析核"。

边界（本模块只被"借用"，不被调用）：
    * 只借 crc16 / parse_intel / parse_pose / parse_route 与 MAGIC/VERSION/MSG_* 常量；
    * sniff / sniff_loop / bridge / --publish（写车侧路线话题）**一律不调用**；
    * PUBLISH_ENABLED 必须保持 False（provenance 单测断言）。
"""

#: 快照来源（教员原件绝对路径，仅留痕；运行时**不**依赖它存在）
PINNED_SOURCE = "D:/User/Desktop/教员交接/udp_bridge.py"
#: 教员原件 SHA-256（与 tests/test_dzudp_core_provenance.py 的 PIN 常量必须一致）
PINNED_SHA256 = "9ff259a36ce40f70437b713d0911d44a122f678909206ad734f0cc94a85b64e8"
#: 教员原件字节数
PINNED_BYTES = 11821
#: 抓取日期
PINNED_DATE = "2026-10-24"
#: 取证镜像（同一 sha256，只读交叉核对用）
PINNED_MIRROR = "_tmp_session/sqc-20261024/tools/udp_bridge.py"
#: 分界行：本行之后（跳过其后的换行）为 udp_bridge.py 逐字节副本
PINNED_MARKER = b"# ==== BEGIN PINNED SNAPSHOT (udp_bridge.py verbatim) ===="

# ==== BEGIN PINNED SNAPSHOT (udp_bridge.py verbatim) ====
#!/usr/bin/env python3
"""Car-side UDP bridge for dzuav broadcasts (UDP :50000, binary protocol).

Frame layout (little-endian), matches onboard UdpBroadcaster.cpp:
  magic(4x0xef) + ver(0x10) + ts(u64 ms) + msg_id(u16) + len(u32)
  0x4001 intel: + count(u16) + count*25B objects(type u8, x f64, y f64,
                  length f32, width f32)
  0x4002 pose : + x(f64) + y(f64) + yaw(f32 rad)
  all frames  : + crc16/modbus over [4:body_end] + suffix(4x0xfe)

Modes:
  (default) sniff   - parse & display, pure python, no ROS
  --ros             - rclpy node 'udp_bridge', same display
  --publish         - enable /nav/waypoint publishing (also needs
                      PUBLISH_ENABLED=True below; OFF by default per user)

0x4003 route frame (observed on air 2026-10-22, 2 Hz from drone):
  magic + ver + ts(u64) + msg_id(0x4003) + declen(u32)
  + u16 len1 + str1 ("reset:N10,x")
  + u16 len2 + str2 ("N10 K10 ... N5,x"  = /nav/waypoint format)
  + crc16 + suffix
"""

import argparse
import math
import socket
import struct
import sys
import time

DEFAULT_PORT = 50000
PUBLISH_ENABLED = False  # master switch for publishing, keep False for now

MAGIC = b"\xef\xef\xef\xef"
VERSION = 0x10
MSG_INTEL = 0x4001
MSG_POSE = 0x4002
MSG_ROUTE = 0x4003
SUFFIX = b"\xfe\xfe\xfe\xfe"
HDR_INTEL = 21   # magic4 + ver1 + ts8 + msgid2 + len4 + count2
HDR_POSE = 19    # magic4 + ver1 + ts8 + msgid2 + len4
OBJ_SIZE = 25    # type1 + x8 + y8 + len4 + width4
CRC_SIZE = 2
TAIL_SIZE = 4

TYPE_NAMES = {
    0: "PAPER", 1: "CELL_LABEL", 2: "CAR", 3: "TARGET_RED", 4: "TARGET_BLUE",
    5: "OBSTACLE", 6: "BUCKET_RED", 7: "BUCKET_GREEN", 8: "BUCKET_CYAN",
    9: "CAR_START", 10: "CAR_END", 11: "UAV_START", 12: "UAV_END",
}


def crc16(data):
    """CRC-16/Modbus, table-driven, equivalent to onboard bitwise version."""
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ _TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


_TABLE = []
for _b in range(256):
    _c = _b
    for _ in range(8):
        _c = (_c >> 1) ^ 0xA001 if _c & 1 else _c >> 1
    _TABLE.append(_c)


def parse_intel(data):
    """Parse 0x4001 frame; raise ValueError on any inconsistency."""
    if len(data) < HDR_INTEL + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"too short: {len(data)}B")
    ts, msg_id, length, count = struct.unpack_from("<QHIH", data, 5)
    if msg_id != MSG_INTEL:
        raise ValueError(f"msg_id 0x{msg_id:04x} not intel")
    if length != 2 + OBJ_SIZE * count:
        raise ValueError(f"len {length} != 2+25*{count}")
    if len(data) != HDR_INTEL + OBJ_SIZE * count + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"size {len(data)} != expected for count={count}")
    body_end = HDR_INTEL + OBJ_SIZE * count
    objs = []
    off = HDR_INTEL
    for _ in range(count):
        t, x, y, ln, wd = struct.unpack_from("<Bddff", data, off)
        if not 0 <= t <= 12:
            raise ValueError(f"unknown type {t}")
        if not all(math.isfinite(v) for v in (x, y, ln, wd)):
            raise ValueError("NaN/Inf in object")
        objs.append((int(t), x, y, ln, wd))
        off += OBJ_SIZE
    (rx_crc,) = struct.unpack_from("<H", data, body_end)
    if rx_crc != crc16(memoryview(data)[4:body_end]):
        raise ValueError("crc mismatch")
    if data[body_end + CRC_SIZE:body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")
    return ts, objs


def parse_pose(data):
    """Parse 0x4002 car-pose frame; raise ValueError on inconsistency."""
    if len(data) != HDR_POSE + 20 + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"pose size {len(data)} != 45")
    if data[0:4] != MAGIC or data[4] != VERSION:
        raise ValueError("bad magic/version")
    ts, msg_id, length = struct.unpack_from("<QHI", data, 5)
    if msg_id != MSG_POSE or length != 20:
        raise ValueError(f"msg_id 0x{msg_id:04x} len {length}, not pose")
    x, y, yaw = struct.unpack_from("<ddf", data, HDR_POSE)
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("NaN/Inf in pose")
    body_end = HDR_POSE + 20
    (rx_crc,) = struct.unpack_from("<H", data, body_end)
    if rx_crc != crc16(memoryview(data)[4:body_end]):
        raise ValueError("crc mismatch")
    if data[body_end + CRC_SIZE:body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")
    return ts, x, y, yaw


def parse_route(data):
    """Parse 0x4003 route frame; raise ValueError on inconsistency."""
    if len(data) < HDR_POSE + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"too short: {len(data)}B")
    if data[0:4] != MAGIC or data[4] != VERSION:
        raise ValueError("bad magic/version")
    ts, msg_id, length = struct.unpack_from("<QHI", data, 5)
    if msg_id != MSG_ROUTE:
        raise ValueError(f"msg_id 0x{msg_id:04x} not route")
    if len(data) != HDR_POSE + length + CRC_SIZE + TAIL_SIZE:
        raise ValueError(f"size {len(data)} != 19+{length}+6")
    off = HDR_POSE
    (len1,) = struct.unpack_from("<H", data, off); off += 2
    if off + len1 + 2 > HDR_POSE + length:
        raise ValueError(f"str1 len {len1} overruns payload")
    str1 = data[off:off + len1].decode("utf-8", errors="replace"); off += len1
    (len2,) = struct.unpack_from("<H", data, off); off += 2
    if off + len2 != HDR_POSE + length:
        raise ValueError(f"str2 len {len2} != remaining payload")
    str2 = data[off:off + len2].decode("utf-8", errors="replace")
    body_end = HDR_POSE + length
    (rx_crc,) = struct.unpack_from("<H", data, body_end)
    if rx_crc != crc16(memoryview(data)[4:body_end]):
        raise ValueError("crc mismatch")
    if data[body_end + CRC_SIZE:body_end + CRC_SIZE + TAIL_SIZE] != SUFFIX:
        raise ValueError("bad suffix")
    return ts, str1, str2


def open_udp(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    return sock


def fmt_intel(objs):
    lines = []
    for t, x, y, ln, wd in objs:
        name = TYPE_NAMES.get(t, f"T{t}")
        lines.append(f"    {name:<11} x={x:7.3f} y={y:7.3f} L={ln:.2f} W={wd:.2f}")
    return "\n".join(lines)


def sniff_loop(sock, on_frame):
    last_intel_key = None
    last_pose_key = None
    last_route_key = None
    stats = {"intel": 0, "pose": 0, "route": 0, "bad": 0}
    n = 0
    last_stat_ts = time.time()
    while True:
        data, addr = sock.recvfrom(65535)
        n += 1
        ts = time.strftime("%H:%M:%S")
        if len(data) >= 5 and data[0:4] == MAGIC and data[4] == VERSION:
            msg_id = struct.unpack_from("<H", data, 13)[0]
            if msg_id == MSG_INTEL:
                try:
                    frame_ts, objs = parse_intel(data)
                except ValueError as e:
                    stats["bad"] += 1
                    print(f"{ts} #{n} intel PARSE FAIL: {e}")
                    continue
                stats["intel"] += 1
                # dedupe identical snapshots, heartbeat every 5 s anyway
                key = tuple((t, round(x, 2), round(y, 2)) for t, x, y, _, _ in objs)
                now = time.time()
                if key != last_intel_key or now - last_stat_ts >= 5.0:
                    last_intel_key = key
                    last_stat_ts = now
                    print(f"{ts} #{n} INTEL from {addr[0]} ts={frame_ts} "
                          f"objs={len(objs)} (ok intel={stats['intel']} "
                          f"pose={stats['pose']} bad={stats['bad']})")
                    if objs:
                        print(fmt_intel(objs))
                    else:
                        print("    (empty frame)")
                    if on_frame:
                        on_frame("intel", objs)
            elif msg_id == MSG_ROUTE:
                try:
                    _, str1, str2 = parse_route(data)
                except ValueError as e:
                    stats["bad"] += 1
                    print(f"{ts} #{n} route PARSE FAIL: {e}")
                    continue
                stats["route"] += 1
                key = (str1, str2)
                now = time.time()
                if key != last_route_key or now - last_stat_ts >= 5.0:
                    last_route_key = key
                    last_stat_ts = now
                    toks = str2.split()
                    print(f"{ts} #{n} ROUTE from {addr[0]} "
                          f"reset={str1!r} ({len(toks)} waypoints) "
                          f"(route={stats['route']} intel={stats['intel']} "
                          f"pose={stats['pose']} bad={stats['bad']})")
                    print(f"    ROUTE: {str2}")
                    if on_frame:
                        on_frame("route", (str1, str2))
            elif msg_id == MSG_POSE:
                try:
                    _, x, y, yaw = parse_pose(data)
                except ValueError as e:
                    stats["bad"] += 1
                    print(f"{ts} #{n} pose PARSE FAIL: {e}")
                    continue
                stats["pose"] += 1
                key = (round(x, 2), round(y, 2), round(yaw, 2))
                now = time.time()
                if key != last_pose_key or now - last_stat_ts >= 5.0:
                    last_pose_key = key
                    last_stat_ts = now
                    print(f"{ts} #{n} POSE  from {addr[0]} "
                          f"x={x:7.3f} y={y:7.3f} yaw={yaw:6.3f}rad"
                          f"({math.degrees(yaw):6.1f}deg) "
                          f"(ok intel={stats['intel']} pose={stats['pose']} "
                          f"bad={stats['bad']})")
                    if on_frame:
                        on_frame("pose", (x, y, yaw))
            else:
                stats["bad"] += 1
                if stats["bad"] % 20 == 1:
                    print(f"{ts} #{n} unknown msg_id 0x{msg_id:04x} "
                          f"len={len(data)} from {addr[0]}")
        else:
            stats["bad"] += 1
            if stats["bad"] % 20 == 1:
                print(f"{ts} #{n} NON-PROTOCOL pkt len={len(data)} "
                      f"from {addr[0]} head={data[:16].hex(' ')}")


def sniff(port):
    sock = open_udp(port)
    print(f"[sniff] listening UDP 0.0.0.0:{port}, Ctrl+C to stop ...")
    sniff_loop(sock, None)


def bridge(port, do_publish):
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("udp_bridge")
    pub = node.create_publisher(String, "/nav/waypoint", 10)
    active = do_publish and PUBLISH_ENABLED
    print(f"[bridge] node=udp_bridge publish={'ON' if active else 'OFF'}")

    def on_frame(kind, payload):
        if kind == "route":
            reset_cmd, route = payload
            # route is already in /nav/waypoint string format; reset_cmd
            # is the "reset:N10,x" anchor command from the drone
            if active:
                pub.publish(String(data=reset_cmd))
                pub.publish(String(data=route))
                print(f"    -> PUBLISHED reset + {route!r}")
            else:
                print(f"    -> (publish disabled; would send reset "
                      f"{reset_cmd!r} then route {route!r})")

    sock = open_udp(port)
    sniff_loop(sock, on_frame)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--ros", action="store_true", help="run as ros bridge node")
    ap.add_argument("--publish", action="store_true",
                    help="enable publishing (also needs PUBLISH_ENABLED)")
    args = ap.parse_args()
    try:
        if args.ros:
            bridge(args.port, args.publish)
        else:
            sniff(args.port)
    except KeyboardInterrupt:
        print("\nbye")
        sys.exit(0)


if __name__ == "__main__":
    main()
