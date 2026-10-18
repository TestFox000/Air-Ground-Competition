#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 drone_sim.py —— 无人机端离线模拟器（车端离线测试用）
================================================================================

作用
----
把一份**易读的地图 YAML** 转成空地协同协议 v2 的二进制帧，通过 UDP 发给车端，
从而在**不接真无人机**的情况下驱动车端的：
    · 排雷规划（40 分）
    · 桥洞穿越（20 分）
    · 云台识别与打击（红蓝面联锁）
    · 任务指令（导航 / 交战 / 待命 / 停车 / 返航）

你只需要改地图 YAML 里的**坐标和参数**，不用碰任何协议代码。

快速开始
--------
    # 1) 车端改成 udp 模式（params/dzmission.yaml）
    #      cmd:
    #        transport: "udp"
    #        udp_bind_port: 9100
    #    然后启动车端：ros2 launch dzmission bringup_mission.launch.py

    # 2) 本机跑模拟器
    python3 tools/drone_sim.py --map tools/maps/example_standard.yaml

    # 3) 只看编码结果、不发送（先确认数据对不对）
    python3 tools/drone_sim.py --map tools/maps/example_standard.yaml --dry-run

    # 4) 发给另一台机器（车端在 192.168.1.100）
    python3 tools/drone_sim.py --map ... --host 192.168.1.100 --port 9100

常用选项
--------
    --dry-run        只打印将要发送的帧（十六进制 + 解码说明），不实际发送
    --loop           发完一轮后循环重发（车端掉线重连时有用）
    --speed 2.0      时间轴加速 2 倍（默认 1.0 按地图里的 t 秒发）
    --once-tasks     任务指令只发一次（默认会按 repeat_task_s 周期重发）
    --verbose        打印每帧的详细解码

协议来源
--------
编码**完全复用** test/air_protocol.py —— 它是与 C++ 侧逐字节对撞验证过的
参考实现（golden 向量由 air_protocol_golden_check.py 校验）。
所以本模拟器发出的帧与真无人机发出的**完全一致**，不存在"模拟器通过、
真机不通"的风险。

★ 安全：本脚本只发送，**不监听**车端回传。车端的上报可用
   `ros2 topic echo /mission_cmd_raw` 单独观察。
================================================================================
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time

# ------------------------------------------------------------------------------
# 让 air_protocol 可被导入（它在 test/ 下）
# ------------------------------------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)                      # dzmission/
for _cand in (os.path.join(_ROOT, 'test'), _ROOT):
    if os.path.isfile(os.path.join(_cand, 'air_protocol.py')):
        sys.path.insert(0, _cand)
        break
else:
    sys.stderr.write(
        "★ 找不到 air_protocol.py。请从 dzmission/ 目录下运行本脚本，\n"
        "   或确认 test/air_protocol.py 存在（它是协议参考实现）。\n")
    sys.exit(2)

import air_protocol as ap                                    # noqa: E402

# ------------------------------------------------------------------------------
# 极简 YAML 读取
# ------------------------------------------------------------------------------
# ★ 为什么不直接 import yaml：车端环境不保证装了 PyYAML，而本脚本要在
#   任何一台 ROS 机器上跑得起来。这里实现一个**只覆盖本 schema** 的子集解析器：
#     键: 值          标量
#     键:             嵌套映射
#     键: [a, b, c]   行内列表
#     - [a, b, c]     列表项
#     - {k: v, ...}   行内映射列表项
#     # 注释
#   够用即可，且报错信息友好。
# ------------------------------------------------------------------------------

class YamlError(Exception):
    pass


def _scalar(tok: str):
    """把 YAML 标量转成 Python 值"""
    t = tok.strip()
    if t == '' or t == '~' or t.lower() == 'null':
        return None
    if (t.startswith('"') and t.endswith('"')) or (t.startswith("'") and t.endswith("'")):
        return t[1:-1]
    low = t.lower()
    if low in ('true', 'yes', 'on'):
        return True
    if low in ('false', 'no', 'off'):
        return False
    try:
        return int(t, 0)          # 支持 0x 前缀
    except ValueError:
        pass
    try:
        return float(t)
    except ValueError:
        pass
    return t


def _inline_list(tok: str):
    """解析 [a, b, c] 或 [[1,2],[3,4]]"""
    body = tok.strip()[1:-1].strip()
    if not body:
        return []
    out, depth, cur = [], 0, ''
    for ch in body:
        if ch in '[{':
            depth += 1
        elif ch in ']}':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur)
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur)
    res = []
    for item in out:
        s = item.strip()
        if s.startswith('[') and s.endswith(']'):
            res.append(_inline_list(s))
        elif s.startswith('{') and s.endswith('}'):
            res.append(_inline_map(s))
        else:
            res.append(_scalar(s))
    return res


def _inline_map(tok: str):
    """解析 {k: v, k2: v2}"""
    body = tok.strip()[1:-1].strip()
    d = {}
    if not body:
        return d
    parts, depth, cur = [], 0, ''
    for ch in body:
        if ch in '[{':
            depth += 1
        elif ch in ']}':
            depth -= 1
        if ch == ',' and depth == 0:
            parts.append(cur)
            cur = ''
        else:
            cur += ch
    if cur.strip():
        parts.append(cur)
    for p in parts:
        if ':' not in p:
            continue
        k, v = p.split(':', 1)
        vs = v.strip()
        if vs.startswith('[') and vs.endswith(']'):
            d[k.strip()] = _inline_list(vs)
        elif vs.startswith('{') and vs.endswith('}'):
            d[k.strip()] = _inline_map(vs)
        else:
            d[k.strip()] = _scalar(vs)
    return d


def load_yaml(path: str) -> dict:
    """读取本 schema 的地图文件"""
    if not os.path.isfile(path):
        raise YamlError("地图文件不存在: %s" % path)

    raw_lines = []
    with open(path, encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            # 去注释（保留引号内的 #）
            s, in_s, quote = '', False, ''
            for ch in line.rstrip('\n'):
                if in_s:
                    if ch == quote:
                        in_s = False
                    s += ch
                else:
                    if ch in '"\'':
                        in_s, quote = True, ch
                        s += ch
                    elif ch == '#':
                        break
                    else:
                        s += ch
            if s.strip():
                raw_lines.append((lineno, s.rstrip()))

    root: dict = {}
    # 栈元素: (缩进, 容器, 当前 key 或 None)
    stack = [(-1, root, None)]

    for lineno, line in raw_lines:
        indent = len(line) - len(line.lstrip(' '))
        content = line.strip()

        while len(stack) > 1 and indent <= stack[-1][0]:
            stack.pop()
        _, container, cur_key = stack[-1]

        # 列表项
        if content.startswith('- '):
            item = content[2:].strip()
            if not isinstance(container, list):
                # 把父容器的该 key 换成列表
                parent = stack[-2][1] if len(stack) >= 2 else root
                if cur_key is None:
                    raise YamlError("第 %d 行: 列表项没有对应的键" % lineno)
                parent[cur_key] = []
                container = parent[cur_key]
                stack[-1] = (stack[-1][0], container, cur_key)
            if item.startswith('{') and item.endswith('}'):
                container.append(_inline_map(item))
            elif item.startswith('[') and item.endswith(']'):
                container.append(_inline_list(item))
            elif ':' in item and not item.startswith('"'):
                container.append(_inline_map('{%s}' % item))
            else:
                container.append(_scalar(item))
            continue

        if ':' not in content:
            raise YamlError("第 %d 行: 不是 k: v 格式 -> %s" % (lineno, content))

        key, val = content.split(':', 1)
        key = key.strip()
        val = val.strip()

        if val == '':
            # 可能是嵌套映射，也可能是列表（先占位为 dict，遇到 '-' 时再换）
            container[key] = {}
            stack.append((indent, container[key], key))
        elif val.startswith('[') and val.endswith(']'):
            container[key] = _inline_list(val)
        elif val.startswith('{') and val.endswith('}'):
            container[key] = _inline_map(val)
        else:
            container[key] = _scalar(val)

    return root


# ==============================================================================
# 地图 -> 帧序列
# ==============================================================================
# 默认场地参数（与 C++ FieldGrid 的 v5 规格一致；地图里可覆盖）
DEFAULTS = {
    'cols': 14,
    'rows': 11,
    'cell_cm': 35,
    'barrier_up_row': 5,
    'barrier_col_lo': 3,
    'barrier_col_hi': 13,
    'bridge_col': 10,
    'bridge_a': [0, 5],
    'bridge_b': [6, 10],
    'start': [12, 10],
    'goal': [13, 5],
    # ★ 无人机起降点（禁入格）—— 无人机起飞前/降落后停在这两格，
    #   地面车经过会撞机。随 FIELD_FIXED v2.1 下发（追加 4 字节，向后兼容）。
    'drone_start': [12, 8],      # M9
    'drone_end': [13, 1],        # N2
    'heading_deg': -90.0,
    'team_color': 1,
    'map_version': 1,
    'repeat_task_s': 1.0,        # 任务指令重发周期
}

# 任务指令名 -> (air 任务码, 是否需要 col/row)
TASK_CMDS = {
    'nop':         (ap.CMD_NOP, False),
    'nav_to':      (ap.CMD_NAV_TO_CELL, True),
    'bridge_cross': (ap.CMD_BRIDGE_CROSS, False),
    'engage':      (ap.CMD_ENGAGE, True),
    'hold':        (ap.CMD_HOLD, False),
    'stop':        (ap.CMD_STOP, False),
    'return_base': (ap.CMD_RETURN_BASE, False),
    'safe_retreat': (ap.CMD_SAFE_RETREAT, False),
}


def build_frames(mp: dict, verbose: bool = False):
    """把地图 dict 转成 [(t_sec, 说明, 帧bytes), ...]（按时间排序）"""
    cfg = dict(DEFAULTS)
    cfg.update(mp.get('field', {}) or {})

    frames = []
    # ★ air_protocol.encode(typ, seq, payload) —— seq 是必填的 32 位序列号。
    #   车端用 seq 做回执/去重，所以这里每帧递增（不要都填 0）。
    seq_counter = [0]

    def add(t, desc, payload, typ):
        seq_counter[0] = (seq_counter[0] + 1) & 0xFFFFFFFF
        frames.append((float(t), desc, ap.encode(typ, seq_counter[0], payload)))

    # ---------------- 上电即发的静态信息 ----------------
    t0 = 0.0
    # ★ 参数名以 air_protocol.py 为准：
    #   up/lo/hi = 隔离线上方行号与覆盖列范围
    #   bcol     = 桥洞列
    #   a/b      = 两个区段的行范围 (lo, hi)
    add(t0, 'FIELD_FIXED 固定结构（隔离线/桥洞列/起终点/无人机起降点）',
        ap.hb_field_fixed(up=cfg['barrier_up_row'],
                          lo=cfg['barrier_col_lo'],
                          hi=cfg['barrier_col_hi'],
                          bcol=cfg['bridge_col'],
                          a=tuple(cfg['bridge_a']),
                          b=tuple(cfg['bridge_b']),
                          # ★ v2.1：无人机起降点 —— 车端会把这两格设为禁入
                          drone_start=tuple(cfg['drone_start']),
                          drone_end=tuple(cfg['drone_end'])),
        ap.T_FIELD_FIXED)

    add(t0, 'TEAM_INFO 队伍信息（颜色/场地尺寸）',
        ap.hb_team_info(cfg['team_color'], cfg['cols'],
                        cfg['rows'], cfg['cell_cm']),
        ap.T_TEAM_INFO)

    # ---------------- 侦察结果 ----------------
    mines = mp.get('mines') or []
    if mines:
        add(mp.get('t_mines', 0.5),
            'MINE_LIST 地雷 %d 颗' % len(mines),
            ap.hb_mine_list(1000, [tuple(m) for m in mines]),
            ap.T_MINE_LIST)

    obstacles = mp.get('obstacles') or []
    if obstacles:
        add(mp.get('t_obstacles', 0.8),
            'OBSTACLE_LIST 障碍 %d 个' % len(obstacles),
            ap.hb_obstacle_list([tuple(o) for o in obstacles]),
            ap.T_OBSTACLE_LIST)

    targets = mp.get('targets') or []
    if targets:
        add(mp.get('t_targets', 1.0),
            'TARGET_LIST 打击区 %d 格' % len(targets),
            ap.hb_target_list(1000, [tuple(t) for t in targets]),
            ap.T_TARGET_LIST)

    gates = mp.get('gates') or []
    for i, g in enumerate(gates):
        # g = [bridge_id, gate_row] 或 [bridge_id, gate_row, conf, flags]
        if len(g) >= 4:
            bid, row, conf, flags = g[0], g[1], g[2], g[3]
        elif len(g) == 3:
            bid, row, conf, flags = g[0], g[1], g[2], 0x01
        else:
            bid, row, conf, flags = g[0], g[1], 100, 0x01
        add(mp.get('t_gates', 1.2) + i * 0.05,
            'BRIDGE_GATE 区段%d 门位 K%d' % (bid, row + 1),
            ap.hb_bridge_gate(bid, row, conf, flags),
            ap.T_BRIDGE_GATE)

    # ---------------- 全图（可选）----------------
    if mp.get('map_full'):
        cells = mp['map_full'].get('cells') or []
        mv = mp['map_full'].get('map_version', cfg['map_version'])
        add(mp.get('t_map_full', 1.5),
            'MAP_FULL 全图 %d 格' % len(cells),
            ap.hb_map_full(mv, 1000, [int(c) for c in cells]),
            ap.T_MAP_FULL)

    # ---------------- 任务指令序列 ----------------
    task_seq = 0
    for item in (mp.get('commands') or []):
        t = item.get('t', 2.0)
        name = str(item.get('cmd', 'nop')).lower()
        if name not in TASK_CMDS:
            raise YamlError("未知任务指令 '%s'，可选: %s"
                            % (name, ', '.join(sorted(TASK_CMDS))))
        code, need_cell = TASK_CMDS[name]
        task_seq = (task_seq + 1) & 0xFF

        if name == 'nav_to':
            col, row = int(item['col']), int(item['row'])
            payload = ap.hb_task_nav_to_cell(task_seq, col, row,
                                             int(item.get('timeout_s', 60)))
            desc = 'TASK_CMD nav_to 格(%d,%d) 超时%ss' % (col, row,
                                                        item.get('timeout_s', 60))
        elif name == 'engage':
            col, row = int(item['col']), int(item['row'])
            face = int(item.get('face', 2))
            payload = ap.hb_task_engage(task_seq, col, row, face)
            desc = 'TASK_CMD engage 格(%d,%d) face=%d(%s)' % (
                col, row, face, {0: '未知', 1: '红/友', 2: '蓝/敌'}.get(face, '?'))
        elif name == 'bridge_cross':
            bid = int(item.get('bridge_id', 1))
            payload = ap.hb_task_bridge_cross(task_seq, bid)
            desc = 'TASK_CMD bridge_cross 区段%d' % bid
        else:
            payload = ap.hb_task_cmd(code, task_seq)
            desc = 'TASK_CMD %s' % name

        add(t, desc, payload, ap.T_TASK_CMD)

    # ---------------- 心跳（每轮结尾发一次，维持链路）----------------
    hb_t = mp.get('heartbeat_interval_s', 1.0)
    total_t = max([f[0] for f in frames] + [2.0]) + hb_t
    add(total_t, 'HEARTBEAT 心跳',
        ap.hb_heartbeat(1000), ap.T_HEARTBEAT)

    frames.sort(key=lambda x: x[0])
    return frames, cfg
# ==============================================================================
# 发送
# ==============================================================================

def hexdump(b: bytes, per_line: int = 16) -> str:
    out = []
    for i in range(0, len(b), per_line):
        chunk = b[i:i + per_line]
        hx = ' '.join('%02X' % c for c in chunk)
        out.append('      %04X  %-*s' % (i, per_line * 3, hx))
    return '\n'.join(out)


def main() -> int:
    p = argparse.ArgumentParser(
        description='无人机端离线模拟器（把地图 YAML 转成空地协议 v2 帧发给车端）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('常用选项')[1] if '常用选项' in __doc__ else '')
    p.add_argument('--map', required=True, help='地图 YAML 文件')
    p.add_argument('--host', default='127.0.0.1', help='车端 IP（默认本机）')
    p.add_argument('--port', type=int, default=9100, help='车端 UDP 端口（默认 9100）')
    p.add_argument('--dry-run', action='store_true', help='只打印不发送')
    p.add_argument('--loop', action='store_true', help='循环重发')
    p.add_argument('--speed', type=float, default=1.0, help='时间轴倍速（默认 1.0）')
    p.add_argument('--verbose', action='store_true', help='打印每帧十六进制')
    args = p.parse_args()

    try:
        mp = load_yaml(args.map)
    except YamlError as e:
        sys.stderr.write('★ 地图解析失败: %s\n' % e)
        return 2

    print('=' * 78)
    print(' 无人机端离线模拟器')
    print('=' * 78)
    print('  地图文件 : %s' % args.map)
    print('  场景名称 : %s' % mp.get('name', '(未命名)'))
    print('  目标地址 : %s:%d' % (args.host, args.port))
    print('  模式     : %s' % ('DRY-RUN（只打印）' if args.dry_run else 'UDP 实际发送'))
    print()

    try:
        frames, cfg = build_frames(mp, args.verbose)
    except YamlError as e:
        sys.stderr.write('★ %s\n' % e)
        return 2

    # ---- 打印清单 ----
    print('  将发送 %d 帧：' % len(frames))
    for (t, desc, fr) in frames:
        print('    t=%6.2fs  %-52s %4d B' % (t, desc, len(fr)))
    print()

    if args.dry_run:
        print('  ---- 逐帧内容（十六进制）----')
        for (t, desc, fr) in frames:
            print('  [t=%.2fs] %s' % (t, desc))
            if args.verbose:
                print(hexdump(fr))
            else:
                print('      %s%s' % (fr[:16].hex(' ').upper(),
                                      ' ...' if len(fr) > 16 else ''))
            # ★ 顺手用参考实现**回解**一次 —— 这是自洽性校验：
            #   decode() 会校验 magic / 长度 / CRC16 / 版本，能解出来就说明
            #   帧本身合法（与车端 C++ air::decodeFrame 同一套判据）。
            #   返回 (ver, typ, seq, payload) 四元组。
            try:
                ver, typ, seq, pl = ap.decode(fr)
                extra = ''
                if 0 < len(pl) <= 24:
                    extra = '  载荷: ' + pl.hex(' ').upper()
                print('      -> 解码 OK: ver=%d typ=0x%02X seq=%u payload=%dB%s'
                      % (ver, typ, seq, len(pl), extra))
            except Exception as e:                      # noqa: BLE001
                print('      -> ★ 解码失败: %s' % e)
        print()
        print('  ✓ DRY-RUN 完成（未发送任何数据）')
        return 0

    # ---- 实际发送 ----
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    try:
        while True:
            t_start = time.monotonic()
            for (t, desc, fr) in frames:
                target = t_start + t / max(args.speed, 1e-6)
                now = time.monotonic()
                if target > now:
                    time.sleep(target - now)
                sock.sendto(fr, (args.host, args.port))
                sent += 1
                print('  [%7.2fs] 已发送 %-52s %4d B'
                      % (time.monotonic() - t_start, desc, len(fr)))
            if not args.loop:
                break
            print('  --- 一轮完成，3s 后重发（Ctrl-C 退出）---')
            time.sleep(3.0)
    except KeyboardInterrupt:
        print()
    finally:
        sock.close()

    print()
    print('  ✓ 共发送 %d 帧' % sent)
    return 0


if __name__ == '__main__':
    sys.exit(main())
