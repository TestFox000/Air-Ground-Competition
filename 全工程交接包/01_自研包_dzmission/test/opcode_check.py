#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
opcode_check.py —— 操作码/类型码**冲突与一致性**检查（本机可跑）

★ 为什么需要它：
  真实编译错误：
      mission_manager.cpp:599: error: duplicate case value
      case kCmdFieldFixed: handleFieldFixed(m); break;
      note: previously used here
      case kCmdCeaseFire: handleCeaseFire(m); break;
  根因：`kCmdFieldFixed` 我取了 **0x11** —— 那是 **air 协议的类型码**
        （kFieldFixed=0x11），但同名空间里 0x11 已归 kCmdCeaseFire。
        C++ 的 switch 不允许重复 case 值，于是编译失败。

  ★ 这个错误**完全可以在编译前静态发现**：把所有常量抽出来查重即可。

检查内容
--------
  1. **L1 内部重名冲突**：同一头文件里两个不同常量取了相同值 -> 必然编译失败
  2. **L2 跨文件值一致性**：C++ 与 Python 参考实现里"同名常量"必须同值，
     否则两端协议理解不一致（比编译错误更危险，会静默错语义）
  3. **L3 命令域重叠提示**：MissionCommand 的"侦察类"常量有意镜像 air 类型码，
     这里把它们与 air::Typ 并排列出，方便人工确认是有意而非笔误

运行:  python3 test/opcode_check.py
"""

from __future__ import annotations

import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

g_fail = 0


def sec(t):
    print()
    print('=' * 76)
    print(t)
    print('=' * 76)


def read(rel):
    p = os.path.join(ROOT, rel.replace('/', os.sep))
    return io.open(p, encoding='utf-8', errors='ignore').read() if os.path.isfile(p) else ''


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    t = re.sub(r'//[^\n]*', ' ', t)
    return t


def find_consts(code, pattern):
    """返回 [(名字, 值)]"""
    out = []
    for m in re.finditer(pattern, code):
        name = m.group(1)
        raw = m.group(2)
        try:
            # 支持 0x.. / 十进制 / 简单表达式（只取字面量部分）
            val = int(raw, 0) if not raw.startswith('(') else None
        except ValueError:
            val = None
        out.append((name, val, raw))
    return out


def check_internal(name, code, pattern, label):
    """L1：同文件内是否有两个不同常量取相同值"""
    consts = find_consts(code, pattern)
    by_val = {}
    for (n, v, raw) in consts:
        if v is None:
            continue
        by_val.setdefault(v, []).append(n)
    dup = {v: ns for v, ns in by_val.items() if len(ns) > 1}
    if dup:
        global g_fail
        g_fail += 1
        for v, ns in sorted(dup.items()):
            print('  [XX] %s 值冲突: 0x%02X 被这些常量共用: %s'
                  % (label, v, ', '.join(ns)))
            print('       ★ C++ switch 里会报 "duplicate case value"；必须改一个')
    else:
        print('  [OK] %s 共 %d 个常量，无重名冲突' % (label, len(consts)))
    return consts


def main():
    global g_fail

    print('操作码 / 类型码 冲突与一致性检查')
    print('★ 查"两个常量取同值"（switch 重复 case）与"C++/Python 同值不同"')

    mc = strip_comments(read('include/dzmission/mission_command.hpp'))
    ap = strip_comments(read('include/dzmission/air_protocol.hpp'))
    mcpy = read('test/air_protocol.py')

    # ------------------------------------------------------------------ L1
    sec('L1. 各文件内部的重名冲突（必然编译失败）')

    # mission_command.hpp：全部 kCmd* 常量（含带行尾注释的写法）
    mc_consts = check_internal(
        'mission_command.hpp', mc,
        r'\bconstexpr\s+uint8_t\s+(kCmd\w+)\s*=\s*'
        r'(0x[0-9A-Fa-f]+|\d+)',
        'mission_command.hpp 的 kCmd*')

    # air_protocol.hpp：kOcc* / kFault*
    check_internal(
        'air_protocol.hpp(kOcc)', ap,
        r'\b(kOcc\w+)\s*=\s*(\d+)', 'air_protocol.hpp 的 kOcc*')

    # air_protocol.hpp 的 enum Typ / TaskCmd / EventCode
    for ename in ('Typ', 'TaskCmd', 'EventCode'):
        m = re.search(r'enum\s+' + ename + r'\s*:\s*uint8_t\s*\{(.*?)\};', ap, re.S)
        if not m:
            continue
        body = m.group(1)
        check_internal(
            'air_protocol.hpp(%s)' % ename, body,
            r'\b(k\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)',
            'air::%s' % ename)

    # ------------------------------------------------------------------ L2
    sec('L2. C++ 与 Python 参考实现的同名常量一致性')
    print('  （两端不一致 = 协议理解分歧，会静默错语义，比编译错误更危险）')

    def val_of_cpp(name, code):
        m = re.search(r'\b' + re.escape(name) + r'\s*=\s*(0x[0-9A-Fa-f]+|\d+)', code)
        if not m:
            return None
        try:
            return int(m.group(1), 0)
        except ValueError:
            return None

    def val_of_py(name, code):
        m = re.search(r'^' + re.escape(name) + r'\s*=\s*(0x[0-9A-Fa-f]+|\d+)',
                      code, re.M)
        if not m:
            return None
        try:
            return int(m.group(1), 0)
        except ValueError:
            return None

    # 成对比较：C++ 名 / Python 名
    PAIRS = [
        ('air_protocol.hpp 的 kMagic0', 'kMagic0', 'MAGIC0'),
        ('kMagic1', 'kMagic1', 'MAGIC1'),
        ('kVer', 'kVer', 'VER'),
        ('kHeaderLen', 'kHeaderLen', 'HEADER_LEN_CPP'),   # Python 的含 CRC，另行说明
        ('kHeartbeat', 'kHeartbeat', 'T_HEARTBEAT'),
        ('kTeamInfo', 'kTeamInfo', 'T_TEAM_INFO'),
        ('kFieldFixed', 'kFieldFixed', 'T_FIELD_FIXED'),
        ('kMapFull', 'kMapFull', 'T_MAP_FULL'),
        ('kMineList', 'kMineList', 'T_MINE_LIST'),
        ('kBridgeGate', 'kBridgeGate', 'T_BRIDGE_GATE'),
        ('kTargetList', 'kTargetList', 'T_TARGET_LIST'),
        ('kObstacleList', 'kObstacleList', 'T_OBSTACLE_LIST'),
        ('kTaskCmd', 'kTaskCmd', 'T_TASK_CMD'),
        ('kEvent', 'kEvent', 'T_EVENT'),
        ('kAck', 'kAck', 'T_ACK'),
        ('kCmdNop', 'kCmdNop', 'CMD_NOP'),
        ('kCmdNavToCell', 'kCmdNavToCell', 'CMD_NAV_TO_CELL'),
        ('kCmdEngage', 'kCmdEngage', 'CMD_ENGAGE'),
        ('kCmdHold', 'kCmdHold', 'CMD_HOLD'),
        ('kCmdStop', 'kCmdStop', 'CMD_STOP'),
        ('kCmdReturnBase', 'kCmdReturnBase', 'CMD_RETURN_BASE'),
        ('kOccUnknown', 'kOccUnknown', 'OCC_UNKNOWN'),
        ('kOccFree', 'kOccFree', 'OCC_FREE'),
        ('kOccBlocked', 'kOccBlocked', 'OCC_BLOCKED'),
        ('kOccMine', 'kOccMine', 'OCC_MINE'),
        ('kOccTarget', 'kOccTarget', 'OCC_TARGET'),
        ('kOccGateUnk', 'kOccGateUnk', 'OCC_GATE_UNK'),
        ('kOccGateOpen', 'kOccGateOpen', 'OCC_GATE_OPEN'),
    ]
    mism = 0
    same = 0
    for (label, cname, pyname) in PAIRS:
        if label == 'kHeaderLen':
            continue                       # 语义不同（含/不含 CRC），跳过
        cv = val_of_cpp(cname, ap)
        pv = val_of_py(pyname, mcpy)
        if cv is None or pv is None:
            continue
        if cv == pv:
            same += 1
        else:
            mism += 1
            print('  [XX] %-20s C++(%s)=0x%02X  !=  Python(%s)=0x%02X'
                  % (label, cname, cv, pyname, pv))
    if mism == 0:
        print('  [OK] %d 对同名常量两端一致' % same)
    else:
        g_fail += 1

    # ------------------------------------------------------------------ L3
    sec('L3. MissionCommand 的侦察类常量 vs air 类型码（确认是有意镜像）')
    typ = {}
    m = re.search(r'enum\s+Typ\s*:\s*uint8_t\s*\{(.*?)\};', ap, re.S)
    if m:
        for mm in re.finditer(r'\b(k\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', m.group(1)):
            typ[int(mm.group(2), 0)] = mm.group(1)
    print('  %-24s %-6s %s' % ('MissionCommand 常量', '值', 'air::Typ 同值者'))
    for (n, v, raw) in sorted(mc_consts, key=lambda t: (t[1] if t[1] is not None else 999)):
        if v is None:
            continue
        same_air = typ.get(v, '')
        tag = '  <== 有意镜像' if same_air else ''
        print('  %-24s 0x%02X   %-18s%s' % (n, v, same_air, tag))
    print()
    print('  ★ 注意：**真正有意镜像** air 类型码的只有侦察类那 5 个：')
    print('         kCmdMapFull / kCmdMineList / kCmdBridgeGate /'
          ' kCmdTargetList / kCmdObstacleList')
    print('     （CmdLink::dispatchAirFrame 把 air 类型码直接映射成这些内部码）')
    print('     其余"同值"多是**巧合**（值域都很小，容易撞），不代表有对应关系。')
    print('     真正必须保证的是：**同一命名空间内不重复** —— 由 L1 强制。')

    # ------------------------------------------------------------------ L4
    sec('L4. Python 参考实现内部冲突（命令码 / 类型码）')

    py_cmds = []
    for m in re.finditer(r'^CMD_(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', mcpy, re.M):
        try:
            py_cmds.append(('CMD_' + m.group(1), int(m.group(2), 0)))
        except ValueError:
            pass
    byv = {}
    for (n, v) in py_cmds:
        byv.setdefault(v, []).append(n)
    dup_py = {v: ns for v, ns in byv.items() if len(ns) > 1}
    if dup_py:
        g_fail += 1
        for v, ns in sorted(dup_py.items()):
            print('  [XX] Python CMD_* 值冲突: 0x%02X -> %s' % (v, ', '.join(ns)))
    else:
        print('  [OK] Python CMD_* 共 %d 个，无冲突' % len(py_cmds))

    py_t = []
    for m in re.finditer(r'^T_(\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)', mcpy, re.M):
        try:
            py_t.append(('T_' + m.group(1), int(m.group(2), 0)))
        except ValueError:
            pass
    byv2 = {}
    for (n, v) in py_t:
        byv2.setdefault(v, []).append(n)
    dup_t = {v: ns for v, ns in byv2.items() if len(ns) > 1}
    if dup_t:
        g_fail += 1
        for v, ns in sorted(dup_t.items()):
            print('  [XX] Python T_* 值冲突: 0x%02X -> %s' % (v, ', '.join(ns)))
    else:
        print('  [OK] Python T_* 共 %d 个，无冲突' % len(py_t))

    # ------------------------------------------------------------------ 汇总
    sec('汇总')
    if g_fail == 0:
        print('  ✓ 通过：无重名冲突，C++/Python 同名常量一致')
    else:
        print('  ✗ 有 %d 类问题（见上）' % g_fail)
    return 0 if g_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
