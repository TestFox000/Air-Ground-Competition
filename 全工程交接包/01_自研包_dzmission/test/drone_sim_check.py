#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_sim_check.py —— 离线模拟器自检（本机可跑，不需要 ROS、不需要网络）

★ 为什么需要它：
  模拟器是**测试工具**，如果它自己发出的帧是错的，会让人误以为"车端有问题"，
  浪费大量时间。所以模拟器必须有自己的自检，并纳入常规自检体系。

检查内容：
  1. 地图 YAML 解析器能正确读出坐标与参数（含嵌套/行内列表/行内映射）
  2. 由地图生成的每一帧都能被参考实现 air_protocol.decode() 解回
     —— decode 会校验 magic / 长度 / CRC16-CCITT / 版本
  3. 解回的 typ 与预期一致；关键载荷字段逐字节正确
  4. 车端 CmdLink 是否真的支持 v2 帧头（源码级检查，防止回退）
  5. 车端 CmdLink 的 v2 分支是否用 air::decodeFrame（而不是自己算 CRC）
  6. 三张示例地图都能生成合理帧数

运行:  python3 test/drone_sim_check.py
"""

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # dzmission/
sys.path.insert(0, HERE)

import air_protocol as ap                          # noqa: E402

SIM = os.path.join(ROOT, 'tools', 'drone_sim.py')
MAPS = os.path.join(ROOT, 'tools', 'maps')
CMDLINK_CPP = os.path.join(ROOT, 'src', 'cmd_link.cpp')
CMDLINK_HPP = os.path.join(ROOT, 'include', 'dzmission', 'cmd_link.hpp')

g_pass = 0
g_fail = 0
problems = []


def check(cond, what):
    global g_pass, g_fail
    if cond:
        g_pass += 1
        print('  [ OK ] %s' % what)
    else:
        g_fail += 1
        problems.append(what)
        print('  [FAIL] %s' % what)


def sec(t):
    print()
    print('=' * 74)
    print(t)
    print('=' * 74)


def load_sim():
    """按模块方式加载 drone_sim.py（它不在包路径里）"""
    import importlib.util
    spec = importlib.util.spec_from_file_location('drone_sim', SIM)
    if spec is None or spec.loader is None:
        raise RuntimeError('无法加载 %s' % SIM)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    print('离线模拟器自检')
    print('★ 本机可跑；确保"模拟器发出的帧是合法的"，避免误导排查方向')

    # ---------------------------------------------------------------- 1
    sec('1. 文件存在性')
    check(os.path.isfile(SIM), 'tools/drone_sim.py 存在')
    check(os.path.isdir(MAPS), 'tools/maps/ 存在')
    check(os.path.isfile(os.path.join(MAPS, 'example_standard.yaml')),
          'example_standard.yaml 存在')
    check(os.path.isfile(os.path.join(MAPS, 'example_redblue.yaml')),
          'example_redblue.yaml 存在')
    check(os.path.isfile(os.path.join(MAPS, 'example_edge.yaml')),
          'example_edge.yaml 存在')

    sim = load_sim()

    # ---------------------------------------------------------------- 2
    sec('2. YAML 子集解析器')
    mp = sim.load_yaml(os.path.join(MAPS, 'example_standard.yaml'))
    check(mp.get('name', '').startswith('标准局'), 'name 读取正确')
    check(mp['field']['barrier_up_row'] == 5, 'field.barrier_up_row == 5')
    check(mp['field']['start'] == [12, 10], 'field.start == [12,10]（行内列表）')
    check(len(mp.get('mines') or []) == 8, 'mines 读出 8 条')
    check(mp['mines'][0] == [3, 2, 90], 'mines[0] == [3,2,90]')
    check(len(mp.get('gates') or []) == 2, 'gates 读出 2 条')
    check(len(mp.get('commands') or []) == 6, 'commands 读出 6 条（行内映射）')
    check(mp['commands'][0].get('cmd') == 'nav_to', 'commands[0].cmd == nav_to')
    check(mp['commands'][0].get('col') == 3, 'commands[0].col == 3')

    # ---------------------------------------------------------------- 3
    sec('3. 逐帧编码 + 回解（校验 magic/长度/CRC/版本）')
    frames, cfg = sim.build_frames(mp)
    check(len(frames) >= 12, 'standard 生成帧数 >= 12（实际 %d）' % len(frames))
    check(cfg['bridge_col'] == 10, 'cfg.bridge_col == 10')
    check(abs(cfg['cell_cm'] - 35) < 1e-9, 'cfg.cell_cm == 35')

    all_ok = True
    types_seen = []
    for (t, desc, fr) in frames:
        try:
            ver, typ, seq, pl = ap.decode(fr)
            types_seen.append(typ)
            if ver != ap.VER:
                all_ok = False
        except Exception as e:                       # noqa: BLE001
            all_ok = False
            print('      ★ 帧解码失败 [%s]: %s' % (desc, e))
    check(all_ok, '全部 %d 帧都能被参考实现解码（CRC 与长度自洽）' % len(frames))

    # ---------------------------------------------------------------- 4
    sec('4. 关键帧的 typ 与载荷逐字节核对')
    by_typ = {}
    for (t, desc, fr) in frames:
        _v, typ, _s, pl = ap.decode(fr)
        by_typ.setdefault(typ, []).append((desc, fr, pl))

    check(ap.T_FIELD_FIXED in by_typ, '含 FIELD_FIXED')
    if ap.T_FIELD_FIXED in by_typ:
        _d, _f, pl = by_typ[ap.T_FIELD_FIXED][0]
        # ★ v2.1：8 -> 12 字节，追加无人机起降点
        #   [0..7]   up col_lo col_hi bcol a_lo a_hi b_lo b_hi
        #   [8..9]   无人机起点 (col,row) = M9 = (12,8)
        #   [10..11] 无人机终点 (col,row) = N2 = (13,1)
        check(pl == bytes([5, 3, 13, 10, 0, 5, 6, 10, 12, 8, 13, 1]),
              'FIELD_FIXED 载荷 == 05 03 0D 0A 00 05 06 0A 0C 08 0D 01'
              '（含无人机起降点 M9/N2）')

    check(ap.T_TEAM_INFO in by_typ, '含 TEAM_INFO')
    if ap.T_TEAM_INFO in by_typ:
        _d, _f, pl = by_typ[ap.T_TEAM_INFO][0]
        check(pl == bytes([1, 14, 11, 35]), 'TEAM_INFO 载荷 == 01 0E 0B 23')

    check(ap.T_MINE_LIST in by_typ, '含 MINE_LIST')
    if ap.T_MINE_LIST in by_typ:
        _d, _f, pl = by_typ[ap.T_MINE_LIST][0]
        # ts(4) count(1) + 8*3
        check(len(pl) == 5 + 8 * 3, 'MINE_LIST 载荷长 = 5+8*3 = 29')
        check(pl[4] == 8, 'MINE_LIST count == 8')
        check(pl[5:8] == bytes([3, 2, 90]), '第 1 颗雷 == (3,2,90)')

    check(ap.T_TARGET_LIST in by_typ, '含 TARGET_LIST')
    if ap.T_TARGET_LIST in by_typ:
        _d, _f, pl = by_typ[ap.T_TARGET_LIST][0]
        check(len(pl) == 5 + 2 * 4, 'TARGET_LIST 载荷长 = 5+2*4 = 13')
        check(pl[4] == 2, 'TARGET_LIST count == 2')
        check(pl[5:9] == bytes([6, 4, 0, 0]), '第 1 格 == (6,4,face=0,conf=0)')

    check(ap.T_BRIDGE_GATE in by_typ, '含 BRIDGE_GATE')
    if ap.T_BRIDGE_GATE in by_typ:
        gates_out = [pl for (_d, _f, pl) in by_typ[ap.T_BRIDGE_GATE]]
        check(bytes([1, 3, 100, 1]) in gates_out, '区段1 门位 K4 == 01 03 64 01')
        check(bytes([2, 8, 100, 1]) in gates_out, '区段2 门位 K9 == 02 08 64 01')

    check(ap.T_TASK_CMD in by_typ, '含 TASK_CMD')
    if ap.T_TASK_CMD in by_typ:
        navs = [pl for (_d, _f, pl) in by_typ[ap.T_TASK_CMD]
                if len(pl) >= 2 and pl[0] == ap.CMD_NAV_TO_CELL]
        check(len(navs) >= 1, '有 nav_to 任务指令')
        if navs:
            check(navs[0][2:5] == bytes([3, 2, 60]),
                  'nav_to(3,2) 载荷 == cmd,seq,03,02,3C')
        engs = [pl for (_d, _f, pl) in by_typ[ap.T_TASK_CMD]
                if len(pl) >= 2 and pl[0] == ap.CMD_ENGAGE]
        check(len(engs) >= 1, '有 engage 任务指令')
        if engs:
            check(engs[0][4] == 2, 'engage 的 face == 2（蓝/敌）')

    # ---------------------------------------------------------------- 5
    sec('5. 三张示例地图都能生成帧')
    for name, min_frames in (('example_standard.yaml', 12),
                             ('example_redblue.yaml', 6),
                             ('example_edge.yaml', 10)):
        try:
            m = sim.load_yaml(os.path.join(MAPS, name))
            fs, _c = sim.build_frames(m)
            ok = len(fs) >= min_frames
            bad = 0
            for (_t, _d, fr) in fs:
                try:
                    ap.decode(fr)
                except Exception:                    # noqa: BLE001
                    bad += 1
            check(ok and bad == 0,
                  '%s -> %d 帧，全部可解码（>=%d）' % (name, len(fs), min_frames))
        except Exception as e:                       # noqa: BLE001
            check(False, '%s 处理失败: %s' % (name, e))

    # ---------------------------------------------------------------- 6
    sec('6. 车端 CmdLink 是否真的支持 v2 帧头（源码级防回退）')
    cpp = io.open(CMDLINK_CPP, encoding='utf-8', errors='ignore').read() \
        if os.path.isfile(CMDLINK_CPP) else ''
    hpp = io.open(CMDLINK_HPP, encoding='utf-8', errors='ignore').read() \
        if os.path.isfile(CMDLINK_HPP) else ''

    check('air::kMagic0' in cpp, 'cmd_link.cpp 引用了 air::kMagic0（A5）')
    check('air::kMagic1' in cpp, 'cmd_link.cpp 引用了 air::kMagic1（C3）')
    check('air::kHeaderLen' in cpp, 'cmd_link.cpp 用了 air::kHeaderLen（10 字节头）')
    check('air::kMaxFrameLen' in cpp, 'cmd_link.cpp 用了 air::kMaxFrameLen（1036 缓冲）')
    check('air::decodeFrame' in cpp,
          '★ cmd_link.cpp 用 air::decodeFrame 解码（不自己算 CRC，避免算法分歧）')
    check('dispatchAirFrame' in cpp, 'cmd_link.cpp 有 dispatchAirFrame 实现')
    check('dispatchAirFrame' in hpp, 'cmd_link.hpp 有 dispatchAirFrame 声明')
    check('kCellSizeM' in cpp, 'cmd_link.cpp 定义了 kCellSizeM（格->米换算）')

    # v2 任务码 -> MissionCommand 的映射是否齐全
    for tok, label in (('air::kCmdNavToCell', 'NAV_TO_CELL'),
                       ('air::kCmdEngage', 'ENGAGE'),
                       ('air::kCmdHold', 'HOLD'),
                       ('air::kCmdStop', 'STOP'),
                       ('air::kCmdReturnBase', 'RETURN_BASE')):
        check(tok in cpp, 'v2 任务码 %s 有映射' % label)

    # 四类侦察消息都要有 case
    for tok, label in (('air::kMineList', 'MINE_LIST'),
                       ('air::kObstacleList', 'OBSTACLE_LIST'),
                       ('air::kTargetList', 'TARGET_LIST'),
                       ('air::kBridgeGate', 'BRIDGE_GATE'),
                       # ★ FIELD_FIXED 承载无人机起降点，**不能**被当成"仅链路保活"忽略
                       ('air::kFieldFixed', 'FIELD_FIXED（无人机起降点）')):
        check(tok in cpp, '★ %s 有解码分支' % label)

    # ★ FIELD_FIXED 必须转成 kCmdFieldFixed 下发，否则起降点禁入无效
    check('kCmdFieldFixed' in cpp,
          '★ cmd_link 把 FIELD_FIXED 映射为 kCmdFieldFixed（下发到任务层）')
    mm = io.open(os.path.join(ROOT, 'src', 'mission_manager.cpp'),
                 encoding='utf-8', errors='ignore').read() \
        if os.path.isfile(os.path.join(ROOT, 'src', 'mission_manager.cpp')) else ''
    check('handleFieldFixed' in mm,
          '★ mission_manager 有 handleFieldFixed（应用无人机起降点）')
    check('setDroneCells' in mm,
          '★ mission_manager 调用 FieldGrid::setDroneCells')
    fg = io.open(os.path.join(ROOT, 'src', 'field_grid.cpp'),
                 encoding='utf-8', errors='ignore').read() \
        if os.path.isfile(os.path.join(ROOT, 'src', 'field_grid.cpp')) else ''
    check('applyDroneCellsAsBlocked' in fg,
          '★ FieldGrid 把无人机起降点落成禁入格（kBlocked）')

    # v1 兼容是否保留
    check('kV1Head0' in cpp and '0xAA' in cpp, 'v1 串口帧头仍保留（向后兼容）')

    # ---------------------------------------------------------------- 7
    sec('7. air 枚举覆盖完整性（防止新增类型时漏掉 case）')

    # ★ 为什么查这个：switch 里漏掉某个枚举值时，它会落到 default: 分支，
    #   表现为"每收到一帧就刷一条未知类型警告"——不致命但会淹没真正的告警。
    #   本轮就漏过 kVehStatus(0x30)，已补。
    air_hpp = os.path.join(ROOT, 'include', 'dzmission', 'air_protocol.hpp')
    hdr = io.open(air_hpp, encoding='utf-8', errors='ignore').read() \
        if os.path.isfile(air_hpp) else ''

    def enum_members(name):
        m = re.search(r'enum\s+' + name + r'\s*:\s*uint8_t\s*\{(.*?)\};', hdr, re.S)
        return set(re.findall(r'(k\w+)\s*=', m.group(1))) if m else set()

    typ_names = enum_members('Typ')
    task_names = enum_members('TaskCmd')
    check(len(typ_names) >= 10, 'air::Typ 抽出 %d 个成员' % len(typ_names))
    check(len(task_names) >= 8, 'air::TaskCmd 抽出 %d 个成员' % len(task_names))

    i = cpp.find('void CmdLink::dispatchAirFrame')
    j = cpp.find('void CmdLink::publishCommand', i)
    body = cpp[i:j] if (i >= 0 and j > i) else ''
    cases = set(re.findall(r'case air::(k\w+):', body))

    miss_typ = sorted(typ_names - cases)
    miss_task = sorted(task_names - cases)
    check(not miss_typ,
          '★ air::Typ 全部有 case（未覆盖: %s）'
          % (', '.join(miss_typ) if miss_typ else '无'))
    check(not miss_task,
          '★ air::TaskCmd 全部有 case（未覆盖: %s）'
          % (', '.join(miss_task) if miss_task else '无'))

    undef = sorted(n for n in cases if n not in typ_names and n not in task_names)
    check(not undef,
          '所有 case 都有定义（无定义: %s）' % (', '.join(undef) if undef else '无'))

    # ---------------------------------------------------------------- 汇总
    sec('汇总')
    print('  通过 %d，失败 %d' % (g_pass, g_fail))
    if g_fail:
        print()
        print('  失败项：')
        for p in problems:
            print('    - %s' % p)
    else:
        print('  ✓ 模拟器与车端接收侧一致')
    return 0 if g_fail == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
