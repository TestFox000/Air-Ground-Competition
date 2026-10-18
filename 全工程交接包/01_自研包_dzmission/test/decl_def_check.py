#!/usr/bin/env python3
"""最可靠的核对：.cpp 里每个 `Class::name(` 必须在头文件的类体内被声明。

不做任何语法解析，只做两件事：
  1. 从 .cpp 抓 `Class::NAME(` 的 NAME 全集（这个正则是可靠的）
  2. 对每个 NAME，在头文件文本里查找 `\\bNAME\\s*\\(` （同样可靠）
若找不到，说明头文件漏了声明 —— 这才是真正的编译错误。
反向（声明未实现）不在此脚本判断，因为那样必须解析类体，容易误报。
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

UNITS = [
    ('include/dzmission/localization_compensator.hpp',
     ['src/localization_compensator.cpp'], 'LocalizationCompensator'),
    ('include/dzmission/mission_manager.hpp', ['src/mission_manager.cpp'], 'MissionManager'),
    ('include/dzmission/cmd_link.hpp', ['src/cmd_link.cpp'], 'CmdLink'),
    ('include/dzmission/motion_arbiter.hpp', ['src/motion_arbiter.cpp'], 'MotionArbiter'),
    ('include/dzmission/safety_monitor.hpp', ['src/safety_monitor.cpp'], 'SafetyMonitor'),
    ('include/dzmission/target_aimer.hpp', ['src/target_aimer.cpp'], 'TargetAimer'),
    ('include/dzmission/referee_board.hpp', ['src/referee_board.cpp'], 'RefereeBoard'),
    ('include/dzmission/eskf.hpp', ['src/eskf.cpp'], 'Eskf'),
    ('include/dzmission/field_grid.hpp', ['src/field_grid.cpp'], 'FieldGrid'),
    ('include/dzmission/mine_planner.hpp', ['src/mine_planner.cpp'], 'MinePlanner'),
    ('include/dzmission/pivot_controller.hpp', ['src/pivot_controller.cpp'],
     'PivotController'),
    # 协议层是命名空间函数（dzmission::air 与 dzmission::air::enc）。
    # 用命名空间名当"类名"复用同一套"定义必须有声明"的检查。
    ('include/dzmission/air_protocol.hpp', ['src/air_protocol.cpp'], 'air'),
    ('include/dzmission/air_protocol.hpp', ['src/air_protocol.cpp'], 'enc'),
]


def read(rel):
    with open(os.path.join(ROOT, rel), encoding='utf-8', errors='replace') as f:
        return f.read()


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    return re.sub(r'//[^\n]*', '', t)


def main():
    total = 0
    for header, cpps, cls in UNITS:
        htext = strip_comments(read(header))
        ctext = strip_comments(''.join(read(c) for c in cpps))

        defined = set(re.findall(r'\b' + cls + r'::(~?\w+)\s*\(', ctext))
        missing = []
        for name in sorted(defined):
            plain = name.lstrip('~')
            if not re.search(r'\b' + re.escape(name) + r'\s*\(', htext) and \
               not re.search(r'\b' + re.escape(plain) + r'\s*\(', htext):
                missing.append(name)

        # --- 反向：头文件里声明为「有返回值/有参数」的成员函数，
        #     是否在 .cpp 里真的有实现？（防止声明了但忘写，会链接失败）
        declared = set(re.findall(r'\b(\w+)\s*\([^;{}]*\)\s*(?:const\s*)?'
                                  r'(?:override\s*)?;', htext))
        skip = {'if', 'for', 'while', 'switch', 'return', 'catch',
                'static_assert', 'defined', 'sizeof'}
        unimplemented = []
        for name in sorted(declared - defined - skip):
            # 头文件内联实现（带函数体）
            if re.search(r'\b' + re.escape(name) + r'\s*\([^;{}]*\)[^;{}]*\{', htext):
                continue
            # 默认实参 / 纯虚 / = default
            if re.search(r'\b' + re.escape(name) + r'\s*\([^;{}]*\)[^;{}]*=\s*'
                         r'(?:default|delete|0)', htext):
                continue
            # 构造/析构在头文件里常见
            if name in (cls, '~' + cls):
                continue
            unimplemented.append(name)

        status = 'OK (%d 个成员函数均有声明)' % len(defined) if not (missing or unimplemented) \
            else '%d 项' % (len(missing) + len(unimplemented))
        print('%-48s %s' % (os.path.basename(header), status))
        for m in missing:
            print('    - [BLOCKER] %s::%s 在 .cpp 中定义，但头文件未声明' % (cls, m))
        for u in unimplemented:
            print('    - [WARN] %s::%s 头文件已声明，.cpp 未见实现（若非内联将链接失败）'
                  % (cls, u))
        total += len(missing)

    print()
    if total == 0:
        print('结论：无「定义未声明」错误。')
    else:
        print('结论：%d 个成员函数缺少头文件声明（会编译失败）。' % total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
