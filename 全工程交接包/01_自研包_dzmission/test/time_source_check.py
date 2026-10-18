#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
time_source_check.py —— rclcpp::Time 时间源一致性检查（本机可跑，不需要 ROS）

★ 为什么需要它：
  真实崩溃（离线测试首次运行）：
      terminate called after throwing an instance of 'std::runtime_error'
        what():  can't subtract times with different time sources [1 != 2]
  根因：`rclcpp::Time` 的**默认构造**用 RCL_SYSTEM_TIME(=1)，
        而 `node->now()` 返回本节点时钟 RCL_ROS_TIME(=2)，
        两者相减**抛异常**（1 != 2 正是这两个枚举值）。

  触发条件：某个 Time 成员**从未被赋值**，就被拿去和 now() 相减。
  正常比赛里数据来得快，这些成员很快被赋值，所以不易暴露；
  而**离线测试/首次启动**会必然命中 —— 节点直接 terminate(-6)。

  这类缺陷完全可以静态发现。

检查：
  1. 收集所有 `rclcpp::Time 名字_;` 成员（默认构造 = 时间源 1）
  2. 收集构造函数里被赋值的成员（时间源被对齐）
  3. 找出"用在减法里、但从未被赋值"的成员 -> ★ 高风险
  4. 找出"从未在任何地方出现第二次"的成员（纯死成员）

运行:  python3 test/time_source_check.py
"""

import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

g_fail = 0


def sec(t):
    print()
    print('=' * 74)
    print(t)
    print('=' * 74)


def read(rel):
    p = os.path.join(ROOT, rel.replace('/', os.sep))
    return io.open(p, encoding='utf-8', errors='ignore').read() if os.path.isfile(p) else ''


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    t = re.sub(r'//[^\n]*', ' ', t)
    return t


def collect_sources():
    """返回 [(type_name, [header...], [cpp...])]"""
    inc = os.path.join(ROOT, 'include', 'dzmission')
    src = os.path.join(ROOT, 'src')
    hdrs = sorted(f for f in os.listdir(inc) if f.endswith('.hpp')) if os.path.isdir(inc) else []
    cpps = sorted(f for f in os.listdir(src) if f.endswith('.cpp')) if os.path.isdir(src) else []

    out = []
    for h in hdrs:
        base = h[:-4]                       # xxx.hpp -> xxx
        rel_cpp = 'src/%s.cpp' % base
        if rel_cpp.replace('/', os.sep) and os.path.isfile(
                os.path.join(ROOT, rel_cpp.replace('/', os.sep))):
            out.append((base, [h], [rel_cpp]))
        else:
            out.append((base, [h], []))
    return out


def main():
    global g_fail

    print('rclcpp::Time 时间源一致性检查')
    print('★ 查"Time 成员从未赋值就被拿来减 now()"——这会抛异常让节点直接退出')

    sec('1. 逐个组件扫描')

    total_risk = 0
    total_members = 0

    for (name, hdrs, cpps) in collect_sources():
        htext = strip_comments('\n'.join(read('include/dzmission/' + h) for h in hdrs))
        ctext = strip_comments('\n'.join(read(c) for c in cpps)) if cpps else ''

        # 1) Time 成员声明
        members = re.findall(r'rclcpp::Time\s+([A-Za-z_]\w*)\s*;', htext)
        if not members:
            continue
        members = sorted(set(members))
        total_members += len(members)

        # 2) 在 .cpp 里被赋值的成员
        #    ★ 成员可能通过 `m` / `obj.m` / `ptr->m` 三种形式访问，
        #      第一版只认裸名 `m`，于是把 motion_arbiter 的 `sl.stamp = ...`
        #      误判成"从未赋值"，产生假阳性。这里覆盖三种形式。
        assigned = set()
        for m in members:
            pat = (r'(?<![\w.>])(?:\w+(?:\.|->))?' + re.escape(m) +
                   r'\s*=(?!=)')
            if re.search(pat, ctext):
                assigned.add(m)

        # 3) 用在减法里的成员： (a - b) 或 a - b 两种形态
        used_in_sub = set()
        for m in members:
            ref = r'(?:\w+(?:\.|->))?' + re.escape(m)
            pat = (r'(?<![\w.>])' + ref + r'\s*\)?\s*-\s*|'
                   r'-\s*' + ref + r'\b')
            if re.search(pat, ctext):
                used_in_sub.add(m)

        # 4) 从未出现第二次的（纯死成员）—— 同样要覆盖 obj.m / ptr->m
        dead = []
        for m in members:
            ref = r'(?:\w+(?:\.|->))?' + re.escape(m)
            if len(re.findall(r'(?<![\w.>])' + ref + r'\b', ctext)) == 0:
                dead.append(m)

        risk = [m for m in members if (m in used_in_sub) and (m not in assigned)]

        print()
        print('  %-28s Time 成员 %2d 个（已赋值 %2d）'
              % (name, len(members), len(assigned)))
        if risk:
            total_risk += len(risk)
            for m in risk:
                print('      [XX] %-26s ★ 用于减法但从未被赋值 -> 会抛异常' % m)
        if dead:
            for m in dead:
                print('      [--] %-26s 在 .cpp 里从未出现（疑似死成员）' % m)
        if not risk and not dead:
            print('      [OK] 所有用于减法的成员都已被赋值')

    sec('2. 逐个核对 MissionManager 的关键成员')
    ctext = strip_comments(read('src/mission_manager.cpp'))
    htext = strip_comments(read('include/dzmission/mission_manager.hpp'))
    key = ['pose_stamp_', 'target_stamp_', 'dztrack_stamp_', 'nav2_stamp_',
           'teleop_stamp_', 'goal_sent_time_', 'goal_last_retry_time_',
           'last_status_time_', 'last_fault_tx_time_', 'link_lost_since_',
           'estop_since_', 'bridge_last_request_time_']
    # 找构造函数体
    mi = ctext.find('MissionManager::MissionManager')
    ctor = ''
    if mi >= 0:
        depth = 0
        started = False
        for k in range(mi, len(ctext)):
            if ctext[k] == '{':
                depth += 1
                started = True
            elif ctext[k] == '}':
                depth -= 1
                if started and depth == 0:
                    ctor = ctext[mi:k]
                    break
    for m in key:
        in_hdr = re.search(r'rclcpp::Time\s+' + re.escape(m) + r'\s*;', htext) is not None
        in_ctor = re.search(r'(?<![\w.>])' + re.escape(m) + r'\s*=(?!=)', ctor) is not None
        in_sub = re.search(r'(?<![\w.>])' + re.escape(m) + r'\s*\)?\s*-\s*|-\s*' + re.escape(m) + r'\b', ctext) is not None
        if not in_hdr:
            continue
        ok = in_ctor or (not in_sub)
        if not ok:
            g_fail += 1
        print('    %s %-28s 声明=%s 构造赋值=%s 用于减法=%s'
              % ('[OK]' if ok else '[XX]', m,
                 'Y' if in_hdr else 'N', 'Y' if in_ctor else 'N',
                 'Y' if in_sub else 'N'))

    sec('汇总')
    print('  共检查 %d 个 Time 成员，高风险 %d 个' % (total_members, total_risk))
    if g_fail == 0 and total_risk == 0:
        print('  ✓ 通过：所有用于减法的 Time 成员都已赋值（时间源一致）')
    else:
        print('  ✗ 有风险项（见上）')
    return 0 if (g_fail == 0 and total_risk == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
