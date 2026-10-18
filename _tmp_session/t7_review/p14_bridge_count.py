# -*- coding: utf-8 -*-
r"""t7 复核：P1-4「route_check 桥洞直穿少计 → 门禁静默放行」可复现实验。

只读：import dzrobot_v2/tools/route_check.py，直接调 _bridge_group_check + summarise，
不改任何仓库文件。

实验口径：
  - 地图真源 map.bridges = ['K5','K9']（第五张图两个通口）
  - _bridge_group_check 内 rows=[5,9]，只有落在 [5,9] 行区间内的水平直穿才计入 n_cross（:413）
  - 每个"物理穿越"若航点正好落在 K 列线上（车端 route 就是这样：J9 K9 L9），
    会被算成 2 个航段 → n_cross 天然翻倍
  - 判级：n_cross>=2 → ✅ok；==1 → ⚠️warn（:434）；0 → ❌bad（:436）
  - summarise 只在 FAIL 非空时返回 1（:1191-1201）→ 任何数量的 warn 都不改退出码
"""
import io
import os
import sys
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
TOOLS = os.path.join(WS, 'dzrobot_v2', 'tools')
sys.path.insert(0, TOOLS)

import route_check as rc  # noqa: E402


def p(cell):
    return rc.block_to_meters(cell)


def run(name, bridges, pts):
    rc.FAIL.clear(); rc.WARN.clear(); rc.OK.clear(); rc.INFO.clear()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc._bridge_group_check(bridges, pts)
        rc_rc = rc.summarise()
    out = buf.getvalue()
    print('=' * 78)
    print('用例: %s' % name)
    print('  bridges=%r  pts=%s' % (bridges, [k for k in []]) if False else
          '  bridges=%r  航段数=%d' % (bridges, len(pts) - 1))
    for line in out.splitlines():
        if any(k in line for k in ('桥洞', '通过', '失败', '提醒')):
            print('    ' + line.strip())
    print('  → FAIL=%d WARN=%d OK=%d  summarise() rc=%d'
          % (len(rc.FAIL), len(rc.WARN), len(rc.OK), rc_rc))
    return dict(name=name, fail=len(rc.FAIL), warn=len(rc.WARN),
                ok=len(rc.OK), rc=rc_rc)


cases = []

# A. 两洞行号正确但方向与真车相反（下半场 9 行向东、上半场 5 行向西）⇒ 硬化后应判红
#    ⚠️ 历史标签「正确两洞」过期：真车是 下半场向西(L9→K9→J9) / 上半场向东(J5→K5→L5)，
#    本用例方向恰好相反，判红是正确行为；引用合法基线请用 p14_composite 的 F/G 案例。
cases.append(run('A 两洞行号对但方向反（下半场向东）——硬化后应判红',
                 ['K5', 'K9'],
                 [p('J9'), p('K9'), p('L9'), p('L5'), p('K5'), p('J5')]))

# B. 第二个通口被"纠正"到行 10（区间 [5,9] 之外）：只剩行 9 的 2 段被计入
cases.append(run('B 一洞在 K5、另一洞错到 K10（区间外）',
                 ['K5', 'K9'],
                 [p('J9'), p('K9'), p('L9'), p('L10'), p('J10')]))

# C. 只穿一个区间内通口 + 一个区间外通口，且列线上无航点 → n_cross 恰好 1
cases.append(run('C 只 1 段计入（命中 :434 warn 分支）',
                 ['K5', 'K9'],
                 [p('J9'), p('L9'), p('L10'), p('J10')]))

# D. 两个通口都错到区间外（K10/K4）→ n_cross=0 → bad
cases.append(run('D 两洞全在区间外(K10,K4)',
                 ['K5', 'K9'],
                 [p('J10'), p('K10'), p('L10'), p('L4'), p('K4'), p('J4')]))

# E. 反向风险：区间内任意行都算"合法穿越"（K7 实际是墙，不是通口）
cases.append(run('E 穿 K7（墙，非通口）却被计为合法穿越',
                 ['K5', 'K9'],
                 [p('J7'), p('K7'), p('L7')]))

print('\n' + '=' * 78)
print('汇总（rc=0 即"门禁放行"）：')
for c in cases:
    print('  %-46s FAIL=%d WARN=%d rc=%d' % (c['name'], c['fail'], c['warn'], c['rc']))
print('\n结论：只有 FAIL 才让 rc=1；n_cross 少计/忽略导致的 ✅/⚠️ 一律 rc=0。')
