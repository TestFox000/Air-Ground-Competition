# -*- coding: utf-8 -*-
r"""t7 复核：P1-4 复合门禁对照实验。

问题：`tools/route_check.py::_bridge_group_check` 只把落进 `map.bridges` 行区间内的
水平直穿计入 n_cross，区间外的穿越**静默消失**；且 n_cross==1 只 warn（不改退出码）。
那么"route_check 绿"能不能当门禁？

做法：同一条路线串，分别喂给
  ① tools/route_check.py（用户侧判据，本次 `--route` 直给）
  ② dzbrain.route.validate（config_selfcheck 用的权威判据，R007/R008/R009）
看两者是否一致。只读，不改任何仓库文件。
"""
import io
import os
import sys
import contextlib
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, '..', '..'))
ROOT = os.path.join(WS, 'dzrobot_v2')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'dzbrain'))

import route_check as rc  # noqa: E402
from dzbrain.core.config import Config  # noqa: E402
from dzbrain.route.parser import parse_mission  # noqa: E402
from dzbrain.route.validate import validate_route  # noqa: E402

CFG = os.path.join(ROOT, 'config', 'robot.yaml')
MAP = os.path.join(ROOT, 'config', 'map.yaml')
cfg = Config.load(CFG)
map_cfg = yaml.safe_load(open(MAP, encoding='utf-8')) or {}

BRIDGES = ['K5', 'K9']
CASES = {
    # 下半场方向 = 向西 L*->K*->J*；上半场 = 向东 J*->K*->L*
    'B 下半场穿了 2 次(L9-K9 / L10-K10，第 2 次在区间 [5,9] 外)，上半场 0 次':
        'L9 K9 J9 L10 K10 J10',
    'E 只在下半场 K7 穿一次（缺上半场）': 'J7 K7 L7',
    'F 正确两洞 K9(下，向西) + K5(上，向东)': 'L9 K9 J9 J5 K5 L5',
    'G 两半场各 1 次，但下半场走 K10（不在 map.bridges 里，仍在 7~11 行段内）':
        'L10 K10 J10 J5 K5 L5',
}

for name, route in CASES.items():
    print('=' * 78)
    print(name)
    print('  路线: %s' % route)
    # ① route_check
    rc.FAIL.clear(); rc.WARN.clear(); rc.OK.clear(); rc.INFO.clear()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc._bridge_group_check(BRIDGES, rc.parse_route(route.split())[0])
        rrc = rc.summarise()
    bridge_lines = [l.strip() for l in buf.getvalue().splitlines()
                    if ('桥洞' in l and ('直穿' in l or '未' in l))]
    print('  ① route_check 桥洞项: %s' % (' || '.join(bridge_lines) or '(无输出)'))
    print('     桥洞层 rc=%d  FAIL=%d WARN=%d' % (rrc, len(rc.FAIL), len(rc.WARN)))

    # ② dzbrain validate
    r = parse_mission(route.split())
    issues = validate_route(r, map_cfg=map_cfg, cfg=cfg)
    cross = [i for i in issues if i.code in ('R007', 'R008', 'R009')]
    errs = [i for i in cross if i.level == 'error']
    print('  ② dzbrain R007/R008/R009: %d 条（error %d）' % (len(cross), len(errs)))
    for i in cross[:4]:
        msg = getattr(i, 'message', None) or getattr(i, 'msg', None) or str(i)
        print('     [%s/%s] %s' % (i.level, i.code, msg[:110]))
    print('  → 复合结论: route_check=%s ; dzbrain=%s'
          % ('绿' if rrc == 0 and not rc.FAIL else '红',
             '红' if errs else '绿'))
