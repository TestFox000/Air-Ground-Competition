#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re, pathlib, sys
p = pathlib.Path('/home/duzhong/dzrobot/config/robot.yaml')
t = p.read_text(encoding='utf-8')
orig = t
subs = [
    (r'(cross_lower_candidates:\s*)\["K7"\]', r'\1["K10"]'),
    (r'(cross_upper_candidates:\s*)\["K5"\]', r'\1["K4"]'),
    (r'(cross_lower_default:\s*)"K7"', r'\1"K10"'),
    (r'(cross_upper_default:\s*)"K5"', r'\1"K4"'),
]
for pat, rep in subs:
    t, n = re.subn(pat, rep, t)
    print('sub', pat[:40], '->', n)
t, n = re.subn(r'(?m)^(    auto_start:\s*)true\b', r'\1false', t)
print('auto_start true->false:', n)
if t != orig:
    p.write_text(t, encoding='utf-8')
    print('WROTE')
import yaml
cfg = yaml.safe_load(t)
print('YAML_OK sections:', len(cfg))
for line in t.splitlines():
    if 'cross_lower' in line or 'cross_upper' in line or 'auto_start:' in line:
        print(' |', line.strip()[:90])
