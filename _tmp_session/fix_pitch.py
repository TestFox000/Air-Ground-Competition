# -*- coding: utf-8 -*-
# 2026-10-15 现场实测：scan_pitch_offset_deg=-10.0 实际是【向上抬 10°】，符号约定反了。
# 改为 +10.0（向下压）。四个落点：robot.yaml :50/:658、cpp :465/:1093、h :174。
import io, sys

def rep(path, pairs):
    s = io.open(path, encoding='utf-8').read()
    for old, new, tag in pairs:
        n = s.count(old)
        if n != 1:
            print('FAIL %s %s count=%d' % (path, tag, n)); sys.exit(1)
        s = s.replace(old, new, 1)
    io.open(path, 'w', encoding='utf-8', newline='\n').write(s)
    print('OK %s (%d pairs)' % (path, len(pairs)))

rep('dzrobot_v2/config/robot.yaml', [
    ('    scan_pitch_offset_deg: -10.0   # ★t52 用户指令（2026-10-14）：水平扫描时把视角【向下压约 10°】⇒ 取 -10.0',
     '    scan_pitch_offset_deg: +10.0   # ★t52 用户指令（2026-10-14）：水平扫描向下压约 10°；★2026-10-15 现场实测：-10.0 实际【向上抬】（符号约定反了）⇒ 改 +10.0',
     'yaml:50'),
    ('      scan_pitch_offset_deg: -10.0    # ★t52 与 /dzrknn_dataprocess 段对齐为 -10.0（负 = 向下压；',
     '      scan_pitch_offset_deg: +10.0    # ★t52 与 /dzrknn_dataprocess 段对齐为 +10.0（★2026-10-15 现场实测：+ 才向下压、',
     'yaml:658'),
])

rep('dzrobot_v2/src/dzrknn_dataprocess/src/dzrknn_dataprocess.cpp', [
    ('// scan_pitch_offset_deg_ 兜底默认 -10.0（t52，用户 2026-10-14 指令"水平扫描向下压约 10°"；\n//   负=向下压）：整体向下压，让画面中心对准稍近处/地面上的靶标。真源是 robot.yaml。',
     '// scan_pitch_offset_deg_ 兜底默认 +10.0（t52，用户 2026-10-14 指令"水平扫描向下压约 10°"；\n//   ★2026-10-15 现场实测：+ 才向下压、- 实际向上抬，符号约定已更正）。真源是 robot.yaml。',
     'cpp:465'),
    ('  this->declare_parameter("scan_pitch_offset_deg", -10.0); // 俯仰整体偏移(度，负=向下压；t52 与 robot.yaml 单一真源对齐)',
     '  this->declare_parameter("scan_pitch_offset_deg", +10.0); // 俯仰整体偏移(度，★实测：+向下压、-向上抬；t52 与 robot.yaml 单一真源对齐)',
     'cpp:1093'),
])

rep('dzrobot_v2/src/dzrknn_dataprocess/include/dzrknn_dataprocess.h', [
    ('	double scan_pitch_offset_deg_ = -10.0; // 俯仰整体偏移(度，负 = 向下压；t52 与 robot.yaml 对齐)',
     '	double scan_pitch_offset_deg_ = +10.0; // 俯仰整体偏移(度，★实测：+向下压、-向上抬；t52 与 robot.yaml 对齐)',
     'h:174'),
])
print('ALL_DONE')
