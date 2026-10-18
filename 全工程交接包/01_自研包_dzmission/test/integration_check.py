#!/usr/bin/env python3
"""端到端联调：接口一致性审计 + 话题配对检查

目的：比赛全程 3 分钟、**无人干预**，任何一处接口错位都会导致零分。
本脚本把"哪些话题被谁发布、被谁订阅"逐条列出来并核对配对，
把"静默失效"（订阅不到、类型不匹配）变成**上线前就能发现**的问题。

检查项：
  1. 每个被订阅的话题，是否真的有人发布
  2. 消息类型是否匹配（同名话题类型不同 -> ROS 2 会直接不连通）
  3. QoS 兼容性（BEST_EFFORT 发布 + RELIABLE 订阅 = 静默丢弃）
  4. launch 文件是否把必需节点都起起来了

运行:  python3 integration_check.py
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if not os.path.isdir(os.path.join(ROOT, 'dzmission')):
    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 工作空间根（含各包目录）
WS = os.path.dirname(ROOT) if os.path.basename(ROOT) == 'dzmission' else ROOT

PACKAGES = ['dzmission', 'rknn_pt', 'dzactuator', 'dzjudgment',
            'dzrknn_dataprocess', 'dztrack']


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


def find_sources(pkg):
    """返回该包下的 .cpp/.cc/.hpp 源码路径列表"""
    base = os.path.join(WS, pkg)
    out = []
    for sub in ('src', 'include', 'test'):
        d = os.path.join(base, sub)
        if not os.path.isdir(d):
            continue
        for dirpath, _dirs, files in os.walk(d):
            for f in files:
                if f.endswith(('.cpp', '.cc', '.hpp', '.h')):
                    out.append(os.path.join(dirpath, f))
    return out


CREATE_RE = re.compile(
    r'create_(publisher|subscription|client|service)\s*'
    r'(?:<([^<>]*(?:<[^<>]*>)?[^<>]*)>)?\s*\(\s*["\']([^"\']+)["\']')

# ★ 第二种写法：话题名来自**参数变量**而非字面量，例如
#     declare_parameter<std::string>("output_topic", "offset_center");
#     det_pub_ = create_publisher<...>(out_topic, 1);
#   若不处理这种写法，会把"实际有发布者"误报为"无人发布"。
CREATE_VAR_RE = re.compile(
    r'create_(publisher|subscription|client|service)\s*'
    r'(?:<([^<>]*(?:<[^<>]*>)?[^<>]*)>)?\s*\(\s*([A-Za-z_]\w*)\s*[,)]')
DECLARE_PARAM_RE = re.compile(
    r'declare_parameter\s*<\s*[^>]*\s*>\s*\(\s*["\']([^"\']+)["\']\s*,\s*'
    r'["\']([^"\']*)["\']')

# 参数被读进哪个变量：get_parameter("output_topic", out_topic)
GET_PARAM_RE = re.compile(
    r'get_parameter\s*\(\s*["\']([^"\']+)["\']\s*,\s*([A-Za-z_]\w*)\s*\)')


def extract_param_topics(flat):
    """返回 {变量名: 话题名}

    ★ 关键：**变量名往往与参数名不同**，例如
         declare_parameter<std::string>("output_topic", "offset_center");
         get_parameter("output_topic", out_topic);
         create_publisher<...>(out_topic, 1);
      参数名是 output_topic，变量名是 out_topic。
      所以必须通过 get_parameter("参数名", 变量名) 建立两者关联，
      否则会把"实际有发布者"误报为"无人发布"（假阳性）。
    """
    decl = {}
    for m in DECLARE_PARAM_RE.finditer(flat):
        pname, pdefault = m.group(1), m.group(2)
        if pdefault.startswith('/') or pdefault.endswith('_center') \
                or 'topic' in pname:
            decl[pname] = pdefault
    out = {}
    for m in GET_PARAM_RE.finditer(flat):
        pname, var = m.group(1), m.group(2)
        if pname in decl:
            out[var] = decl[pname]
    return out


def scan_pkg(pkg):
    """扫描一个包，返回 [(file, kind, msgtype, topic)]"""
    rows = []
    for path in find_sources(pkg):
        try:
            s = open(path, encoding='utf-8', errors='ignore').read()
        except OSError:
            continue
        flat = re.sub(r'\s+', ' ', s)
        # ① 字面量话题
        for m in CREATE_RE.finditer(flat):
            kind, mtype, topic = m.group(1), m.group(2) or '', m.group(3)
            rows.append((os.path.relpath(path, WS), kind, mtype.strip(), topic))
        # ② 参数变量形式
        pmap = extract_param_topics(flat)
        for m in CREATE_VAR_RE.finditer(flat):
            kind, mtype, var = m.group(1), m.group(2) or '', m.group(3)
            topic = pmap.get(var)
            if topic:
                rows.append((os.path.relpath(path, WS), kind,
                             mtype.strip(), topic))
    return rows


def norm_type(t):
    """把消息类型规范化到短名（去掉命名空间与 msg::）"""
    if not t:
        return ''
    t = t.replace('::msg::', '/').replace('::srv::', '/').replace('::action::', '/')
    t = t.replace('::', '/')
    parts = [p for p in t.split('/') if p]
    return parts[-1] if parts else t


def main():
    print("端到端联调：接口一致性审计")
    print("工作空间根: %s" % WS)
    print()
    print("检查目标：比赛全程无人干预，接口错位 = 零分。")

    # ---- 扫描 ----
    all_rows = {}
    for pkg in PACKAGES:
        if not os.path.isdir(os.path.join(WS, pkg)):
            continue
        all_rows[pkg] = scan_pkg(pkg)

    sec("1. 各包对外接口清单")
    total = 0
    for pkg, rows in all_rows.items():
        pubs = [r for r in rows if r[1] == 'publisher']
        subs = [r for r in rows if r[1] == 'subscription']
        total += len(rows)
        print("  [%s]  发布 %d 个 / 订阅 %d 个" % (pkg, len(pubs), len(subs)))
        for (f, _k, mt, tp) in pubs:
            print("      PUB  %-28s %s" % (tp, norm_type(mt)))
        for (f, _k, mt, tp) in subs:
            print("      SUB  %-28s %s" % (tp, norm_type(mt)))
    print()
    print("  合计 %d 条接口" % total)

    # ---- 话题配对 ----
    sec("2. ★ 话题配对检查（谁发布 / 谁订阅）")

    pub_map = {}     # topic -> set(msgtype)
    sub_map = {}     # topic -> list(pkg, msgtype)
    for pkg, rows in all_rows.items():
        for (f, kind, mt, tp) in rows:
            nt = norm_type(mt)
            if kind == 'publisher':
                pub_map.setdefault(tp, set()).add(nt)
            elif kind == 'subscription':
                sub_map.setdefault(tp, []).append((pkg, nt))

    problems = []

    print("  %-30s %-16s %s" % ("话题", "类型", "状态"))
    print("  " + "-" * 72)
    for tp in sorted(set(pub_map) | set(sub_map)):
        pub_types = pub_map.get(tp, set())
        subs = sub_map.get(tp, [])
        if pub_types and subs:
            # 类型是否一致
            sub_types = {t for (_p, t) in subs}
            if sub_types - pub_types:
                problems.append(("类型不匹配", tp, pub_types, sub_types))
                status = "✗ 类型不匹配 pub=%s sub=%s" % (
                    "/".join(pub_types), "/".join(sub_types))
            else:
                status = "✓ 已配对"
        elif subs and not pub_types:
            problems.append(("无人发布", tp, set(), {t for (_p, t) in subs}))
            status = "✗ ★ 有人订阅但**无人发布**"
        else:
            status = "  （仅发布，供外部/调试消费）"
        shown = "/".join(sorted(pub_types | {t for (_p, t) in subs})) or "?"
        print("  %-30s %-16s %s" % (tp, shown, status))

    print()
    if problems:
        # ★ 分类：区分"真缺口"与"历史遗留/外部依赖"
        EXPECTED_EXTERNAL = {
            '/image_raw': '相机节点（usb_cam 或 dzactuator/camera.launch.py）'
                          '不在本工作空间内，必须由 launch 起',
            'cmd_vel_nav_raw': 'nav2 velocity_smoother 的输出，由 launch 重映射',
            'dztrack_raw_cmd': 'dztrack 的 cmd_vel_dztrack，需在 dztrack 侧或'
                               'launch 里重映射（见 doc/INTEGRATION.md）',
            'logicontrol': '遥控话题，由遥控器/上位机发布（比赛阶段不发布是正常的）',
            'scan': '激光雷达（★ 本赛题**禁止**雷达，应移除该订阅）',
            'amcl_pose': '定位模块（本方案用航位推算 + anchor，可能不启用 amcl）',
            'trafficLightArray': '历史遗留（交通灯，与本赛题无关）',
            'savemapping': '历史遗留（建图，比赛不用）',
            'passenger_waypoints': '历史遗留（载客路线，与本赛题无关）',
            'avp_parking_Num': '历史遗留（自动泊车，与本赛题无关）',
            'StopCmd': '底盘急停，由外部或调试工具发布',
            'HpAndHit': '旧版裁判板话题名（现用 HpAndHitmsg）—— 应删除该订阅',
            '/move_base/stop_signal': '旧版到位信号（dzrknn_dataprocess 用）'
                                      '—— 与 dzmission 的到位判定重叠，需确认由谁发',
            # ★ 由 AirLink 用**变量话题名**发布（AirLink::publishMulti(topic, ...)）。
            #   静态扫描只认 create_publisher<...>("字面量", ...)，认不出这些，
            #   但它们确实是本包发布的 —— 只需有订阅者即可，不是缺口。
            'field': 'AirLink::publishMulti("field", ...) —— 场地固定结构'
                     '（含无人机起降点），变量话题名故静态扫描看不到发布点',
            'gate': 'AirLink::publishMulti("gate", ...) —— 桥洞门位',
            'mine': 'AirLink::publishMulti("mine", ...) —— 地雷清单',
            'obs': 'AirLink::publishMulti("obs", ...) —— 障碍清单',
            'tgt': 'AirLink::publishMulti("tgt", ...) —— 打击区清单',
            'map': 'AirLink::publishMulti("map", ...) —— 全图栅格',
            'task': 'AirLink::publishMulti("task", ...) —— 任务指令',
            'ev': 'AirLink::publishMulti("ev", ...) —— 事件',
            'ack': 'AirLink::publishMulti("ack", ...) —— 回执',
        }
        real_gap, legacy, external = [], [], []
        for (kind, tp, p, s) in problems:
            if tp in EXPECTED_EXTERNAL:
                note = EXPECTED_EXTERNAL[tp]
                if '历史遗留' in note or '应移除' in note or '应删除' in note:
                    legacy.append((tp, note))
                else:
                    external.append((tp, note))
            else:
                real_gap.append((kind, tp, p, s))

        if real_gap:
            print("  ★★ 真正的问题（%d 个）—— 必须处理：" % len(real_gap))
            for (kind, tp, p, s) in real_gap:
                print("     [%s] %s" % (kind, tp, "/".join(p), "/".join(s)))
            print()
        if external:
            print("  ○ 需要 launch 或 remap 解决（%d 个）：" % len(external))
            for (tp, note) in external:
                print("     %-26s %s" % (tp, note))
            print()
        if legacy:
            print("  · 历史遗留（%d 个）—— 建议清理但对本赛题无影响：" % len(legacy))
            for (tp, note) in legacy:
                print("     %-26s %s" % (tp, note))
            print()
    else:
        print("  ✓ 所有被订阅的话题都有人发布，且类型一致")

    # ---- ★★ 资源冲突检测（多个节点抢同一资源）----
    sec("3. ★★ 资源冲突检测（多节点抢同一话题）")
    print("  ★ 为什么这是最危险的一类问题：")
    print("     ROS 2 允许多个发布者写同一话题，**不报错**，只是指令互相覆盖。")
    print("     云台/激光这类执行器被两个节点同时写时，表现为'抖动/有时不动'，")
    print("     现场极难定位，而比赛只有 3 分钟、无人干预。")
    print()

    # 收集每个话题的发布方（按包）与订阅方（按包）
    pub_by_pkg = {}
    sub_by_pkg = {}
    for pkg, rows in all_rows.items():
        for (f, kind, mt, tp) in rows:
            if kind == 'publisher':
                pub_by_pkg.setdefault(tp, set()).add(pkg)
            elif kind == 'subscription':
                sub_by_pkg.setdefault(tp, set()).add(pkg)

    conflicts = []
    for tp, pkgs in sorted(pub_by_pkg.items()):
        if len(pkgs) > 1:
            conflicts.append(('多发布者', tp, sorted(pkgs)))

    if conflicts:
        print("  ★ 发现 %d 处**多发布者冲突**：" % len(conflicts))
        print()
        for (kind, tp, pkgs) in conflicts:
            print("     [%s] %s" % (kind, tp))
            print("        发布方: %s" % ", ".join(pkgs))
            # 给出针对性说明
            if tp == 'LaserShot_Command':
                print("        ⚠ 这是**激光发射**：两个节点都能触发开火，")
                print("          会导致'没到对准就开火'或重复发弹（弹量只有 10 发！）")
            elif tp == 'CameMo_Conl':
                print("        ⚠ 这是**云台指令**：两个节点同时写会互相覆盖，")
                print("          表现为云台抖动或跟丢。★ 必须只保留一个控制者。")
            elif tp == 'pursuitAngle':
                print("        ⚠ 这是**转向指令**（舵机角度）")
            elif tp == 'cmd_vel':
                print("        ⚠ 这是**底盘速度指令**")
            print()
    else:
        print("  ✓ 未发现多发布者冲突")

    # 多订阅者通常是正常的（广播语义），但某些资源重复消费要提醒
    multi_sub = {tp: sorted(p) for tp, p in sub_by_pkg.items() if len(p) > 1}
    if multi_sub:
        print("  ○ 多订阅者（通常正常，但需确认语义）：")
        for tp, pkgs in sorted(multi_sub.items()):
            note = ""
            if tp == 'offset_center':
                note = ("  ⚠ 视觉结果被多个节点消费：dzrknn_dataprocess 会**直接驱动云台**，"
                        "与 dzmission 的 TargetAimer **抢云台**")
            print("     %-26s %s%s" % (tp, ", ".join(pkgs), note))
        print()


    sec(chr(34) + '5. ' + chr(9733)*2 + ' 坐标系与单位约定核对（最隐蔽的一类错误）' + chr(34))
    print(chr(34) + '  这类错误不报错，只是车往反方向走、或角度差 90 度。' + chr(34))
    print()
    yaml_path = os.path.join(WS, 'dzmission', 'params', 'dzmission.yaml')
    conv = {}
    if os.path.isfile(yaml_path):
        txt = open(yaml_path, encoding='utf-8', errors='ignore').read()
        for key in ('initial_map_yaw_deg', 'initial_col', 'initial_row',
                    'barrier_up_row', 'bridge_col', 'cell_size',
                    'yaw_ticks_per_deg', 'gimbal_max_speed'):
            m = re.search(r'^\s*' + key + r'\s*:\s*([-\w.]+)', txt, re.M)
            if m:
                conv[key] = m.group(1)
    EXPECT = {
        'initial_map_yaw_deg': ('-90.0', '起点车头朝 -y（M11 -> M10）'),
        'initial_col': ('12', '起点 M 列'),
        'initial_row': ('10', '起点第 11 行'),
    }
    print('  (A) 定位参数（dzmission.yaml 的 loc.*）：')
    bad = 0
    for k, (want, desc) in EXPECT.items():
        got = conv.get(k, '(缺)')
        ok = got.startswith(want) or want.startswith(got)
        if not ok:
            bad += 1
        print('     %s %-24s = %-10s 期望 %-8s %s' % (
            'OK ' if ok else 'XX ', k, got, want, desc))
    print()
    # ★ 拓扑常量在 C++ Spec 里（不是参数），必须从源码读
    spec_path = os.path.join(WS, 'dzmission', 'include', 'dzmission', 'field_grid.hpp')
    print('  (B) 拓扑常量（C++ FieldGrid::Spec，不是参数）：')
    SPEC_EXPECT = {
        'barrier_up_row': ('5', '隔离线在 row6/row7 之间'),
        'bridge_col': ('10', '桥洞在 K 列'),
        'start_col': ('12', '起点 M 列'),
        'start_row': ('10', '起点第 11 行'),
        'goal_col': ('13', '终点 N 列'),
        'goal_row': ('5', '终点第 6 行'),
    }
    spec_txt = ''
    if os.path.isfile(spec_path):
        spec_txt = open(spec_path, encoding='utf-8', errors='ignore').read()
    for k, (want, desc) in SPEC_EXPECT.items():
        m = re.search(r'spec_?\.?\s*' + k + r'\s*=\s*([-\w.]+)', spec_txt)
        if not m:
            m = re.search(k + r'\s*[={\[]\s*([-\w.]+)', spec_txt)
        got = m.group(1) if m else '(未找到)'
        ok = (got == want)
        if not ok:
            bad += 1
        print('     %s %-24s = %-10s 期望 %-8s %s' % (
            'OK ' if ok else 'XX ', k, got, want, desc))
    print()
    if bad == 0:
        print('  OK 坐标系/拓扑约定与 v5 一致')
    else:
        print('  XX %d 项与拓扑 v5 不符 -> 会导致定位/规划错位' % bad)
    print()
    print('  ' + chr(9733)*2 + ' 单位约定专项：pursuitAngle vs cmd_vel')
    print('     cmd_vel.angular.z       单位 = rad/s  (角速度)')
    print('     pursuitAngle.angular.z  单位 = 度     (舵机角度)')
    print('     同名 angular.z 但量纲不同。下游若按 rad/s 解释舵机角度，')
    print('     30 度会被当成 30 rad/s -> 完全错误的转向量。')
    print()
    print('     上线前目视确认：ros2 topic echo /pursuitAngle')
    print('     数值范围应是 +/-35 (度)，不是 +/-1 (rad/s)。')

    # ---- 关键链路专项核对 ----

    sec("4. ★ 关键链路专项核对（比赛必需）")

    # 这些是比赛必需的数据流，逐条核对
    CHAINS = [
        ("相机 -> 检测", [
            ("image_raw", "sensor_msgs::msg::Image", "pub"),
            ("offset_center", "", "sub"),
        ]),
        ("检测 -> 瞄准", [
            ("offset_center", "", "pub"),
            ("offset_center", "", "sub"),
        ]),
        ("裁判板 -> 车端", [
            ("referee/ammo", "", "pub"),
            ("Battery_Percentage", "", "sub"),
        ]),
        ("dztrack -> 仲裁", [
            ("dztrack_raw_cmd", "", "pub"),
            ("dztrack_raw_cmd", "", "sub"),
        ]),
    ]
    for name, items in CHAINS:
        print("  [%s]" % name)
        for (tp, _mt, kind) in items:
            chain = pub_map if kind == 'pub' else sub_map
            present = tp in chain
            who = ''
            if kind == 'pub' and present:
                who = "(%s)" % ", ".join(sorted(pub_map[tp]))
            elif kind == 'sub' and present:
                who = "(%s)" % ", ".join(sorted({p for (p, _t) in sub_map[tp]}))
            mark = "✓" if present else "✗"
            print("     %s %-6s %-26s %s" % (mark, kind.upper(), tp, who))
        print()

    # ---- launch 覆盖 ----
    sec("5. launch 文件覆盖检查")
    print("  ★ 比赛需要一个**统一入口**把必需节点都起起来。")
    print()
    required_nodes = {
        'dzactuator': '（底盘/云台/激光驱动，必须起）',
        'rknn_pt': '（视觉检测，必须起）',
        'usb_cam 或 dzactuator/camera.launch.py': '（相机，必须起）',
        'dzmission': '（任务编排，必须起）',
    }
    print("  比赛必需的节点：")
    for k, v in required_nodes.items():
        print("     - %-38s %s" % (k, v))
    print()
    lf = os.path.join(WS, 'dzmission', 'launch', 'bringup_mission.launch.py')
    if os.path.isfile(lf):
        s = open(lf, encoding='utf-8', errors='ignore').read()
        started = []
        for probe, label in (('dzmission_node', 'dzmission'),
                             ('det_node', 'rknn_pt/det_node'),
                             ('usb_cam', 'usb_cam'),
                             ('camera.launch', 'camera'),
                             ('dzcontrol', 'dzactuator/dzcontrol'),
                             ('dztrack', 'dztrack')):
            if probe in s:
                started.append(label)
        print("  dzmission.launch.py 当前启动的节点：")
        if started:
            for x in started:
                print("     ✓ %s" % x)
        else:
            print("     （未检出节点）")
        missing = [l for x, l in (
            ('det_node', 'rknn_pt/det_node'),
            ('usb_cam', 'usb_cam / camera'),
            ('dzcontrol', 'dzactuator/dzcontrol')) if x not in s]  if 'bringup_mission' in os.path.basename(lf) else missing
        if missing:
            print()
            print("  ★ 未包含：%s" % ", ".join(missing))
            print("     ⇒ 需要**总启动文件**把相机/检测/驱动一起拉起，")
            print("       否则上场时要开 4 个终端，2 分钟准备时间不够且易漏。")
    else:
        print("  ★ 未找到 dzmission.launch.py")

    sec("总结论")
    print("  1. 接口清单已列出（%d 条）" % total)
    print("  2. 话题配对：%s" % (
        "✓ 全部正常" if not problems else "★ 有 %d 个问题（见上）" % len(problems)))
    print("  3. 关键链路：见 §3")
    print("  4. launch 覆盖：见 §4（建议做统一入口）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
