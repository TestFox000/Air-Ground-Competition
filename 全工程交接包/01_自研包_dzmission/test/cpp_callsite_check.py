#!/usr/bin/env python3
"""C++ 调用点审计：参数个数 + 基础类型粗查

★ 为什么需要这个脚本：
  本机（Windows 沙箱）**没有 C++ 编译器**，无法 colcon build。
  但"参数个数不匹配"是 C++ 编译错误的第一大来源，而且是完全可以静态检查的。
  本脚本把这类错误在上 Ubuntu 之前就抓出来。

能力边界（务必理解）：
  ✓ 能查：函数参数个数不匹配；调用了未声明的函数；明显的作用域问题
  ✗ 不能查：模板实例化、类型推导、重载决议、链接错误
  => 它能**减少**去 Ubuntu 后的试错次数，但**不能替代编译**

运行:  python3 cpp_callsite_check.py
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if not os.path.isdir(os.path.join(ROOT, 'dzmission')):
    ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WS = ROOT if os.path.isdir(os.path.join(ROOT, 'dzmission')) else \
    os.path.dirname(os.path.dirname(ROOT))

HPP_DIR = os.path.join(WS, 'dzmission', 'include', 'dzmission')
SRC_DIR = os.path.join(WS, 'dzmission', 'src')


def sec(t):
    print()
    print("=" * 78)
    print(t)
    print("=" * 78)


def strip_comments(t):
    """去掉注释与字符串字面量，避免误匹配"""
    t = re.sub(r'//[^\n]*', '', t)
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    t = re.sub(r'"(?:\\.|[^"\\])*"', '""', t)
    t = re.sub(r"'(?:\\.|[^'\\])*'", "''", t)
    return t


def split_params(s):
    """按顶层逗号切分参数列表（跳过 <> 与 () 内部）"""
    out = []
    depth_ang = 0
    depth_par = 0
    cur = ''
    for ch in s:
        if ch == '<':
            depth_ang += 1
        elif ch == '>':
            depth_ang -= 1
        elif ch == '(':
            depth_par += 1
        elif ch == ')':
            depth_par -= 1
        if ch == ',' and depth_ang == 0 and depth_par == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def count_params(s):
    """返回 (最少参数个数, 最多参数个数)，考虑默认值"""
    s = s.strip()
    if not s or s == 'void':
        return 0, 0
    parts = split_params(s)
    mn = 0
    mx = len(parts)
    for p in parts:
        if '=' in p:
            break
        mn += 1
    return mn, mx


DECL_RE = re.compile(
    r'(?:^|[;{}]\s*)'                       # 语句起始
    r'(?:static\s+|inline\s+|virtual\s+|explicit\s+|constexpr\s+)*'
    r'([A-Za-z_][\w:<>,\s\*&]*?)\s+'        # 返回类型
    r'([A-Za-z_]\w*)\s*'                    # 函数名
    r'\(([^;{}]*?)\)\s*'                    # 参数
    r'(?:const\s*)?(?:noexcept\s*)?(?:override\s*)?(?:=\s*0\s*)?;',
    re.M)


def scan_decls(text):
    """返回 {函数名: [(min,max)]}（同名可能重载）"""
    out = {}
    for m in DECL_RE.finditer(text):
        name = m.group(2)
        mn, mx = count_params(m.group(3))
        out.setdefault(name, []).append((mn, mx))
    return out


# 我们关心的函数（新调用/新定义，最可能出错）
WATCHED = [
    'event', 'cellDecode', 'cellEncode', 'encodeFrame', 'sendFrame',
    'sendEvent', 'setGates', 'update', 'configure', 'selfCheck',
    'markMineByRecon', 'markBlockedByRecon', 'markTargetByRecon',
    'rdU16Pub', 'openCrossing', 'handleMineList', 'handleObstacleList',
    'handleTargetList', 'handleBridgeGate', 'handleMapFull',
    'applyReconToGrid', 'tickBridgeCrossing', 'worldToCell',
    'inBounds', 'prepare', 'parse', 'hashRecon',
]

# ★★ 关键：同名函数可能分属不同命名空间，签名完全不同，必须分开登记 ★★
#   例：air::encodeFrame(typ, seq, payload)   —— 3 参
#       dzmission::encodeFrame(cmd, payload, out, cap) —— 4 参
#   若不区分，会把正确代码误报为"参数个数不符"（我第一版就踩了这个坑）。
MANUAL = {
    # air_protocol.hpp  namespace air::enc
    'event': [(2, 5)],            # (ts_ms, ev, a0=0, a1=0, a2=0)
    'cellDecode': [(4, 4)],       # (v, occ&, conf&, age&)
    'cellEncode': [(1, 3)],       # (occ, conf=15, age=0)
    'heartbeat': [(1, 4)],
    'bridgeGate': [(2, 4)],       # (bridge_id, gate_row, conf=100, flags=0x01)
    'mineList': [(2, 2)],
    'obstacleList': [(1, 1)],
    'targetList': [(2, 2)],
    'mapFull': [(3, 3)],
    'ack': [(1, 3)],
    'taskCmd': [(2, 3)],
    'navToCell': [(3, 5)],
    # mission_command.hpp
    'encodeFrame': [(4, 4)],      # dzmission:: (cmd, payload, out, out_cap)
    'air::encodeFrame': [(3, 3)], # air:: (typ, seq, payload)
    'encodeVehicleStatus': [(3, 3)],
    'encodeTargetAck': [(3, 3)],
    'decodeMissionCommand': [(2, 2)],
    'crc16_modbus': [(2, 2)],
    # cmd_link.hpp
    'sendFrame': [(2, 2)],        # (buf, len)
    # bridge_tracker.hpp
    'setGates': [(2, 2)],
    'update': [(2, 2)],
    'configure': [(7, 7)],
    'selfCheck': [(0, 0)],
    # field_grid.hpp
    'markMineByRecon': [(2, 2)],
    'markBlockedByRecon': [(2, 2)],
    'markTargetByRecon': [(2, 2)],
    'rdU16Pub': [(1, 1)],
    'openCrossing': [(1, 1)],
    'closeCrossing': [(1, 1)],
    'worldToCell': [(4, 4)],
    'inBounds': [(2, 2)],
    'cellName': [(2, 2)],
    'markBlocked': [(2, 2)],
    'markMine': [(2, 2)],
    'markTarget': [(2, 2)],
    'markFree': [(2, 2)],
    # mission_manager.hpp（成员函数，含 this 隐式）
    'handleMineList': [(1, 1)],
    'handleObstacleList': [(1, 1)],
    'handleTargetList': [(1, 1)],
    'handleBridgeGate': [(1, 1)],
    'handleMapFull': [(1, 1)],
    'applyReconToGrid': [(0, 0)],
    'tickBridgeCrossing': [(0, 0)],
    'sendEvent': [(1, 4)],
    'setFaultBit': [(2, 2)],
    'handleTargetReport': [(1, 1)],
    'handleSetHome': [(1, 1)],
}

# 非函数调用（宏、局部 lambda、局部变量同名等），跳过
IGNORE_CALLS = {
    'if', 'while', 'for', 'switch', 'return', 'sizeof', 'static_cast',
    'reinterpret_cast', 'const_cast', 'dynamic_cast', 'decltype', 'alignof',
    'catch', 'throw', 'new', 'delete', 'defined',
    'std::', 'RCLCPP_INFO', 'RCLCPP_WARN', 'RCLCPP_ERROR', 'RCLCPP_DEBUG',
    'RCLCPP_INFO_THROTTLE', 'RCLCPP_WARN_THROTTLE', 'RCLCPP_ERROR_THROTTLE',
    'std::make_unique', 'std::make_shared', 'std::min', 'std::max',
    'std::abs', 'std::hypot', 'std::to_string', 'std::round', 'std::floor',
    'std::ceil', 'std::lround', 'std::sort', 'std::find', 'std::count',
    'std::vector', 'std::string', 'std::pair', 'std::array', 'std::chrono',
    'std::lock_guard', 'std::mutex', 'std::atomic', 'std::shared_ptr',
    'std::unique_ptr', 'std::move', 'std::function', 'std::bind',
}

CALL_RE = re.compile(r'([A-Za-z_][\w:]*)\s*\(')


def find_call_args(text, start):
    """从 '(' 的下标开始，返回顶层参数列表字符串与结束位置"""
    depth = 0
    i = start
    while i < len(text):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[start + 1:i], i
        i += 1
    return None, start


def main():
    print("C++ 调用点审计：参数个数 + 作用域粗查")
    print("★ 本机无 C++ 编译器，本脚本用于**减少上 Ubuntu 后的试错次数**")
    print("  （不能替代编译：模板/重载/链接仍需真实编译器）")

    if not os.path.isdir(HPP_DIR):
        print("找不到头文件目录: %s" % HPP_DIR)
        return 1

    # ---- 收集头文件里的声明（补充 MANUAL） ----
    decls = dict((k, list(v)) for k, v in MANUAL.items())
    for fn in sorted(os.listdir(HPP_DIR)):
        if not fn.endswith('.hpp'):
            continue
        t = strip_comments(open(os.path.join(HPP_DIR, fn),
                                encoding='utf-8', errors='ignore').read())
        for name, sigs in scan_decls(t).items():
            decls.setdefault(name, [])
            for s in sigs:
                if s not in decls[name]:
                    decls[name].append(s)

    # ★ 也要扫 .cpp 里的**文件内定义**（匿名 namespace / static 函数）。
    #   这类函数不在头文件里，若只扫头文件会误报"未找到声明"。
    #   （hashRecon 就是这种情况，第一版误报了 3 次）
    for fn in sorted(os.listdir(SRC_DIR)):
        if not fn.endswith('.cpp'):
            continue
        t = strip_comments(open(os.path.join(SRC_DIR, fn),
                                encoding='utf-8', errors='ignore').read())
        # 只收"定义"形式：返回类型 名字(参数) {  —— 以 { 结尾
        for m in re.finditer(
                r'(?:^|[;{}]\s*)'
                r'(?:static\s+|inline\s+|constexpr\s+)*'
                r'([A-Za-z_][\w:<>,\s\*&]*?)\s+'
                r'([A-Za-z_]\w*)\s*\(([^;{}]*?)\)\s*'
                r'(?:const\s*)?(?:noexcept\s*)?\{', t, re.M):
            name = m.group(2)
            mn, mx = count_params(m.group(3))
            decls.setdefault(name, [])
            if (mn, mx) not in decls[name]:
                decls[name].append((mn, mx))

    sec("1. 已知签名表（参数个数范围）")
    for fn in WATCHED:
        if fn in decls:
            sigs = decls[fn]
            txt = ", ".join("%d~%d" % (a, b) if a != b else str(a)
                            for (a, b) in sigs)
            print("  %-24s %s" % (fn, txt))
        else:
            print("  %-24s ★ 未找到声明" % fn)

    # ---- 扫描源文件里的调用点 ----
    sec("2. 调用点逐条核对")
    problems = []
    checked = 0

    targets = []
    for d in (SRC_DIR,):
        for fn in sorted(os.listdir(d)):
            if fn.endswith('.cpp'):
                targets.append(os.path.join(d, fn))

    for path in targets:
        raw = open(path, encoding='utf-8', errors='ignore').read()
        text = strip_comments(raw)
        base = os.path.basename(path)
        for m in CALL_RE.finditer(text):
            full = m.group(1)
            name = full.split('::')[-1]
            if name in IGNORE_CALLS or full in IGNORE_CALLS:
                continue
            if name not in WATCHED:
                continue
            # ★ 带命名空间限定的调用（如 air::encodeFrame）优先用限定名查表，
            #   避免与同名的另一命名空间函数混淆（假阳性来源）
            key = None
            if '::' in full:
                for cand in (full, full.split('::', 1)[1] if '::' in full else full):
                    if cand in decls:
                        key = cand
                        break
            lookup = key if key else name
            args_str, _end = find_call_args(text, m.end() - 1)
            if args_str is None:
                continue
            args_str = args_str.strip()
            nargs = 0 if not args_str else len(split_params(args_str))
            checked += 1

            sigs = decls.get(lookup)
            if not sigs:
                problems.append((base, full, nargs, '未找到声明（可能需 include）'))
                continue

            # 函数名字面量出现在声明处也会被匹配到，排除：公告型调用无变量上下文
            ok = any(mn <= nargs <= mx for (mn, mx) in sigs)
            if not ok:
                rng = ", ".join("%d~%d" % (a, b) if a != b else str(a)
                                for (a, b) in sigs)
                problems.append((base, name, nargs,
                                 '参数个数不符（声明接受 %s）' % rng))

    print("  共核对 %d 个调用点（仅限本脚本关注的函数）" % checked)
    print()
    if problems:
        print("  ★ 发现 %d 处可疑：" % len(problems))
        for (f, n, c, why) in problems:
            print("     %-22s %-22s 传了 %d 个 -> %s" % (f, n, c, why))
    else:
        print("  ✓ 未发现参数个数不符")

    # ---- 专项：我本轮新增的高风险调用 ----
    sec("3. ★ 本轮新增代码的高风险调用专项")

    checks = [
        ("air::enc::event(ts, ev, a0, a1, a2)",
         "mission_manager.cpp", "enc::event(", (2, 5), 5),
        ("air::enc::cellDecode(cell, occ, conf, age)",
         "mission_manager.cpp", "enc::cellDecode(", (4, 4), 4),
        ("encodeFrame(cmd, payload, out, cap)",
         "mission_manager.cpp", "encodeFrame(", (4, 4), 4),
        ("cmd_link_->sendFrame(buf, n)",
         "mission_manager.cpp", "sendFrame(", (2, 2), 2),
        ("bridge_->setGates(row_a, row_b)",
         "mission_manager.cpp", "setGates(", (2, 2), 2),
        ("bridge_->update(col, row)",
         "mission_manager.cpp", "update(", (2, 2), 2),
        ("bridge_->configure(7 args)",
         "mission_manager.cpp", "configure(", (7, 7), 7),
        ("grid_->worldToCell(x, y, c, r)",
         "mission_manager.cpp", "worldToCell(", (4, 4), 4),
        ("grid_->markBlockedByRecon(c, r)",
         "mission_manager.cpp", "markBlockedByRecon(", (2, 2), 2),
        ("grid_->markMineByRecon(c, r)",
         "mission_manager.cpp", "markMineByRecon(", (2, 2), 2),
        ("FieldGrid::rdU16Pub(p)",
         "mission_manager.cpp", "rdU16Pub(", (1, 1), 1),
        ("grid_->openCrossing(row)",
         "mission_manager.cpp", "openCrossing(", (1, 1), 1),
    ]
    allok = True
    for (desc, fname, pat, rng, expect) in checks:
        p = os.path.join(SRC_DIR, fname)
        t = strip_comments(open(p, encoding='utf-8', errors='ignore').read())
        hits = []
        for m in re.finditer(re.escape(pat), t):
            a, _ = find_call_args(t, m.end() - 1)
            if a is None:
                continue
            hits.append(0 if not a.strip() else len(split_params(a)))
        if not hits:
            print("  ○ %-46s 未找到调用" % desc)
            continue
        bad = [h for h in hits if not (rng[0] <= h <= rng[1])]
        if bad:
            allok = False
            print("  ✗ %-46s 传了 %s，期望 %d" % (desc, bad, expect))
        else:
            print("  ✓ %-46s 全部调用传 %d 个参数" % (desc, expect))

    sec("总结论")
    print("  1. 扫描 %d 个调用点，可疑 %d 处" % (checked, len(problems)))
    print("  2. 本轮新增高风险调用：%s" % ("✓ 全部参数个数正确" if allok else "✗ 有问题"))
    print()
    print("  ★ 本脚本**不能替代编译**。它只保证'参数个数'这一层正确；")
    print("     模板实例化、重载决议、类型推导、链接错误仍需在 Ubuntu 上 build。")
    print("  ★ 但参数个数不匹配是编译错误的第一大来源，过了这一关能显著")
    print("     减少上机后的试错次数。")
    return 0 if (not problems and allok) else 1


if __name__ == "__main__":
    sys.exit(main())
