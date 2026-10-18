#!/usr/bin/env python3
"""跨头文件常量可见性检查（本机可跑，不需要编译器）

★ 为什么需要它：
  本轮真实编译错误：
      target_aimer.cpp:435: error: 'kTargetNone' was not declared in this scope
  根因：kTargetNone 定义在 mission_command.hpp（dzmission 命名空间），
        而 target_aimer.cpp 只 include 了 target_aimer.hpp + 标准库，
        **没有直接或间接包含 mission_command.hpp**。
        （对比：mission_manager.cpp 经 cmd_link.hpp 间接获得，所以能用 ——
          这正是"间接包含"最危险的地方：同一个常量在 A 文件能用、B 文件不能。）

  "依赖间接包含"是脆弱写法：上游头文件一改动，就会连带编译失败，
  而且报错位置与被改的地方毫无关系，极难定位。

检查：
  1. 收集各头文件对外提供的常量名（constexpr k*）
  2. 对每个 .cpp，找出它用到的这些常量
  3. 判断该 .cpp 的直接 include 闭包（递归展开本项目的头文件）
     是否包含定义这些常量的头文件
  4. 报告"用了但没包含"的情况

运行:  python3 include_closure_check.py
"""

import io
import os
import re
import sys

ROOT = r'C:\Users\Ran\Desktop\无人车\src-car\dzmission'
INC = os.path.join(ROOT, 'include', 'dzmission')
SRC = os.path.join(ROOT, 'src')


def sec(t):
    print()
    print("=" * 76)
    print(t)
    print("=" * 76)


def read(p):
    return io.open(p, encoding='utf-8', errors='ignore').read()


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//[^\n]*', ' ', text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    return text


def header_provides():
    """返回 {头文件名: set(常量名)}"""
    out = {}
    for f in os.listdir(INC):
        if not f.endswith(('.hpp', '.h')):
            continue
        code = strip_comments(read(os.path.join(INC, f)))
        names = set(re.findall(r'\bconstexpr\s+[\w:<>]+\s+(k\w+)', code))
        # 也算上 enum 里的名字（kXxx 形式）
        names |= set(re.findall(r'\b(k[A-Z]\w*)\s*=', code))
        if names:
            out[f] = names
    return out


def direct_includes(path):
    """本 .cpp/.hpp 直接 include 的【项目内】头文件名

    ★ 必须用**原始内容**，不能先 strip_comments：
      strip_comments 会把字符串字面量替换成 ""，而 `#include "..."` 的路径
      本身就是字符串 —— 结果变成 `#include ""`，所有 include 都被抹掉。
      本检查器第一版就是这么坏掉的（恰恰是它要查的那类问题）。
      include 行里不可能出现注释，所以用原文完全安全。
    """
    code = read(path)
    incs = re.findall(r'#\s*include\s+"dzmission/([\w./]+)"', code)
    return set(incs)


def include_closure(start_incs):
    """递归展开项目内头文件的 include 闭包"""
    seen = set()
    stack = list(start_incs)
    while stack:
        h = stack.pop()
        if h in seen:
            continue
        seen.add(h)
        p = os.path.join(INC, h)
        if os.path.isfile(p):
            stack.extend(direct_includes(p))
    return seen


def main():
    print("跨头文件常量可见性检查")
    print("★ 本机无编译器；本工具查'用了某常量但未（间接）包含定义它的头文件'")

    provides = header_provides()
    if not provides:
        print("没找到任何提供 k* 常量的头文件")
        return 1

    sec("1. 各头文件提供的常量")
    for h, names in sorted(provides.items()):
        print("  %-28s %d 个（%s…）"
              % (h, len(names), ", ".join(sorted(names)[:4])))

    # 常量名 -> 定义它的头文件（★ 多对多：一个名字可能被多个头文件定义）
    owner = {}
    owner_multi = {}
    for h, names in provides.items():
        for n in names:
            owner.setdefault(n, h)
            owner_multi.setdefault(n, []).append(h)

    sec("2. 逐个 .cpp 检查")

    problems = []
    checked = 0
    for f in sorted(os.listdir(SRC)):
        if not f.endswith(('.cpp', '.cc')):
            continue
        p = os.path.join(SRC, f)
        code = strip_comments(read(p))
        direct = direct_includes(p)
        closure = include_closure(direct)

        # 该 .cpp 用到的、由头文件提供的常量
        used = {}
        for n, h in owner.items():
            if re.search(r'(?<![\w:])' + re.escape(n) + r'\b', code):
                used.setdefault(h, []).append(n)
        if not used:
            continue
        checked += 1

        missing = {h: ns for h, ns in used.items() if h not in closure}
        if missing:
            for h, ns in sorted(missing.items()):
                problems.append(
                    "%s 用到 %s（定义在 %s），但直接 include 闭包里没有它"
                    % (f, ", ".join(sorted(ns)[:5]), h))
        else:
            print("  [OK] %-30s (用到 %d 个头文件的常量，闭包完整)"
                  % (f, len(used)))

    sec("3. 常量重名冲突（★ 按危险程度分级）")
    # =====================================================================
    # 为什么要查、怎么分级：
    #   本项目里多个常量名被两个头文件各定义一次。危险程度不同：
    #
    #   【危险】同一命名空间层级 + 值不同 -> 静默取错值
    #     例：kMaxFrameLen  air=1036（空地 UDP 帧） vs dzmission=96（串口帧）
    #         在能看到 air:: 的文件里写裸名 kMaxFrameLen，想开 1036 缓冲，
    #         实际拿到 96 -> **栈上缓冲区溢出**。
    #
    #   【冗余但安全】值相同 -> 同名同义
    #     例：kFaultNoAmmo 两边都是 1。
    #
    #   【安全】enum 成员 -> 本来就**必须**用 Enum::name 限定，裸用不可能
    #     例：State::kIdle / Phase::kIdle / Reason::kNone。
    #         把它们算作冲突会制造噪音（本工具第一版就报了 14 条，其中
    #         10 条是这种安全情况）。
    # =====================================================================
    VALUE_PAT = re.compile(r'\b(k\w+)\s*=\s*([^,;\n}]+?)\s*(?:,|;|$)')

    def values_of(header):
        code = strip_comments(read(os.path.join(INC, header)))
        out = {}
        for m in VALUE_PAT.finditer(code):
            v = m.group(2).strip()
            v = re.sub(r'//.*$', '', v).strip()
            out[m.group(1)] = v
        return out

    hdr_vals = {h: values_of(h) for h in provides}
    # 哪些名字出现在 enum 体内（这类是 enum 成员）
    enum_names = set()
    for h in provides:
        code = strip_comments(read(os.path.join(INC, h)))
        for m in re.finditer(r'\benum\s+(?:class\s+)?\w*\s*\{([^}]*)\}', code, re.S):
            for mm in re.finditer(r'\b(k\w+)', m.group(1)):
                enum_names.add(mm.group(1))

    dangerous, redundant, safe_enum = [], [], []
    for n, hs in sorted(owner_multi.items()):
        if len(hs) < 2:
            continue
        vals = {hdr_vals[h].get(n) for h in hs}
        if n in enum_names:
            safe_enum.append((n, hs))
        elif len(vals) > 1:
            detail = ", ".join("%s=%s" % (h, hdr_vals[h].get(n)) for h in hs)
            dangerous.append((n, detail))
        else:
            redundant.append((n, hs, list(vals)[0]))

    if dangerous:
        print("  【危险】值不同 —— 会静默取错值，建议改名：")
        for n, d in dangerous:
            print("    [XX] %-20s %s" % (n, d))
        print()
    if redundant:
        print("  【冗余但安全】值相同 —— 同名同义，可不改：")
        for n, hs, v in redundant:
            print("    [--] %-20s 值都是 %s  (%s)" % (n, v, ", ".join(hs)))
        print()
    if safe_enum:
        print("  【安全】enum 成员 —— 本来就必须 Enum::name 限定，裸用不可能：")
        print("    [--] " + ", ".join(n for n, _ in safe_enum))
        print()

    if not dangerous:
        print("  ✓ 没有'值与层级都冲突'的常量")

    sec("4. 用了但没包含（低优先级：多为间接包含或同族命名）")
    if problems:
        for pr in problems:
            print("  [--] %s" % pr)
    else:
        print("  ✓ 无")

    sec("汇总")
    print("  检查 %d 个 .cpp" % checked)
    print("  【危险】值不同的重名 : %d 条" % len(dangerous))
    print("  【安全】enum 成员     : %d 条" % len(safe_enum))
    print("  【冗余】值相同的重名 : %d 条" % len(redundant))
    print("  用了未直接包含       : %d 条（低优先级）" % len(problems))
    if not dangerous:
        print("  ✓ 通过（无危险重名）")
    else:
        print("  ✗ 需处理上面的【危险】项")
    return 0 if not dangerous else 1


if __name__ == '__main__':
    sys.exit(main())
