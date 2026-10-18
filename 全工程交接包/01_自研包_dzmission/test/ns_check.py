#!/usr/bin/env python3
"""限定名引用检查（本机可跑，不需要编译器）

★ 为什么需要它：
  本轮真实编译错误之一：
      error: 'kBridgeIdA' is not a member of 'dzmission::air';
             did you mean 'dzmission::kBridgeIdA'?
      note:  'dzmission::kBridgeIdA' declared here
  根因：`kEvBridgePassed` 确实在 `air::` 里（air_protocol.hpp），
        而我在同一行里把 `kBridgeIdA` 也写成了 `air::kBridgeIdA` ——
        它其实定义在 `dzmission::`（mission_command.hpp）。
        "同一行里一个带前缀、一个不带" 是很容易犯的错。

  这类错误完全可以在编译前静态发现：把项目里所有 `air::X` / `dzmission::X`
  形式的限定引用抽出来，核对 X 是否真的在那个命名空间里声明。

检查：
  1. 所有 `air::NAME` 引用 -> NAME 是否在 air 命名空间声明
  2. 所有 `dzmission::NAME` 引用 -> NAME 是否在 dzmission 命名空间（非 air）声明
  3. 报告"应改成什么样"（附声明位置）

运行:  python3 ns_check.py
"""

import io
import os
import re
import sys

ROOT = r'C:\Users\Ran\Desktop\无人车\src-car\dzmission'


def sec(t):
    print()
    print("=" * 76)
    print(t)
    print("=" * 76)


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//[^\n]*', ' ', text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    return text


def find_namespace_ranges(code):
    """返回 [(ns_name, start, end)] —— 每个 namespace 的名字与内容范围"""
    out = []
    for m in re.finditer(r'\bnamespace\s+([A-Za-z_]\w*)\s*\{', code):
        ns = m.group(1)
        i = m.end() - 1
        depth = 0
        while i < len(code):
            if code[i] == '{':
                depth += 1
            elif code[i] == '}':
                depth -= 1
                if depth == 0:
                    out.append((ns, m.end(), i))
                    break
            i += 1
    return out


def collect_decls():
    """返回 {namespace: {名字: (文件, 行号)}}"""
    decls = {}
    for sub in ('include', 'src'):
        base = os.path.join(ROOT, sub)
        for dp, _dn, fn in os.walk(base):
            for f in fn:
                if not f.endswith(('.hpp', '.h', '.cpp')):
                    continue
                p = os.path.join(dp, f)
                raw = io.open(p, encoding='utf-8', errors='ignore').read()
                code = strip_comments(raw)
                rel = os.path.relpath(p, ROOT)
                for (ns, a, b) in find_namespace_ranges(code):
                    body = code[a:b]
                    line0 = raw[:a].count('\n') + 1
                    # constexpr / const / enum 值 / 函数 / 类型
                    for m in re.finditer(
                            r'(?:constexpr|const|static\s+constexpr)\s+'
                            r'[\w:<>]+\s+([A-Za-z_]\w*)\s*[=({;]', body):
                        decls.setdefault(ns, {}).setdefault(
                            m.group(1), (rel, line0 + body[:m.start()].count('\n')))
                    for m in re.finditer(r'\benum\s+(?:class\s+)?(\w+)', body):
                        decls.setdefault(ns, {}).setdefault(
                            m.group(1), (rel, line0 + body[:m.start()].count('\n')))
    return decls


def main():
    print("限定名引用检查（air:: / dzmission:: 前缀用对没有）")
    print("★ 本机无编译器；本工具查'限定名写错命名空间'这类必然编译失败的错误")

    decls = collect_decls()

    sec("1. 各命名空间里声明的常量/类型（节选）")
    for ns in sorted(decls):
        names = sorted(decls[ns])
        print("  %-14s 共 %d 个" % (ns + "::", len(names)))
        if ns == 'air':
            print("      " + ", ".join(n for n in names if n.startswith('kEv'))[:150])
        if ns == 'dzmission':
            print("      " + ", ".join(n for n in names if n.startswith('k'))[:150])
    print()

    # 扫描所有源文件里的限定引用
    sec("2. 逐个核对限定引用")

    problems = []
    checked = 0
    for sub in ('include', 'src'):
        base = os.path.join(ROOT, sub)
        for dp, _dn, fn in os.walk(base):
            for f in fn:
                if not f.endswith(('.hpp', '.h', '.cpp')):
                    continue
                p = os.path.join(dp, f)
                raw = io.open(p, encoding='utf-8', errors='ignore').read()
                code = strip_comments(raw)
                rel = os.path.relpath(p, ROOT)

                for m in re.finditer(r'\b(air|dzmission)::([A-Za-z_]\w*)', code):
                    ns, name = m.group(1), m.group(2)
                    # 跳过 std:: 之类、跳过类型名（msg/srv 命名空间常为外部）
                    if name in ('msg', 'srv', 'action', 'enc'):
                        continue
                    checked += 1
                    if name in decls.get(ns, {}):
                        continue
                    # 找它实际在哪个命名空间
                    others = [o for o in decls if name in decls[o]]
                    if others:
                        ln = raw[:m.start()].count('\n') + 1
                        loc = decls[others[0]][name]
                        problems.append(
                            "%s:%d  %s::%s -> 实际在 %s::%s（%s:%d），"
                            "应写成 %s::%s"
                            % (rel, ln, ns, name, others[0], name,
                               loc[0], loc[1], others[0], name))
                    else:
                        # 可能是外部符号（ROS 等），只做低优先级提示
                        pass

    if problems:
        seen = set()
        for pr in problems:
            if pr in seen:
                continue
            seen.add(pr)
            print("  [XX] %s" % pr)
    else:
        print("  ✓ 未发现限定名用错命名空间")

    sec("汇总")
    print("  核对 %d 处限定引用，问题 %d 条" % (checked, len(problems)))
    if not problems:
        print("  ✓ 通过")
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
