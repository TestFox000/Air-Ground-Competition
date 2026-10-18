#!/usr/bin/env python3
"""读/写辅助函数完整性检查（本机可跑，不需要编译器）

★ 为什么需要它：
  本轮真实编译错误：
      mission_command.cpp:238: error: 'rdU32' was not declared in this scope;
                                did you mean 'rdI32'?
  根因：mission_command.hpp 里有一组小端读辅助
        rdU16 / rdI16 / rdI32 / rdBearingDeg / rdRangeCm
        —— 唯独漏了 rdU32，而 .cpp 里用它读时间戳。
        即"用了没声明的函数"。

  ★ 为什么 cpp_callsite_check.py 没抓到：
    那个工具查的是"**声明了**的函数被用错参数个数"，
    而这里是"**根本没声明**"，属于另一种缺陷。

  这类错误完全可以在编译前静态发现：把项目里所有 `xxU16/xxI32/...` 形式的
  调用抽出来，核对同名函数是否在某个头文件里声明过。

检查：
  1. 收集项目内所有函数声明/定义名（头文件 + 源文件）
  2. 抽出所有"看起来像项目内辅助函数"的调用（rd*/wr*/to*/from* 等前缀）
  3. 报告"被调用但任何地方都没有声明"的名字

为什么限定前缀而不是全量函数：
  全量分析需要真正的 C++ 解析器（成员函数、模板、重载、ADL……），
  用正则做会产生大量误报。限定在"项目自有的小工具函数命名族"上，
  判据明确、误报率低 —— 与 atomic_check.py 同样的取舍。
"""

import io
import os
import re
import sys

ROOT = r'C:\Users\Ran\Desktop\无人车\src-car\dzmission'

# 只检查这些命名族（项目自有小端读/写工具函数）
# ★★ 为什么只留 rd/wr，去掉了 enc/dec ★★
#   第一版把 enc/dec 也纳入，结果全是误报：
#     · declare_parameter 是 **ROS 成员函数**（this-> 可省略），正则分不清
#     · encodeMissionCommand 等**确实在头文件里声明了**，只是返回类型
#       是 `size_t`，我的类型正则没覆盖到 -> 被误判成"未声明"
#   教训与 atomic_check.py 相同：**正则做不了 C++ 的名字解析**。
#   rd/wr 族是项目自有的小工具函数（约定前缀、返回类型简单），
#   判据明确；enc/dec 混合了成员函数与自由函数，交给编译器更准。
PREFIXES = ('rd', 'wr')

# 这些不是函数，或来自外部库，跳过
WHITELIST = {
    'read', 'write', 'rdbuf', 'reason', 'recovery', 'reset', 'remove',
    'real', 'red', 'referee', 'region', 'record', 'recv',
    'wr', 'rd',
}

# 声明处可能的前导类型关键字（用于区分"声明"与"调用"）
TYPE_RE = (r'(?:void|bool|char|short|long|int|unsigned|signed|float|double|'
           r'size_t|auto|'
           r'u?int(?:8|16|32|64)_t|'
           r'std::\w+(?:::\w+)*|'
           r'const\s+\w+|[A-Z]\w*)')


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


def collect_files():
    out = []
    for sub in ('include', 'src'):
        base = os.path.join(ROOT, sub)
        for dp, _dn, fn in os.walk(base):
            for f in fn:
                if f.endswith(('.hpp', '.h', '.cpp', '.cc')):
                    out.append(os.path.join(dp, f))
    return out


def main():
    print("读/写辅助函数完整性检查")
    print("★ 本机无编译器；本工具查'调用了但从未声明'的函数（本轮 rdU32 的错误形态）")

    files = collect_files()
    if not files:
        print("找不到源文件: %s" % ROOT)
        return 1

    declared = set()
    called = {}          # 名字 -> [(文件, 行号), ...]

    for p in files:
        raw = io.open(p, encoding='utf-8', errors='ignore').read()
        code = strip_comments(raw)
        rel = os.path.relpath(p, ROOT)

        # 1) 收集声明/定义：形如 `类型 名字(` 或 `名字(`
        for m in re.finditer(r'\b([A-Za-z_]\w*)\s*\(', code):
            name = m.group(1)
            if not name.startswith(PREFIXES):
                continue
            # 判断是否为"声明/定义"而非"调用"：
            # 往前看 40 字符里是否有类型名（说明是声明/定义）
            before = code[max(0, m.start() - 40):m.start()]
            if re.search(r'(?:^|[;{}\s])(?:inline\s+|static\s+|constexpr\s+)?'
                         r'(?:void|bool|int|uint\d+_t|int\d+_t|float|double|'
                         r'std::\w+|auto|[A-Z]\w*)\s+$', before):
                declared.add(name)
        # 头文件里的 inline 定义也算（上面的规则已覆盖）

        # 2) 收集调用（排除声明行本身）
        for m in re.finditer(r'(?<![\w.:>])([A-Za-z_]\w*)\s*\(', code):
            name = m.group(1)
            if not name.startswith(PREFIXES):
                continue
            if name in WHITELIST:
                continue
            # 排除声明/定义（往前是类型名）
            before = code[max(0, m.start() - 40):m.start()]
            if re.search(r'(?:^|[;{}\s])(?:inline\s+|static\s+|constexpr\s+)?'
                         r'(?:void|bool|int|uint\d+_t|int\d+_t|float|double|'
                         r'std::\w+|auto|[A-Z]\w*)\s+$', before):
                continue
            ln = raw[:m.start()].count('\n') + 1
            called.setdefault(name, []).append((rel, ln))

    sec("1. 项目内声明/定义的小工具函数")
    print("  " + ", ".join(sorted(declared)))

    sec("2. 被调用但**任何地方都没有声明**的")
    missing = {n: v for n, v in called.items() if n not in declared}
    if missing:
        for n in sorted(missing):
            locs = ", ".join("%s:%d" % (f, l) for (f, l) in missing[n][:4])
            print("  [XX] %-16s 未声明，调用处: %s" % (n, locs))
            # 找相似名字给建议
            import difflib
            near = difflib.get_close_matches(n, sorted(declared), n=3, cutoff=0.6)
            if near:
                print("       也许你想用: %s" % ", ".join(near))
    else:
        print("  ✓ 未发现未声明的辅助函数调用")

    sec("汇总")
    print("  声明 %d 个，调用 %d 个不同名字，缺失 %d 个"
          % (len(declared), len(called), len(missing)))
    if not missing:
        print("  ✓ 通过")
    return 0 if not missing else 1


if __name__ == '__main__':
    sys.exit(main())
