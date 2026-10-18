#!/usr/bin/env python3
"""C++ 头文件与 include 静态检查（本机可跑，不需要编译器）

★ 为什么需要它：
  本机（Windows 沙箱）没有 C++ 编译器，而**头文件冲突**与**缺失 include**
  是最常见的编译失败原因，且完全可以静态检查。
  本轮就是这么抓到 glibc <termios.h> 与内核 <asm/termbits.h>
  都定义 struct termios 的冲突。

能力边界：
  ✓ 能查：已知的头文件冲突组合、用了符号但没 include 对应头文件
  ✗ 不能查：模板实例化、类型推导、重载决议、链接错误
  => 减少上机试错，不能替代编译

运行:  python3 cpp_include_check.py [文件或目录...]
"""

import os
import re
import sys

# 已知会互相冲突的头文件组合（同包含 -> 编译错误）
CONFLICTS = [
    ('<termios.h>', '<asm/termbits.h>',
     "glibc 与内核头文件都定义 struct termios"),
    ('<termios.h>', '<linux/termios.h>',
     "用户态与内核态 termios 定义冲突"),
    ('<sys/ioctl.h>', '<linux/ioctl.h>',
     "ioctl 宏可能重复定义"),
    ('<sys/socket.h>', '<linux/socket.h>',
     "socket 结构可能冲突"),
    ('<netinet/in.h>', '<linux/in.h>',
     "in_addr 结构可能冲突"),
    ('<ifaddrs.h>', '<linux/if.h>',
     "ifaddrs 与内核 if 定义冲突"),
]

# =============================================================================
# ★ Linux/glibc 已经定义过的宏清单
# =============================================================================
# 用途：如果代码里出现
#       #ifndef XXX
#       #define XXX ...
#       #endif
#   而 XXX 已由系统头文件定义，那**我们的定义会被静默跳过**，
#   然后按系统的定义展开 —— 这会产生很难懂的编译错误。
#
# ★ 本清单来自实战：serial.cpp 曾这样写
#       #ifndef TCGETS2
#       #define TCGETS2 _IOR('T', 0x2A, struct serial_termios2)
#       #endif
#   但 glibc <bits/ioctls.h> 已定义 TCGETS2 并引用**不完整的**
#   `struct termios2` -> 报
#       invalid application of 'sizeof' to incomplete type 'struct termios2'
#   正确做法：用自己的宏名（如 DZ_TCGETS2）。
# =============================================================================
SYSTEM_MACROS = {
    # ioctl 请求码（<sys/ioctl.h> -> <bits/ioctls.h> 提供）
    'TCGETS2': "glibc 已定义，且引用不完整的 struct termios2",
    'TCSETS2': "glibc 已定义，且引用不完整的 struct termios2",
    'TCGETS': "glibc 已定义",
    'TCSETS': "glibc 已定义",
    'TCSETSW': "glibc 已定义",
    'TCSETSF': "glibc 已定义",
    'TCFLSH': "glibc 已定义",
    'TIOCEXCL': "glibc 已定义",
    'TIOCMGET': "glibc 已定义",
    'TIOCMSET': "glibc 已定义",
    # 串口标志
    'BOTHER': "内核/glibc 可能已定义",
    'CBAUD': "glibc termios.h 已定义",
    'CS8': "glibc termios.h 已定义",
    'CLOCAL': "glibc termios.h 已定义",
    'CREAD': "glibc termios.h 已定义",
    'CRTSCTS': "glibc termios.h 已定义",
    'PARENB': "glibc termios.h 已定义",
    'CSTOPB': "glibc termios.h 已定义",
    'VMIN': "glibc termios.h 已定义",
    'VTIME': "glibc termios.h 已定义",
    # 系统调用号
    '__NR_ioctl': "glibc <sys/syscall.h> 已定义",
}

# 符号 -> 需要的头文件（可多个候选，任一存在即通过）
NEEDS = {
    'std::this_thread': ['<thread>'],
    'std::chrono': ['<chrono>'],
    'std::vector': ['<vector>'],
    'std::string': ['<string>'],
    'std::exception': ['<exception>'],
    'std::array': ['<array>'],
    'std::function': ['<functional>'],
    'std::mutex': ['<mutex>'],
    'std::atomic': ['<atomic>'],
    'std::unique_ptr': ['<memory>'],
    'std::shared_ptr': ['<memory>'],
    'std::make_unique': ['<memory>'],
    'std::sort': ['<algorithm>'],
    'std::min': ['<algorithm>'],
    'std::max': ['<algorithm>'],
    'std::clamp': ['<algorithm>'],
    'std::hypot': ['<cmath>'],
    'std::abs': ['<cmath>', '<cstdlib>'],
    'std::round': ['<cmath>'],
    'std::floor': ['<cmath>'],
    'std::ceil': ['<cmath>'],
    'std::lround': ['<cmath>'],
    'std::printf': ['<cstdio>'],
    'std::snprintf': ['<cstdio>'],
    'std::atoi': ['<cstdlib>'],
    'std::stoi': ['<string>'],
    'std::to_string': ['<string>'],
    'memset': ['<string.h>', '<cstring>'],
    'memcpy': ['<string.h>', '<cstring>'],
    'strerror': ['<string.h>', '<cstring>'],
    'strlen': ['<string.h>', '<cstring>'],
    'ioctl': ['<sys/ioctl.h>'],
    'select': ['<sys/select.h>'],
    'tcgetattr': ['<termios.h>'],
    'tcsetattr': ['<termios.h>'],
    'cfsetispeed': ['<termios.h>'],
    'errno': ['<errno.h>', '<cerrno>'],
    'O_RDWR': ['<fcntl.h>'],
    'O_NONBLOCK': ['<fcntl.h>'],
    'F_GETFL': ['<fcntl.h>'],
    'usleep': ['<unistd.h>'],
    'getpid': ['<unistd.h>'],
}


def strip_comments_and_strings(text):
    """去掉注释与字符串字面量

    ★ 必须做这一步：否则注释里提到的 #include <asm/termbits.h>
      会被当成真的 include，产生**假阳性**（本检查器第一版就踩了）。
    """
    # 块注释
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    # 行注释
    text = re.sub(r'//[^\n]*', ' ', text)
    # 字符串字面量（保留占位，避免把里面的 #include 当代码）
    text = re.sub(r'"(?:\\.|[^"\\])*"', '""', text)
    text = re.sub(r"'(?:\\.|[^'\\])*'", "''", text)
    return text


def get_includes(code):
    return re.findall(r'#\s*include\s+(<[^>\n]+>|"[^"\n]+")', code)


def check_file(path):
    raw = open(path, encoding='utf-8', errors='ignore').read()
    code = strip_comments_and_strings(raw)
    incs = get_includes(code)
    # 规范化：带双引号的自定义头不算系统头
    sys_incs = [i for i in incs if i.startswith('<')]

    rel = os.path.basename(path)
    problems = []

    # 1) 冲突组合
    for a, b, why in CONFLICTS:
        if a in sys_incs and b in sys_incs:
            problems.append('头文件冲突: %s + %s —— %s' % (a, b, why))

    # 2) 用了符号但未 include
    for token, cands in NEEDS.items():
        if token not in code:
            continue
        if any(c in sys_incs for c in cands):
            continue
        problems.append('用到 %-22s 但未 include %s' % (token, ' 或 '.join(cands)))

    # 3) ★ 自查：对系统已定义的宏做 #ifndef 保护 -> 我们的定义会被静默跳过
    for mac, why in SYSTEM_MACROS.items():
        # 匹配  #ifndef MAC ... #define MAC
        pat = re.compile(r'#\s*ifndef\s+' + re.escape(mac) +
                         r'\b[\s\S]{0,400}?#\s*define\s+' + re.escape(mac) + r'\b')
        if pat.search(code):
            problems.append(
                '对系统宏 %-14s 做了 #ifndef+#define 保护 —— %s；'
                '应改用自有宏名（如 DZ_%s）' % (mac, why, mac))

    # 4) #pragma once / include guard 检查（仅头文件）
    if path.endswith(('.h', '.hpp')):
        if '#pragma once' not in code and '#ifndef' not in code[:600]:
            problems.append('头文件缺少 include guard（#pragma once 或 #ifndef）')

    return incs, problems


def main():
    args = sys.argv[1:]
    if not args:
        # 默认检查本仓库新增/改动的关键文件
        root = r'C:\Users\Ran\Desktop\无人车\src-car'
        args = [
            os.path.join(root, 'serial', 'src', 'serial.cpp'),
            os.path.join(root, 'serial', 'include', 'serial', 'serial.h'),
            os.path.join(root, 'serial', 'test', 'serial_selftest.cpp'),
            os.path.join(root, 'dzmission', 'src', 'bridge_tracker.cpp'),
            os.path.join(root, 'dzmission', 'include', 'dzmission', 'bridge_tracker.hpp'),
            os.path.join(root, 'dzmission', 'test', 'bridge_selftest.cpp'),
        ]

    targets = []
    for a in args:
        if os.path.isdir(a):
            for dp, _dn, fn in os.walk(a):
                for f in fn:
                    if f.endswith(('.cpp', '.cc', '.hpp', '.h')):
                        targets.append(os.path.join(dp, f))
        elif os.path.isfile(a):
            targets.append(a)

    print("C++ 头文件与 include 静态检查")
    print("★ 本机无编译器；本工具查'头文件冲突'与'缺失 include'这两类最常见错误")
    print()

    total = 0
    bad_files = 0
    for p in targets:
        incs, problems = check_file(p)
        total += 1
        name = os.path.basename(p)
        if problems:
            bad_files += 1
            print("  [XX] %s" % name)
            for pr in problems:
                print("        - %s" % pr)
        else:
            print("  [OK] %-34s (%d 个 include)" % (name, len(incs)))

    print()
    print("=" * 70)
    print(" 检查 %d 个文件，有问题 %d 个" % (total, bad_files))
    if bad_files == 0:
        print(" ✓ 未发现头文件冲突与缺失 include")
    else:
        print(" ✗ 需修复（见上）")
    print("=" * 70)
    return 0 if bad_files == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
