#!/usr/bin/env python3
"""成员变量声明核对 + include 自洽检查（比 cpp_consistency_check.py 更保守、误报更少）

思路：
  - 头文件里凡是形如 `Type name_;` 或 `Type name_{...};` 或 `Type name_ = ...;`
    的语句都视为成员变量声明（不论访问修饰符）。
  - .cpp 里凡是形如 `name_` 的标识符都视为成员引用。
  - 前者必须覆盖后者，否则编译必然报 "was not declared in this scope"。

  另外对每个 .hpp/.cpp 单独检查常见类型是否缺少直接 include。
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

UNITS = [
    ('include/dzmission/mission_manager.hpp', ['src/mission_manager.cpp']),
    ('include/dzmission/cmd_link.hpp', ['src/cmd_link.cpp']),
    ('include/dzmission/motion_arbiter.hpp', ['src/motion_arbiter.cpp']),
    ('include/dzmission/safety_monitor.hpp', ['src/safety_monitor.cpp']),
    ('include/dzmission/target_aimer.hpp', ['src/target_aimer.cpp']),
    ('include/dzmission/referee_board.hpp', ['src/referee_board.cpp']),
    ('include/dzmission/localization_compensator.hpp', ['src/localization_compensator.cpp']),
    ('include/dzmission/eskf.hpp', ['src/eskf.cpp']),
    ('include/dzmission/field_grid.hpp', ['src/field_grid.cpp']),
    ('include/dzmission/mine_planner.hpp', ['src/mine_planner.cpp']),
    ('include/dzmission/pivot_controller.hpp', ['src/pivot_controller.cpp']),
]

TYPE_PREFIX = (
    r'(?:mutable\s+|static\s+|const\s+|constexpr\s+)*'
    r'(?:unsigned\s+)?'
    r'(?:int|long|short|char|bool|float|double|size_t|'
    r'uint8_t|uint16_t|uint32_t|uint64_t|int8_t|int16_t|int32_t|int64_t|'
    r'std::[\w:]+|rclcpp::[\w:]+|geometry_msgs::[\w:]+|nav_msgs::[\w:]+|'
    r'sensor_msgs::[\w:]+|std_msgs::[\w:]+|nav2_msgs::[\w:]+|Eigen::[\w:]+|'
    r'tf2[\w:]*|serial::[\w:]+|\w+)(?:\s*<[^;{}]*>)?'
)

INCLUDE_HINTS = [
    (r'\bstd::atomic\b', '<atomic>'),
    (r'\bstd::thread\b', '<thread>'),
    (r'\bstd::mutex\b|\bstd::lock_guard\b|\bstd::unique_lock\b', '<mutex>'),
    (r'\bstd::function\b', '<functional>'),
    (r'\bstd::vector\b', '<vector>'),
    (r'\bstd::string\b', '<string>'),
    (r'\bstd::array\b', '<array>'),
    (r'\bstd::deque\b', '<deque>'),
    (r'\bstd::unique_ptr\b|\bstd::shared_ptr\b|\bstd::make_unique\b|\bstd::make_shared\b', '<memory>'),
    (r'\bstd::clamp\b|\bstd::lround\b|\bstd::hypot\b|\bstd::isfinite\b|\bstd::atan2\b|\bM_PI\b', '<cmath>'),
    (r'\bstd::memcpy\b|\bstd::memset\b|\bstd::strerror\b', '<cstring>'),
    (r'\bstd::min\b|\bstd::max\b|\bstd::copy\b|\bstd::clamp\b', '<algorithm>'),
    (r'\bstd::chrono\b', '<chrono>'),
    (r'\bEigen::', '<Eigen/Dense>'),
]


def read(rel):
    with open(os.path.join(ROOT, rel), encoding='utf-8', errors='replace') as f:
        return f.read()


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)
    text = re.sub(r'//[^\n]*', '', text)
    return text


def strip_strings(text):
    return re.sub(r'"(?:[^"\\]|\\.)*"', '""', text)


def declared_members(htext):
    """头文件中所有 `Type name_;` 形式的成员名"""
    names = set()
    # 逐行找以 name_ 结尾的声明
    for m in re.finditer(
            r'(?:^|[;{}]\s*)'
            r'((?:mutable\s+|static\s+|const\s+)*[\w:<>,\.\s\*&]+?)\s+'      # 类型
            r'(\w+_)\s*'                                                     # 名字
            r'(?:\{[^{}]*\}|=\s*[^;{}]+)?\s*;', htext, flags=re.M):
        names.add(m.group(2))
    # 兼容 `Type name_{init};`（聚合初始化）
    for m in re.finditer(r'(\w+_)\s*\{[^{}]*\}\s*;', htext):
        names.add(m.group(1))
    return names


def used_members(ctext):
    """忽略字符串/注释后，.cpp 中出现的 name_ 标识符"""
    return set(re.findall(r'\b([a-zA-Z]\w*_)\b', ctext))


def check_includes(name, text):
    problems = []
    incs = re.findall(r'#include\s*[<"]([^>"]+)[>"]', text)
    # 同时比较完整路径与 basename，兼容 <Eigen/Dense> 这类带目录的 include
    incs_full = set(incs)
    incs_base = {i.split('/')[-1] for i in incs}
    for pat, need in INCLUDE_HINTS:
        if re.search(pat, text):
            core = need.strip('<>')
            if core in incs_full or core in incs_base:
                continue
            problems.append('%s: 用到 %s 但未直接 #include %s'
                            % (os.path.basename(name), pat, need))
    return problems


def main():
    total = 0
    for header, cpps in UNITS:
        htext_raw = read(header)
        htext = strip_comments(htext_raw)
        decl = declared_members(htext)

        ctext = ''
        for c in cpps:
            ctext += strip_strings(strip_comments(read(c))) + '\n'
        used = used_members(ctext)

        # 过滤掉 .cpp 内部声明的局部变量（同名的 name_ 局部变量很少，这里保守处理）
        missing = sorted(u for u in used - decl)

        probs = []
        for u in missing:
            # 再确认：如果该名字在本 cpp 里被显式声明/赋值过，视为局部变量
            if re.search(r'\b(?:auto|int|double|bool|float|size_t|uint\w*|int\w*|std::\w+)\s*&?\s*'
                         + re.escape(u) + r'\b', ctext):
                continue
            probs.append('使用了头文件未声明的成员: %s' % u)

        probs += check_includes(header, htext_raw)
        for c in cpps:
            probs += check_includes(c, read(c))

        status = 'OK' if not probs else '%d 项' % len(probs)
        print('%-52s %s' % (os.path.basename(header), status))
        for p in probs:
            print('    - %s' % p)
        total += len(probs)

    # 独立 TU 检查
    for c in ['src/mission_command.cpp', 'src/mission_main.cpp']:
        probs = check_includes(c, read(c))
        status = 'OK' if not probs else '%d 项' % len(probs)
        print('%-52s %s' % (os.path.basename(c), status))
        for p in probs:
            print('    - %s' % p)
        total += len(probs)

    print()
    print('总计 %d 个问题' % total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
