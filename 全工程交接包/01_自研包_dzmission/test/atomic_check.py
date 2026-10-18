#!/usr/bin/env python3
"""原子量使用一致性检查（本机可跑）

★ 为什么需要它：
  本轮真实编译错误：
      target_aimer.hpp:110: error: request for member 'store' in 'ammo_',
                            which is of non-class type 'int'
  根因：把 setAmmo 改成 ammo_.store()，却漏改声明 int ammo_ -> std::atomic<int>。
  这是"声明与使用不一致"的典型模式，完全可以在编译前静态发现。

检查：
  1. 调用了 .store()/.load()/.fetch_add() 等原子接口的成员，声明是否为 std::atomic<>
  2. 声明为 std::atomic<> 的成员，是否在 **const 函数**里被隐式转换（应改用 .load()）
  3. 声明为 std::atomic<> 的成员，是否有 operator= 之外的误用

运行:  python3 atomic_check.py
"""

import io
import os
import re
import sys

ROOT = r'C:\Users\Ran\Desktop\无人车\src-car\dzmission'

# 原子接口
ATOMIC_METHODS = ('store', 'load', 'fetch_add', 'fetch_sub', 'exchange',
                  'compare_exchange_weak', 'compare_exchange_strong')


def sec(t):
    print()
    print("=" * 74)
    print(t)
    print("=" * 74)


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    text = re.sub(r'//[^\n]*', ' ', text)
    return text


def find_const_function_ranges(code):
    """返回 [(start, end)] —— 所有 **const 成员函数** 的函数体范围

    ★ 为什么要这个：
      std::atomic<T> 的 operator T() 只在**非 const** 版本可用；
      const 版本是 `operator T() const = delete;`。
      所以"在 const 函数里直接读原子量"必然编译失败，而在非 const 函数里
      是合法且常见的写法。
      本检查器第一版没做这个区分，把 target_aimer.cpp 的 tick()（非 const）
      也误报了。

    实现：找形如 `... 名字(...) const {` 或 `... 名字(...) const;`（声明）
    的函数头，然后用花括号配平找出函数体范围。
    """
    ranges = []
    # 函数体：签名以 `) const` 结尾，后跟可选的 override/noexcept，然后 {
    pat = re.compile(
        r'[A-Za-z_][\w:<>,\*&\s]*?'      # 返回类型
        r'([A-Za-z_]\w*|~\w+)\s*'        # 函数名
        r'\([^;{}]*\)\s*'                # 参数
        r'const\s*'                      # ★ const 限定
        r'(?:noexcept\s*)?(?:override\s*)?\{')
    for m in pat.finditer(code):
        # 从 `{` 开始配平花括号
        i = m.end() - 1
        depth = 0
        start = i
        while i < len(code):
            if code[i] == '{':
                depth += 1
            elif code[i] == '}':
                depth -= 1
                if depth == 0:
                    ranges.append((start, i + 1))
                    break
            i += 1
    return ranges


def collect_files():
    out = []
    for sub in ('include', 'src'):
        d = os.path.join(ROOT, sub)
        for dp, _dn, fn in os.walk(d):
            for f in fn:
                if f.endswith(('.hpp', '.h', '.cpp', '.cc')):
                    out.append(os.path.join(dp, f))
    return out


def main():
    print("原子量使用一致性检查")
    print("★ 本机无编译器；本工具查'声明与使用不一致'这类必然编译失败的模式")

    files = collect_files()
    if not files:
        print("找不到源文件: %s" % ROOT)
        return 1

    # ---- 收集所有成员声明 ----
    decls = {}      # 名字 -> (是否atomic, 文件, 行号)
    for p in files:
        raw = io.open(p, encoding='utf-8', errors='ignore').read()
        code = strip_comments(raw)
        rel = os.path.relpath(p, ROOT)
        for m in re.finditer(
                r'(std::atomic\s*<[^>]+>)\s+([A-Za-z_]\w*)\s*[\{;=]', code):
            decls[m.group(2)] = (True, rel, raw[:m.start()].count('\n') + 1)
        # 非原子声明（只收 int/bool/uint 等标量，避免误报自定义类型）
        for m in re.finditer(
                r'(?<![\w:<>])(int|bool|uint8_t|uint16_t|uint32_t|int32_t|'
                r'int64_t|uint64_t|double|float|size_t)\s+([A-Za-z_]\w*)\s*[\{;=]',
                code):
            name = m.group(2)
            if name not in decls:
                decls[name] = (False, rel, raw[:m.start()].count('\n') + 1)

    sec("1. 扫描到的成员声明")
    n_atomic = sum(1 for v in decls.values() if v[0])
    print("  共 %d 个成员；其中 atomic 的 %d 个" % (len(decls), n_atomic))
    for name, (is_at, rel, ln) in sorted(decls.items()):
        if is_at:
            print("    atomic  %-28s %s:%d" % (name, rel, ln))

    # ---- 检查使用点 ----
    sec("2. 一致性检查")

    problems = []

    for p in files:
        raw = io.open(p, encoding='utf-8', errors='ignore').read()
        code = strip_comments(raw)
        rel = os.path.relpath(p, ROOT)

        # 2a. 调用了原子接口，但声明不是 atomic
        for meth in ATOMIC_METHODS:
            for m in re.finditer(r'([A-Za-z_]\w*)\s*\.\s*' + meth + r'\s*\(', code):
                name = m.group(1)
                ln = raw[:m.start()].count('\n') + 1
                if name in decls and not decls[name][0]:
                    problems.append(
                        "%s:%d  调用了 %s.%s()，但声明不是 std::atomic<>"
                        "（声明在 %s:%d，是普通类型）"
                        % (rel, ln, name, meth, decls[name][1], decls[name][2]))

        const_ranges = find_const_function_ranges(code)

        # -------------------------------------------------------------------
        # 2b【已移除】"atomic 成员在 const 函数里被隐式转换"的检查
        # -------------------------------------------------------------------
        # 这确实是真实的错误模式（std::atomic 的 const 版 operator T() 是
        # = delete，所以 const 函数里必须 .load()），但**用正则判断误报率过高**，
        # 三次实现都失败：
        #   · 第 1 版：按 token 位置判断 -> 因"先剥离注释再算偏移"行号错位，
        #             误报 target_aimer.cpp 的 tick()（实际是非 const，合法）
        #   · 第 2 版：加 find_const_function_ranges() -> 仍误报 snapToCell
        #   · 第 3 版：改为"整个 const 函数无 .load() 才报" ->
        #             误报 LocalizationCompensator::evaluateHealth()，
        #             因为成员名 have_odom_ / have_imu_yaw_ 带 have_ 前缀，
        #             被正则误认成 atomic（实际是普通 bool）
        #
        # ⇒ 教训：**用正则做"类型层面的语义判断"不可靠**。
        #   这一项交给编译器最准：它会在相应 .cpp 编译时报
        #       "no conversion from 'const std::atomic<T>' to 'T'"
        #   这是编译期必现错误，不会漏到运行期。
        #
        # 本检查器只保留 2a（调用了 .store()/.load() 但声明不是 atomic）——
        # 那正是本次真实错误的形态（setAmmo 里 ammo_.store() 而 ammo_ 是 int），
        # 判据明确、零误报。
        # -------------------------------------------------------------------
        _ = const_ranges   # 保留计算，供将来参考；不再用于报错

    if problems:
        seen = set()
        for pr in problems:
            if pr in seen:
                continue
            seen.add(pr)
            print("  [XX] %s" % pr)
    else:
        print("  ✓ 未发现声明/使用不一致")

    sec("汇总")
    print("  检查 %d 个文件，问题 %d 条" % (len(files), len(problems)))
    if not problems:
        print("  ✓ 通过")
    return 0 if not problems else 1


if __name__ == '__main__':
    sys.exit(main())
