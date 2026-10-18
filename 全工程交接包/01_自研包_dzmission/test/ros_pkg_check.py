#!/usr/bin/env python3
"""ROS 2 包结构自检（本机可跑，不需要编译）

★ 为什么需要它：
  colcon 在**编译前**会先检查包的"声明一致性"。不一致时的报错往往很隐晦，
  例如本轮真实遇到的：

      Failed to find the following files:
      - install/serial/share/serial/package.sh
      Check that the following packages have been built:
      - serial

  根因是 package.xml 声明 `<build_type>ament_cmake</build_type>`，
  而 CMakeLists 里 `ament_package()` 被写成**条件调用**
  （find_package(ament_cmake QUIET) + if(...)），没被执行 ->
  不会生成 package.sh -> colcon 找不到 -> 该包安装目录为空 ->
  依赖它的包也编不过。

  这类"声明与实现不一致"完全可以在编译前静态发现。

检查项：
  1. package.xml 的 <build_type> 与 CMakeLists 的构建方式是否一致
  2. 声明 ament 的包，是否**无条件**调用 ament_package()
  3. CMakeLists 的 project() 名与 package.xml 的 <name> 是否一致
  4. <buildtool_depend> 是否包含 ament_cmake（声明 ament 时）
  5. CMakeLists 引用的源文件是否都存在
  6. install() 引用的文件是否都存在
  7. 声明了 <depend> 但环境里找不到的包（ROS 1 残留 / 拼写错误）

运行:  python3 ros_pkg_check.py [工作空间根]
"""

import os
import re
import sys

# 已知 ROS 1 才有的依赖（ROS 2 里不存在，声明了必然构建失败）
ROS1_ONLY = {
    'roscpp', 'rospy', 'roslib', 'std_msgs', 'tf', 'actionlib',
    'message_generation', 'message_runtime', 'catkin', 'rosbuild',
    'nav_msgs', 'sensor_msgs', 'geometry_msgs',  # 注意：这几个 ROS 2 也有，见 SPECIAL
}
# 上面几个在 ROS 2 里也存在，不能一概而论 -> 用单独清单
ROS1_REAL = {'roscpp', 'rospy', 'roslib', 'tf', 'actionlib',
             'message_generation', 'message_runtime', 'catkin', 'rosbuild'}

# 已确认不存在的包名（拼写错误）
KNOWN_TYPOS = {
    'rclcib': 'rclcpp',
    'rclcpp_action_msgs': 'rclcpp_action',
    'nav2_msg': 'nav2_msgs',
}


def sec(t):
    print()
    print("=" * 76)
    print(t)
    print("=" * 76)


def strip_cmake_comments(text):
    return re.sub(r'#[^\n]*', '', text)


def read(path):
    return open(path, encoding='utf-8', errors='ignore').read()


def parse_package_xml(text, path=None):
    """解析 package.xml

    ★★ 为什么必须用 ElementTree 而不是正则 ★★
      我最初用正则抓 <depend>...</depend>，踩了两个坑：
        坑 1：`<depend>` 会匹配到 `<build_depend>` / `<exec_depend>` 的**尾部**
              -> 把 buildtool_depend 的 ament_cmake 也误当成普通依赖
        坑 2（更隐蔽）：**注释里的包名也被当成真依赖**
              -> 我删掉依赖后留了说明性注释（"删除了 <depend>actionlib</depend>"），
                 检查器仍报 actionlib 存在，让我误以为文件没改成功
      改用 ElementTree 后两个问题都自然消失：它按 XML 语法解析，
      注释是独立节点，标签名也不会互相包含。
    """
    out = {'name': None, 'build_type': None, 'buildtool': [], 'depend': []}

    if path and os.path.isfile(path):
        try:
            import xml.etree.ElementTree as ET
            root = ET.parse(path).getroot()
        except Exception:
            root = None
        if root is not None:
            name = root.findtext('name')
            out['name'] = name.strip() if name else None
            bt = root.findtext('build_type')
            out['build_type'] = bt.strip() if bt else None
            DEP_TAGS = ('depend', 'build_depend', 'exec_depend',
                        'build_export_depend')
            for e in root:
                # ★ 只认真实元素；注释在 ElementTree 里不是 Element，天然被跳过
                if not isinstance(e.tag, str):
                    continue
                v = (e.text or '').strip()
                if not v:
                    continue
                if e.tag == 'buildtool_depend':
                    out['buildtool'].append(v)
                elif e.tag == 'build_depend':
                    out['buildtool'].append(v)
                    out['depend'].append(v)
                elif e.tag in DEP_TAGS:
                    out['depend'].append(v)
            return out

    # 退化路径：没有文件路径时用带锚定的正则（避免坑 1）
    m = re.search(r'<name>\s*([^<\s]+)\s*</name>', text)
    if m:
        out['name'] = m.group(1)
    m = re.search(r'<build_type>\s*([^<\s]+)\s*</build_type>', text)
    if m:
        out['build_type'] = m.group(1)
    return out


def check_pkg(pkg_dir):
    """返回 (pkg_name, [问题...], [提示...])"""
    problems = []
    notes = []

    px = os.path.join(pkg_dir, 'package.xml')
    cm = os.path.join(pkg_dir, 'CMakeLists.txt')
    if not os.path.isfile(px):
        return (os.path.basename(pkg_dir),
                ['缺 package.xml（colcon 会跳过该目录）'], notes)

    pxml = parse_package_xml(read(px), px)
    name = pxml['name'] or os.path.basename(pkg_dir)

    if not os.path.isfile(cm):
        # 纯 Python 包也可能没有 CMakeLists
        if pxml['build_type'] in (None, 'ament_python'):
            notes.append('无 CMakeLists.txt（Python 包正常）')
            return name, problems, notes
        problems.append('缺 CMakeLists.txt，但 build_type 是 %s' % pxml['build_type'])
        return name, problems, notes

    code = strip_cmake_comments(read(cm))

    # ---- 1. build_type 与 CMakeLists 是否一致 ----
    is_ament = pxml['build_type'] == 'ament_cmake'
    calls_ament_package = 'ament_package(' in code
    if is_ament and not calls_ament_package:
        problems.append(
            'package.xml 声明 ament_cmake，但 CMakeLists **没有** ament_package() '
            '-> 不会生成 package.sh，colcon 会报 '
            '"Failed to find .../share/%s/package.sh"' % name)

    # ---- 2. ament_package() 是否被条件包裹 ----
    if calls_ament_package:
        # 找 ament_package 之前最近的 if(/endif 配对情况
        idx = code.find('ament_package(')
        head = code[:idx]
        n_if = len(re.findall(r'\bif\s*\(', head))
        n_endif = len(re.findall(r'\bendif\s*\(', head))
        n_else = len(re.findall(r'\belse\s*\(', head))
        # 粗略判断：if 多于 endif 说明 ament_package 位于某个未闭合的 if 内
        if n_if > n_endif:
            problems.append(
                'ament_package() 被包在条件分支里（if 未闭合数 %d）'
                '-> 可能不执行，导致同上错误。应**无条件**调用'
                % (n_if - n_endif))
        if 'ament_cmake_FOUND' in code or 'ament_cmake QUIET' in code:
            problems.append(
                'find_package(ament_cmake QUIET) 或 ament_cmake_FOUND 条件存在 '
                '-> 与 package.xml 的 ament_cmake 声明矛盾，应改为 REQUIRED 且无条件')

    # ---- 3. project() 名 vs package.xml name ----
    m = re.search(r'project\s*\(\s*([A-Za-z0-9_\-]+)', code)
    if m:
        proj = m.group(1)
        if pxml['name'] and proj != pxml['name']:
            problems.append('CMakeLists project() = %s，package.xml <name> = %s '
                            '-> ament install 路径不匹配' % (proj, pxml['name']))
    else:
        if is_ament:
            problems.append('未找到 project() 调用')

    # ---- 4. ament 包应有 ament_cmake buildtool ----
    if is_ament and 'ament_cmake' not in pxml['buildtool']:
        problems.append('声明 ament_cmake 但缺 <buildtool_depend>ament_cmake</buildtool_depend>')

    # ---- 5. add_executable/add_library 的源文件是否存在 ----
    for m in re.finditer(r'add_(?:executable|library)\s*\(\s*(\w+)([^)]*)\)', code):
        body = m.group(2)
        for f in re.findall(r'([A-Za-z_][\w/.\-]*\.(?:cpp|cc|cxx|hpp|h))', body):
            if not os.path.isfile(os.path.join(pkg_dir, f)):
                problems.append('源文件不存在: %s（被 %s 引用）' % (f, m.group(1)))

    # ---- 6. install() 引用的文件 ----
    for m in re.finditer(r'install\s*\(\s*(PROGRAMS|FILES|DIRECTORY)\s+([^)]*)\)', code):
        kind, body = m.group(1), m.group(2)
        for f in re.findall(r'([A-Za-z_][\w/.\-]*\.(?:py|yaml|yml|md|txt|xml|json|csv))', body):
            if not os.path.isfile(os.path.join(pkg_dir, f)):
                problems.append('install(%s) 引用的文件不存在: %s' % (kind, f))

    # ---- 7. 依赖里的 ROS 1 残留 / 拼写错误 ----
    for d in sorted(set(pxml['depend'])):
        if d in ROS1_REAL:
            problems.append(
                '依赖 %s 是 **ROS 1 的包**，ROS 2 里不存在 -> 该包无法构建。'
                '应删除（若代码未使用）' % d)
        if d in KNOWN_TYPOS:
            problems.append(
                '依赖 %s 疑似拼写错误，应为 %s' % (d, KNOWN_TYPOS[d]))

    return name, problems, notes


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else \
        r'C:\Users\Ran\Desktop\无人车\src-car'

    print("ROS 2 包结构自检")
    print("★ 本机无编译器；本工具查'声明与实现不一致'这类编译前就会被 colcon 拦下的问题")
    print("  典型：package.xml 说自己是 ament 包，CMakeLists 却没调 ament_package()")
    print()

    pkgs = []
    for e in sorted(os.listdir(root)):
        d = os.path.join(root, e)
        if os.path.isdir(d) and os.path.isfile(os.path.join(d, 'package.xml')):
            pkgs.append(d)

    # 第三方大包单独归类，避免噪音
    THIRD_PARTY = ('navigation2', 'rplidar_ros-ros2',
                   'robot_localization-humble-devel', 'slam_gmapping-eloquent-devel')

    sec("1. 本工作空间的包")
    total_p = 0
    bad = 0
    for d in pkgs:
        name = os.path.basename(d)
        is_tp = any(name.startswith(t) for t in THIRD_PARTY)
        pname, probs, notes = check_pkg(d)
        if probs:
            bad += 1
            mark = "  [XX]"
        else:
            mark = "  [OK]"
        tag = "  (第三方总包，仅供参考)" if is_tp else ""
        print("%s %-34s%s" % (mark, pname, tag))
        for p in probs:
            total_p += 1
            print("        - %s" % p)
        for n in notes:
            print("        · %s" % n)

    sec("汇总")
    print("  检查 %d 个包，有问题 %d 个（共 %d 条）" % (len(pkgs), bad, total_p))
    if bad == 0:
        print("  ✓ 包结构自检通过")
    else:
        print("  ✗ 需修复（见上）。带 (第三方总包) 标记的可暂时忽略。")
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
