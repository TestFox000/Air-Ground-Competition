#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_car_wait_for_drone.py —— 让小车【起栈后原地待命，只等外部指令再动】

问题：`cd ~/dzrobot && ./start_all.sh start` 之后车会自己在 10 秒后出发
      （path_executor 用内置 34 点路线自动下发），现场来不及接管。

改动（全部在车上 ~/dzrobot）：
  1) config/robot.yaml
       mission.auto_start: true -> false          ← 默认待命
  2) src/car_localization/car_localization/path_executor.py
       · 常量区注释更新（说明 AUTO_START 的取值优先级）
       · 自动出发定时器：先算 DZ_AUTO_START 环境变量覆盖，再打印★横幅
         DZ_AUTO_START=1 -> 强制开启（比赛一键出发，起栈 10s 后自动下发内置路线）
         DZ_AUTO_START=0 -> 强制关闭（待命）
         未设置          -> 用 YAML 的 mission.auto_start（打完补丁后 = false = 待命）

安全性设计：
  · 只做【精确整串替换】，每处断言命中恰好 1 次；命中数不符 -> 不写任何文件
  · 改前自动备份到 ~/dzrobot/_backups/autostart_wait_<时间戳>/
  · 改后自检：py_compile + yaml.safe_load；任一失败自动回滚
  · 幂等：重复运行会识别出已打过补丁并跳过

用法（在车上执行）：
    python3 patch_car_wait_for_drone.py            # 打补丁（默认）
    python3 patch_car_wait_for_drone.py --check    # 只看状态，一个字节都不写
    python3 patch_car_wait_for_drone.py --revert   # 从最近一次备份回滚
"""

import argparse
import difflib
import glob
import os
import py_compile
import shutil
import sys
import time

WS = os.path.expanduser('~/dzrobot')
PE = os.path.join(WS, 'src/car_localization/car_localization/path_executor.py')
YML = os.path.join(WS, 'config/robot.yaml')
BACKUP_ROOT = os.path.join(WS, '_backups')
MARKER = 'DZ_AUTO_START'

# ---------------------------------------------------------------------------
# 替换 1：常量区注释（说明取值优先级）
# ---------------------------------------------------------------------------
R1_OLD = (
    "AUTO_START = True           # 想纯手动就改成 False\n"
)
R1_NEW = (
    "AUTO_START = True           # 默认值；实际值被 config mission.auto_start 覆盖，\n"
    "                            #   环境变量 DZ_AUTO_START 优先级最高（=1 强制出发 / =0 强制待命）\n"
)

# ---------------------------------------------------------------------------
# 替换 2：自动出发定时器 —— 环境变量覆盖 + ★启动横幅
# ---------------------------------------------------------------------------
R2_OLD = (
    "        self._auto_started = False\n"
    "        self._auto_start_timer = None\n"
    "        if AUTO_START:\n"
    "            self._auto_start_timer = self.create_timer(\n"
    "                AUTO_START_DELAY, self._auto_start_mission)\n"
)
R2_NEW = (
    "        self._auto_started = False\n"
    "        self._auto_start_timer = None\n"
    "        # ★ 2026-10-07：起栈后是否自动出发，三级取值（后者覆盖前者）\n"
    "        #     1) 代码默认 / YAML mission.auto_start\n"
    "        #     2) 环境变量 DZ_AUTO_START: 1/true/yes/on=强制出发, 0/false/no/off=强制待命\n"
    "        #   默认待命时：车只认外部 /nav/waypoint，收到路线才动（不会自己跑）\n"
    "        _as_env = os.environ.get('DZ_AUTO_START', '').strip().lower()\n"
    "        _as_eff = AUTO_START\n"
    "        _as_src = 'config mission.auto_start'\n"
    "        if _as_env in ('1', 'true', 'yes', 'on'):\n"
    "            _as_eff = True\n"
    "            _as_src = 'env DZ_AUTO_START=' + _as_env\n"
    "        elif _as_env in ('0', 'false', 'no', 'off'):\n"
    "            _as_eff = False\n"
    "            _as_src = 'env DZ_AUTO_START=' + _as_env\n"
    "        if _as_eff:\n"
    "            self._auto_start_timer = self.create_timer(\n"
    "                AUTO_START_DELAY, self._auto_start_mission)\n"
    "            self.get_logger().warn(\n"
    "                '★自动出发【已开启】来源=' + _as_src + '：'\n"
    "                + format(AUTO_START_DELAY, '.1f') + 's 后自动下发内置路线 '\n"
    "                + str(MISSION_BLOCKS))\n"
    "        else:\n"
    "            self.get_logger().warn(\n"
    "                '★自动出发【已关闭】来源=' + _as_src\n"
    "                + '：车原地待命，只等外部 /nav/waypoint 指令，不会自己动')\n"
)

YML_NEW_LINE = ('    auto_start: false            '
                '# ★起栈后是否自动出发；false = 原地待命，等外部指令再动\n'
                '                                 #   临时要一键出发：DZ_AUTO_START=1 ./start_all.sh start')


def md5(p):
    import hashlib
    with open(p, 'rb') as f:
        return hashlib.md5(f.read()).hexdigest()


def install_copies():
    """ros2 run 实际执行的是 install 空间的副本，必须一起同步。
    实测（2026-10-07）：source install/setup.bash 后 ros2 run 走的是
      ~/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py
    而不是 src/ 下的同名文件；纯 Python 包 colcon build 就是逐字节拷贝，
    所以 src -> install 直接 cp 与重新 build 等价。"""
    out = []
    root = os.path.join(WS, 'install')
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in filenames:
            if fn == 'path_executor.py':
                out.append(os.path.join(dirpath, fn))
    return out


def sync_install(bdir=None):
    """把 src（已打补丁）同步到所有 install 副本。返回 True/False。"""
    copies = install_copies()
    if not copies:
        print('[install] 未找到 install 副本，跳过同步')
        return True
    src_md5 = md5(PE)
    all_ok = True
    for c in copies:
        cur = md5(c)
        if cur == src_md5:
            print('[install] 已一致，无需同步: %s' % c)
            continue
        if bdir:
            os.makedirs(bdir, exist_ok=True)
            shutil.copy2(c, os.path.join(bdir, 'install_' + os.path.basename(os.path.dirname(c)) + '_path_executor.py'))
        shutil.copy2(PE, c)
        now = md5(c)
        ok = (now == src_md5)
        all_ok = all_ok and ok
        print('[install] 同步 %s -> %s  %s' % (cur[:8], now[:8], 'OK' if ok else '失败'))
        print('[install]   目标: %s' % c)
    return all_ok


def read(p):
    with open(p, 'r', encoding='utf-8') as f:
        return f.read()


def write(p, s):
    with open(p, 'w', encoding='utf-8', newline='') as f:
        f.write(s)


def show_diff(name, old, new):
    d = difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                             fromfile=name + ' (改前)', tofile=name + ' (改后)', n=2)
    out = ''.join(d)
    print(out if out else '  (无差异)')


def yaml_auto_start_value():
    """读 yaml 里 mission.auto_start 的原始行（找不到返回 None）"""
    for line in read(YML).splitlines():
        if line.strip().startswith('auto_start:'):
            return line
    return None


def do_check():
    print('=== 状态检查（只读，不写任何文件）===')
    pe = read(PE)
    print('  path_executor.py :', PE)
    print('    行数           :', len(pe.splitlines()))
    print('    已打补丁       :', '是' if MARKER in pe else '否')
    print('    R1 命中次数    :', pe.count(R1_OLD))
    print('    R2 命中次数    :', pe.count(R2_OLD))
    print('  robot.yaml       :', YML)
    print('    auto_start 行  :', (yaml_auto_start_value() or '<未找到>').rstrip())
    print('  install 副本（ros2 run 实际执行的那份）:')
    copies = install_copies()
    if not copies:
        print('    <未找到>')
    for c in copies:
        body = read(c)
        print('   ', c)
        print('      已打补丁   :', '是' if MARKER in body else '否')
        print('      md5        :', md5(c), '（src =', md5(PE) + '）')
        print('      与 src 一致:', '是' if md5(c) == md5(PE) else '★否 —— 需 --sync-install 或 colcon build')
    return 0


def do_revert():
    cands = sorted(glob.glob(os.path.join(BACKUP_ROOT, 'autostart_wait_*')))
    if not cands:
        print('!! 没找到任何备份目录：%s/autostart_wait_*' % BACKUP_ROOT)
        return 1
    src = cands[-1]
    print('=== 回滚（从最近备份 %s）===' % src)
    for f, dst in (('path_executor.py', PE), ('robot.yaml', YML)):
        s = os.path.join(src, f)
        if os.path.exists(s):
            shutil.copy2(s, dst)
            print('  已还原:', dst)
        else:
            print('  跳过（备份里没有 %s）' % f)
    # ★ 回滚也必须同步 install 副本，否则 ros2 run 跑的还是打完补丁的那份
    print('  同步 install 副本 ...')
    sync_install()
    print('回滚完成（src 与 install 都已还原到备份状态）。')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='只看状态，不修改')
    ap.add_argument('--revert', action='store_true', help='从最近备份回滚')
    ap.add_argument('--sync-install', action='store_true',
                    help='只把 src 的补丁同步到 install 副本（ros2 run 读的是它）')
    a = ap.parse_args()

    if a.check:
        return do_check()
    if a.revert:
        return do_revert()
    if a.sync_install:
        print('=== 只同步 install 副本 ===')
        return 0 if sync_install() else 4

    print('=' * 72)
    print('让小车起栈后【原地待命】，只等外部 /nav/waypoint 指令')
    print('=' * 72)

    pe_old = read(PE)
    yml_old = read(YML)

    # ---- 幂等判断 ---------------------------------------------------------
    if MARKER in pe_old:
        print('\n[跳过] path_executor.py 已经打过补丁（含 %s）' % MARKER)
        print('       当前 auto_start 行：', (yaml_auto_start_value() or '').rstrip())
        print('\n[同步] ros2 run 执行的是 install 空间的副本，检查是否需要同步 ...')
        ok = sync_install()
        print('       如需回滚：python3 %s --revert' % os.path.basename(__file__))
        return 0 if ok else 4

    # ---- 替换前先全部校验命中次数，任何一个不对就整体不写 ------------------
    n1 = pe_old.count(R1_OLD)
    n2 = pe_old.count(R2_OLD)
    print('\n[校验] R1 注释    命中 %d 次（需要 1）' % n1)
    print('[校验] R2 定时器  命中 %d 次（需要 1）' % n2)
    yml_line = yaml_auto_start_value()
    if yml_line is None:
        print('!! robot.yaml 里找不到 auto_start 行，放弃。')
        return 2
    if n1 != 1 or n2 != 1:
        print('!! 命中次数不符 —— 源码与预期不一致，【未修改任何文件】。')
        print('   R1 期望片段：%r' % R1_OLD)
        print('   R2 期望片段：%r' % R2_OLD)
        return 2

    # ---- 备份 -------------------------------------------------------------
    stamp = time.strftime('%Y%m%d_%H%M%S')
    bdir = os.path.join(BACKUP_ROOT, 'autostart_wait_' + stamp)
    os.makedirs(bdir, exist_ok=True)
    shutil.copy2(PE, os.path.join(bdir, 'path_executor.py'))
    shutil.copy2(YML, os.path.join(bdir, 'robot.yaml'))
    print('\n[备份] %s' % bdir)

    # ---- 生成新内容 -------------------------------------------------------
    pe_new = pe_old.replace(R1_OLD, R1_NEW, 1).replace(R2_OLD, R2_NEW, 1)
    yml_new = yml_old.replace(yml_line, YML_NEW_LINE, 1)

    print('\n--- path_executor.py diff ---')
    show_diff('path_executor.py', pe_old, pe_new)
    print('--- robot.yaml diff ---')
    show_diff('robot.yaml', yml_old, yml_new)

    # ---- 写入 + 自检（失败自动回滚）---------------------------------------
    try:
        write(PE, pe_new)
        py_compile.compile(PE, doraise=True)
        print('\n[自检] py_compile 通过')

        write(YML, yml_new)
        try:
            import yaml as _y
        except ImportError:
            print('[自检] 无 pyyaml，跳过 YAML 解析（已用整行替换，风险低）')
        else:
            with open(YML, 'r', encoding='utf-8') as f:
                doc = _y.safe_load(f)
            # 真实结构：  path_executor: -> ros__parameters: -> auto_start: <bool>
            # 证据：path_executor.py:292-310 `_load_config_center()` 读的就是这一层
            sec = doc.get('path_executor') if isinstance(doc, dict) else None
            params = sec.get('ros__parameters', sec) if isinstance(sec, dict) else None
            got = params.get('auto_start') if isinstance(params, dict) else None
            assert got is False, \
                'path_executor.ros__parameters.auto_start 期望 False，实际 %r' % (got,)
            print('[自检] yaml.safe_load 通过，path_executor.ros__parameters.auto_start =', got)
    except Exception as e:  # noqa: BLE001
        print('\n!! 自检失败：%r' % (e,))
        print('   正在回滚 ...')
        shutil.copy2(os.path.join(bdir, 'path_executor.py'), PE)
        shutil.copy2(os.path.join(bdir, 'robot.yaml'), YML)
        print('   已回滚到改动前状态，车上文件未被破坏。')
        return 3

    print('\n' + '=' * 72)
    if not sync_install(bdir):
        print('!! install 副本同步失败 —— 请手动 colcon build 后再试')
        return 4
    print('完成。现在的行为：')
    print('  ./start_all.sh start                  -> 车原地待命，只等 /nav/waypoint')
    print('  DZ_AUTO_START=1 ./start_all.sh start  -> 比赛一键出发（10s 后自动下发内置路线）')
    print('  启动横幅（7_path.log 开头）会打印 ★自动出发【已关闭/已开启】来源=...')
    print('回滚：python3 %s --revert' % os.path.basename(__file__))
    print('=' * 72)
    return 0


if __name__ == '__main__':
    sys.exit(main())
