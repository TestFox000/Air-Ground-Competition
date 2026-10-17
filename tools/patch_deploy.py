#!/usr/bin/env python3
"""dzpx4 增强补丁一键部署：onboard/dzpx4_patch -> Orin（韧性定位体系核心件）。

背景：px4_odom_convert 是定位链的咽喉（SLAM -> EKF）。增强版在官方版之上
叠加两层韧性防护（全部来自两天四轮实飞的实测证据）：
  * jump guard + armed 门控：SLAM 跟丢跳变（实测 37~62m、9.11m）永不进 EKF
  * z 换源：SLAM z 漂移（实测 5.56m / 8.9m）与飞控高度彻底绝缘（走气压计）

用法（密码走环境变量，与 AG_ORIN_PASS 同约定）:
    set AG_ORIN_PASS=******
    python tools/patch_deploy.py --host 10.171.116.104 --user duzhong
    python tools/patch_deploy.py --host ... --no-restart   # 只推+编译，不动运行中节点
    python tools/patch_deploy.py --host ... --rollback     # 恢复机上 .bak 原版

流程：备份机上现版(.bak_patch_<时刻>) -> 推送增强版 -> 强制 colcon 重编
      -> 重启 px4_odom_convert 节点 -> 验证（韧性层横幅 + 进程 + delta 流）。
依赖: pip install paramiko（复用 tools/remote_run.py 原语）
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402  连接重试 / sftp_put 原语

REPO = TOOLS.parent
PATCH = REPO / "onboard" / "dzpx4_patch"

REMOTE_CPP = "~/dzpx4/src/px4_odom_convert/src/px4_odom_convert.cpp"
REMOTE_HPP = "~/dzpx4/src/px4_odom_convert/include/px4_odom_convert.hpp"
BUILD_LOG = "/tmp/patch_build.log"


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少 Orin 密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def step(host, user, pw, port, desc, cmd, timeout=60, expect=None):
    """一步远程操作，断链自动重试；expect 不中则重试整步。"""
    for attempt in range(1, 6):
        try:
            code, out, err = remote_run.run(host, user, pw, cmd, timeout=timeout, port=port)
            if expect is None or expect in out:
                print(f"[OK] {desc}" + (f"（{attempt} 次尝试）" if attempt > 1 else ""))
                return out
            print(f"[..] {desc}: 未见到期望标志，重试（{attempt}/5）")
        except Exception as e:  # noqa: BLE001 - 断链是常态，重试是本工具存在的意义
            print(f"[..] {desc}: {type(e).__name__}（{attempt}/5）")
        time.sleep(6)
    raise SystemExit(f"[失败] {desc}：5 次尝试未成功（用 --host 确认无人机可达）")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="patch_deploy.py",
                                 description="部署韧性定位补丁到 Orin")
    ap.add_argument("--host", required=True, help="Orin IP（现场网络以实际为准）")
    ap.add_argument("--user", default="duzhong", help="SSH 用户（默认 duzhong）")
    ap.add_argument("--password", default=None, help="密码；推荐 AG_ORIN_PASS 环境变量")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--no-restart", action="store_true", help="只部署编译，不重启运行中节点")
    ap.add_argument("--rollback", action="store_true",
                    help="回滚到机上 .bak 原版（找最近的备份）并重启")
    args = ap.parse_args(argv)
    pw = read_password(args)
    host, user, port = args.host, args.user, args.port

    if args.rollback:
        step(host, user, pw, port, "查找最近备份并回滚",
             "C=$(ls -t ~/dzpx4/src/px4_odom_convert/src/*.bak_* 2>/dev/null | head -1); "
             "H=$(ls -t ~/dzpx4/src/px4_odom_convert/include/*.bak_* 2>/dev/null | head -1); "
             "echo ROLLBACK_FROM=$C; "
             "cp $C ~/dzpx4/src/px4_odom_convert/src/px4_odom_convert.cpp && "
             "cp $H ~/dzpx4/src/px4_odom_convert/include/px4_odom_convert.hpp && "
             "cd ~/dzpx4 && source /opt/ros/humble/setup.bash && "
             "colcon build --packages-select px4_odom_convert 2>&1 | tail -1; "
             "echo ROLLBACK_DONE", timeout=300, expect="ROLLBACK_DONE")
        return 0

    # 1. 备份机上现版
    stamp = time.strftime("%Y%m%d_%H%M%S")
    step(host, user, pw, port, f"备份机上现版（.bak_patch_{stamp}）",
        f"cp {REMOTE_CPP} {REMOTE_CPP}.bak_patch_{stamp} && "
        f"cp {REMOTE_HPP} {REMOTE_HPP}.bak_patch_{stamp} && echo BACKUP_OK",
        expect="BACKUP_OK")

    # 2. 推送增强版
    for name, remote in [("px4_odom_convert.cpp", REMOTE_CPP),
                         ("px4_odom_convert.hpp", REMOTE_HPP)]:
        for attempt in range(1, 6):
            try:
                remote_run.sftp_put(host, user, pw, str(PATCH / name), remote,
                                    port=port, timeout=90)
                print(f"[OK] 推送 {name}")
                break
            except Exception as e:  # noqa: BLE001
                print(f"[..] 推送 {name}: {type(e).__name__}（{attempt}/5）")
                time.sleep(6)
        else:
            raise SystemExit(f"[失败] 推送 {name}")

    # 3. 强制重编（删产物绕过 colcon 增量坑；nohup 防 SSH 断连杀构建）
    step(host, user, pw, port, "强制重编 px4_odom_convert",
         "rm -f ~/dzpx4/build/px4_odom_convert/px4_odom_convert; "
         f"cd ~/dzpx4 && rm -f {BUILD_LOG} && "
         f"nohup setsid bash -lc 'source /opt/ros/humble/setup.bash; "
         f"colcon build --packages-select px4_odom_convert > {BUILD_LOG} 2>&1; "
         f"echo rc=$? >> {BUILD_LOG}' < /dev/null > /dev/null 2>&1 & "
         "echo BUILD_SUBMITTED", expect="BUILD_SUBMITTED")
    for i in range(18):
        time.sleep(15)
        try:
            _, out, _ = remote_run.run(host, user, pw, f"tail -2 {BUILD_LOG}",
                                        timeout=45, port=port)
            if "rc=" in out:
                rc = out.strip().splitlines()[-2] if "Summary" not in out else "done"
                print(f"[OK] 编译完成（{i + 1} 轮等待）\n{out.strip()[-200:]}")
                if "rc=0" not in out:
                    raise SystemExit("[失败] 编译报错，见机上 " + BUILD_LOG)
                break
        except SystemExit:
            raise
        except Exception:  # noqa: BLE001
            pass
    else:
        raise SystemExit("[失败] 编译超时（4.5 分钟），查机上 " + BUILD_LOG)

    # 4. 重启节点（setsid 后台 + 日志落 drone_logs）
    if not args.no_restart:
        step(host, user, pw, port, "重启 px4_odom_convert（setsid）",
            "pkill -f 'ros2 launch px4_odom_convert' 2>/dev/null; "
            "pkill -f px4_odom_convert 2>/dev/null; sleep 1; "
            "L=~/drone_logs/node_odom_$(date +%H%M%S).log; "
            "nohup setsid bash -lc 'source /opt/ros/humble/setup.bash; "
            "source ~/dzpx4/install/setup.bash; "
            "ros2 launch px4_odom_convert px4_odom_convert.launch.py >> $L 2>&1' "
            "< /dev/null > /dev/null 2>&1 & echo RESTARTED",
            expect="RESTARTED")
        time.sleep(15)
        out = step(host, user, pw, port, "验证韧性层横幅 + 进程",
                   "pgrep -f px4_odom_convert >/dev/null && echo PROC_OK; "
                   "grep -h 'Resilience layer\\|jump guard' ~/drone_logs/node_odom_*.log 2>/dev/null | tail -1; "
                   "grep -c 'position delta' ~/drone_logs/node_odom_*.log 2>/dev/null | tail -1",
                   timeout=60)
        print(out)

    print(
        "\n==================== 韧性补丁部署完成 ====================\n"
        "已生效：jump guard(armed 永不接受) + z 换源(气压计高度)\n"
        "回滚： python tools/patch_deploy.py --host " + host + " --rollback\n"
        "悬停验证判据：0.5m 悬停 10 秒，位移 < 0.3m、零突跳\n"
        "=========================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
