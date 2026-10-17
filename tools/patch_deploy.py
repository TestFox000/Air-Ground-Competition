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
    python tools/patch_deploy.py --host ... --rollback     # 按回滚索引恢复原版备份（指纹校验后重编）

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
# 回滚索引：部署时写入"备份路径+指纹"，回滚只按索引取件（绝不再用 ls -t 猜）
REMOTE_INDEX = "~/dzpx4/src/px4_odom_convert/.patch_deploy_index"
BUILD_LOG = "/tmp/patch_build.log"


def backup_cmd(stamp: str) -> str:
    """备份机上现版 + 写回滚索引（备份路径与 md5 指纹落档）。

    索引的意义：*.bak_* 里会混进手工备份（.bak_local_*），且二次部署的备份
    本身就是补丁版——回滚若靠 ls -t 挑"最新"，既可能拿错别人的备份，也可能
    把补丁版当原版"回滚"回去。指纹用于回滚前校验备份没被改动过。"""
    return (
        f"cp {REMOTE_CPP} {REMOTE_CPP}.bak_patch_{stamp} && "
        f"cp {REMOTE_HPP} {REMOTE_HPP}.bak_patch_{stamp} && "
        f"C={REMOTE_CPP}.bak_patch_{stamp} && H={REMOTE_HPP}.bak_patch_{stamp} && "
        "{ "
        'echo "CPP_BAK=$C"; echo "HPP_BAK=$H"; '
        'echo "CPP_MD5=$(md5sum "$C" | cut -d\' \' -f1)"; '
        'echo "HPP_MD5=$(md5sum "$H" | cut -d\' \' -f1)"; '
        f"}} > {REMOTE_INDEX} && echo BACKUP_OK")


def rollback_cmd() -> str:
    """按索引回滚：全程 && 串联（任一环节失败链条即断，ROLLBACK_DONE 不再假阳性）。

    校验顺序：索引在 -> 索引指向的备份文件在 -> 备份 md5 与索引指纹一致 ->
    才 cp 回原位并重编。任一不满足打印 ROLLBACK_REFUSED + 现场证据后退出，
    绝不盲滚（对端确定性拒绝，step() 的 refuse 分支会立即中止不重试）。"""
    return (
        f"IDX={REMOTE_INDEX}; "
        # 索引缺失 = 不是本工具部署的（或部署不完整）：拒绝盲选，亮现场证据
        '[ -f "$IDX" ] || { echo "ROLLBACK_REFUSED: 无回滚索引 $IDX"; '
        "ls -la ~/dzpx4/src/px4_odom_convert/src/ ~/dzpx4/src/px4_odom_convert/include/ 2>&1; "
        "exit 1; }; "
        '. "$IDX" && '
        # 只认索引记录的 .bak_patch_* 备份；显式校验非空且存在
        '[ -n "$CPP_BAK" ] && [ -f "$CPP_BAK" ] && [ -n "$HPP_BAK" ] && [ -f "$HPP_BAK" ] '
        '|| { echo "ROLLBACK_REFUSED: 索引指向的备份缺失"; cat "$IDX"; exit 1; }; '
        # 指纹校验：备份必须还是部署时那一份（被覆盖/污染过的备份不能当原版）
        '[ "$(md5sum "$CPP_BAK" | cut -d\' \' -f1)" = "$CPP_MD5" ] && '
        '[ "$(md5sum "$HPP_BAK" | cut -d\' \' -f1)" = "$HPP_MD5" ] '
        '|| { echo "ROLLBACK_REFUSED: 备份指纹与索引不符"; cat "$IDX"; '
        'md5sum "$CPP_BAK" "$HPP_BAK"; exit 1; }; '
        'echo "ROLLBACK_FROM=$CPP_BAK" && '
        f'cp "$CPP_BAK" {REMOTE_CPP} && cp "$HPP_BAK" {REMOTE_HPP} && '
        "cd ~/dzpx4 && source /opt/ros/humble/setup.bash && "
        f"colcon build --packages-select px4_odom_convert > {BUILD_LOG} 2>&1 && "
        f"tail -1 {BUILD_LOG} && echo ROLLBACK_DONE")


def read_password(args) -> str:
    pw = args.password or os.environ.get("AG_ORIN_PASS") or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit("缺少 Orin 密码：set AG_ORIN_PASS=******（或 --password）")
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


def step(host, user, pw, port, desc, cmd, timeout=60, expect=None, refuse=None):
    """一步远程操作，断链自动重试；expect 不中则重试整步。

    refuse 命中 = 对端**确定性拒绝**（如回滚校验不过），重试不会有不同结果：
    原样打印对端输出（现场证据）后立即失败退出。"""
    for attempt in range(1, 6):
        try:
            code, out, err = remote_run.run(host, user, pw, cmd, timeout=timeout, port=port)
            if refuse and refuse in out:
                print(out.strip())
                raise SystemExit(f"[失败] {desc}：远端明确拒绝（{refuse}），按上面证据处理")
            if expect is None or expect in out:
                print(f"[OK] {desc}" + (f"（{attempt} 次尝试）" if attempt > 1 else ""))
                return out
            tail = (out + err).strip()
            print(f"[..] {desc}: 未见到期望标志，重试（{attempt}/5）"
                  + (f"\n     输出尾部: {tail[-200:]}" if tail else ""))
        except SystemExit:
            raise
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
                    help="回滚到部署时索引记录的原版备份（校验指纹后重编并重启）")
    args = ap.parse_args(argv)
    pw = read_password(args)
    host, user, port = args.host, args.user, args.port

    if args.rollback:
        step(host, user, pw, port, "按回滚索引校验并回滚",
             rollback_cmd(), timeout=300, expect="ROLLBACK_DONE",
             refuse="ROLLBACK_REFUSED")
        return 0

    # 1. 备份机上现版 + 写回滚索引（备份指纹落档，回滚只按索引取件）
    stamp = time.strftime("%Y%m%d_%H%M%S")
    step(host, user, pw, port, f"备份机上现版（.bak_patch_{stamp}）并写回滚索引",
         backup_cmd(stamp), expect="BACKUP_OK")

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
            # 只杀装好的二进制本体：裸 pkill -f px4_odom_convert 会误杀
            # colcon/编辑器等同名串进程（包名匹配太宽）
            'pkill -f "$HOME/dzpx4/install/px4_odom_convert/lib/px4_odom_convert/px4_odom_convert" '
            "2>/dev/null; sleep 1; "
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
                   timeout=60, expect="PROC_OK")
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
