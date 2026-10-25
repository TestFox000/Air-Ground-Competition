"""Orin（机载 Jetson）一键部署：把 dzuav 源码 + 模型 + Orin 工具链推上无人机。

为什么需要它：车端有 car_deploy.py，机载端此前靠人肉 scp，且路径出现过三处
口径不一（preflight 查 ~/dzuav/build、run 脚本默认 ~/ag_test/dzuav、
build_verify 产物在仓库内）。本脚本统一约定为【仓库形状】远端布局：

    ~/dzuav_deploy/air_ground/          仓库侧内容
        onboard/dzuav/**                    dzuav C++ 源码（13 文件）
        models/release/                     current 指针 + 指针指向的 ONNX
        tools/orin_*.sh                     编译验证/引擎转换/全栈拉起
        tools/orin_*.py                     机载 Python 工具（显式名单见 ORIN_PY_TOOLS）

三个 orin_* 脚本都从 $0 自推 REPO 路径，落位后原位即用（无需搬动）；
引擎目录沿用 orin_run_dzuav.sh / orin_convert_engines.sh 的默认
~/dzuav/engines（机上转换，TRT 引擎与架构/版本绑定，本机无法预生成）。

用法（密码走环境变量，与 AG_CAR_PASS 同款约定）:
    set AG_ORIN_PASS=******
    python tools/orin_deploy.py --host <Orin IP> --user <用户>
    python tools/orin_deploy.py --host ... --skip-models      # 只推代码
    python tools/orin_deploy.py --host ... --all-models       # 推整个 release（含历史版本）
    python tools/orin_deploy.py --host ... --verify           # 部署后远程 L1 编译+L2 自检

部署完成后在 Orin 上（脚本会打印）:
    cd ~/dzuav_deploy/air_ground
    tools/orin_convert_engines.sh -o ~/dzuav/engines   # ONNX -> TRT FP16（换机/刷机后必跑）
    tools/orin_build_verify.sh                        # L0~L2 分层验证
    tools/orin_bringup.sh up                          # 全栈启动（tmux）

依赖: pip install paramiko（连接/SFTP 复用 tools/remote_run.py 的原语与重试约定）
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import remote_run  # noqa: E402  复用连接重试 / sftp_put / run（同一套坑都踩过）

REPO = TOOLS.parent
REMOTE_ROOT = "~/dzuav_deploy/air_ground"

# 与 preflight_checklist.md B3-4 / models/release/README.md 对齐的指针名
POINTER_FILES = ("current_hbb_detector", "current_cell_classifier")

# 机载 Python 工具全量名单（显式登记，新增 orin_*.py 时同步加进来）：
#   orin_flight_blackbox.py   飞行黑匣子：PX4 遥测/控制指令 JSONL 落盘 + UDP 镜像
#                             （flight_console.py 也会自动上传，此处兜底）
#   orin_log_capture.py       dzuav stdout -> JSONL（uav_link 健康通道优先读它；
#                             main.cpp 的日志行格式与之严格对应）
#   orin_plan_sidecar.py      机载自主规划旁挂进程（官方控制要求「规划须由机载
#                             平台自主完成」）：消费 dzuav 本地 UDP:50000 -> 世界
#                             模型/落格矩阵/任务路由 -> 落 route_plan.json 等三份
#                             产物 + 广播 0x4003 车命令。**依赖 src/ag_integrate**，
#                             故本脚本必须同时推送该包（见 AG_INTEGRATE_DIR）。
# 2026-10-18 合并线说明：repo 线全量名单另含 orin_watchdog/orin_agent_watchdog/
# orin_capture_dataset，其中前两个对应 repo 线 orin_bringup.sh 的 flight_wd/
# wdog 窗口；合并线 bringup 采用部署现实（fix 线）9 窗口版，无此二窗口、
# 部署包内亦无对应文件，故不列入——显式登记 + 部署前缺失校验，
# 漏传在本地即被拦下，不再靠机上 bringup 亮红灯才发现。
ORIN_PY_TOOLS = (
    "orin_flight_blackbox.py",
    "orin_log_capture.py",
    "orin_plan_sidecar.py",
)

# 机载规划包（相对仓库根）：orin_plan_sidecar 直接 sys.path 复用这个包，
# 单独推 sidecar 会 ImportError。它此前不在部署范围（旧口径把 ag_integrate
# 当纯车端包），规划上机后成为机上必需件。
AG_INTEGRATE_DIR = Path("src") / "ag_integrate"


def read_password(args) -> str:
    """密码守卫已收编到 remote_run.read_password（2026-10-17 横切扫描）；
    本壳保留仅为向后兼容，语义不变：--password > AG_ORIN_PASS > 交互提问。"""
    return remote_run.read_password(args, env_name="AG_ORIN_PASS")


def resolve_current_models() -> list[tuple[Path, str]]:
    """按 current 指针挑出必须上机的模型文件（指针 + 指向的 ONNX + release README）。

    只推指针指向的当前版本（约 43MB）——历史版本留在仓库，机上用不到，
    慢速 WiFi 下全量推 release（>120MB）得不偿失。
    """
    rel = REPO / "models" / "release"
    out: list[tuple[Path, str]] = []
    for pointer in POINTER_FILES:
        p = rel / pointer
        if not p.is_file():
            raise SystemExit(f"缺少指针文件: {p}（先跑 tools/release_models.py 发布模型）")
        target = p.read_text(encoding="utf-8").strip()
        onnx = rel / target
        if not onnx.is_file():
            raise SystemExit(f"指针 {pointer} -> {target} 在 models/release/ 不存在")
        out.append((p, f"models/release/{pointer}"))
        out.append((onnx, f"models/release/{target}"))
    readme = rel / "README.md"
    if readme.is_file():
        out.append((readme, "models/release/README.md"))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="orin_deploy.py",
        description="Orin 一键部署：dzuav 源码 + 当前模型 + orin 工具链（远端布局统一）")
    ap.add_argument("--host", required=True, help="Orin IP，如 192.168.50.10")
    ap.add_argument("--user", default=os.environ.get("AG_ORIN_USER"),
                    help="Orin 登录用户（或设 AG_ORIN_USER 环境变量）")
    ap.add_argument("--password", default=None, help="密码；更推荐 AG_ORIN_PASS 环境变量")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--skip-models", action="store_true",
                    help="只推代码与工具（模型 43MB，慢网省时）")
    ap.add_argument("--all-models", action="store_true",
                    help="推整个 models/release/（含历史版本，>120MB）")
    ap.add_argument("--verify", action="store_true",
                    help="部署后远程执行 L1 编译 + L2 二进制自检（不做 apt/镜像操作）")
    ap.add_argument("--timeout", type=float, default=600.0,
                    help="SFTP/命令超时（模型较大，默认 600s）")
    args = ap.parse_args(argv)

    if not args.user:
        ap.error("需要 --user（或设 AG_ORIN_USER 环境变量）；Jetson 出厂常见为 nvidia，以实际为准")
    pw = read_password(args)
    host, user, port = args.host, args.user, args.port

    def put(local: Path, remote: str) -> None:
        n = remote_run.sftp_put(host, user, pw, str(local), remote,
                                port=port, timeout=args.timeout)
        print(f"[已上传] {n} 个文件 -> {remote}")

    # ---- 1. dzuav 源码（13 文件，build/ 目录留在机上不回传不覆盖） ----
    print("==> 部署 onboard/dzuav 源码")
    put(REPO / "onboard" / "dzuav", f"{REMOTE_ROOT}/onboard/dzuav")

    # ---- 2. orin 工具链（自推 REPO，落位即用；.py 名单见 ORIN_PY_TOOLS 注释） ----
    print("==> 部署 orin 工具链")
    orin_tools = sorted(TOOLS.glob("orin_*.sh")) + [TOOLS / n for n in ORIN_PY_TOOLS]
    missing = [t.name for t in orin_tools if not t.is_file()]
    if not orin_tools or missing:
        raise SystemExit(f"tools/ 下缺少 orin 工具: {missing or 'orin_*.sh'} —— 仓库状态异常")
    for t in orin_tools:
        put(t, f"{REMOTE_ROOT}/tools/{t.name}")
    # SFTP put 不保留权限位：.sh 落盘成 0644，机上第一条 tools/orin_*.sh
    # 指令就是 Permission denied——上传后显式补可执行位
    # （flight_console 对 bb_start.sh 也是同一做法）
    code, _, cerr = remote_run.run(host, user, pw,
                                   f"chmod +x {REMOTE_ROOT}/tools/orin_*.sh",
                                   timeout=30, port=port)
    if code != 0:
        raise SystemExit(f"[失败] 远端 chmod +x 工具脚本失败: {cerr.strip()[:120]}")

    # ---- 2.2 机载依赖工具（orin_convert/orin_accept 的硬依赖） ----
    # tools/ml/verify_contract.py：orin_convert_engines.sh 转换前无条件调用
    # （$REPO/tools/ml/ 路径），名单缺它时换装直接 exit 5 拒转（2026-10-26
    # ft_v1 换装实测踩坑）。
    # tools/fake_udp_sender.py：orin_accept_plan.sh 的态势注入器，同日踩坑
    # （机上缺失时验收②③无法执行）。
    for dep, rel in ((REPO / "tools" / "ml" / "verify_contract.py", "tools/ml/verify_contract.py"),
                     (REPO / "tools" / "fake_udp_sender.py", "tools/fake_udp_sender.py")):
        if not dep.is_file():
            raise SystemExit(f"[失败] 仓库缺少机载依赖 {dep}")
        put(dep, f"{REMOTE_ROOT}/{rel}")

    # ---- 2.5 机载规划包：orin_plan_sidecar 的 import 依赖（机上必需件） ----
    # 官方控制要求「无人机的定位、建图、识别、规划与控制等实时任务须由机载平台
    # 自主完成」→ 规划层在机上跑，sidecar 直接 sys.path 复用这个包；只推 sidecar
    # 会 ImportError。机上这份是无 .git 的纯拷贝，覆盖前先整目录备份（可回滚）。
    print("==> 部署机载规划包 src/ag_integrate（先备份机上原副本）")
    planner = REPO / AG_INTEGRATE_DIR
    probe = planner / "ag_integrate" / "intel_protocol.py"
    if not probe.is_file():
        raise SystemExit(f"[失败] 仓库缺少规划包 {probe} —— 仓库状态异常，拒绝部署")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    bcmd = (
        f'P={REMOTE_ROOT}/src/ag_integrate; '
        f'if [ -d "$P" ]; then cp -a "$P" "$P.bak_{stamp}" && echo "BACKUP=$P.bak_{stamp}"; '
        f'else echo NO_EXISTING_PLANNER; fi'
    )
    code, out, err = remote_run.run(host, user, pw, bcmd, timeout=120, port=port)
    print("  " + (out.strip() or err.strip()))
    if code != 0:
        raise SystemExit("[失败] 机上规划包备份失败（拒绝在无备份的情况下覆盖机上文件）")
    put(planner, f"{REMOTE_ROOT}/{AG_INTEGRATE_DIR.as_posix()}")

    # ---- 3. 模型：默认只推 current 指针 + 指向的 ONNX ----
    if args.skip_models:
        print("==> --skip-models：跳过模型（机上引擎由 orin_convert_engines.sh 转换）")
    elif args.all_models:
        print("==> 部署整个 models/release/（含历史版本）")
        put(REPO / "models" / "release", f"{REMOTE_ROOT}/models/release")
    else:
        print("==> 部署当前指针模型（hbb + cell）")
        planned = resolve_current_models()
        # 把指针实际指向亮出来：若机上引擎已比这些模型新（如手动部署过更新的
        # ONNX 并重建了引擎），再跑 orin_convert_engines.sh 就是用旧模型覆盖
        # 新引擎（静默降级）。先确认指针版本符合预期再继续。
        for _local, relpath in planned:
            if relpath.endswith(".onnx"):
                print(f"  [指针指向] {relpath}")
        print("  ⚠️ 若机上引擎比上述模型更新，部署后勿跑引擎转换覆盖（降级风险）")
        for local, relpath in planned:
            n = remote_run.sftp_put(host, user, pw, str(local),
                                    f"{REMOTE_ROOT}/{relpath}",
                                    port=port, timeout=args.timeout)
            if n:
                print(f"[已上传] {local.name} -> {REMOTE_ROOT}/{relpath}")

    # ---- 4. 可选：远程编译验证（纯 L1/L2，不碰 apt/镜像/sudo） ----
    if args.verify:
        print("==> 远程 L1 编译 + L2 自检（首次约 1~3 分钟）")
        verify_cmd = (
            "bash -lc 'mkdir -p ~/dzuav_deploy/air_ground/onboard/dzuav/build && "
            "cd ~/dzuav_deploy/air_ground && "
            "cmake -S onboard/dzuav -B onboard/dzuav/build -DCMAKE_BUILD_TYPE=Release && "
            "cmake --build onboard/dzuav/build --parallel \"$(nproc)\"; build_rc=$?; "
            # 编译失败必须显式退出：原写法靠 && 短路，dzuav 不存在时 PIPESTATUS
            # 取到 2 导致 L2_OK 假阳性（2026-10-16 Orin 实机踩坑，编译错被误报通过）。
            "[ $build_rc -ne 0 ] && echo BUILD_FAIL && exit 1; "
            "onboard/dzuav/build/dzuav 2>&1 | head -2; "
            "rc=${PIPESTATUS[0]}; [ $rc -eq 2 ] && echo L2_OK || exit 1'"
        )
        code, out, err = remote_run.run(host, user, pw, verify_cmd,
                                        timeout=args.timeout, port=port)
        print(out)
        if err.strip():
            print("--- stderr ---\n" + err[:800])
        if code != 0:
            print(f"[失败] 远程验证退出码 {code}（无参数运行应打印 usage 退出码 2）",
                  file=sys.stderr)
            return 1
        print("[通过] L1 编译 + L2 usage 自检")

    # ---- 5. 打印机上后续命令（部署只管"送到位"，转换/验证在机上跑） ----
    print(
        "\n==================== 部署完成 ====================\n"
        "在 Orin 上按序执行：\n"
        "  tools/orin_build_verify.sh                          # L0~L2；加 --run <engines> <图> 做 L3\n"
        "  tools/orin_accept_plan.sh                           # 机载规划一键验收（只写 /tmp）\n"
        "  tools/orin_bringup.sh up                            # 全栈启动（规划默认只落盘）\n"
        "  PLAN_SEND=1 tools/orin_bringup.sh up                # 确认产物后再开 0x4003 实发\n"
        "\n"
        "⚠️ 换模型后（指针变动）必须跑 tools/orin_convert_engines.sh 重转引擎：\n"
        "   它按 current_* 指针选 ONNX（转换前自带契约门禁），引擎文件名沿用\n"
        "   hbb_detector_d13_fp16.engine（与 run/verify 脚本三处逐字一致，测试锁定），\n"
        "   旧引擎自动留 .bak_时间戳 备份。前置：python3 -m pip install --user onnx。\n"
        "   顺序见 DEPLOY_RUNBOOK ft_v1 契约换装段。\n"
        "无无人机时车端自测：tools/fake_udp_sender.py --scenario \"...\"\n"
        "====================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
