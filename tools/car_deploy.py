"""车端部署：把 ag_integrate 上传到香橙派并 colcon build + 验证。

流程：
    1. 本机打包 src/ag_integrate（排除 __pycache__）
    2. SSH 上传到车端工作空间 <ws>/src/
    3. colcon build --packages-select ag_integrate
    4. ros2 pkg list | grep ag_integrate  验证安装
    5. 可选：--dry-run-test 启动 integrate_node 5 秒，确认节点能起来

用法：
    # 密码走环境变量（推荐）
    set AG_CAR_PASS=<车端密码>
    python tools/car_deploy.py --host 192.168.1.188

    # 指定工作空间和 ROS 发行版
    python tools/car_deploy.py --host 192.168.1.188 --ws ~/ag_ws --ros-distro humble

    # 只上传不编译
    python tools/car_deploy.py --host 192.168.1.188 --no-build

    # 编译后跑 dry_run 5 秒
    python tools/car_deploy.py --host 192.168.1.188 --dry-run-test

依赖：pip install paramiko（仅本机）
车端不需要装任何额外东西（已有 ROS2 + colcon）。
"""
from __future__ import annotations

import argparse
import io
import os
import sys
import tarfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC_PKG = REPO / "src" / "ag_integrate"

try:
    import paramiko
except ImportError:
    print("缺少 paramiko，请先执行: python -m pip install paramiko", file=sys.stderr)
    raise SystemExit(2)


# ---------------------------------------------------------------- 输出辅助
def step(msg: str) -> None:
    print(f"\033[1;36m==>\033[0m {msg}")


def ok(msg: str) -> None:
    print(f"\033[1;32m[通过]\033[0m {msg}")


def warn(msg: str) -> None:
    print(f"\033[1;33m[警告]\033[0m {msg}")


def fail(msg: str) -> None:
    print(f"\033[1;31m[失败]\033[0m {msg}", file=sys.stderr)
    raise SystemExit(1)


# ---------------------------------------------------------------- 打包
def make_tarball() -> bytes:
    """把 src/ag_integrate 打包成 tar.gz（排除 __pycache__、.pyc、.bak）。"""
    if not SRC_PKG.is_dir():
        fail(f"找不到源码目录: {SRC_PKG}")
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for path in sorted(SRC_PKG.rglob("*")):
            rel = path.relative_to(SRC_PKG)
            # 排除缓存与临时文件
            parts = rel.parts
            if any(p == "__pycache__" for p in parts):
                continue
            if path.suffix in (".pyc", ".bak"):
                continue
            if path.is_file():
                tar.add(path, arcname=str(Path("ag_integrate") / rel))
    buf.seek(0)
    return buf.read()


# ---------------------------------------------------------------- SSH 执行
def run(ssh: paramiko.SSHClient, cmd: str, timeout: int = 300) -> tuple[int, str]:
    """执行命令，返回 (退出码, 合并输出)。"""
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out + err


def get_password(args) -> str:
    pw = args.password or os.environ.get("AG_CAR_PASS")
    if pw:
        return pw
    import getpass
    if not sys.stdin.isatty():
        raise SystemExit(
            "缺少车端密码（无交互终端）：\n"
            "  ① 设环境变量：set AG_CAR_PASS=<密码>\n"
            "  ② 或 --password <密码>"
        )
    return getpass.getpass(f"{args.user}@{args.host} 的密码: ")


# ---------------------------------------------------------------- 主流程
def main() -> int:
    ap = argparse.ArgumentParser(description="车端部署 ag_integrate")
    ap.add_argument("--host", required=True, help="车端 IP（香橙派）")
    ap.add_argument("--user", default="duzhong", help="车端用户名（默认 duzhong）")
    ap.add_argument("--password", default=None, help="密码（或设 AG_CAR_PASS 环境变量）")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--ws", default="~/ag_ws", help="车端 colcon 工作空间（默认 ~/ag_ws）")
    ap.add_argument("--ros-distro", default=None,
                    help="ROS 发行版（foxy/humble/...），不指定则自动探测 /opt/ros/ 下可用的")
    ap.add_argument("--no-build", action="store_true", help="只上传源码，不编译")
    ap.add_argument("--dry-run-test", action="store_true",
                    help="编译后启动 integrate_node 5 秒（dry_run），确认节点可起")
    args = ap.parse_args()

    # 1. 打包
    step("打包 src/ag_integrate")
    data = make_tarball()
    ok(f"打包完成，{len(data) / 1024:.1f} KB")

    # 2. 连接
    step(f"SSH 连接 {args.user}@{args.host}")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(args.host, port=args.port, username=args.user,
                    password=get_password(args), timeout=30, banner_timeout=30)
    except Exception as exc:
        fail(f"SSH 连接失败: {exc}")
    ok("SSH 已连接")

    try:
        # 3. 探测 ROS 发行版
        if not args.ros_distro:
            rc, out = run(ssh, "ls /opt/ros/ 2>/dev/null | head -1")
            distro = out.strip().splitlines()[0] if out.strip() else ""
            if not distro:
                fail("车端 /opt/ros/ 下没有 ROS 发行版，请用 --ros-distro 指定")
            args.ros_distro = distro
            ok(f"自动探测到 ROS 发行版: {args.ros_distro}")
        else:
            ok(f"使用指定 ROS 发行版: {args.ros_distro}")

        ros_setup = f"/opt/ros/{args.ros_distro}/setup.bash"
        ws = args.ws

        # 4. 创建工作空间并上传
        step(f"上传到车端 {ws}/src/")
        run(ssh, f"mkdir -p {ws}/src")
        sftp = ssh.open_sftp()
        remote_tar = f"/tmp/ag_integrate.tar.gz"
        with sftp.file(remote_tar, "wb") as f:
            f.write(data)
        sftp.close()
        ok("tarball 已上传")

        # 解压（覆盖旧版本）
        rc, out = run(ssh, f"cd {ws}/src && rm -rf ag_integrate && tar xzf {remote_tar}")
        if rc != 0:
            fail(f"解压失败:\n{out}")
        run(ssh, f"rm -f {remote_tar}")
        ok("解压完成")

        if args.no_build:
            warn("--no-build：跳过编译，部署结束")
            return 0

        # 5. colcon build
        step(f"colcon build（工作空间 {ws}）")
        build_cmd = (
            f"source {ros_setup} && "
            f"cd {ws} && "
            f"colcon build --packages-select ag_integrate --symlink-install 2>&1"
        )
        rc, out = run(ssh, build_cmd, timeout=600)
        print(out)
        if rc != 0 or "Failed   <<< ag_integrate" in out:
            fail("colcon build 失败，见上方输出")
        ok("colcon build 成功")

        # 6. 验证安装
        step("验证安装：ros2 pkg list | grep ag_integrate")
        verify_cmd = (
            f"source {ros_setup} && "
            f"source {ws}/install/setup.bash && "
            f"ros2 pkg list | grep ag_integrate"
        )
        rc, out = run(ssh, verify_cmd, timeout=60)
        if rc != 0 or "ag_integrate" not in out:
            fail(f"验证失败，ros2 pkg list 中没有 ag_integrate:\n{out}")
        ok(f"ag_integrate 已安装: {out.strip()}")

        # 7. 可选 dry_run 测试
        if args.dry_run_test:
            step("dry_run 测试：启动 integrate_node 5 秒")
            test_cmd = (
                f"source {ros_setup} && "
                f"source {ws}/install/setup.bash && "
                f"timeout 5 ros2 run ag_integrate integrate_node "
                f"--ros-args -p dry_run:=true -p udp_port:=50099 2>&1 || true"
            )
            rc, out = run(ssh, test_cmd, timeout=30)
            print(out)
            if "integrate_node 就绪" in out:
                ok("integrate_node 启动成功（dry_run 模式）")
            else:
                warn("未看到'就绪'日志，可能 ROS 环境或依赖有问题，见上方输出")

    finally:
        ssh.close()

    print()
    print("==================== 部署完成 ====================")
    print(f"工作空间: {ws}")
    print(f"启动命令:")
    print(f"  source {ros_setup}")
    print(f"  source {ws}/install/setup.bash")
    print(f"  ros2 launch ag_integrate integrate.launch.py")
    print("==================================================")
    return 0


if __name__ == "__main__":
    sys.exit(main())
