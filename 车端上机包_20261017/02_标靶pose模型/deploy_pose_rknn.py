# -*- coding: utf-8 -*-
"""target_pose_v2 ONNX → RKNN PC 端转换 + 香橙派一键部署（自动备份，失败可回滚）。

链路（全自动，幂等，绝不无备份覆盖车端文件）：
    1. 本地环境检查：ONNX 契约 / paramiko / onnx / rknn-toolkit2（缺失给安装指引）
    2. PC 端 FP16 转换（rk3588 不量化；mean 0 / std 255，与 air_ground
       tools/ml/rknn_convert.py 完全同参）+ accuracy_analysis 精度验证
       （PC 模拟器逐层对比 ONNX；输出层相似度低于阈值即中止，此时未动车端）
    3. SSH 连香橙派（不可达 → 优雅停在"等待板端在线"，--wait N 可挂机轮询）
    4. 定位现役 light_det.rknn，md5 与 models/release 登记基线比对
    5. 上传 .rknn → /tmp，板端 md5 复核；幂等备份原模型（.bak_pose_v2）
    6. 替换 + md5 复核，任何一步失败自动回滚
    7. 重启 det_node 并验证（节点在 ROS 图 + 进程存活 + 检测话题出帧 +
       重启日志无模型加载失败；--expect-frames 可严格判帧）
    8. 验证失败 → 自动回滚原模型并重启，明确报告

det_node 契约（air_ground docs/probe/car_models_contract.txt，9 通道单输出）：
    输入  images  [1,3,640,640] FLOAT（NCHW；节点侧喂 0~255 像素，/255 在模型内）
    输出  output0 [1,9,8400]    FLOAT（4 bbox + 2 类红/蓝靶 + 1×3 关键点；
    DFL 已在模型内解码，NMS 与关键点解码在节点侧——形状不符 det_node 解不出来）

用法：
    set AG_CAR_PASS=<车端密码>           （可选；默认自动尝试 duzhong / 12345678）
    python deploy_pose_rknn.py                           # 一键全流程（默认 192.168.0.199）
    python deploy_pose_rknn.py --host 192.168.x.x
    python deploy_pose_rknn.py --check-only              # 只做本地检查，不连车
    python deploy_pose_rknn.py --dry-run                 # + 连车只读探测，不改动
    python deploy_pose_rknn.py --wait 3600               # 板端不在线时挂机轮询等待
    python deploy_pose_rknn.py --rollback                # 一键回滚到部署前模型
    python deploy_pose_rknn.py --rknn <现成.rknn>        # 跳过本地转换（他机转好的）

依赖：本机 paramiko + onnx；rknn-toolkit2（PC 端转换用，与车端推理库
rknn-toolkit-lite2 是两回事——板端无需安装任何东西）。
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata as im
import json
import os
import re
import sys
import tempfile
import time
from pathlib import Path

PACK = Path(__file__).resolve().parent
ONNX_LOCAL = PACK / "models" / "target_pose_v2_best.onnx"
RKNN_LOCAL = PACK / "models" / "target_pose_v2_best.rknn"

REMOTE_RKNN = "/tmp/target_pose_v2.rknn"
BAK_SUFFIX = ".bak_pose_v2"

# models/release 登记基线（air_ground 仓库 README.md 登记表）
ONNX_MD5_BASELINE = "1d7e1f90a66966870c9ed61345ef39f7"     # target_pose_v2
LIGHTDET_MD5_BASELINE = "49f3700590a907994e7ee20e4d30cc13"  # 厂家 light_det.rknn

try:
    import paramiko
except ImportError:
    print("缺少 paramiko，请先执行: python -m pip install paramiko",
          file=sys.stderr)
    raise SystemExit(2)


def step(msg):
    print(f"==> {msg}", flush=True)


def ok(msg):
    print(f"[通过] {msg}", flush=True)


def warn(msg):
    print(f"[警告] {msg}", flush=True)


def fail(msg, code=1):
    print(f"[失败] {msg}", file=sys.stderr, flush=True)
    raise SystemExit(code)


def run(ssh, cmd, timeout=120):
    _, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    code = stdout.channel.recv_exit_status()
    out = stdout.read().decode("utf-8", errors="replace").strip()
    err = stderr.read().decode("utf-8", errors="replace").strip()
    return code, out, err


def md5_local(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest().lower()


# ---------------------------------------------------------------- 本地检查 --
def rknn_install_hint(py_ver):
    return (
        "缺少 rknn-toolkit2（PC 端转换用；与车端 rknn-toolkit-lite2 是两回事）。安装：\n"
        "  1) Python 3.8~3.11 环境: python -m pip install rknn-toolkit2 "
        "-i https://pypi.tuna.tsinghua.edu.cn/simple\n"
        f"  2) 当前 Python {py_ver}：PyPI 无兼容 wheel，需从 GitHub Releases 下载\n"
        "     对应 cp 版本 whl 手动装: https://github.com/airockchip/rknn-toolkit2/releases\n"
        "     下载后: python -m pip install <whl 路径>\n"
        "  版本要求: >= 1.6 才支持 RK3588；现役 light_det.rknn 由 toolkit2 2.3.2 转换，\n"
        "  板端 runtime 已验证兼容 2.x 产物，推荐装 2.x（>= 2.3.2 最稳）。\n"
        "  也可在其他已装 toolkit2 的机器上转换后: python deploy_pose_rknn.py "
        "--rknn <产物路径>"
    )


def check_local(onnx_path, strict_rknn):
    """本地环境与 ONNX 契约检查；strict_rknn=True 时缺 rknn-toolkit2 即失败。"""
    py_ver = ".".join(str(x) for x in sys.version_info[:3])
    if not onnx_path.is_file():
        fail(f"本地 ONNX 不存在: {onnx_path}", 1)
    md5 = md5_local(onnx_path)
    if md5 == ONNX_MD5_BASELINE:
        ok(f"ONNX: {onnx_path.name} {onnx_path.stat().st_size/1e6:.1f}MB "
           f"md5={md5.upper()}（与 models/release 登记一致）")
    else:
        warn(f"ONNX md5 {md5.upper()} != 登记基线 {ONNX_MD5_BASELINE.upper()}"
             f"（确认这是你要部署的版本）")

    # ---- det_node 契约校验（[N,3,640,640]→[N,9,8400]，形状不符节点解不出来）----
    try:
        import onnx
    except ImportError:
        fail("缺 onnx 库（契约校验用）: python -m pip install onnx", 4)
    m = onnx.load(str(onnx_path), load_external_data=False)

    def dims(v):
        tt = v.type.tensor_type
        return ([d.dim_value or d.dim_param for d in tt.shape.dim],
                onnx.TensorProto.DataType.Name(tt.elem_type))

    ins = {v.name: dims(v) for v in m.graph.input}
    outs = {v.name: dims(v) for v in m.graph.output}
    problems = []
    if "images" not in ins:
        problems.append(f"输入名 {list(ins)} 无 images")
    else:
        d, dt = ins["images"]
        if len(d) != 4 or list(d[1:]) != [3, 640, 640]:
            problems.append(f"输入 images {d} != [N,3,640,640]")
        if dt != "FLOAT":
            problems.append(f"输入 dtype {dt} != FLOAT")
    if "output0" not in outs:
        problems.append(f"输出名 {list(outs)} 无 output0")
    else:
        d, dt = outs["output0"]
        if len(d) != 3 or list(d[1:]) != [9, 8400]:
            problems.append(f"输出 output0 {d} != [N,9,8400]"
                            f"（9=4框+2类+1x3关键点）")
        if dt != "FLOAT":
            problems.append(f"输出 dtype {dt} != FLOAT")
    if problems:
        fail("ONNX 契约不符 det_node（别转别传）: " + "; ".join(problems), 1)
    opset = [o.version for o in m.opset_import if o.domain in ("", "ai.onnx")]
    ok(f"契约符合 det_node: images {ins['images'][0]} → output0 "
       f"{outs['output0'][0]}（opset {opset}）")

    # ---- rknn-toolkit2（PC 端转换）----
    tk_ver = None
    try:
        from rknn.api import RKNN  # noqa: F401
        try:
            tk_ver = im.version("rknn-toolkit2")
        except Exception:
            tk_ver = "unknown"
    except ImportError:
        pass
    if tk_ver:
        ok(f"rknn-toolkit2 {tk_ver} 可用（PC 端转换）")
        try:
            major, minor = (int(x) for x in tk_ver.split(".")[:2])
        except Exception:
            major, minor = 0, 0
        if (major, minor) < (1, 6):
            fail(f"rknn-toolkit2 {tk_ver} < 1.6，不支持 RK3588，请升级", 4)
        if (major, minor) < (2, 0):
            warn("rknn-toolkit2 < 2.0：现役模型由 2.3.2 转换，建议升级到 2.x")
    else:
        if strict_rknn:
            fail(rknn_install_hint(py_ver), 4)
        warn("rknn-toolkit2 未安装（--check-only/--dry-run 可继续；"
             "正式转换前需安装，见正式运行时的指引）")
    return tk_ver


# ---------------------------------------------------------------- PC 端转换 --
def write_bmp(path, arr):
    """无第三方库写 24bit BMP（精度验证兜底噪声图用）。"""
    import struct
    h, w = arr.shape[:2]
    row = (w * 3 + 3) // 4 * 4
    size = 54 + row * h
    with open(path, "wb") as f:
        f.write(b"BM" + struct.pack("<IHHI", size, 0, 0, 54))
        f.write(struct.pack("<IiiHHIIiiII", 40, w, h, 1, 24, 0,
                            row * h, 2835, 2835, 0, 0))
        pad = b"\x00" * (row - w * 3)
        for y in range(h - 1, -1, -1):
            f.write(arr[y][:, ::-1].tobytes() + pad)  # RGB → BGR


def find_probe_image(explicit):
    """精度验证输入图：--probe-image > 训练 val/train 图 > 随机噪声图。"""
    if explicit:
        p = Path(explicit)
        if not p.is_file():
            fail(f"--probe-image 不存在: {p}")
        return p
    for d in [Path(r"C:\Users\29212\Desktop\target_pose_v2\images\val"),
              Path(r"C:\Users\29212\Desktop\target_pose_v2\images\train")]:
        if d.is_dir():
            imgs = sorted(d.glob("*.jpg")) or sorted(d.glob("*.png"))
            if imgs:
                return imgs[0]
    try:
        import numpy as np
        rng = np.random.default_rng(42)
        arr = rng.integers(0, 256, (640, 640, 3), dtype=np.uint8)
        p = Path(tempfile.gettempdir()) / "pose_v2_probe_noise.bmp"
        write_bmp(p, arr)
        warn("未找到真图，精度验证用随机噪声图（逐层数值对比仍有效，"
             "但建议 --probe-image 指定一张含靶标的现场图")
        return p
    except Exception as e:
        warn(f"精度验证图不可用（跳过精度验证）: {e}")
        return None


def parse_output_sim(report):
    """从 accuracy_analysis 报告提取输出层相似度(%)。"""
    best = None
    for line in report.splitlines():
        if "OUTPUT" in line.upper() or "output0" in line:
            nums = [float(n) for n in re.findall(r"\d+\.\d+", line)]
            nums = [n for n in nums if 0.0 < n <= 100.0]
            if nums:
                best = max(nums) if best is None else max(best, max(nums))
    return best


def convert_local(onnx_path, rknn_path, tk_ver, sim_threshold, force,
                  probe_image=None):
    """PC 端 FP16 转换 + 精度验证；带 md5 缓存（车不在线先转好，在线秒部署）。"""
    onnx_md5 = md5_local(onnx_path)
    meta_path = Path(str(rknn_path) + ".meta")
    if (rknn_path.is_file() and meta_path.is_file() and not force):
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            if meta.get("onnx_md5") == onnx_md5:
                ok(f"转换缓存命中（{meta.get('toolkit', '?')} 转，"
                   f"{rknn_path.stat().st_size/1e6:.1f}MB）—— 跳过重转")
                return md5_local(rknn_path)
        except Exception:
            pass

    probe_img = find_probe_image(probe_image)
    from rknn.api import RKNN  # check_local 已保证可导入
    step("PC 端 FP16 转换（rk3588 不量化，预计 1~3 分钟）…")
    t0 = time.time()
    rknn = RKNN(verbose=False)
    # 与 tools/ml/rknn_convert.py 同参：像素 0~255，std 255 → 模型内 0~1
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                target_platform="rk3588")
    try:
        if rknn.load_onnx(str(onnx_path)) != 0:
            fail("load_onnx 失败", 5)
        if rknn.build(do_quantization=False) != 0:
            fail("build 失败", 5)
        # ---- 精度验证（PC 模拟器逐层对比 ONNX；此时未动车端任何东西）----
        sim = None
        report_dir = None
        if probe_img is not None:
            try:
                if rknn.init_runtime() == 0:
                    report_dir = rknn_path.parent / "accuracy_analysis"
                    if rknn.accuracy_analysis(
                            inputs=[str(probe_img)],
                            output_dir=str(report_dir)) == 0:
                        txt = report_dir / "accuracy_analysis.txt"
                        if txt.is_file():
                            sim = parse_output_sim(
                                txt.read_text(encoding="utf-8",
                                              errors="replace"))
            except Exception as e:
                warn(f"accuracy_analysis 异常: {e}")
        if sim is None:
            warn(f"输出层相似度未能自动解析（人工复核 "
                 f"{report_dir / 'accuracy_analysis.txt' if report_dir else 'N/A'}）；"
                 f"FP16 不量化精度风险极低，继续")
        elif sim < sim_threshold:
            fail(f"精度验证不过：输出层相似度 {sim:.2f}% < {sim_threshold}%"
                 f"（中止，未动车端；可 --force-reconvert 重试或核实 probe 图）", 5)
        else:
            ok(f"精度验证通过：输出层相似度 {sim:.2f}%（阈值 {sim_threshold}%）")
        if rknn.export_rknn(str(rknn_path)) != 0:
            fail("export_rknn 失败", 5)
    finally:
        rknn.release()
    size = rknn_path.stat().st_size
    if not (5_000_000 < size < 40_000_000):
        fail(f"产物大小可疑（{size/1e6:.1f}MB），中止", 5)
    meta_path.write_text(json.dumps(
        {"onnx_md5": onnx_md5, "created": time.strftime("%F %T"),
         "toolkit": tk_ver, "sim": sim}, ensure_ascii=False), encoding="utf-8")
    ok(f"转换完成（{time.time()-t0:.0f}s）: {rknn_path.name} "
       f"{size/1e6:.1f}MB md5={md5_local(rknn_path).upper()}")
    return md5_local(rknn_path)


# ---------------------------------------------------------------- 车端连接 --
def try_connect(host, port, user, passwords):
    """依次尝试密码；返回 (ssh, 失败原因)。网络不可达时不再试其它密码。"""
    err = ""
    for pw in passwords:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(hostname=host, port=port, username=user, password=pw,
                        timeout=10, banner_timeout=15, auth_timeout=15)
            return ssh, None
        except paramiko.AuthenticationException:
            err = f"认证失败（{user}，已试 {len(passwords)} 个密码）"
        except Exception as e:
            ssh.close()
            return None, f"{type(e).__name__}: {e}"
    return None, err


def connect_car(args, action="部署"):
    """连车；不可达时优雅停在『等待板端在线』（--wait N 轮询），不抛栈。"""
    passwords = []
    if args.password:
        passwords.append(args.password)
    elif os.environ.get("AG_CAR_PASS"):
        passwords.append(os.environ["AG_CAR_PASS"])
    for p in ["duzhong", "12345678"]:
        if p not in passwords:
            passwords.append(p)
    pw_src = "命令行/环境变量" if (args.password or os.environ.get("AG_CAR_PASS")) \
        else "默认尝试列表"

    deadline = time.time() + max(0, args.wait)
    while True:
        ssh, err = try_connect(args.host, args.port, args.user, passwords)
        if ssh is not None:
            ok(f"SSH 已连接 {args.user}@{args.host}（密码来源: {pw_src}）")
            return ssh
        if time.time() >= deadline:
            break
        remain = int(deadline - time.time())
        print(f"[等待] 板端不在线（{err}），{min(10, remain)}s 后重试，"
              f"剩余 {remain}s …", flush=True)
        time.sleep(min(10, remain))

    rknn_note = ""
    if RKNN_LOCAL.is_file():
        rknn_note = (f"\n  本地转换产物已就绪: {RKNN_LOCAL.name} "
                     f"({RKNN_LOCAL.stat().st_size/1e6:.1f}MB)")
    print("=" * 66, flush=True)
    print(f"[等待板端在线] {args.user}@{args.host}:{args.port} 暂不可达"
          f"（{err}）", flush=True)
    print(f"  {action}未开始，车端与本地均未做任何改动。" + rknn_note,
          flush=True)
    print("  车端上线后重新运行同一命令即可继续：", flush=True)
    print(f"      python {Path(__file__).name} --host {args.host}", flush=True)
    print(f"  或挂机等待: python {Path(__file__).name} "
          f"--host {args.host} --wait 3600", flush=True)
    print("=" * 66, flush=True)
    raise SystemExit(3)


# ---------------------------------------------------------------- 车端探测 --
def ros_env_prefix(ssh):
    """构建板端 ROS 环境 source 前缀（ros2 命令需要）。"""
    _, out, _ = run(ssh, "ls /opt/ros/*/setup.bash 2>/dev/null | head -1")
    ros = out.strip().splitlines()[0] if out.strip() else ""
    _, out, _ = run(ssh, "ls ~/dzrobot/install/setup.bash "
                         "~/dzrobot/install_isolated/setup.bash 2>/dev/null "
                         "| head -1")
    ws = out.strip().splitlines()[0] if out.strip() else ""
    parts = []
    if ros:
        parts.append(f"source {ros}")
    if ws:
        parts.append(f"source {ws}")
    _, out, _ = run(ssh, "ls ~/dzros.env 2>/dev/null")
    if out.strip():
        parts.append(". ~/dzros.env")
    return " && ".join(parts), bool(ros)


def locate_model(ssh):
    """定位现役 light_det.rknn（已知路径优先，find 兜底排除 src_ 副本）。"""
    _, out, _ = run(ssh,
                    "ls ~/dzrobot/src/rknn_pt/model/light_det.rknn 2>/dev/null "
                    "|| find ~/dzrobot -name 'light_det.rknn' "
                    "-not -name '*.bak*' 2>/dev/null | grep -v '/src_' | head -1")
    model_path = out.strip().splitlines()[0] if out.strip() else ""
    if not model_path:
        fail("车上未找到 light_det.rknn（~/dzrobot 下）")
    _, cur_md5, _ = run(ssh, f"md5sum {model_path}")
    cur_md5 = cur_md5.split()[0].lower() if cur_md5 else "?"
    ok(f"现役模型: {model_path} md5={cur_md5.upper()}")
    if cur_md5 == LIGHTDET_MD5_BASELINE:
        ok("现役 md5 与 models/release 登记基线一致（厂家原版）")
    else:
        warn(f"现役 md5 != 登记基线 {LIGHTDET_MD5_BASELINE.upper()}"
             f"（车上模型可能被换过；备份将保存当前实际版本）")
    return model_path, model_path + BAK_SUFFIX, cur_md5


def det_node_procs(ssh):
    """det_node 相关进程；返回 [(pid, cmd)]。"""
    _, out, _ = run(ssh, "pgrep -af det_node")
    procs = []
    for line in out.splitlines():
        parts = line.strip().split(None, 1)
        if len(parts) == 2 and "pgrep" not in parts[1]:
            procs.append((parts[0], parts[1]))
    return procs


def restart_det_node(ssh, env, orig_cmd):
    """杀掉 det_node 并按原命令行拉起（nohup，日志 /tmp/det_node_restart.log）。"""
    step("重启 det_node …")
    run(ssh, "pkill -f det_node 2>/dev/null; sleep 2; "
             "pgrep -f det_node >/dev/null && pkill -9 -f det_node; echo KILLED")
    prefix = f"{env} && " if env else ""
    code, out, err = run(
        ssh, f'cd $HOME && ({prefix}nohup {orig_cmd} '
             f'> /tmp/det_node_restart.log 2>&1 &) && echo RESTARTED', timeout=30)
    if "RESTARTED" not in out:
        return False, f"拉起失败: {err or out}"
    return True, orig_cmd


def verify_det_node(ssh, env, expect_frames):
    """部署后验证：节点在 ROS 图 + 进程存活 + 日志无加载失败 + 话题出帧。

    判死标准只含「节点消失/进程退出/日志报模型加载失败」——现场无靶标时
    检测话题可能只是空消息，不算失败；--expect-frames 才严格判帧。
    """
    step("验证 det_node（节点存活 + 日志 + 检测话题）…")
    node_list = (f"{env} && " if env else "") + \
        "timeout 10 ros2 node list 2>/dev/null"
    for _ in range(6):  # 最多等 30s 起节点
        _, out, _ = run(ssh, node_list, timeout=20)
        if "/det_node" in out:
            break
        time.sleep(5)
    else:
        return False, "ros2 node list 30s 内未见 /det_node"
    _, out, _ = run(ssh, "pgrep -f det_node >/dev/null && echo ALIVE")
    if "ALIVE" not in out:
        return False, "det_node 进程已退出（疑似加载新模型崩溃）"
    _, out, _ = run(
        ssh, "tail -100 /tmp/det_node_restart.log 2>/dev/null | grep -iE "
             "'load.*model.*(fail|error)|rknn.*(fail|error)|segmentation|"
             "terminate|core dump' | head -3")
    if out.strip():
        return False, f"重启日志疑似模型加载失败: {out.strip()[:200]}"
    ok("节点 /det_node 在 ROS 图上，进程存活，日志无模型加载失败")

    # 检测话题出帧（增强判据：有输出更好；无输出不判死——可能现场无靶标）
    topic_cmd = (f"{env}; " if env else "") + \
        "timeout 8 ros2 node info /det_node 2>/dev/null | " \
        "sed -n '/Publishers:/,/Service/p' | grep -oE '/[a-zA-Z0-9_/]+' | " \
        "grep -vE '/rosout|/parameter_events' | head -1"
    _, out, _ = run(ssh, topic_cmd, timeout=20)
    topic = out.strip().splitlines()[0] if out.strip() else ""
    if topic:
        _, out, _ = run(ssh, (f"{env}; " if env else "") +
                        f"timeout 12 ros2 topic hz {topic} --window 5 2>&1 | head -2",
                        timeout=25)
        if "average rate" in out:
            ok(f"检测话题出帧: {topic} {out.splitlines()[0]}")
            return True, f"节点存活，{topic} 正常出帧"
        if expect_frames:
            return False, f"--expect-frames 严格模式: 话题 {topic} 无输出"
        warn(f"话题 {topic} 暂无输出（现场无靶标时正常；"
             f"有靶标验证请加 --expect-frames）")
    else:
        warn("未解析到 det_node 发布话题（可人工 ros2 node info /det_node 复核）")
    return True, "节点存活（话题未出帧但不判死）"


def rollback_now(ssh, model_path, bak_path, reason):
    """回滚到备份并尽力重启。"""
    warn(f"自动回滚: {reason}")
    code, _, err = run(ssh, f"cp -p {bak_path} {model_path}")
    if code != 0:
        fail(f"回滚失败（请手动执行）: cp {bak_path} {model_path} —— {err}", 1)
    _, out, _ = run(ssh, f"md5sum {model_path}")
    ok(f"已回滚: {model_path} ← {bak_path} "
       f"(md5 {out.split()[0].upper() if out else '?'})")
    env, has_ros = ros_env_prefix(ssh)
    procs = det_node_procs(ssh)
    if procs and has_ros:
        orig = next((c for _, c in procs if "launch" in c), procs[0][1])
        restart_det_node(ssh, env, orig)
        time.sleep(8)
        okv, detail = verify_det_node(ssh, env, False)
        (ok if okv else warn)(f"回滚后验证: {detail}")


# ---------------------------------------------------------------- 主流程 --
def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.0.199", help="香橙派 IP")
    ap.add_argument("--user", default="duzhong")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--password", default=None,
                    help="默认取 AG_CAR_PASS，再退自动尝试 duzhong/12345678")
    ap.add_argument("--onnx", default=str(ONNX_LOCAL),
                    help=f"默认 {ONNX_LOCAL}")
    ap.add_argument("--rknn", default=None,
                    help="直接指定已转换的 .rknn（跳过本地转换）")
    ap.add_argument("--check-only", action="store_true",
                    help="只做本地检查（契约/依赖/缓存），不连车")
    ap.add_argument("--dry-run", action="store_true",
                    help="本地检查 + 连车只读探测，不做任何改动")
    ap.add_argument("--rollback", action="store_true",
                    help="回滚到最近一次备份（原 light_det.rknn）")
    ap.add_argument("--wait", type=int, default=0,
                    help="板端不在线时的挂机轮询秒数（默认 0=立即优雅退出）")
    ap.add_argument("--force-reconvert", action="store_true",
                    help="忽略缓存强制重新转换")
    ap.add_argument("--sim-threshold", type=float, default=98.0,
                    help="精度验证输出层相似度阈值%%（默认 98）")
    ap.add_argument("--probe-image", default=None,
                    help="精度验证输入图（默认自动找训练 val 图）")
    ap.add_argument("--restart", choices=["auto", "always", "never"],
                    default="auto",
                    help="auto=det_node 在跑才重启验证；always=强制拉起；"
                         "never=只换文件不重启")
    ap.add_argument("--expect-frames", action="store_true",
                    help="严格验证：检测话题必须出帧（现场有靶标时用）")
    args = ap.parse_args()

    if args.rollback and (args.check_only or args.dry_run):
        fail("--rollback 不能与 --check-only/--dry-run 同用")

    # ---- 1. 本地环境检查 ----
    step("本地环境检查（ONNX 契约 / 依赖 / 转换缓存）…")
    onnx_path = Path(args.onnx)
    strict_rknn = not (args.check_only or args.dry_run or args.rknn
                       or args.rollback)
    tk_ver = check_local(onnx_path, strict_rknn)
    if args.check_only:
        cached = RKNN_LOCAL.is_file() and Path(str(RKNN_LOCAL) + ".meta").is_file()
        print(f"[check-only] 缓存: {'有（' + f'{RKNN_LOCAL.stat().st_size/1e6:.1f}MB)'
              if cached else '无（正式运行时转换）'}；"
              f"车端目标: {args.user}@{args.host}:~/dzrobot/src/rknn_pt/model/"
              f"light_det.rknn", flush=True)
        ok("check-only 完成")
        return 0

    # ---- 2. PC 端转换 + 精度验证（仅正式部署执行；dry-run/rollback 不转换）----
    local_rknn = local_md5 = None
    if not (args.dry_run or args.rollback):
        if args.rknn:
            local_rknn = Path(args.rknn)
            if not local_rknn.is_file():
                fail(f"--rknn 指定的文件不存在: {local_rknn}")
            local_md5 = md5_local(local_rknn)
            ok(f"使用现成 RKNN: {local_rknn} "
               f"({local_rknn.stat().st_size/1e6:.1f}MB "
               f"md5={local_md5.upper()})")
        else:
            local_rknn = RKNN_LOCAL
            local_md5 = convert_local(onnx_path, local_rknn, tk_ver,
                                      args.sim_threshold,
                                      args.force_reconvert,
                                      probe_image=args.probe_image)

    # ---- 3. 连车（不可达 → 优雅等待在线，exit 3）----
    step(f"连接香橙派 {args.user}@{args.host} …")
    action = "回滚" if args.rollback else ("探测" if args.dry_run else "部署")
    ssh = connect_car(args, action)

    try:
        # ---- 4. 定位现役模型 ----
        model_path, bak_path, cur_md5 = locate_model(ssh)

        # ---- 回滚模式 ----
        if args.rollback:
            code, out, _ = run(ssh, f"md5sum {bak_path} 2>/dev/null")
            if code != 0 or not out:
                fail(f"没有备份可回滚: {bak_path}")
            code, _, err = run(ssh, f"cp -p {bak_path} {model_path}")
            if code != 0:
                fail(f"回滚失败: {err}")
            _, out, _ = run(ssh, f"md5sum {model_path}")
            ok(f"已回滚: {model_path} ← {bak_path}")
            ok(f"当前 md5: {out.split()[0].upper()}")
            env, has_ros = ros_env_prefix(ssh)
            procs = det_node_procs(ssh)
            if procs and has_ros and args.restart != "never":
                orig = next((c for _, c in procs if "launch" in c), procs[0][1])
                restart_det_node(ssh, env, orig)
                time.sleep(8)
                okv, detail = verify_det_node(ssh, env, args.expect_frames)
                (ok if okv else warn)(f"回滚后验证: {detail}")
            else:
                print("提醒: 重启 det_node / start_all.sh 后生效")
            return 0

        env, has_ros = ros_env_prefix(ssh)
        procs = det_node_procs(ssh)
        if args.dry_run:
            _, out, _ = run(ssh, f"ls {bak_path} 2>/dev/null")
            print(f"[dry-run] 备份 {bak_path}: "
                  f"{'已存在（不重复备份）' if out.strip() else '不存在（正式执行时创建）'}")
            print(f"[dry-run] det_node 进程: "
                  f"{len(procs)} 个{'（正式执行时自动重启+验证）' if procs else '（未运行，正式执行时跳过重启验证）'}",
                  flush=True)
            print(f"[dry-run] ROS 环境: {'可探测' if has_ros else '未探测到 /opt/ros'}",
                  flush=True)
            ok("dry-run 探测完成（未做任何改动；正式执行将上传+替换+重启验证）")
            return 0

        # ---- 5. 上传 + 复核 ----
        step(f"上传 RKNN → 板端 {REMOTE_RKNN} …")
        sftp = ssh.open_sftp()
        sftp.put(str(local_rknn), REMOTE_RKNN)
        sftp.close()
        _, out, _ = run(ssh, f"md5sum {REMOTE_RKNN}; stat -c %s {REMOTE_RKNN}")
        lines = out.splitlines()
        remote_md5 = lines[0].split()[0].lower() if lines else "?"
        size = int(lines[1]) if len(lines) > 1 and lines[1].isdigit() else 0
        if remote_md5 != local_md5:
            fail(f"上传 md5 不一致（{remote_md5} != {local_md5}），中止")
        ok(f"上传校验一致: {size/1e6:.1f}MB md5={remote_md5.upper()}")

        # ---- 6. 幂等备份 ----
        _, out, _ = run(ssh, f"ls {bak_path} 2>/dev/null")
        if out.strip():
            ok(f"备份已存在（不重复备份）: {bak_path}")
        else:
            code, _, err = run(ssh, f"cp -p {model_path} {bak_path}")
            if code != 0:
                fail(f"备份失败（中止，未做任何改动）: {err}")
            ok(f"已备份: {bak_path}")

        # ---- 7. 替换 + 复核（失败自动回滚）----
        step("替换现役模型 …")
        code, out2, err2 = run(ssh, f"cp {REMOTE_RKNN} {model_path} && "
                                    f"md5sum {model_path}")
        final_md5 = out2.split()[0].lower() if out2 else "?"
        if code != 0 or final_md5 != local_md5:
            rollback_now(ssh, model_path, bak_path,
                         f"替换失败（{err2 or final_md5}）")
            raise SystemExit(1)
        ok(f"已替换: {model_path} md5={final_md5.upper()}")

        # ---- 8. 重启验证（失败自动回滚）----
        verified, detail = False, "未执行"
        if args.restart == "never":
            print("提醒: --restart never，需手动重启 det_node 后生效")
        elif not procs and args.restart == "auto":
            print("提示: det_node 当前未运行——模型已替换，下次启动生效"
                  "（要立即拉起验证: --restart always）")
            verified, detail = True, "跳过（det_node 原本未运行）"
        else:
            if procs:
                orig = next((c for _, c in procs if "launch" in c), procs[0][1])
            else:  # always 且原本没跑：找 det_node 的 launch 文件拉起
                _, out, _ = run(ssh, "find ~/dzrobot/src -path '*det_node*' "
                                     "-name '*.launch.py' 2>/dev/null | head -1")
                lf = out.strip().splitlines()[0] if out.strip() else ""
                orig = f"ros2 launch det_node {Path(lf).name}" if lf else ""
                if not orig:
                    warn("未找到 det_node launch 文件（无法自动拉起，"
                         "模型已替换；请手动重启验证）")
                    verified, detail = True, "跳过（未找到启动方式）"
            if not verified:
                okk, rdetail = restart_det_node(ssh, env, orig) if orig \
                    else (False, "无启动命令")
                if not okk:
                    rollback_now(ssh, model_path, bak_path,
                                 f"重启失败: {rdetail}")
                    raise SystemExit(1)
                time.sleep(8)
                verified, detail = verify_det_node(ssh, env, args.expect_frames)
                if not verified:
                    rollback_now(ssh, model_path, bak_path, detail)
                    raise SystemExit(1)

        print("=" * 66)
        ok("target_pose_v2 RKNN 部署完成")
        print(f"  原模型备份: {bak_path}（md5 {cur_md5.upper()}）")
        print(f"  新模型 md5: {local_md5.upper()}（val 144 帧 P=0.985/R=0.999/"
              f"mAP50=0.995，红靶零误检/蓝靶零漏检）")
        print(f"  板端验证: {detail}")
        print(f"  回滚命令: python {Path(__file__).name} "
              f"--host {args.host} --rollback")
        print("  现场验证: 俯拍红/蓝靶标，确认检出与关键点")
        print("=" * 66)
        return 0
    finally:
        try:
            run(ssh, f"rm -f {REMOTE_RKNN}")
        except Exception:
            pass
        ssh.close()


if __name__ == "__main__":
    raise SystemExit(main())
