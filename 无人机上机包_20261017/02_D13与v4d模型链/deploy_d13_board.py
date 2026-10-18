# -*- coding: utf-8 -*-
"""D13 上板收尾：L2 自检 + ONNX/ag_integrate 推送 + TRT 引擎转换(nohup) + 轮询。

前置：orin_deploy.py 已完成代码推送与 L1 编译（本脚本接续断连遗留的 L2）。
引擎落位 ~/dzuav/engines/hbb_detector_d13_fp16.engine（独立命名，不碰现役引擎）。
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, r"C:\Users\29212\Documents\kimi\tasks\2026-10-14"
                 r"\18-59-27-5bee8702\air_ground\tools")
import remote_run  # noqa: E402  标准车端/机载操作规范

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HOST, USER, PW = "192.168.55.1", "duzhong", "duzhong"
REPO = Path(r"C:\Users\29212\Documents\kimi\tasks\2026-10-14"
            r"\18-59-27-5bee8702\air_ground")
PACK = Path(r"C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016")
RROOT = "~/dzuav_deploy/air_ground"
ONNX_NAME = "hbb_detector_d13_20261016.onnx"
ENGINE_PATH = "$HOME/dzuav/engines/hbb_detector_d13_fp16.engine"
LOG = "$HOME/d13_engine_build.log"
TRTEXEC = "/usr/src/tensorrt/bin/trtexec"


def run(cmd, timeout=180):
    return remote_run.run(HOST, USER, PW, cmd, timeout=timeout)


# ---------- 1. L2 二进制自检（接续被断连打断的一步） ----------
print("==> [1] L2 二进制自检")
code, out, err = run(
    "set -o pipefail; ~/dzuav_deploy/air_ground/onboard/dzuav/build/dzuav "
    "2>&1 | head -4; rc=${PIPESTATUS[0]}; echo EXIT=$rc")
print((out or err).strip())
ok = "EXIT=2" in (out or "") and "usage:" in (out or "")
print("[L2 通过]" if ok else "[L2 失败]")
if not ok:
    sys.exit(2)

# ---------- 2. 推 D13 ONNX 到 models/release（不动指针） ----------
print("==> [2] 推送 D13 ONNX (38.5MB)")
n = remote_run.sftp_put(HOST, USER, PW, str(PACK / "models" / "hbb_d13.onnx"),
                        f"{RROOT}/models/release/{ONNX_NAME}", timeout=900)
code, out, _ = run(f"md5sum {RROOT}/models/release/{ONNX_NAME}")
print(out.strip())
local_md5 = remote_run.run  # noqa: F841  占位避免误用
import hashlib  # noqa: E402
h = hashlib.md5((PACK / "models" / "hbb_d13.onnx").read_bytes()).hexdigest()
remote_md5 = (out.split()[0] if out else "").strip()
print(f"本地 md5 {h}")
print(f"[md5 {'一致' if h == remote_md5 else '不一致!'}]")
if h != remote_md5:
    sys.exit(2)

# ---------- 3. 补推 ag_integrate（L3 监听器依赖 intel_protocol） ----------
print("==> [3] 推送 src/ag_integrate")
n = remote_run.sftp_put(HOST, USER, PW, str(REPO / "src" / "ag_integrate"),
                        f"{RROOT}/src/ag_integrate", timeout=300)
print(f"[已上传] {n} 个文件")

# ---------- 4. 启动引擎转换（nohup 防断连） ----------
print("==> [4] 启动 trtexec FP16 引擎转换（后台）")
code, out, err = run(
    f"nohup {TRTEXEC} --onnx={RROOT}/models/release/{ONNX_NAME} "
    f"--saveEngine={ENGINE_PATH} --fp16 > {LOG} 2>&1 & echo PID=$!")
print((out or err).strip())

# ---------- 5. 轮询直到引擎生成（最长 40 分钟） ----------
print("==> [5] 轮询引擎构建...")
deadline = time.time() + 40 * 60
last_len = -1
while time.time() < deadline:
    time.sleep(30)
    try:
        code, out, _ = run(
            f"pgrep -f 'trtexec.*d13' >/dev/null && echo RUNNING || echo DONE; "
            f"ls -la {ENGINE_PATH} 2>/dev/null; tail -c 400 {LOG}")
    except Exception as ex:
        print(f"[poll 重试] {type(ex).__name__}")
        continue
    txt = (out or "").strip()
    lines = txt.splitlines()
    state = lines[0] if lines else "?"
    if len(txt) != last_len:
        print(f"[{time.strftime('%H:%M:%S')}] {state}")
        for ln in lines[1:]:
            print("   " + ln)
        last_len = len(txt)
    if state == "DONE":
        if "hbb_detector_d13_fp16.engine" in txt:
            print("\n[done] D13 引擎生成完毕")
            sys.exit(0)
        print("\n[FAIL] trtexec 结束但引擎未生成，看上方日志")
        sys.exit(2)
print("\n[FAIL] 40 分钟超时")
sys.exit(2)
