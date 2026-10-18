# -*- coding: utf-8 -*-
"""D13 引擎 L3 全链验证：17 通道契约 + 推理 + UDP 协议闭环（车端解析器）。

验证编排：
  1. 硬链接组独立引擎目录 ~/dzuav/engines_d13/（d13 hbb + 现役 cell），
     避免与 engines/ 里旧 hbb 引擎产生 *hbb* 匹配歧义
  2. 跑 orin_build_verify.sh --no-mirror --run（L0~L3 全链）
  3. L3a 关键点：HbbDetector 加载 d13 引擎时执行 17 通道契约校验（fe1d9df）
  4. L3b 关键点：UDP 帧过车端 intel_protocol.parse + 13 类 names 显示
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, r"C:\Users\29212\Documents\kimi\tasks\2026-10-14"
                 r"\18-59-27-5bee8702\air_ground\tools")
import remote_run  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HOST, USER, PW = "192.168.55.1", "duzhong", "duzhong"
RROOT = "~/dzuav_deploy/air_ground"
ENGINE_SRC = "~/dzuav/engines/hbb_detector_d13_fp16.engine"
EDIR = "~/dzuav/engines_d13"
TEST_IMG = f"{RROOT}/l3_test.jpg"


def run(cmd, timeout=300):
    return remote_run.run(HOST, USER, PW, cmd, timeout=timeout)


# ---------- 1. 组独立引擎目录（硬链接，-type f 可见） ----------
print("==> [1] 组装独立引擎目录")
code, out, err = run(
    f"mkdir -p {EDIR} && "
    f"ln -f {ENGINE_SRC} {EDIR}/hbb_detector_d13_fp16.engine && "
    f"ln -f ~/dzuav/engines/cell_classifier_fp16.engine "
    f"{EDIR}/cell_classifier_fp16.engine && ls -la {EDIR}")
print((out or err).strip())
if "d13" not in (out or ""):
    print("[FAIL] 引擎目录组装失败（引擎是否已生成？）")
    sys.exit(2)

# ---------- 2. L3 全链验证 ----------
print("==> [2] orin_build_verify.sh --run（L0~L3，含 17 通道契约）")
t0 = time.time()
code, out, err = run(
    f"cd {RROOT} && tools/orin_build_verify.sh --no-mirror "
    f"--run {EDIR} {TEST_IMG} 2>&1 | tail -45",
    timeout=900)
print((out or err).strip())
print(f"[耗时 {time.time()-t0:.0f}s，退出码 {code}]")

ok = "L3 全链路通过" in (out or "")
partial = ("L3a 通过" in (out or "")) and not ok
if ok:
    print("\n[done] D13 L3 全链通过：17通道契约 + 推理 + UDP 协议闭环")
elif partial:
    print("\n[PARTIAL] L3a 推理通过但本图定位失败（localized=0）——"
          "引擎与契约已验证，换实拍图可补 L3b")
else:
    print("\n[FAIL] L3 未通过，检查上方输出")
    sys.exit(2)
