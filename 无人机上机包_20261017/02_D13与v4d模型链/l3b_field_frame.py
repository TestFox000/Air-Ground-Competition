# -*- coding: utf-8 -*-
"""L3b 补验（链路鲁棒版）：幂等推送 88 格号帧 + L3 全链，每阶段独立重试。"""
import sys
import time
from pathlib import Path

sys.path.insert(0, r"C:\Users\29212\Documents\kimi\tasks\2026-10-14"
                 r"\18-59-27-5bee8702\air_ground\tools")
import remote_run  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HOST, USER, PW = "192.168.55.1", "duzhong", "duzhong"
IMG = Path(r"C:\Users\29212\Documents\Trae-CN\default\plan_compare_pack_20261016"
           r"\aerial_field_v2_paper_local\images\train\frame_00124.jpg")
RROOT = "~/dzuav_deploy/air_ground"
EDIR = "~/dzuav/engines_d13"
REMOTE_IMG = f"{RROOT}/l3_field_88cells.jpg"
EXPECT_KB = IMG.stat().st_size // 1024


def try_phase(name, fn, tries=6, wait=20):
    for i in range(tries):
        try:
            r = fn()
            if r is not None:
                print(f"[{name}] 第 {i+1} 次成功")
                return r
        except SystemExit:
            raise
        except Exception as ex:
            print(f"[{name}] 第 {i+1}/{tries} 次失败: {type(ex).__name__} {ex}")
        time.sleep(wait)
    print(f"[{name}] 多次失败，放弃")
    return None


def img_onboard():
    code, out, _ = remote_run.run(HOST, USER, PW,
                                  f"stat -c %s {REMOTE_IMG} 2>/dev/null",
                                  timeout=60)
    size = (out or "").strip()
    return int(size) if size.isdigit() else None


def push_img():
    remote_run.sftp_put(HOST, USER, PW, str(IMG), REMOTE_IMG, timeout=300)
    size = img_onboard()
    if size and abs(size - IMG.stat().st_size) < 1024:
        return size
    raise IOError(f"推送后校验不符: {size} vs {IMG.stat().st_size}")


def run_l3():
    t0 = time.time()
    code, out, err = remote_run.run(
        HOST, USER, PW,
        f"cd {RROOT} && tools/orin_build_verify.sh --no-mirror "
        f"--run {EDIR} {REMOTE_IMG} 2>&1 | tail -30",
        timeout=900)
    txt = (out or err or "").strip()
    if not txt:
        raise IOError("L3 输出为空（断连）")
    print(txt)
    print(f"[耗时 {time.time()-t0:.0f}s]")
    return txt


# 阶段 1：图在位即可跳过推送
size = try_phase("查图", img_onboard, tries=3, wait=10)
if size is None:
    print(f"==> 图不在位，推送 ({EXPECT_KB}KB)")
    size = try_phase("推图", push_img)
    if size is None:
        sys.exit(2)
else:
    print(f"==> 图已在位 {size//1024}KB")

# 阶段 2：L3
txt = try_phase("L3", run_l3, tries=4, wait=25)
if txt is None:
    sys.exit(2)
if "L3 全链路通过" in txt:
    print("\n[done] D13 L3 全链通过：17通道契约 + 推理 + 定位 + UDP 协议闭环")
elif "L3a 通过" in txt:
    print("\n[PARTIAL] 仍 localized=0")
else:
    print("\n[FAIL]")
    sys.exit(2)
