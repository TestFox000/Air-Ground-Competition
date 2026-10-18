# -*- coding: utf-8 -*-
"""v4d 引擎转换（nohup+短连轮询）+ engines_d13 切换 + L3 重跑。"""
import sys
import time

sys.path.insert(0, r"C:\Users\29212\Documents\kimi\tasks\2026-10-14"
                 r"\18-59-27-5bee8702\air_ground\tools")
import remote_run  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

HOST, USER, PW = "192.168.55.1", "duzhong", "duzhong"
RROOT = "~/dzuav_deploy/air_ground"
EDIR = "~/dzuav/engines_d13"
TRTEXEC = "/usr/src/tensorrt/bin/trtexec"
LOG = "$HOME/v4d_engine.log"
ENGINE = f"{EDIR}/cell_classifier_v4d_fp16.engine"


def short(cmd, timeout=45):
    """短命令：只连 45s 内能完成的操作，断连抛异常由外层重试。"""
    return remote_run.run(HOST, USER, PW, cmd, timeout=timeout)


def phase(name, fn, tries=8, wait=15):
    for i in range(tries):
        try:
            return fn()
        except Exception as ex:
            print(f"[{name}] {i+1}/{tries} {type(ex).__name__}", flush=True)
            time.sleep(wait)
    return None


# ---------- 1. nohup 启动 v4d 转换（幂等：已有引擎则跳过） ----------
def kick():
    # 注意：--saveEngine=~/... 的 ~ 在 = 后不被 bash 展开（词首才展开），
    # trtexec 会往字面 '~' 目录保存而失败 —— 必须用 $HOME 显式展开。
    code, out, err = short(
        f"if [ -f {ENGINE} ]; then echo ALREADY; else "
        f"nohup bash -c '{TRTEXEC} --onnx=$HOME/dzuav/models/"
        f"cell_classifier_v4d_20261015.onnx --saveEngine=$HOME/dzuav/engines_d13/"
        f"cell_classifier_v4d_fp16.engine --fp16 > {LOG} 2>&1 && "
        f"echo BUILD_DONE >> {LOG} || echo BUILD_FAIL >> {LOG}' "
        f">/dev/null 2>&1 & echo KICKED; fi",
        timeout=30)
    txt = (out or err or "").strip()
    if txt not in ("ALREADY", "KICKED"):
        raise IOError(txt)
    return txt


print("==> [1] 启动 v4d 转换（nohup 后台）", flush=True)
r = phase("kick", kick, tries=6, wait=12)
if r is None:
    sys.exit(2)
print(f"    {r}", flush=True)

# ---------- 2. 短连轮询转换结果 ----------
print("==> [2] 轮询 v4d 转换", flush=True)
done = False
for i in range(40):  # 最长 ~10 分钟
    time.sleep(15)
    def poll():
        code, out, _ = short(
            f"grep -c BUILD_DONE {LOG} 2>/dev/null; "
            f"grep -c BUILD_FAIL {LOG} 2>/dev/null; ls -la {ENGINE} 2>/dev/null",
            timeout=30)
        return (out or "").strip()
    r = phase("poll", poll, tries=3, wait=8)
    if r is None:
        continue
    lines = r.splitlines()
    if len(lines) >= 2 and lines[0].strip() == "1":
        print(f"    转换完成: {lines[2] if len(lines) > 2 else ''}", flush=True)
        done = True
        break
    if len(lines) >= 2 and lines[1].strip() == "1":
        print("[FAIL] trtexec 失败", flush=True)
        phase("log", lambda: print(short(f"tail -5 {LOG}", 30)[1]), tries=2)
        sys.exit(2)
    print(f"    [{i}] 构建中...", flush=True)
if not done:
    print("[FAIL] 轮询超时")
    sys.exit(2)


# ---------- 3. engines_d13 切换 v4d ----------
def swap():
    code, out, err = short(
        f"rm -f {EDIR}/cell_classifier_fp16.engine && ls {EDIR}", timeout=30)
    txt = (out or err or "")
    if "cell_classifier_v4d_fp16.engine" not in txt:
        raise IOError(txt)
    return txt


print("==> [3] engines_d13 切到 v4d", flush=True)
r = phase("swap", swap)
if r is None:
    sys.exit(2)
print("    " + r.replace("\n", " "), flush=True)


# ---------- 4. L3 重跑 ----------
def run_l3():
    code, out, err = remote_run.run(
        HOST, USER, PW,
        f"cd {RROOT} && nohup bash -c 'tools/orin_build_verify.sh --no-mirror "
        f"--run {EDIR} $HOME/dzuav_deploy/air_ground/l3_field_88cells.jpg "
        f"> $HOME/l3_run.log 2>&1; echo EXIT=$? >> $HOME/l3_run.log' "
        f">/dev/null 2>&1 & echo L3_STARTED", timeout=30)
    txt = (out or err or "").strip()
    if "L3_STARTED" not in txt:
        raise IOError(txt)
    return True


def poll_l3():
    code, out, _ = short(f"tail -25 $HOME/l3_run.log 2>/dev/null", timeout=30)
    txt = (out or "").strip()
    if "EXIT=" in txt:
        return txt
    return None


print("==> [4] L3 重跑（后台+轮询）", flush=True)
r = phase("L3启动", run_l3, tries=5, wait=15)
if r is None:
    sys.exit(2)
result = None
for i in range(20):
    time.sleep(12)
    result = phase("L3轮询", poll_l3, tries=3, wait=8)
    if result:
        break
    print(f"    [{i}] L3 运行中...", flush=True)
if not result:
    print("[FAIL] L3 轮询超时")
    sys.exit(2)
print(result, flush=True)
if "L3 全链路通过" in result:
    print("\n[done] L3 全链通过（v4d 格号引擎生效）")
elif "L3a 通过" in result:
    print("\n[PARTIAL] 仍 localized=0")
else:
    sys.exit(2)
