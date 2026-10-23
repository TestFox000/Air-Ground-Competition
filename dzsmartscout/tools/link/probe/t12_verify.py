#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t12_verify.py —— t12 修复的真机验证总控（本地 Windows 跑，指挥无人机 + 小车 /tmp 探针）

流程：
  S0 复现（改之前）：无人机上跑旧版 listen，抓 UnboundLocalError 栈 + 退出码
  S1 部署：备份旧文件 → scp 修复版 supervisor/sender/两个单测 → md5 核对 → 远端 py_compile → 远端跑单测
  S2 D2 真机：`run --bridge lower=K9 --bridge upper=K4 --duration 8` → intel_emitted>0 + 事件输出
  S3 D1 真机：无人机后台 listen（记录退出码），小车 /tmp 探针发 ack+car_state → 退出码 0 且无栈
  S4 收尾：停掉探针、列出证据

车侧只在 /tmp 放探针（t4_car_sender.py），不碰 ~/dzrobot/** 与 dzrobot_v2/**。
"""
import argparse
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh as S  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
RAW = os.path.join(ROOT, "_tmp_session", "t12", "raw")
os.makedirs(RAW, exist_ok=True)
LINK = os.path.join(ROOT, "dzsmartscout", "tools", "link")
REMOTE = "/home/duzhong/dzlink_tools"


def save(name, text):
    p = os.path.join(RAW, name)
    with open(p, "w", encoding="utf-8", errors="replace") as f:
        f.write(text)
    print("    [saved] %s (%d chars)" % (name, len(text)))
    return p


def ex(dev, cmd, timeout=180):
    c = S.connect(dev)
    try:
        return S.run(c, cmd, timeout=timeout)
    finally:
        c.close()


def step(t):
    print("\n" + "=" * 78 + "\n" + t + "\n" + "=" * 78)


def s0_reproduce():
    step("S0 复现（改之前，无人机上现役旧版）：listen 退出崩溃")
    rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash -c "
                          "'cd %s && timeout 20 python3 dzlink_supervisor.py listen --duration 3; "
                          "echo LISTEN_EXIT=$?'" % REMOTE)
    save("s0_pre_fix_listen.txt", out)
    print(out[-1200:])
    return {"unbound": "UnboundLocalError" in out,
            "exit0": "LISTEN_EXIT=0" in out}


def s1_deploy():
    step("S1 备份 + 部署修复版 + md5 + 远端自检")
    ts = time.strftime("%Y%m%d_%H%M%S")
    rc, out = ex("drone", "mkdir -p %s/_backups/t12_%s && cp -p %s/dzlink_supervisor.py "
                          "%s/dz_send_waypoint.py %s/_backups/t12_%s/ 2>/dev/null; "
                          "ls -l %s/_backups/t12_%s/; md5sum %s/dzlink_supervisor.py %s/dz_send_waypoint.py"
                % (REMOTE, ts, REMOTE, REMOTE, REMOTE, ts, REMOTE, ts, REMOTE, REMOTE))
    save("s1_backup.txt", out)
    print(out)

    files = [
        (os.path.join(LINK, "dzlink_supervisor.py"), REMOTE + "/dzlink_supervisor.py"),
        (os.path.join(LINK, "dz_send_waypoint.py"), REMOTE + "/dz_send_waypoint.py"),
        (os.path.join(LINK, "tests", "test_dzlink_supervisor.py"), REMOTE + "/test_dzlink_supervisor.py"),
        (os.path.join(LINK, "tests", "test_dz_send_waypoint.py"), REMOTE + "/test_dz_send_waypoint.py"),
    ]
    c = S.connect("drone")
    local_md5 = {}
    try:
        for local, remote in files:
            n = S.put(c, local, remote)
            import hashlib
            h = hashlib.md5(open(local, "rb").read().replace(b"\r\n", b"\n")).hexdigest()
            local_md5[remote] = h
            print("  put %-58s %6d bytes  md5=%s" % (remote, n, h))
        rc, out = S.run(c, "md5sum " + " ".join(r for _, r in files))
        save("s1_remote_md5.txt", out)
        print(out)
        ok = all(local_md5[r] in out for _, r in files)
        rc, out2 = S.run(c, "python3 -m py_compile %s/dzlink_supervisor.py %s/dz_send_waypoint.py "
                            "&& echo PY_COMPILE_OK" % (REMOTE, REMOTE))
        save("s1_remote_py_compile.txt", out2)
        print(out2.strip())
        rc, out3 = S.run(c, "cd %s && python3 test_dzlink_supervisor.py 2>&1 | tail -4; "
                            "python3 test_dz_send_waypoint.py 2>&1 | tail -4" % REMOTE, timeout=300)
        save("s1_remote_unit_tests.txt", out3)
        print(out3)
    finally:
        c.close()
    return ok


def s2_d2_real():
    step("S2 D2 真机：run --bridge/--target/--mines/--obstacles 生效")
    cmd = ("bash /tmp/t4_rosenv.sh bash -c 'cd %s && python3 dzlink_supervisor.py run "
           "--bridge lower=K9 --bridge upper=K4 --target G10,H10 --mines A3 --obstacles K7 "
           "--duration 8 --heartbeat-hz 1 --bridge-period 2 --log-file /tmp/t12_run.jsonl'"
           % REMOTE)
    rc, out = ex("drone", cmd, timeout=120)
    save("s2_run_real.txt", out)
    print(out[-2500:])
    rc, out2 = ex("drone", "grep -c '\"kind\": \"intel_emit\"' /tmp/t12_run.jsonl 2>/dev/null || "
                           "grep -c intel_emit /tmp/t12_run.jsonl; echo ---; "
                           "grep -o '\"kind\": \"[a-z_]*\"' /tmp/t12_run.jsonl | sort | uniq -c")
    save("s2_run_log_kinds.txt", out2)
    print(out2)
    m = re.search(r'"intel_emitted":\s*(\d+)', out)
    intel = int(m.group(1)) if m else -1
    return {"intel_emitted": intel,
            "bridge_lines": out.count("bridge_passable"),
            "target_lines": out.count("target_zone"),
            "field_lines": out.count("field_verify")}


def s3_d1_real():
    step("S3 D1 真机：listen 退出码 0 且无 UnboundLocalError 栈（复跑 t4 的 s6 场景）")
    # 车侧探针（只放 /tmp）
    c = S.connect("car")
    try:
        S.put(c, os.path.join(HERE, "t4_car_sender.py"), "/tmp/t12_car_sender.py")
    finally:
        c.close()

    wrapper = ("#!/bin/bash\n"
               "bash /tmp/t4_rosenv.sh bash -c 'cd %s && python3 dzlink_supervisor.py listen "
               "--duration 14 --log-file /tmp/t12_listen.jsonl' > /tmp/t12_listen.out 2>&1\n"
               "echo \"LISTEN_EXIT=$?\" >> /tmp/t12_listen.out\n" % REMOTE)
    with open(os.path.join(RAW, "t12_listen_run.sh"), "w", newline="\n") as f:
        f.write(wrapper)
    c = S.connect("drone")
    try:
        S.put(c, os.path.join(RAW, "t12_listen_run.sh"), "/tmp/t12_listen_run.sh")
        rc, out = S.run(c, "setsid nohup bash /tmp/t12_listen_run.sh > /tmp/t12_listen_outer.log "
                           "2>&1 < /dev/null & sleep 1; echo STARTED")
        print("  listen started: %s" % out.strip())
    finally:
        c.close()

    time.sleep(4)
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t12_car_sender.py --ack-of 7 "
                        "--car-state --block K9 --yaw-deg 180.0 --phase DRIVE --period 0.7 "
                        "--duration 9 --out /tmp/t12_car_sent.jsonl", timeout=120)
    save("s3_car_sender.txt", out)
    print(out[-900:])
    time.sleep(8)
    rc, out = ex("drone", "cat /tmp/t12_listen.out")
    save("s3_drone_listen.txt", out)
    print("\n" + out[-2500:])
    return {"unbound": "UnboundLocalError" in out,
            "exit0": "LISTEN_EXIT=0" in out,
            "count_line": [l for l in out.splitlines() if l.startswith("[listen] 收到")]}


def s4_cleanup():
    step("S4 收尾：停探针 + 车侧零改动复查")
    rc, out = ex("car", "pkill -f '/tmp/t12_car_sender.py' ; sleep 1; "
                        "ps -eo pid,cmd | grep -F '/tmp/t12_' | grep -v grep || echo '(车侧 t12 探针已清)'; "
                        "pgrep -af 'dzbrain|path_executor' || echo '(车栈仍冷态)'")
    save("s4_car_cleanup.txt", out)
    print(out)
    rc, out = ex("car", "find /home/duzhong/dzrobot -type f -newermt '2026-10-12 19:35:00' "
                        "-not -path '*/install/*' -not -path '*/build/*' -not -path '*/log/*' "
                        "2>/dev/null | head; echo '(空=~/dzrobot 未被触碰)'")
    save("s4_car_untouched.txt", out)
    print(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", default="0,1,2,3,4")
    a = ap.parse_args()
    want = set(a.steps.split(","))
    res = {}
    if "0" in want:
        res["s0"] = s0_reproduce()
    if "1" in want:
        res["s1_md5_ok"] = s1_deploy()
    if "2" in want:
        res["s2"] = s2_d2_real()
    if "3" in want:
        res["s3"] = s3_d1_real()
    if "4" in want:
        s4_cleanup()
    print("\n### 汇总: " + json.dumps(res, ensure_ascii=False, indent=2))
    with open(os.path.join(RAW, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
