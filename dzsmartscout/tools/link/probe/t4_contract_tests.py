#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
t4_contract_tests.py —— t4 契约层真机验证总控（本地 Windows 跑，指挥两端）

九步（每步原始输出都落 _tmp_session/t4/raw/，报告里的每条证据都能回溯）：
  S1 互发现：两端各挂一个话题，交叉 ros2 topic list / topic info -v
  S2 契约 QoS 线上抓取：/dzlink/drone_to_car 上对端 Publisher 的真实 QoSProfile
  S3 载荷对账 机→车：dz_send_waypoint.py 的真实报文 → 车侧只读消费者逐字段
  S4 负向对照（真契约话题）：BEST_EFFORT 发 → RELIABLE 收（应 0 条）
  S5 /nav/waypoint 正向：supervisor route（RELIABLE）→ 车侧 RELIABLE 消费者
  S6 载荷对账 车→机：车侧按 dzbrain 逐字复刻的 ack/car_state → 无人机生产监听器
  S7 多发布者：/dzlink/drone_to_car 从 0 → 1 → 2 个 Publisher 的现场计数
  S8 定量组播：双向 20 包计数
  S9 车侧零改动：freeze begin/end + 新建文件清单
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t4_ssh as S  # noqa: E402

ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
RAW = os.path.join(ROOT, "_tmp_session", "t4", "raw")
os.makedirs(RAW, exist_ok=True)

STEP = [0]


def save(name, text):
    p = os.path.join(RAW, name)
    with open(p, "w", encoding="utf-8", errors="replace") as f:
        f.write(text)
    print("    [saved] %s (%d chars)" % (name, len(text)))
    return p


def ex(dev, cmd, timeout=180):
    c = S.connect(dev)
    try:
        rc, out = S.run(c, cmd, timeout=timeout)
        return rc, out
    finally:
        c.close()


def bg(dev, inner, log):
    cmd = ("setsid nohup bash -c %s > %s 2>&1 < /dev/null & echo BG_PID=$!; sleep 1; exit 0"
           % (S.shell_quote(inner), log))
    return ex(dev, cmd)


def getf(dev, remote, name):
    c = S.connect(dev)
    try:
        local = os.path.join(RAW, name)
        try:
            S.get(c, remote, local)
            print("    [got] %s <- %s" % (name, remote))
            return local
        except Exception as e:
            print("    [get failed] %s: %s" % (remote, e))
            return None
    finally:
        c.close()


def ros(dev, cmd):
    """在远端以正确 ROS 环境跑命令"""
    return ex(dev, "bash /tmp/t4_rosenv.sh %s" % cmd)


def step(title):
    STEP[0] += 1
    print("\n" + "=" * 78)
    print("STEP %d  %s  [%s]" % (STEP[0], title, time.strftime("%H:%M:%S")))
    print("=" * 78)


# --------------------------------------------------------------------------- #
def s1b_drone_topicinfo():
    step("S1b 无人机侧互发现（补做：ros2 CLI daemon 修好之后）")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub --topic /t4/disc/car2 "
              "--qos best_effort --rate 1 --duration 60 --tag CAR2 > /dev/null",
       "/tmp/t4_disc_car2.log")
    bg("drone", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub --topic /t4/disc/drone2 "
                "--qos reliable --rate 1 --duration 60 --tag DRONE2 > /dev/null",
       "/tmp/t4_disc_drone2.log")
    time.sleep(10)
    rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash /tmp/t4_topicinfo.sh /t4/disc/car2")
    save("s1b_drone_topicinfo.txt", out)
    print(out)
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh bash /tmp/t4_topicinfo.sh /t4/disc/drone2")
    save("s1b_car_topicinfo.txt", out)


def s3b_payload_round2():
    step("S3b 载荷对账补全（正确语法 target/field + 逗号误用对照 + 容错对照）")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py --qos best_effort "
              "--duration 50 --out /tmp/t4_car_recv_contract2.jsonl > /tmp/t4_s3b.log 2>&1",
       "/tmp/t4_s3b_outer.log")
    time.sleep(6)
    cmds = [
        ("正确语法 target", "python3 ~/dzlink_tools/dz_send_waypoint.py target G10 H10"),
        ("正确语法 field", "python3 ~/dzlink_tools/dz_send_waypoint.py field --mines A3 B4 --obstacles K7"),
        ("★误用（逗号形式）field", "python3 ~/dzlink_tools/dz_send_waypoint.py field --mines A3,B4 --obstacles K7"),
        ("★误用 target --blocks", "python3 ~/dzlink_tools/dz_send_waypoint.py target --blocks G10,H10"),
        ("容错：非 JSON 裸串", "python3 ~/dzlink_tools/dz_send_waypoint.py route 'HELLO-NOT-JSON'"),
    ]
    allc = ""
    for title, c in cmds:
        rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash -lc %s" % S.shell_quote(c))
        allc += "\n### %s\n$ %s\n(exit=%d)\n%s\n" % (title, c, rc, out)
    save("s3b_drone_sends.txt", allc)
    print(allc[:6000])
    time.sleep(3)
    getf("car", "/tmp/t4_car_recv_contract2.jsonl", "s3b_car_received.jsonl")
    rc, out = ex("car", "tail -60 /tmp/t4_s3b.log")
    save("s3b_car_consumer_summary.txt", out)
    print(out[-3500:])


def s6b_ack_loop():
    step("S6b 闭环：机发 seq → 车按 dzbrain 规则回 ack → 机侧 ack 台账 matched")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py --qos best_effort "
              "--auto-ack --duration 40 --out /tmp/t4_car_recv_loop.jsonl "
              "> /tmp/t4_s6b_car.log 2>&1", "/tmp/t4_s6b_car_outer.log")
    time.sleep(6)
    bg("drone", "bash /tmp/t4_rosenv.sh bash -c "
                "'cd /home/duzhong/dzlink_tools && python3 dzlink_supervisor.py run "
                "--domain 118 --duration 30 --bridge lower=K9 --bridge upper=K4 "
                "--heartbeat-hz 1 --bridge-period 2 --status-period 10 "
                "--log-file /tmp/t4_s6b_sup.jsonl > /tmp/t4_s6b_sup.out 2>&1'",
       "/tmp/t4_s6b_outer.log")
    time.sleep(36)
    rc, out = ex("drone", "cat /tmp/t4_s6b_sup.out")
    save("s6b_drone_supervisor_out.txt", out)
    print(out[-4000:])
    rc, out = ex("car", "grep -c 'rx OK' /tmp/t4_s6b_car.log; tail -8 /tmp/t4_s6b_car.log")
    save("s6b_car_consumer.txt", out)
    print(out[-2000:])
    getf("drone", "/tmp/t4_s6b_sup.jsonl", "s6b_drone_supervisor.jsonl")


def s7a_recapture():
    step("S7a 重采：冷态 /dzlink 话题端点（完整输出，不用 grep）")
    for t in ("/dzlink/drone_to_car", "/dzlink/car_to_drone", "/nav/waypoint", "/dzlink/car_to_drone"):
        rc, out = ex("car", "bash /tmp/t4_rosenv.sh ros2 topic info -v %s 2>&1" % t)
        save("s7a_car_%s.txt" % t.replace("/", "_"), out)
        print("  [car] %s -> %s" % (t, out.strip().splitlines()[-1] if out.strip() else ""))
    for t in ("/dzlink/drone_to_car", "/dzlink/car_to_drone", "/nav/waypoint"):
        rc, out = ex("drone", "bash /tmp/t4_rosenv.sh ros2 topic info -v %s 2>&1" % t)
        save("s7a_drone_%s.txt" % t.replace("/", "_"), out)
        print("  [drone] %s -> %s" % (t, out.strip().splitlines()[-1] if out.strip() else ""))


def s8b_mcast_loss():
    step("S8b 组播丢包率复测（30 包 @2.5Hz，双向）+ WiFi 信号")
    for tag, sender, recv in (("d2c", "drone", "car"), ("c2d", "car", "drone")):
        bg(recv, "python3 /tmp/t4_mcast_count.py recv --duration 25 --out /tmp/t4_mc2_%s.json" % tag,
           "/tmp/t4_mc2_%s_outer.log" % tag)
        time.sleep(3)
        rc, out = ex(sender, "python3 /tmp/t4_mcast_count.py send --count 30 --interval 0.4 "
                             "--tag %s-%s" % (sender, tag))
        save("s8b_%s_send.txt" % tag, out)
        time.sleep(20)
        getf(recv, "/tmp/t4_mc2_%s.json" % tag, "s8b_%s_recv.json" % tag)
    for dev in ("drone", "car"):
        rc, out = ex(dev, "ip -4 addr show | awk '/wl|wlan/{i=$2} /inet /{print i, $2}'; "
                          "(command -v iw >/dev/null && iw dev 2>/dev/null | grep -E 'Interface|ssid' ; "
                          "iw dev 2>/dev/null | grep -A6 Interface | grep -E 'ssid|signal' ) ; "
                          "cat /proc/net/wireless 2>/dev/null")
        save("s8b_wifi_%s.txt" % dev, out)
        print("  [%s] %s" % (dev, out.strip()[:300]))


def s1_discovery():
    step("S1 互发现（两端交叉看到对端话题）")
    bg("drone", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub --topic /t4/disc/drone "
                "--qos reliable --rate 1 --duration 90 --tag DRONE > /dev/null",
       "/tmp/t4_disc_drone.log")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub --topic /t4/disc/car "
              "--qos best_effort --rate 1 --duration 90 --tag CAR > /dev/null",
       "/tmp/t4_disc_car.log")
    time.sleep(10)
    rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash /tmp/t4_topicinfo.sh /t4/disc/car")
    save("s1_drone_topicinfo.txt", out)
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh bash /tmp/t4_topicinfo.sh /t4/disc/drone "
                        "/dzlink/drone_to_car /dzlink/car_to_drone /nav/waypoint")
    save("s1_car_topicinfo.txt", out)
    print(out)


def s2_contract_qos():
    step("S2 契约 QoS 线上抓取（/dzlink/drone_to_car 的 Publisher 真实 QoSProfile）")
    bg("drone", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub "
                "--topic /dzlink/drone_to_car --qos best_effort --rate 1 --duration 60 "
                "--tag CONTRACT > /dev/null", "/tmp/t4_s2_pub.log")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py --qos best_effort "
              "--duration 55 --out /tmp/t4_car_recv_contract.jsonl > /tmp/t4_s2_consumer.log 2>&1",
       "/tmp/t4_s2_consumer_outer.log")
    time.sleep(6)
    rc, out = ros("car", "python3 /tmp/t4_qos_info.py --topic /dzlink/drone_to_car "
                         "--role sub --qos best_effort --duration 8")
    save("s2_car_sees_drone_publisher_qos.json", out)
    print(out)


def s3_payload_d2c():
    step("S3 载荷对账 机→车（dz_send_waypoint.py 真实报文 → 车侧只读消费者）")
    cmds = [
        "python3 ~/dzlink_tools/dz_send_waypoint.py heartbeat",
        "python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K9 upper=K4 --seq 101",
        "python3 ~/dzlink_tools/dz_send_waypoint.py target --blocks G10,H10",
        "python3 ~/dzlink_tools/dz_send_waypoint.py field --mines A3,B4 --obstacles K7",
    ]
    allc = ""
    for c in cmds:
        rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash -lc %s" % S.shell_quote(c))
        allc += "\n$ %s\n%s\n" % (c, out)
    save("s3_drone_sends.txt", allc)
    print(allc)
    time.sleep(3)
    getf("car", "/tmp/t4_car_recv_contract.jsonl", "s3_car_received.jsonl")
    rc, out = ex("car", "cat /tmp/t4_s2_consumer.log | tail -40")
    save("s3_car_consumer_summary.txt", out)
    print(out[-3000:])


def s4_negative_control():
    step("S4 负向对照：真契约话题上 BEST_EFFORT 发 → RELIABLE 收（应 0 条）")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py --qos reliable "
              "--duration 20 --out /tmp/t4_car_recv_neg.jsonl > /tmp/t4_s4_neg.log 2>&1",
       "/tmp/t4_s4_neg_outer.log")
    time.sleep(5)
    rc, out = ros("drone", "python3 /tmp/t4_qos_probe.py pub --topic /dzlink/drone_to_car "
                           "--qos best_effort --rate 5 --duration 8 --tag NEGCTRL")
    save("s4_drone_pub.txt", out)
    print(out[-1200:])
    time.sleep(12)
    rc, out = ex("car", "tail -30 /tmp/t4_s4_neg.log")
    save("s4_car_neg_consumer.txt", out)
    print(out[-2500:])


def s5_waypoint_reliable():
    step("S5 /nav/waypoint 正向（supervisor route，RELIABLE → 车侧 RELIABLE 消费者）")
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_consumer.py --topic /nav/waypoint "
              "--qos reliable --duration 25 --out /tmp/t4_car_recv_route.jsonl "
              "> /tmp/t4_s5_route.log 2>&1", "/tmp/t4_s5_outer.log")
    time.sleep(5)
    rc, out = ex("drone", "bash /tmp/t4_rosenv.sh bash -c %s"
                % S.shell_quote("cd /home/duzhong/dzlink_tools && python3 dzlink_supervisor.py "
                                "route 'N10 M10 L10* L9 K9' --domain 118"))
    save("s5_drone_route.txt", out)
    print(out[-2000:])
    time.sleep(8)
    rc, out = ex("car", "tail -30 /tmp/t4_s5_route.log")
    save("s5_car_route_consumer.txt", out)
    print(out[-2500:])


def s6_payload_c2d():
    step("S6 载荷对账 车→机（dzbrain 逐字复刻的 ack/car_state → 无人机生产监听器）")
    bg("drone", "bash /tmp/t4_rosenv.sh bash -c "
                "'cd /home/duzhong/dzlink_tools && python3 dzlink_supervisor.py listen "
                "--duration 34 --log-file /tmp/t4_sup_listen.jsonl > /tmp/t4_sup_listen.out 2>&1'",
       "/tmp/t4_s6_outer.log")
    time.sleep(5)
    bg("car", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_car_sender.py --ack-of 101 --car-state "
              "--block K9 --yaw-deg 180.0 --phase DRIVE --period 0.7 --duration 18 "
              "--out /tmp/t4_car_sent.jsonl > /tmp/t4_s6_sender.log 2>&1",
       "/tmp/t4_s6_sender_outer.log")
    time.sleep(5)
    rc, out = ros("drone", "python3 /tmp/t4_qos_info.py --topic /dzlink/car_to_drone "
                           "--role sub --qos best_effort --duration 6")
    save("s6_drone_sees_car_publisher_qos.json", out)
    print(out[-1500:])
    time.sleep(12)
    rc, out = ex("car", "tail -25 /tmp/t4_s6_sender.log")
    save("s6_car_sent.txt", out)
    print(out[-2500:])
    getf("car", "/tmp/t4_car_sent.jsonl.summary.json", "s6_car_sender_summary.json")
    getf("car", "/tmp/t4_car_sent.jsonl", "s6_car_sent_bodies.jsonl")
    rc, out = ex("drone", "cat /tmp/t4_sup_listen.out; echo '--- jsonl ---'; "
                          "cat /tmp/t4_sup_listen.jsonl 2>/dev/null | head -40")
    save("s6_drone_listen.txt", out)
    print(out[-4000:])


def s7_multi_publisher():
    step("S7 多发布者检查（/dzlink/drone_to_car 的 Publisher 数：0 → 1 → 2）")
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh bash -c "
                        "'ros2 topic info -v /dzlink/drone_to_car 2>&1 | grep -A3 -i publisher | head -20'")
    save("s7a_cold_publishers.txt", out)
    print("  [冷态] " + out.strip()[:600])

    bg("drone", "bash /tmp/t4_rosenv.sh bash -c "
                "'cd /home/duzhong/dzlink_tools && python3 dzlink_supervisor.py run "
                "--domain 118 --duration 40 --bridge lower=K9 --bridge upper=K4 "
                "--no-log --heartbeat-hz 1 > /tmp/t4_s7_sup.log 2>&1'",
       "/tmp/t4_s7_sup_outer.log")
    time.sleep(9)
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh bash -c "
                        "'ros2 topic info -v /dzlink/drone_to_car 2>&1 | grep -A3 -i publisher | head -20'")
    save("s7b_one_writer.txt", out)
    print("  [1 个写者] " + out.strip()[:600])

    bg("drone", "bash /tmp/t4_rosenv.sh python3 /tmp/t4_qos_probe.py pub "
                "--topic /dzlink/drone_to_car --qos best_effort --rate 1 --duration 20 "
                "--payload '{\"type\":\"heartbeat\",\"seq\":999}' > /dev/null",
       "/tmp/t4_s7_second.log")
    time.sleep(7)
    rc, out = ex("car", "bash /tmp/t4_rosenv.sh bash -c "
                        "'ros2 topic info -v /dzlink/drone_to_car 2>&1 | grep -A6 -i publisher | head -30'")
    save("s7c_two_writers.txt", out)
    print("  [2 个写者] " + out.strip()[:900])


def s8_mcast():
    step("S8 定量组播（双向各 20 包）")
    for tag, sender, recv in (("d2c", "drone", "car"), ("c2d", "car", "drone")):
        bg(recv, "python3 /tmp/t4_mcast_count.py recv --duration 22 --out /tmp/t4_mcast_%s.json" % tag,
           "/tmp/t4_mcast_%s_outer.log" % tag)
        time.sleep(3)
        rc, out = ex(sender, "python3 /tmp/t4_mcast_count.py send --count 20 --interval 0.4 "
                             "--tag %s-%s" % (sender, tag))
        save("s8_%s_send.txt" % tag, out)
        time.sleep(17)
        getf(recv, "/tmp/t4_mcast_%s.json" % tag, "s8_%s_recv.json" % tag)


def s9_freeze():
    step("S9 车侧零改动声明（freeze begin/end + 新建文件清单）")
    rc, out = ex("car", "bash /tmp/t4_car_freeze.sh end")
    save("s9_car_freeze_end.txt", out)
    rc, out = ex("car", "diff /tmp/t4_freeze_begin.txt /tmp/t4_freeze_end.txt; echo DIFF_EXIT=$?")
    save("s9_car_freeze_diff.txt", out)
    print(out)
    rc, out = ex("car", "find /home/duzhong -xdev -type f -newermt '2026-10-12 19:09:00' "
                        "-not -path '*/.cache/*' -not -path '*/.ros/log/*' "
                        "-not -path '*/robot_logs/*' 2>/dev/null | head -60")
    save("s9_car_new_files.txt", out)
    print("  HOME 下 19:09 之后被改/新建的文件：\n" + (out.strip() or "  (无)"))
    rc, out = ex("car", "ls -l --time-style=+%Y-%m-%dT%H:%M:%S /tmp/t4_* 2>/dev/null | wc -l; "
                        "du -sh /tmp 2>/dev/null")
    save("s9_car_tmp.txt", out)
    print(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", default="1,2,3,4,5,6,7,8,9")
    a = ap.parse_args()
    want = set(a.steps.split(","))
    fns = {"1": s1_discovery, "1b": s1b_drone_topicinfo, "2": s2_contract_qos,
           "3": s3_payload_d2c, "3b": s3b_payload_round2, "4": s4_negative_control,
           "5": s5_waypoint_reliable, "6": s6_payload_c2d, "6b": s6b_ack_loop,
           "7": s7_multi_publisher, "7a": s7a_recapture,
           "8": s8_mcast, "8b": s8b_mcast_loss, "9": s9_freeze}
    t0 = time.time()
    for k in sorted(fns):
        if k in want:
            fns[k]()
    print("\n### t4_contract_tests done in %.0fs; raw evidence in %s" % (time.time() - t0, RAW))
    return 0


if __name__ == "__main__":
    sys.exit(main())
