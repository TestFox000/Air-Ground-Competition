import hashlib, io, os, subprocess

CAR = "duzhong@10.29.122.188"
KH = r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools\drone\known_hosts_dz"
LOCAL_ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"

FILES = [
    "src/dzbrain/dzbrain/main.py",
    "src/dzbrain/dzbrain/core/config.py",
    "src/dzbrain/dzbrain/core/geometry.py",
    "src/dzbrain/dzbrain/core/blackboard.py",
    "src/dzbrain/dzbrain/core/task.py",
    "src/dzbrain/dzbrain/io/ros_io.py",
    "src/dzbrain/dzbrain/io/contract.py",
    "src/dzbrain/dzbrain/pursuit/path.py",
    "src/dzbrain/dzbrain/pursuit/lookahead.py",
    "src/dzbrain/dzbrain/pursuit/controller.py",
    "src/dzbrain/dzbrain/route/parser.py",
    "src/dzbrain/dzbrain/route/planner.py",
    "src/dzbrain/dzbrain/route/physics.py",
    "src/dzbrain/dzbrain/tasks/localization.py",
    "src/dzbrain/dzbrain/tasks/safety.py",
    "src/dzbrain/dzbrain/tasks/tracker.py",
    "src/dzbrain/dzbrain/tasks/mission.py",
    "src/dzbrain/dzbrain/tasks/aim.py",
    "src/dzbrain/dzbrain/tasks/fix.py",
    "src/dzbrain/dzbrain/tasks/drone.py",
    "src/dzbrain/dzbrain/tasks/telemetry.py",
]

def sha_local(rel):
    p = os.path.join(LOCAL_ROOT, rel.replace("/", os.sep))
    if not os.path.exists(p):
        return None
    return hashlib.sha256(io.open(p, "rb").read()).hexdigest()[:16]

# one ssh call for all remote hashes
script = "; ".join("sha256sum /home/duzhong/dzrobot/%s 2>/dev/null" % f for f in FILES)
r = subprocess.run(["ssh", "-4", "-o", "BatchMode=yes", "-o", "UserKnownHostsFile=" + KH,
                    CAR, script], capture_output=True, text=True, encoding="utf-8",
                   errors="replace", timeout=300)
remote = {}
for line in r.stdout.splitlines():
    parts = line.split()
    if len(parts) == 2:
        remote[parts[1].replace("/home/duzhong/dzrobot/", "")] = parts[0][:16]

print("%-46s %-18s %-18s %s" % ("file", "local", "car", "verdict"))
print("-" * 104)
same = diff = missing = 0
for f in FILES:
    l = sha_local(f)
    c = remote.get(f)
    if l is None:
        v = "LOCAL-MISSING"; missing += 1
    elif c is None:
        v = "CAR-MISSING"; missing += 1
    elif l == c:
        v = "SAME"; same += 1
    else:
        v = "**DIFF**"; diff += 1
    print("%-46s %-18s %-18s %s" % (f.split("/")[-1], l or "-", c or "-", v))
print()
print("same=%d  diff=%d  missing=%d" % (same, diff, missing))
