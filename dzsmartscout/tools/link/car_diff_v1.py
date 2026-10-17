import hashlib, io, os, subprocess, sys

CAR = "duzhong@10.29.122.188"
KH = r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools\drone\known_hosts_dz"
LOCAL_ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
REMOTE_ROOT = "/home/duzhong/dzrobot"

def ssh(cmd):
    r = subprocess.run(["ssh", "-4", "-o", "BatchMode=yes",
                        "-o", "UserKnownHostsFile=" + KH,
                        CAR, cmd], capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=180)
    return r.stdout, r.stderr, r.returncode

def sha(b):
    return hashlib.sha256(b).hexdigest()

# 1) car map.yaml essentials
out, err, code = ssh("python3 -c \"import io,sys; s=io.open('/home/duzhong/dzrobot/config/map.yaml',encoding='utf-8').read(); print(s)\"")
print("===== CAR config/map.yaml =====")
print(out[:4000])

# 2) car robot.yaml key params (ASCII only)
out2, _, _ = ssh("grep -nE '^[[:space:]]*(auto_start|auto_start_delay_s|cruise_speed_mps|linear_speed|accel_up_mps2|accel_down_mps2|mission_reset|ticksPerMeter|ticksPer2PI|drift_stop)' /home/duzhong/dzrobot/config/robot.yaml | sed 's/#.*//'")
print("===== CAR config/robot.yaml (params) =====")
print(out2)

# 3) hashes on car
out3, _, _ = ssh("sha256sum /home/duzhong/dzrobot/config/robot.yaml /home/duzhong/dzrobot/config/map.yaml")
print("===== CAR hashes =====")
print(out3)

# 4) local hashes
print("===== LOCAL hashes =====")
for f in ["config/robot.yaml", "config/map.yaml"]:
    b = io.open(os.path.join(LOCAL_ROOT, f), "rb").read()
    print(sha(b)[:32] + "  " + f)
