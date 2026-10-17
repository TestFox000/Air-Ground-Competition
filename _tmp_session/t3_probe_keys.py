# -*- coding: utf-8 -*-
"""t3 独立复核（只读）：验证 w1 的 B3「vehicle.*/field.* 键路径断裂」是否成立。

做法：把 config/robot.yaml 复制到 _tmp_session（不改真源），只把 /dzactuator 段下的
length_m 改成哨兵值 0.99，再用 core.config.Config 读规范名 vehicle.length_m，看是否变化。
"""
import io, os, sys, shutil, re

ROOT = r"D:\User\Desktop\比赛存档\v1\dzrobot_v2"
sys.path.insert(0, os.path.join(ROOT, "src", "dzbrain"))
TMP = r"D:\User\Desktop\比赛存档\v1\_tmp_session\t3_probe_robot.yaml"

src = os.path.join(ROOT, "config", "robot.yaml")
txt = io.open(src, encoding="utf-8").read()
# 只改 dzactuator 段（文件末尾段）里的 length_m/width_m
new = txt.replace("      length_m: 0.28", "      length_m: 0.99").replace(
    "      width_m: 0.135", "      width_m: 0.50")
assert new != txt, "哨兵替换失败"
io.open(TMP, "w", encoding="utf-8").write(new)

from dzbrain.core.config import Config

c = Config.load(TMP)
print("== 哨兵注入后（dzactuator.vehicle.length_m 应为 0.99 / width 0.50）==")
for k in ("vehicle.length_m", "vehicle.width_m", "vehicle.coverage_radius_m",
          "vehicle.ref_offset_m", "field.obstacle_radius_m", "field.mine_pad_m",
          "field.barrier_width_m", "field.barrier_cols",
          "dzactuator.vehicle.length_m", "dzactuator.vehicle.width_m",
          "dzactuator.vehicle.ref_offset_m", "dzactuator.field.obstacle_radius_m"):
    print("   %-34s = %r" % (k, c.get(k)))

print()
print("== 真源（未注入）==")
c0 = Config.load(src)
for k in ("vehicle.length_m", "vehicle.width_m", "vehicle.ref_offset_m",
          "dzactuator.vehicle.length_m", "dzactuator.vehicle.width_m",
          "dzactuator.vehicle.ref_offset_m", "dzactuator.field.obstacle_radius_m"):
    print("   %-34s = %r" % (k, c0.get(k)))

# route_check.py 的读法：从 /dzbrain 段找 vehicle —— 该段不存在
import yaml
d = yaml.safe_load(io.open(src, encoding="utf-8"))
dz = (d.get("/dzbrain") or {}).get("ros__parameters", {}) or {}
print()
print("== route_check.py:1260 的读法（/dzbrain 段里的 vehicle / field）==")
print("   dzbrain 段有 'vehicle' 键?", "vehicle" in dz, " 有 'field' 键?", "field" in dz)
print("   dzbrain.field 内容 =", dz.get("field"))
