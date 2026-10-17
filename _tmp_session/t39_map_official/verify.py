# -*- coding: utf-8 -*-
r"""确定性校验：① 格网相位自证（满格色块是否落在单一格内）② 隔板是否存在
③ 与现役 map.yaml 的逐项差异 ④ 对队长 8 个疑点格逐格取证。"""

from __future__ import annotations

import json
import pathlib
import re
import sys

import numpy as np
from PIL import Image

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, r"D:\User\Desktop\比赛存档\v1\dzsmartscout\tools")
from perception.colors import as_rgb_array                      # noqa: E402
from perception.grid import GridSpec                            # noqa: E402

SPEC = GridSpec()
MAP_YAML = pathlib.Path(
    r"D:\User\Desktop\比赛存档\v1\dzrobot_v2\config\map.yaml")
res = json.loads((HERE / "read_map_result.json").read_text(encoding="utf-8"))
lat = res["lattice"]
cov = res["coverage"]

img = as_rgb_array(Image.open(HERE / "map_official.png"))
r, g, b = (img[..., i].astype(np.int16) for i in range(3))
mx, mn = np.max(img, axis=2).astype(np.int16), np.min(img, axis=2).astype(np.int16)
sat = mx - mn
red = (r > 200) & (g < 140) & (b < 140) & (sat > 80)
yellow = (r > 210) & (g > 170) & (b < 150) & (sat > 80)
mag = (r > 190) & (b > 150) & (g < 160) & (sat > 60)
grn = (g > 150) & (r < 130) & (b < 130) & (sat > 60)
luma = img.astype(np.float64) @ np.array([0.299, 0.587, 0.114])


def rect(ci, ri):
    x0 = int(round(lat["x0"] + ci * lat["sx"]))
    y0 = int(round(lat["y0"] + ri * lat["sy"]))
    x1 = int(round(lat["x0"] + (ci + 1) * lat["sx"]))
    y1 = int(round(lat["y0"] + (ri + 1) * lat["sy"]))
    return x0, y0, x1, y1


def cov_at(mask, ci, ri, inset=3):
    x0, y0, x1, y1 = rect(ci, ri)
    sub = mask[y0 + inset:y1 - inset, x0 + inset:x1 - inset]
    return float(sub.mean()) if sub.size else 0.0


MASKS = {"obstacle": red, "mine/box": yellow, "target": mag, "bridge": grn}

# ---------- ① 相位自证：满格色块 vs 四邻 ----------
print("=== ① 格网相位自证（色块在本格 vs 四邻的覆盖率） ===")
blocks = ([("obstacle", d) for d in res["obstacles"]] +
          [("target", d) for d in res["targets"]] +
          [("bridge", d) for d in res["bridges"]] +
          [("mine/box", d) for d in res["mines"] if d["block"] != "M2"] +
          [("mine/box", d) for d in res["label_boxes"]])
worst_self, worst_neigh = 1.0, 0.0
for cls, d in blocks:
    ci, ri = d["col_idx"], d["row_idx"]
    self_c = cov_at(MASKS[cls], ci, ri)
    neigh = []
    for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        c2, r2 = ci + dc, ri + dr
        if 0 <= c2 < SPEC.cols and 0 <= r2 < SPEC.rows:
            neigh.append(cov_at(MASKS[cls], c2, r2))
    worst_self = min(worst_self, self_c)
    worst_neigh = max(worst_neigh, max(neigh))
    print(f"  {cls:9s} {d['block']:4s} 本格 {self_c:.3f}  四邻 max {max(neigh):.3f}")
print(f"  ⇒ 所有满格块：本格最低 {worst_self:.3f}，四邻最高 {worst_neigh:.3f}")

# ---------- ② 隔板（白色单面胶长条）是否存在 ----------
print("\n=== ② 隔板检测：逐条行分界的灰线覆盖率（列 D~N = index 3~13） ===")
grayline = ((np.abs(r - 208) < 16) & (np.abs(g - 208) < 16) & (np.abs(b - 208) < 16))
white = (luma > 235) & (sat < 20)
for k in range(1, SPEC.rows):
    y = lat["y0"] + k * lat["sy"]
    band = slice(int(round(y)) - 1, int(round(y)) + 2)
    covs_g, covs_w = [], []
    for ci in range(3, 14):
        x0, _, x1, _ = rect(ci, 0)
        covs_g.append(float(grayline[band, x0:x1].mean()))
        covs_w.append(float(white[band, x0:x1].mean()))
    print(f"  行{k}/{k+1} 分界 y≈{y:6.1f}: 灰线均值 {np.mean(covs_g):.3f} "
          f"(最小 {min(covs_g):.3f})  近白均值 {np.mean(covs_w):.3f}")

# ---------- ③ 与 map.yaml 差异 ----------
print("\n=== ③ 与现役 config/map.yaml 的差异 ===")
text = MAP_YAML.read_text(encoding="utf-8")


def ylist(key):
    m = re.search(rf"^\s*{key}:\s*\[(.*?)\]", text, re.M)
    return sorted(s.strip().strip('"') for s in m.group(1).split(",") if s.strip())


def yscalar(key):
    m = re.search(rf"^\s*{key}:\s*([^#\n]+)", text, re.M)
    return m.group(1).strip().strip('"')


live = {"mines": ylist("mines"), "obstacles": ylist("obstacles"),
        "targets": ylist("targets"), "bridges": ylist("bridges"),
        "start": yscalar("start"), "end": yscalar("end")}
mine_read = sorted(d["block"] for d in res["mines"] if d["block"] != "M2")
read = {"mines": mine_read,
        "obstacles": sorted(d["block"] for d in res["obstacles"]),
        "targets": sorted(d["block"] for d in res["targets"]),
        "bridges": sorted(d["block"] for d in res["bridges"]),
        "boxes": sorted((d["block"], round(d["dark_frac"], 4)) for d in res["label_boxes"])}
for k in ("mines", "obstacles", "targets", "bridges"):
    same = set(live[k]) & set(read[k])
    print(f"  {k:9s} map.yaml {live[k]}")
    print(f"  {'':9s} 官方图   {read[k]}")
    print(f"  {'':9s} 交集 {sorted(same)}  仅map.yaml {sorted(set(live[k]) - set(read[k]))}"
          f"  仅官方图 {sorted(set(read[k]) - set(live[k]))}")
print(f"  起终点：map.yaml start={live['start']} end={live['end']}；"
      f"官方图标注框 {read['boxes']}")

# ---------- ④ 队长 8 个疑点格 + E/F 2/10 行 ----------
print("\n=== ④ 疑点格逐格取证（本格四类覆盖率 / 是否有深色文字） ===")
watch = ["A1", "K5", "K9", "B7", "D8", "H10", "B11", "C11", "E2", "F2", "E10", "F10",
         "F11", "E11", "I8", "G9", "D9", "G4", "I6", "E3", "A3", "C1", "B1"]
live_map = {"mine": set(live["mines"]), "obstacle": set(live["obstacles"]),
            "target": set(live["targets"]), "bridge": set(live["bridges"])}
rows = []
for blk in watch:
    ri, ci = SPEC.block_to_index(blk)
    vals = {k: round(cov_at(m, ci, ri), 3) for k, m in MASKS.items()}
    dark = float((luma[rect(ci, ri)[1]:rect(ci, ri)[3],
                       rect(ci, ri)[0]:rect(ci, ri)[2]] < 150).mean())
    live_cls = [k for k, s in live_map.items() if blk in s]
    top = max(vals, key=lambda k: vals[k])
    rows.append({"block": blk, "coverage": vals, "dark_frac": round(dark, 4),
                 "read_top": top if vals[top] > 0.3 else "空",
                 "map_yaml": live_cls or ["空"]})
    print(f"  {blk:4s} 覆盖率 {vals}  深色 {dark:.4f}  ⇒ 官方图={rows[-1]['read_top']:9s}"
          f" map.yaml={rows[-1]['map_yaml']}")

out = {"phase_self_check": {"worst_self_coverage": round(worst_self, 3),
                            "worst_neighbour_coverage": round(worst_neigh, 3),
                            "n_blocks": len(blocks)},
       "live_map_yaml": live, "read_official": read, "watch": rows}
(HERE / "verify_result.json").write_text(
    json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8", newline="\n")
print("\n-> verify_result.json")
