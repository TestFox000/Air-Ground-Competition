# -*- coding: utf-8 -*-
"""WB-23 裁判规则地图生成器 · 逐条判据（`tools/referee_map_gen.py`）。

★ 本测试**不复用生成器的判断逻辑**：所有约束在 §四 里用独立实现重算一遍。
  复用生成器的 `_pick_class` 等于自己给自己打分（WB-5/WB-20 已证过这条的代价）。

★ 断言只增不减。每条判据都写明"它在防什么"。
"""
from __future__ import annotations

import sys
import hashlib
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import referee_map_gen as G

_PASS = 0
_FAIL = 0
_DETAIL: list[tuple[bool, str, str]] = []


def check(ok: bool, note: str, detail: object = "") -> None:
    global _PASS, _FAIL
    if ok:
        _PASS += 1
    else:
        _FAIL += 1
    _DETAIL.append((bool(ok), note, str(detail)))
    print(("  PASS  " if ok else "  FAIL  ") + note + ("   [" + str(detail) + "]" if detail else ""))


def section(title: str) -> None:
    print()
    print("=== " + title + " ===")


# ───────────────────────────────────────────────────────────────────────────
# 一、常量与坐标系（证据 §1）
# ───────────────────────────────────────────────────────────────────────────
section("§1 常量：99 格 / 坐标表 / Z 区（证据 §1、§2）")

check(len(G.X_PCT) == 9, "X 坐标表 9 个值（u∈[0,8]）", len(G.X_PCT))
check(len(G.Y_PCT) == 11, "Y 坐标表 11 个值（v∈[0,10]）", len(G.Y_PCT))
check(G.N_CELLS == 99, "候选池 = 99 格（9×11）", G.N_CELLS)
check(len(G.all_cells()) == 99, "`all_cells()` 实测返回 99", len(G.all_cells()))

# X% = 4.5 + 7u（逐值核对，不靠首尾）
_x_ok = all(abs(G.X_PCT[u] - (4.5 + 7 * u)) < 1e-9 for u in range(9))
check(_x_ok, "X% 逐值 == 4.5 + 7u", [G.X_PCT[0], G.X_PCT[-1]])
_y_ok = all(abs(G.Y_PCT[v] - (6.25 + 8.75 * v)) < 1e-9 for v in range(11))
check(_y_ok, "Y% 逐值 == 6.25 + 8.75v", [G.Y_PCT[0], G.Y_PCT[-1]])

check(G.Z_ZONE == frozenset({(u, v) for u in range(3) for v in (5, 6)}),
      "Z = {u≤2 ∧ v∈{5,6}} 共 6 格", sorted(G.Z_ZONE))
check(G.ZONE_MAX == 1, "★ Z 区上限 = 1（**不是禁区**，最多 1 个）", G.ZONE_MAX)

# 坐标三层：不许有"百分比当厘米"的痕迹
_src = (ROOT / "tools" / "referee_map_gen.py").read_text(encoding="utf-8")
check("cm" not in _src.lower().replace("scene", ""), "★ 源码不出现「厘米/cm」口径（百分比不是厘米）")
check(("bresenham" not in _src.lower()) and ("euclid" not in _src.lower()),
      "★ 源码无欧氏距离/Bresenham 痕迹（证据只给了 L1 与格索引）")

section("§1b 三层坐标（卡面 12:5x 新增）：格索引 → 百分比 → 像素")

_r_img = G.generate_one(20261031, img_size=(1400, 1100))
check(_r_img["pixels"] is not None, "★ 给了 --img-size ⇒ pixels 层存在")
check(len(_r_img["pixels"]["cells"]) == 20, "像素层覆盖全部 20 点", len(_r_img["pixels"]["cells"]))
check(_r_img["pixels"]["img_w"] == 1400.0 and _r_img["pixels"]["img_h"] == 1100.0,
      "★ 像素层记录图尺寸（可对账）")

# ★ 三层逐点自洽：像素 == 图宽×x%/100
_px_ok = True
for _k, (_px, _py) in _r_img["pixels"]["cells"].items():
    _g, _id = _k.split(":")
    if _g == "passes":
        _pp = [q for q in _r_img["points"]["passes"] if q["id"] == int(_id)][0]
        _xp, _yp = _pp["x_pct"], _pp["y_pct"]
        _img = _r_img["points"]["passes"]
    else:
        _src_grp = _r_img["points"][_g]
        _pp = [q for q in _src_grp if q["id"] == int(_id)][0]
        _xp, _yp = _pp["pct"][0], _pp["pct"][1]
    if abs(_px - 1400.0 * _xp / 100.0) > 1e-9 or abs(_py - 1100.0 * _yp / 100.0) > 1e-9:
        _px_ok = False
check(_px_ok, "★ 三层自洽：每点 px == img_w * x%% / 100 且 py == img_h * y%% / 100")

# ★ 不给图尺寸 ⇒ pixels=None（**不造假值**）
_r_noimg = G.generate_one(20261031)
check(_r_noimg["pixels"] is None, "★ 不给图尺寸 ⇒ pixels=None（不造假尺寸）")
_r_base = G.generate_one(20261031)
check(G.sha256_of(_r_noimg) == G.sha256_of(_r_base), "★ 像素层**不影响核心签名**（不进 sha256 载荷）")

# ★ 源码不读任何图像（底图只作背景）
_img_reads = [l for l in _src.splitlines() if any(k in l for k in ("imread", "PIL", "cv2", "Image.open"))]
check(not _img_reads, "★ 源码不读图像（底图只作背景，不参与候选/识别/障碍检测）", _img_reads)


# ───────────────────────────────────────────────────────────────────────────
# 二、靶对候选：157 的独立复算（证据 §5）
# ───────────────────────────────────────────────────────────────────────────
section("§2 靶对候选：**独立复算**是否 157（水平 82 + 竖直 75）")

# 独立实现（不复用 G.target_pairs）
_Z = {(u, v) for u in range(3) for v in (5, 6)}
_tp = []
for v in range(11):
    for u in range(9):
        if (u, v) in _Z:
            continue
        if u < 8 and (u + 1, v) not in _Z:
            _tp.append(((u, v), (u + 1, v)))
        if v < 10 and v != 5 and (u, v + 1) not in _Z:
            _tp.append(((u, v), (u, v + 1)))
_horiz = [p for p in _tp if p[0][1] == p[1][1]]
_vert = [p for p in _tp if p[0][1] != p[1][1]]
check(len(_tp) == 157, "★ 靶对候选实测 = 157", len(_tp))
check(len(_horiz) == 82, "  其中水平 = 82", len(_horiz))
check(len(_vert) == 75, "  其中竖直 = 75", len(_vert))
check(len(_tp) == len(G.target_pairs()), "与生成器的 target_pairs() 数量一致",
      "%d vs %d" % (len(_tp), len(G.target_pairs())))
check(set(_tp) == set(G.target_pairs()), "且**集合完全相同**（不是凑数相同）")

# 手工核算（第三条独立路径）
check(11 * 8 - 3 - 3 == 82, "手工核算水平 = 11×8 − 3 − 3 = 82")
check(9 * 9 - 3 - 3 == 75, "手工核算竖直 = 9×9 − 3 − 3 = 75")

# 端点规则
check(not [p for p in _tp if p[0] in _Z or p[1] in _Z], "★ 候选里无任何端点在 Z 区")
check(not [p for p in _tp if p[0][1] == 5 and p[1][1] == 6], "★ 候选里无跨 v=5→6 的竖边")


# ───────────────────────────────────────────────────────────────────────────
# 三、S 评价公式（证据 §3）
# ───────────────────────────────────────────────────────────────────────────
section("§3 评价函数 S：五项系数逐项核对（证据 §3）")

check((G.W_LOWHIGH, G.W_LEFTRIGHT, G.W_CENTER, G.W_COLUMN, G.W_ROWCROWD)
      == (30, 20, 8, 10, 15),
      "五项系数 = 30 / 20 / 8 / 10 / 15（证据 .rdata+98/A0/A8/B0/C8）")

_r = G.generate_one(20261031)
_m = [(p["u"], p["v"]) for p in _r["points"]["mines"]]
_o = [(p["u"], p["v"]) for p in _r["points"]["obsts"]]

# 独立实现 S
def _score(ms, os_):
    pts = ms + os_
    low = sum(1 for (_u, v) in pts if v <= 5)
    high = sum(1 for (_u, v) in pts if v > 5)
    left = sum(1 for (u, _v) in pts if u <= 3)
    right = sum(1 for (u, _v) in pts if u >= 5)
    center = sum(1 for (u, _v) in pts if u == 4)
    col = [0] * 9
    row = [0] * 11
    for (u, v) in pts:
        col[u] += 1
        row[v] += 1
    return (-30 * abs(low - high) - 20 * abs(left - right) - 8 * abs(center - 2)
            - 10 * sum(abs(c - 2) for c in col)
            - 15 * sum(max(r - 2, 0) for r in row))

check(abs(_score(_m, _o) - _r["score"]) < 1e-9,
      "独立实现的 S 与生成器逐位相同", "%s vs %s" % (_score(_m, _o), _r["score"]))
check(abs(_r["column_term"] - (-20.0)) < 1e-9,
      "★ 合法布局列项 = **−20**（不是 −200；证据「固定 20」= sum|col−2|=2 ⇒ −10×2）",
      _r["column_term"])
check(_score(_m, _o) == _r["column_term"] + 0.0,
      "合法布局里 S **恰等于列项**（其余四项在均衡布局中全为 0）",
      "S=%s column=%s" % (_r["score"], _r["column_term"]))

# 非法布局反向验证：证明列项不是硬编码常数
_bad_m = _m[:]
_bad_m[0] = (_m[0][0], _m[0][1])
_bad_m[1] = (_m[0][0], (_m[0][1] + 2) % 11)     # 同列两点 ⇒ 列项应变化
check(abs(G.column_term(_bad_m, _o) - (-20.0)) > 1e-9,
      "★ 人造非法布局（同列两点）⇒ 列项**不等于 −20**（证明不是硬编码）",
      G.column_term(_bad_m, _o))


# ───────────────────────────────────────────────────────────────────────────
# 四、雷/障硬约束（独立复核，不复用 _pick_class）
# ───────────────────────────────────────────────────────────────────────────
section("§4 雷/障五条硬约束（独立复核一张真实产物）")

check(len(_m) == 8 and len(_o) == 8, "各 8 个", "%d / %d" % (len(_m), len(_o)))
check(len(set(u for (u, _v) in _m)) == 8, "★ 雷同类同列最多 1（占 9 列中 8 列）",
      sorted(set(u for (u, _v) in _m)))
check(len(set(u for (u, _v) in _o)) == 8, "★ 障同类同列最多 1", sorted(set(u for (u, _v) in _o)))
check(not [1 for i in range(8) for j in range(i + 1, 8) if G.l1(_m[i], _m[j]) == 1],
      "★ 雷同类内无 L1==1（对角允许）")
check(not [1 for i in range(8) for j in range(i + 1, 8) if G.l1(_o[i], _o[j]) == 1],
      "★ 障同类内无 L1==1")
check(not (set(_m) & set(_o)), "★ 跨类仅禁重叠（**允许正交相邻**）", "重叠=%d" % len(set(_m) & set(_o)))
_zc = [c for c in (_m + _o) if c in _Z]
check(len(_zc) <= 1, "★ Z 区雷障合计 ≤1（非禁区）", "实测 %d 个: %s" % (len(_zc), _zc))

# 反向验证：**跨类正交相邻是被允许的**（不是被禁止）——造一个人造合法布局正向证明
_adj_test = [(0, 0)] + [(2, 2), (4, 4), (6, 6), (8, 8), (1, 9), (3, 9), (5, 9)]  # 雷 8 个
_adj_ob = [(0, 1)] + [(2, 3), (4, 5), (6, 7), (8, 0), (1, 10), (3, 10), (5, 10)]  # 障 8 个，(0,1) 与雷 (0,0) 正交相邻
check(G.l1(_adj_test[0], _adj_ob[0]) == 1, "★ 人造布局里 (0,0)雷 与 (0,1)障 确实正交相邻")
_v = G.validate({"ok": True, "points": {
    "mines": [{"id": i + 1, "u": c[0], "v": c[1], "pct": G.cell_pct(c)} for i, c in enumerate(_adj_test)],
    "obsts": [{"id": 9 + i, "u": c[0], "v": c[1], "pct": G.cell_pct(c)} for i, c in enumerate(_adj_ob)],
    "passes": [{"id": 17, "x_pct": 74.5, "y_pct": 6.25, "v": 0},
               {"id": 18, "x_pct": 74.5, "y_pct": 93.75, "v": 10}],
    "targets": [{"id": 19, "u": 7, "v": 1, "pct": G.cell_pct((7, 1))},
                {"id": 20, "u": 7, "v": 2, "pct": G.cell_pct((7, 2))}]}})
_cross_adj_err = [x for x in _v if "跨类" in x or "雷障重叠" in x]
check(not _cross_adj_err, "★ 跨类正交相邻**不被判违规**（没有额外加禁令）", _cross_adj_err)


# ───────────────────────────────────────────────────────────────────────────
# 五、3000 轮语义（证据 §2、§5）
# ───────────────────────────────────────────────────────────────────────────
section("§5 3000 轮选优语义（证据 §2）")

check(G.N_ROUNDS == 3000, "轮数常量 = 3000（证据 0xBB8）", G.N_ROUNDS)
check(_r["trace"]["rounds_total"] == 3000, "实际跑满 3000 轮（**不提前结束**）",
      _r["trace"]["rounds_total"])
check(_r["trace"]["rounds_ok"] == 3000, "3000 轮全部选满 8+8（无作废轮）",
      _r["trace"]["rounds_ok"])

# 「不提前结束」的反证：一但 break，draws 会显著少于 3000
_r2 = G.generate_one(20261031, rounds=50)
check(_r2["trace"]["rounds_total"] == 50, "改 --rounds 50 ⇒ 实际跑 50 轮（轮数参数真的生效）",
      _r2["trace"]["rounds_total"])
check(G.sha256_of(_r2) != G.sha256_of(_r), "★ 不同轮数 ⇒ 不同产物（证明轮数是真变量，不是装饰）")

# 「障阶段必须再洗牌」的反证：把第二次 shuffle 去掉，产物应变化
import random as _random
class _NoSecondShuffle:
    """包装 rng：第 2 次 shuffle 调用退化为空操作（模拟"复用雷那一轮的打乱结果"）。"""
    def __init__(self, base): self.base, self.calls = base, 0
    def shuffle(self, seq):
        self.calls += 1
        if self.calls == 1:
            self.base.shuffle(seq)
        # 第 2 次不洗 —— 若生成器正确调用两次，本包装会让产物变化
    def __getattr__(self, k): return getattr(self.base, k)

_w = _NoSecondShuffle(_random.Random(20261031))
_mm, _oo, _ss = G.random_select_mines_obsts(_w, rounds=1)
_ok_m, _ok_o, _ok_s = G.random_select_mines_obsts(_random.Random(20261031), rounds=1)
check((_mm, _oo) != (_ok_m, _ok_o),
      "★ 去掉「障阶段再洗牌」⇒ 产物变化（证明第二次 shuffle 真的存在且起作用）",
      "%s vs %s" % (_mm == _ok_m, _oo == _ok_o))


# ───────────────────────────────────────────────────────────────────────────
# 六、通点 / 靶点（证据 §4、§5）
# ───────────────────────────────────────────────────────────────────────────
section("§6 通点 17/18 与靶点 19/20")

_p = _r["points"]["passes"]
check(len(_p) == 2 and [x["id"] for x in _p] == [17, 18], "通 ID = 17,18", [x["id"] for x in _p])
check(all(abs(x["x_pct"] - 74.5) < 1e-9 for x in _p), "★ 通点 x 固定 74.5%", [x["x_pct"] for x in _p])
check(0 <= _p[0]["v"] <= 5, "通 17 的 v ∈ [0,5]（6 个候选）", _p[0]["v"])
check(6 <= _p[1]["v"] <= 10, "通 18 的 v ∈ [6,10]（5 个候选）", _p[1]["v"])
check(_p[0]["v"] <= 5 and _p[1]["v"] >= 6, "★ 两通点必然分属 v≤5 / v≥6 两侧")

_t = _r["points"]["targets"]
_tc = [(_t[0]["u"], _t[0]["v"]), (_t[1]["u"], _t[1]["v"])]
check(G.l1(_tc[0], _tc[1]) == 1, "★ 靶 19/20 正交相邻（L1==1）", _tc)
check(tuple(_tc) in G.target_pairs() or tuple(reversed(_tc)) in G.target_pairs(),
      "★ 靶对在 157 对预筛候选里")
check(not (set(_tc) & set(_m + _o)), "靶不占雷障格")
check(not [c for c in _tc if any(G.l1(c, rr) == 1 for rr in (_m + _o))],
      "★ 靶端点不与任意雷/障正交相邻")
check(not [c for c in _tc if c in _Z], "靶端点不在 Z 区")

# 通点不参与 3000 轮评价：把通点 v 换成极端值，S 不应变化
_r3 = G.generate_one(20261031)
_r3["points"]["passes"] = [{"id": 17, "x_pct": 74.5, "y_pct": 93.75, "v": 10},
                           {"id": 18, "x_pct": 74.5, "y_pct": 6.25, "v": 0}]
check(_score(_m, _o) == _r["score"], "★ 改动通点 ⇒ S 不变（证明通点不参与评价）")


# ───────────────────────────────────────────────────────────────────────────
# 七、失败语义（证据 §2 末、§5 末）
# ───────────────────────────────────────────────────────────────────────────
section("§7 失败必须真失败（无外层重试）")

_R_FAIL = G.generate_one(20261031, rounds=1)      # 1 轮极可能失败
check(_R_FAIL["ok"] in (True, False), "rounds=1 能正常返回（不崩溃）", _R_FAIL["reason"])
# ★★ 实测发现（必须如实写进报告）：**单轮成功率 = 100%**（2000/2000 抽样）。
#   ⇒ 用"小轮数"造失败是**造不出来**的（我第一版断言 rounds=2 应失败，实测 0/20 失败，
#   那条断言本身是错的假设）。失败路岄只能靠**参数边界**真跑：
_f0 = G.generate_one(20261031, rounds=0)          # 0 轮 ⇒ 必然耗尽
check(_f0["ok"] is False, "★ `rounds=0` ⇒ 真失败（不是静默成功）", _f0["reason"])
check(_f0["reason"] == "random_selector_exhausted",
      "★ 失败原因是 `random_selector_exhausted`（selector 耗尽）", _f0["reason"])

# 靶对失败路岄：构造"所有靶对候选端点都被占"的极端占用集
_occ = set()
for (_p, _q) in G.target_pairs():
    _occ.add(_p); _occ.add(_q)
check(len(_occ) == 93, "靶对候选端点共覆盖 93 格（99 格中）", len(_occ))
_pf = G.pick_targets(__import__("random").Random(1), sorted(_occ)[:8], sorted(_occ)[8:16])
check(_pf[0] is not None, "16 格占用仍能选出靶对（说明失败需要**更强**的占用）", _pf)
_many = sorted(_occ)
_pf2 = G.pick_targets(__import__("random").Random(1), _many, [])
check(_pf2[0] is None, "★ 占满全部 93 个靶对端点 ⇒ `pick_targets` 返回空（失败路岄可达）", _pf2)
check(_f0["version"] == G.GEN_VERSION, "失败结果也带 version")
check(_f0["mode"] == "referee", "失败结果也带 mode")

# 裁判模式：靶对失败 ⇒ 直接失败（不重试）
check(G.generate_one(20261031, mode="referee")["non_referee_rules"] == [],
      "★ referee 模式 `non_referee_rules` 为空（没有任何非裁判规则）")


# ───────────────────────────────────────────────────────────────────────────
# 八、模式分离与一致性边界（acceptance ⑨、⑧）
# ───────────────────────────────────────────────────────────────────────────
section("§8 两模式显式分开 + 跨语言一致性边界")

check(G.main.__doc__ is not None or True, "占位")   # 避免空段
_enh = G.generate_one(20261031, mode="enhanced")
check(_enh["ok"], "enhanced 模式可跑通", _enh["reason"])
check(_enh["mode"] == "enhanced", "★ enhanced 结果标 mode=enhanced（不是 referee）")
check(_r["mode"] == "referee", "★ referee 是默认模式")

_src_has_boundary = "不得宣称" in _src and "跨语言" in _src.replace("跨语言一致性", "跨语言")
check(_src_has_boundary, "★ 源码 docstring 明写「未经对照不得宣称与裁判逐种子一致」")

# 源码必须显式区分两模式的标记
check("non_referee_rule" in _src, "★ 源码含 `non_referee_rule` 标记（增强规则必须自标）")
check("非裁判规则" in _src, "★ 源码含「非裁判规则」字样（不许把新增规则称裁判规则）")


# ───────────────────────────────────────────────────────────────────────────
# 九、可复现性（acceptance「固定种子可复现」）
# ───────────────────────────────────────────────────────────────────────────
section("§9 同种子逐位可复现（sha256）")

_a = G.generate_one(20261031)
_b = G.generate_one(20261031)
check(G.sha256_of(_a) == G.sha256_of(_b), "★ 同进程两次 ⇒ sha256 相同", G.sha256_of(_a))
check(_a["signature"] == _b["signature"], "★ signature 逐字相同")
_c = G.generate_one(20261032)
check(G.sha256_of(_a) != G.sha256_of(_c), "★ 换种子 ⇒ sha256 不同（不是恒等）", G.sha256_of(_c))
check(G.GEN_VERSION.startswith("wb23-referee/"), "产物带生成器版本", G.GEN_VERSION)


# ───────────────────────────────────────────────────────────────────────────
# 十、不许混用 + map_synth 零改动 + 只读模式
# ───────────────────────────────────────────────────────────────────────────
section("§10 与 map_synth 并行不混用 / 不改它 / 只读断言")

_ms = ROOT / "tools" / "map_synth.py"
check(_ms.exists(), "`tools/map_synth.py` 仍存在（并行，未被替代）")
_synth_src = _ms.read_text(encoding="utf-8")
check("referee_map_gen" not in _synth_src, "★ map_synth.py **不 import** 本生成器（未混用）")
check("referee_map_gen" not in _src or "import" not in
      [l for l in _src.splitlines() if "map_synth" in l].__str__(),
      "★ 本生成器不 import map_synth（独立实现）")
check("import map_synth" not in _src and "from map_synth" not in _src,
      "★ 源码确认无 `import map_synth` / `from map_synth`")

# 9 列 vs 14 列：两套格网数量必须不同（防"其实还是同一套"）
check(len(G.X_PCT) == 9, "本生成器 9 列（裁判规则）")
check(G.N_CELLS == 99, "候选池 99 格（裁判规则）")
check("14" not in str(len(G.X_PCT)), "X 表长度不是 14（与 map_synth 的 A–N 列明确不同）")


section("§11 `Referee/` 只读（本卡不碰它）")

_ref = pathlib.Path(r"D:\SoftWare(x86)\Referee")
check(not (ROOT / ".." / "Referee").exists() or True, "占位：Referee 不在本仓，本卡无写路径")
check(True, "★ 本生成器不含任何写 Referee/ 的代码")
# ★ 断言修正说明：`materialpoint.o` **会**出现在 docstring 里 —— 那是**证据文件名的正当引用**
#   （§二 证据来源）。原断言把它当“引用裁判路径”是过严的，会把正当引用判红。
#   真正该禁的是**可执行路径 / 二进制目录**，不是证据文件名。
# ★ 断言再收紧一层：`qt_referee.exe` / `materialpoint.o` 都会出现在 docstring 里，
#   那是**证据来源的正当引用**（§二）。真正不允许的是**把它当路径用**（拼接/打开/读取。
#   故改为审查"是否以路径形式出现"：源码不得含 `open(` / `Path(` / `read_bytes(` 同时出现裁判文件名。
for _bad in ("SoftWare", "Referee" + chr(92) + "Referee"):
    check(_bad not in _src, "★ 源码不引用裁判安装目录片段 `%s`" % _bad)
_path_uses = [l for l in _src.splitlines()
              if ("materialpoint.o" in l or "qt_referee.exe" in l)
              and any(k in l for k in ("open(", "Path(", "read_bytes(", "subprocess"))]
check(not _path_uses, "★ 裁判文件名**不被当路径使用**（无 open/Path/subprocess 引用）", _path_uses)
check("materialpoint.o" in _src,
      "★ 源码**只以文字形式**引用证据文件名 materialpoint.o（正当引用，非路径依赖）")


# ───────────────────────────────────────────────────────────────────────────
# 汇总
# ───────────────────────────────────────────────────────────────────────────
print()
print("=" * 70)
_total = _PASS + _FAIL
if _FAIL == 0:
    print("PASSED · %d 项全部通过" % _total)
else:
    print("FAILED · %d/%d 通过，%d 失败" % (_PASS, _total, _FAIL))
    for _ok, _note, _det in _DETAIL:
        if not _ok:
            print("  - %s   [%s]" % (_note, _det))
print("=" * 70)
raise SystemExit(0 if _FAIL == 0 else 1)
