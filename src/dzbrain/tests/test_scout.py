# -*- coding: utf-8 -*-
r"""机端侦察仿真（T28 / 协议 v2）测试 —— 纯脚本，无 pytest。

运行：python src\dzbrain\tests\test_scout.py

覆盖（逐条对应任务卡）：
  1. 视场模型：格点姿态 2x4（航向 0°）<-> 4x2（航向 90°）= 8 格；45° 旋转取覆盖格；
     高 1.5 m 假设；足迹面积/重叠面积（同一位置旋转 90° = 50%）。
  2. 帧位生成：全图蛇形条带（先下半场再上半场）、相邻帧面积重叠 >= 30%、
     覆盖率 154/154、帧数/时长、确定性。
  3. 布局 v2：known 单调增长、特征 ⊆ known、version 严格递增、target_cell 合法；
     终点已知可达 -> 终点；前沿决策可达性（隔板 A/B/C、K 列桥洞口径）。
  4. 心跳：变化即发 + 每秒重发（同 version）；纯函数 schedule 与 LayoutFeed 状态机。
  5. 对照：用户手绘路线（M8 -> ... -> M2）覆盖率/帧数/时长 vs 推荐路线。
  6. 闭环：每份布局都能被车端 normalize_field_layout 接受（v2 校验）；v1 无 known 仍兼容。
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from dzbrain.core import geometry as geo                                      # noqa: E402
from dzbrain.scout import (FOV_LENGTH_CELLS, FOV_LENGTH_M, FOV_WIDTH_CELLS,   # noqa: E402
                           FOV_WIDTH_M, SCOUT_ALTITUDE_M, LayoutFeed,
                           LayoutStream, Pose, TruthMap, all_blocks,
                           build_layout, cells_in_view, choose_target_cell,
                           compare_routes, coverage, footprint_corners,
                           frames_from_cell_route, generate_snake_timeline,
                           heartbeat_messages, heartbeat_schedule,
                           overlap_fraction, reachable_cells, step_ok,
                           user_route_timeline,
                           USER_ROUTE_BLOCKS, USER_ROUTE_SOURCE, simulate)
from dzbrain.scout.frames import (SCAN_LINE_YS, adjacent_overlaps,          # noqa: E402
                                  cell_overlaps, timeline_duration_s)
from dzbrain.tasks.drone import (FIELD_LAYOUT_VERSION, normalize_field_layout)  # noqa: E402

from harness import check, finish, section                                     # noqa: E402

WS = pathlib.Path(__file__).resolve().parents[3]          # <repo> = dzrobot_v2/
MAP = WS / "config" / "map.yaml"
ALL = set(all_blocks())
TOTAL = geo.MAP_COLS * geo.MAP_ROWS


def row_of(block: str) -> int:
    return geo.block_to_index(block)[0] + 1


def col_of(block: str) -> int:
    return geo.block_to_index(block)[1]


# --------------------------------------------------------------------------- #
section("① 视场模型：2x4（0°）/ 4x2（90°）/ 任意 yaw 旋转取覆盖格")
check("视场尺寸 = 4x2 格 = 1.4x0.7 m", FOV_LENGTH_CELLS == 4 and FOV_WIDTH_CELLS == 2
      and abs(FOV_LENGTH_M - 1.4) < 1e-12 and abs(FOV_WIDTH_M - 0.7) < 1e-12,
      f"{FOV_LENGTH_M}x{FOV_WIDTH_M}")
check("高度假设 = 1.5 m", abs(SCOUT_ALTITUDE_M - 1.5) < 1e-12)

NODE = Pose(1.05, 1.05, 0.0)
c0 = cells_in_view(NODE)
rows0 = {row_of(b) for b in c0}
cols0 = {col_of(b) for b in c0}
check("航向 0°（+x）在格点上恰好 8 格、行跨 4、列跨 2",
      len(c0) == 8 and max(rows0) - min(rows0) + 1 == 4
      and max(cols0) - min(cols0) + 1 == 2, f"{sorted(c0)}")
NODE90 = Pose(1.05, 1.05, 3.141592653589793 / 2.0)
c90 = cells_in_view(NODE90)
rows90 = {row_of(b) for b in c90}
cols90 = {col_of(b) for b in c90}
check("★ 航向 90°（+y）旋转成宽条：恰好 8 格、行跨 2、列跨 4",
      len(c90) == 8 and max(rows90) - min(rows90) + 1 == 2
      and max(cols90) - min(cols90) + 1 == 4, f"{sorted(c90)}")
check("0° 与 90° 的格集不同（确实旋转，不是换名）", set(c0) != set(c90))
c180 = cells_in_view(Pose(NODE.x, NODE.y, 3.141592653589793))
check("180° 与 0° 覆盖同一组格（矩形中心对称）", set(c180) == set(c0))
c45 = cells_in_view(Pose(NODE.x, NODE.y, 3.141592653589793 / 4.0))
check("45° 斜视场：非空、格名全合法、与 0° 不同",
      4 <= len(c45) <= 9 and set(c45) <= ALL and set(c45) != set(c0),
      f"{len(c45)} 格 {sorted(c45)}")
check("视场足迹面积 = 1.4*0.7 = 0.98 m^2",
      abs(__import__("dzbrain.scout.fov", fromlist=["polygon_area"]).polygon_area(
          footprint_corners(NODE)) - 0.98) < 1e-9)
check("同一位置旋转 90°：物理面积重叠 = 50%（>=30% 判据）",
      abs(overlap_fraction(NODE, NODE90) - 0.5) < 1e-9,
      f"{overlap_fraction(NODE, NODE90):.4f}")
check("沿航向平移 0.70 m（4 格长的半程）：面积重叠 = 50%",
      abs(overlap_fraction(NODE, Pose(NODE.x + 0.7, NODE.y, 0.0)) - 0.5) < 1e-9)
check("沿航向平移 0.35 m：面积重叠 = 75%",
      abs(overlap_fraction(NODE, Pose(NODE.x + 0.35, NODE.y, 0.0)) - 0.75) < 1e-9)
check("边界姿态会按场地裁剪（<8 格，不补虚拟格）",
      len(cells_in_view(Pose(0.0, 0.0, 0.0))) < 8,
      f"{len(cells_in_view(Pose(0.0, 0.0, 0.0)))}")

# --------------------------------------------------------------------------- #
section("② 帧位生成：先下半场->上半场 / 覆盖 154 / 相邻重叠 >=30% / 帧数时长")
frames = generate_snake_timeline(start_t=-30.0, frame_hz=2.0)
total = coverage(frames)
area = adjacent_overlaps(frames)
cell_ov = cell_overlaps(frames)
check("帧数 = 68（2 Hz，含转向补帧）", len(frames) == 68, f"{len(frames)}")
check("时长 = 33.5 s（首末帧 -30.0 .. 3.5）",
      abs(timeline_duration_s(frames) - 33.5) < 1e-9
      and abs(frames[0].t - (-30.0)) < 1e-9 and abs(frames[-1].t - 3.5) < 1e-9,
      f"{frames[0].t}..{frames[-1].t}")
check("★ 覆盖率 = 154/154（全图无缝）", total == ALL, f"{len(total)}/{TOTAL}")
check("★ 相邻帧物理足迹重叠 min >= 30%（实测 >=50%，留 1e-6 浮点余量）",
      min(area) >= 0.3 - 1e-9 and min(area) >= 0.5 - 1e-6,
      f"min={min(area):.4f} mean={sum(area)/len(area):.4f}")
check("相邻帧格集合 Jaccard min >= 30%（诊断口径同样达标）",
      min(cell_ov) >= 0.3 - 1e-9, f"min={min(cell_ov):.4f}")
check("每帧格数 6~8（边界裁剪最多 8）",
      min(len(f.cells) for f in frames) >= 6
      and max(len(f.cells) for f in frames) <= 8)
check("帧间隔恒为 0.5 s（2 Hz）",
      all(abs((frames[i + 1].t - frames[i].t) - 0.5) < 1e-9
          for i in range(len(frames) - 1)))
first_rows = sorted({row_of(b) for f in frames[:10] for b in f.cells})
last_rows = sorted({row_of(b) for f in frames[-10:] for b in f.cells})
check("★ 先下半场：前 10 帧全部在行 7~11", all(r >= 7 for r in first_rows),
      f"{first_rows}")
check("★ 后上半场：最后 10 帧已到上半场（含行 1、且不再含行 10~11）",
      1 in last_rows and max(last_rows) <= 9,
      f"{last_rows}")
check("帧位全部落在格点（0.35 m 整数倍；2x4=8 格的前提）",
      all(abs(f.pose.x / geo.BLOCK_SIZE - round(f.pose.x / geo.BLOCK_SIZE)) < 1e-9
          and abs(f.pose.y / geo.BLOCK_SIZE - round(f.pose.y / geo.BLOCK_SIZE)) < 1e-9
          for f in frames))
check("扫描线 y 坐标 = 0.35,1.05,...,4.55（7 条，覆盖 14 列）",
      len(SCAN_LINE_YS) == 7 and abs(SCAN_LINE_YS[0] - 0.35) < 1e-9
      and abs(SCAN_LINE_YS[-1] - 4.55) < 1e-9)
again = generate_snake_timeline(start_t=-30.0, frame_hz=2.0)
check("生成确定性：两次调用逐帧相同",
      [(f.t, f.pose, f.cells) for f in frames] == [(f.t, f.pose, f.cells) for f in again])

# --------------------------------------------------------------------------- #
section("③ 布局 v2：known 单调 / 特征 ⊆ known / version 递增 / target 合法")
truth = TruthMap.load(MAP)
check("真值图读出：起 N10 终 N5、雷/障/靶/桥洞非空",
      truth.start == "N10" and truth.end == "N5" and truth.mines
      and truth.obstacles and truth.targets and truth.bridges,
      f"{sorted(truth.bridges)}")
stream = LayoutStream(truth)
layouts = stream.layouts_for(frames)
knowns = [len(x["known"]) for x in layouts]
versions = [int(x["version"]) for x in layouts]
check("布局序列非空（每有新格才发）", len(layouts) > 30, f"{len(layouts)}")
check("★ known 单调增长（只增不减）",
      all(knowns[i] <= knowns[i + 1] for i in range(len(knowns) - 1))
      and knowns[-1] == TOTAL, f"{knowns[0]}..{knowns[-1]}")
check("★ version 严格递增（1..N）", versions == list(range(1, len(layouts) + 1)),
      f"{versions[:3]}..{versions[-1]}")
check("最终 known = 全图 154 格", set(layouts[-1]["known"]) == ALL)
check("★ 特征 ⊆ known（mines/obstacles/targets 都只报已观测格）",
      all(set(x["mines"]) | set(x["obstacles"]) | set(x["targets"])
          <= set(x["known"]) for x in layouts))
check("特征从真值图取（全 known 后 = 真值集合）",
      set(layouts[-1]["mines"]) == set(truth.mines)
      and set(layouts[-1]["obstacles"]) == set(truth.obstacles)
      and set(layouts[-1]["targets"]) == set(truth.targets))
check("bridges/隔板照真值图", set(layouts[-1]["bridges"]) == set(truth.bridges)
      and layouts[-1]["barrier_col_from"] == truth.barrier_col_from)
check("★ target_cell 每份都合法且 ∈ known",
      all(x["target_cell"] in set(x["known"]) and x["target_cell"] in ALL
          for x in layouts))
check("全 known 后终点可达 -> target_cell = end N5",
      layouts[-1]["target_cell"] == "N5", layouts[-1]["target_cell"])
check("segment_id = version（车回报原样带回）",
      all(int(x["segment_id"]) == int(x["version"]) for x in layouts))

_known = {"N10", "M10", "N9", "M9", "N8", "M8", "A1"}      # A1 是已知孤岛
_target = choose_target_cell(_known, truth, "N10")
check("★ 目标格必须是可达前沿：孤岛 A1 不被选",
      _target != "A1" and _target in _known, f"{_target}")
_reach = reachable_cells(_known, truth, "N10")
check("孤岛不在可达集合里", "A1" not in _reach, f"{sorted(_reach)}")

check("★ 隔板 D 列不可穿（D7->D6 非法）",
      not step_ok(truth, {"D7", "D6"}, "D7", "D6"))
check("★ 隔板 A/B/C 列可穿（B7->B6 合法）",
      step_ok(truth, {"B7", "B6"}, "B7", "B6"))
_bridge_rows = sorted(truth.bridge_rows)
check("真值图桥洞都在 K 列（1..11）", bool(_bridge_rows)
      and all(1 <= r <= geo.MAP_ROWS for r in _bridge_rows), f"{_bridge_rows}")
_br = _bridge_rows[0]
_non_br = next(r for r in range(1, geo.MAP_ROWS + 1) if r not in truth.bridge_rows)
_k = geo.COL_LETTERS.index("K")
_kn = geo.index_to_block(_br - 1, _k)
_kx = geo.index_to_block(_non_br - 1, _k)
check("★ K 列只能在桥洞行进出（K{} 可进 / K{} 不可进）".format(_br, _non_br),
      step_ok(truth, {_kn, geo.index_to_block(_br - 1, _k - 1)},
              geo.index_to_block(_br - 1, _k - 1), _kn)
      and not step_ok(truth, {_kx, geo.index_to_block(_non_br - 1, _k - 1)},
                      geo.index_to_block(_non_br - 1, _k - 1), _kx))
check("★ K 列内不得上下挪格（K 列垂直移动非法）",
      not step_ok(truth, {_kn, geo.index_to_block(_br, _k)},
                  _kn, geo.index_to_block(_br, _k)))

_bad = 0
for item in layouts:
    norm, why = normalize_field_layout(item)
    if norm is None:
        _bad += 1
        print(f"  !! 车端拒收 v{item['version']}: {why}")
check("★ 闭环：全部布局被车端 normalize_field_layout 接受（v2 字段名/语义一致）",
      _bad == 0 and FIELD_LAYOUT_VERSION >= 2, f"bad={_bad}")
_v1 = {"version": 1, "start": "N10", "end": "N5", "mines": [], "obstacles": [],
       "targets": [], "bridges": ["K7"]}
_n1, _ = normalize_field_layout(_v1)
check("v1 旧布局仍兼容（无 known 键，不触发车端步进模式）",
      _n1 is not None and "known" not in _n1)

# --------------------------------------------------------------------------- #
section("④ 心跳：变化即发 + 每秒重发最新版（同 version）")
sched = heartbeat_schedule([0.0, 2.0, 5.0], 6.0, interval_s=1.0)
check("★ schedule 时刻 = 0..6 每秒一条（变化时刻不重复）",
      [round(t, 3) for t, _ in sched] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
      f"{sched}")
check("★ 版本序号：0,0,1,1,1,2,2（变化之间重发最新版）",
      [i for _, i in sched] == [0, 0, 1, 1, 1, 2, 2], f"{sched}")
check("无变化 -> 无发送", heartbeat_schedule([], 10.0) == [])
_msgs = heartbeat_messages(layouts, t_end=frames[-1].t, interval_s=1.0)
check("布局序列 + 心跳：消息数 >= 布局数、版本非降",
      len(_msgs) >= len(layouts)
      and all(int(_msgs[i][1]["version"]) <= int(_msgs[i + 1][1]["version"])
              for i in range(len(_msgs) - 1)),
      f"{len(_msgs)} >= {len(layouts)}")
_change_ts = {round(float(x["ts"]), 6) for x in layouts}
check("心跳时间单调、且每份布局的 ts 都作为变化帧被发出",
      [t for t, _ in _msgs] == sorted(t for t, _ in _msgs)
      and _change_ts <= {round(t, 6) for t, _ in _msgs},
      f"{len(_msgs)} 条 / {len(_change_ts)} 次变化")

feed = LayoutFeed(interval_s=1.0)
check("LayoutFeed：未 push 前 poll=None", feed.poll(0.0) is None)
_l = dict(layouts[0])
check("push 新版本 -> poll 立即返回", feed.push(_l) and feed.poll(0.0) is not None)
check("0.4 s 内不重复发", feed.poll(0.4) is None)
_again_msg = feed.poll(1.0)
check("满 1 s 重发同一版本（version 不变）",
      _again_msg is not None and _again_msg["version"] == _l["version"])
check("旧版本 push 被忽略", feed.push(dict(_l, version=0)) is False)
feed.push(dict(layouts[1]))
check("新版本 push -> 立即发新版",
      feed.poll(1.1)["version"] == layouts[1]["version"])

# --------------------------------------------------------------------------- #
section("⑤ 对照：用户手绘路线（M8 -> ... -> M2） vs 推荐路线")
check("用户路线起点 M8 / 终点 M2 / 格序列 59 格",
      USER_ROUTE_BLOCKS[0] == "M8" and USER_ROUTE_BLOCKS[-1] == "M2"
      and len(USER_ROUTE_BLOCKS) == 59, f"{len(USER_ROUTE_BLOCKS)}")
check("来源元数据可复核（图路径/方法/起点/终点）",
      "scout_route_user.png" in USER_ROUTE_SOURCE["image"]
      and USER_ROUTE_SOURCE["start"] == "M8")
ref = user_route_timeline(stride=1)
ref2 = user_route_timeline(stride=2)
cmp = compare_routes(frames)
check("★ 用户手绘：84 帧 / 41.5 s / 覆盖 146/154（94.8%，N 列有缺口）",
      cmp["user_route"]["frames"] == 84
      and abs(cmp["user_route"]["duration_s"] - 41.5) < 1e-9
      and cmp["user_route"]["coverage_cells"] == 146
      and set(cmp["user_route"]["missing"]) == {"N1", "N2", "N3", "N6",
                                                "N9", "N10", "M11", "N11"},
      f"{cmp['user_route']['frames']}/{cmp['user_route']['coverage_cells']}")
check("★ 推荐路线覆盖更高（154 > 146）、帧数更少（68 < 84）、时长更短（33.5 < 41.5）",
      cmp["delta"]["coverage_cells"] > 0 and cmp["delta"]["frames"] < 0
      and cmp["delta"]["duration_s"] < 0, f"{cmp['delta']}")
check("同帧距（0.70 m）口径：用户路线 35 帧 / 17.0 s / 覆盖 126/154",
      cmp["user_route_same_step"]["frames"] == 35
      and abs(cmp["user_route_same_step"]["duration_s"] - 17.0) < 1e-9
      and cmp["user_route_same_step"]["coverage_cells"] == 126,
      f"{cmp['user_route_same_step']}")
check("用户路线相邻帧重叠同样 >=30%（同一视场模型）",
      min(adjacent_overlaps(ref)) >= 0.3 - 1e-9
      and min(adjacent_overlaps(ref2)) >= 0.3 - 1e-9)

# --------------------------------------------------------------------------- #
section("⑥ 一站式 simulate + CLI 冒烟")
res = simulate(start_t=-30.0, frame_hz=2.0)
check("simulate：帧/布局/心跳/对照一次产出",
      len(res.frames) == 68 and res.layouts and res.heartbeat
      and res.stats["coverage_cells"] == TOTAL and res.known_count == TOTAL,
      f"frames={len(res.frames)} layouts={len(res.layouts)} hb={len(res.heartbeat)}")
check("simulate 的布局末版 target = end N5（车已能到终点）",
      res.layouts[-1]["target_cell"] == "N5")
_d = res.as_dict()
check("as_dict 可 JSON 序列化（T29 直接落盘）",
      bool(json.dumps(_d, ensure_ascii=False)) and _d["known_count"] == TOTAL)

_proc = subprocess.run([sys.executable, str(WS / "tools" / "scout_sim.py"), "--json"],
                       cwd=str(WS), capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=120)
check("CLI tools/scout_sim.py --json exit 0", _proc.returncode == 0,
      f"rc={_proc.returncode} stderr={_proc.stderr[-300:]}")
try:
    _payload = json.loads(_proc.stdout)
    _cli_ok = (_payload.get("known_count") == TOTAL
               and _payload.get("stats", {}).get("coverage_cells") == TOTAL
               and bool(_payload.get("layouts")))
except Exception as exc:                        # noqa: BLE001
    _payload, _cli_ok = {}, False
    print(f"  !! CLI JSON 解析失败: {exc}")
check("CLI JSON 含 stats/layouts/known_count（覆盖率 154）", _cli_ok,
      f"known={_payload.get('known_count')}")

finish()