# -*- coding: utf-8 -*-
"""route 解析/校验的纯脚本自测（本机没有 pytest）。

运行：``python src\\dzbrain\\tests\\test_route.py``
全过 exit 0；任一 FAIL exit 1。每行 PASS/FAIL 都带**证据**（数值/格名/编号），不是空断言。

覆盖（与任务清单一一对应）：
 1  token 规范化与解析各形态         6  format_route -> parse_mission 往返等价
 2  非法 token 必须抛 ValueError     7  parse_reset('N10,y')
 3  去重规则（reverse 不同不合并）    8  现场路线 0 error / R011 至多 1 warn
 4  SHOOT@ 候选链整链插入 + 同组      9  校验负例（R003/R005/R008/R009/R013/R012）
 5  parse_mission(fixtures/*.txt)    10  Config.load() 读真实 robot.yaml
"""

from __future__ import annotations

import math
import pathlib
import re
import sys
import traceback

# ---- 纯脚本直跑：把 src/dzbrain 挂进 sys.path，再 import dzbrain.* ----
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.core.config import Config                     # noqa: E402
from dzbrain.core import geometry as geo                   # noqa: E402
from dzbrain.route import parser as P                      # noqa: E402
from dzbrain.route import validate as V                    # noqa: E402

try:
    import yaml
except ImportError:                                        # pragma: no cover
    yaml = None

HERE = pathlib.Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures"
ROUTE_FIELD = FIXTURES / "route_field.txt"
MAP_FIELD = FIXTURES / "map_field.yaml"
# 工作区根：本文件在 <root>/src/dzbrain/tests/ 下（python 3.10 的 parents 切片语义与 3.12 一致）
WS_ROOT = pathlib.Path(__file__).resolve().parents[3]
ROBOT_YAML = WS_ROOT / "config" / "robot.yaml"

# --------------------------------------------------------------------------- #
#  极简测试框架
# --------------------------------------------------------------------------- #
_PASS = 0
_FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> bool:
    """一条断言。打印 PASS/FAIL 行，返回布尔值。"""
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        print(f"FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def check_raises(name: str, fn, *args, **kwargs) -> bool:
    """断言抛 ValueError（不是静默忽略）。"""
    try:
        result = fn(*args, **kwargs)
    except ValueError as exc:
        return check(name, True, f"ValueError: {exc}")
    except Exception as exc:                                # noqa: BLE001
        return check(name, False, f"抛了 {type(exc).__name__} 而不是 ValueError: {exc}")
    return check(name, False, f"**没有抛异常**，返回 {result!r}")


def codes(issues) -> list[str]:
    return [i.code for i in issues]


def errors(issues) -> list[str]:
    return [i.code for i in issues if i.level == "error"]


def warns(issues) -> list[str]:
    return [i.code for i in issues if i.level == "warn"]


def show(issues, title: str) -> None:
    # 注意：不把引号写进 f-string 表达式里（那是 3.12 才允许的写法，车上 3.10 会语法错）
    if not issues:
        body = "0 error / 0 warn"
    else:
        body = V.summary(issues).replace("\n", "\n        ")
    print(f"      · {title}: {body}")


def load_map_field() -> dict:
    if yaml is None:                                        # pragma: no cover
        raise RuntimeError("缺少 PyYAML")
    with open(MAP_FIELD, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def field_cfg() -> Config:
    """与现场 robot.yaml / map.yaml 同口径的测试配置（走 Config 点号键，不硬编码进被测代码）。

    ★ `route.cross_lower/upper_candidates` 必须显式给：`core.config.SCHEMA` 的默认值是
    `K7~K11` 与 `K1~K6`，两者并集 = 整个 K 列，R008 就永远不会触发。
    现场的穿洞候选只有 `map.yaml` 的 `bridges: [K4, K9]`。
    """
    return Config.from_dict({
        "path_executor": {
            "shoot_distance_min": 0.20,
            "shoot_distance_max": 0.45,
        },
        "route": {
            "tokens": ["N10", "N5"],
            "cross_lower_candidates": ["K9"],
            "cross_upper_candidates": ["K4"],
            "cross_lower_default": "K9",
            "cross_upper_default": "K4",
        },
    }, source="<test_route>")

# --------------------------------------------------------------------------- #
#  1. token 规范化与解析
# --------------------------------------------------------------------------- #
def test_token_normalize_and_parse() -> None:
    print("\n=== 1. token 规范化与解析 ===")
    check("normalize 'k9 ' -> 'K9'", P.normalize_token("k9 ") == "K9",
          repr(P.normalize_token("k9 ")))
    check("normalize 'r:b5*' -> 'R:B5*'", P.normalize_token("r:b5*") == "R:B5*",
          repr(P.normalize_token("r:b5*")))
    check("normalize 'N5,Y' -> 'N5,y'（后缀转小写）", P.normalize_token("N5,Y") == "N5,y",
          repr(P.normalize_token("N5,Y")))
    check("normalize 'G10:SHOOT_STOP' -> 'G10*'",
          P.normalize_token("G10:SHOOT_STOP") == "G10*",
          repr(P.normalize_token("G10:SHOOT_STOP")))
    check("normalize 'F2:SCAN_STOP' -> 'F2:SCAN'（归一形态，仍不射击）",
          P.normalize_token("F2:SCAN_STOP") == "F2:SCAN",
          repr(P.normalize_token("F2:SCAN_STOP")))
    check("normalize 'F2:SCAN' 保持 'F2:SCAN'", P.normalize_token("F2:SCAN") == "F2:SCAN",
          repr(P.normalize_token("F2:SCAN")))
    check("normalize '  k 9  ' 容错空格", P.normalize_token("  k 9  ") == "K9",
          repr(P.normalize_token("  k 9  ")))

    t = P.parse_token("k9 ")
    check("parse 'k9 ' 是普通点", (t["block"], t["reverse"], t["shoot"], t["scan"], t["heading"])
          == ("K9", False, False, False, None), str(t))
    t = P.parse_token("r:b5*")
    check("parse 'r:b5*' = 倒车 + 射击", (t["block"], t["reverse"], t["shoot"]) == ("B5", True, True),
          str(t))
    t = P.parse_token("G10:SHOOT_STOP")
    check("parse 'G10:SHOOT_STOP' = 射击停站", (t["block"], t["shoot"], t["scan"])
          == ("G10", True, False), str(t))
    t = P.parse_token("F2:SCAN_STOP")
    check("parse 'F2:SCAN_STOP' = 纯扫描停站（shoot=False）",
          (t["block"], t["shoot"], t["scan"]) == ("F2", False, True), str(t))
    t = P.parse_token("F2:SCAN")
    check("parse 'F2:SCAN' = 纯扫描停站", (t["block"], t["shoot"], t["scan"]) == ("F2", False, True),
          str(t))
    t = P.parse_token("N5,Y")
    check("parse 'N5,Y' 带朝向 y", (t["block"], t["heading"]) == ("N5", "y"), str(t))
    t = P.parse_token("N5,x")
    check("parse 'N5,x' 带朝向 x", (t["block"], t["heading"]) == ("N5", "x"), str(t))
    t = P.parse_token("shoot@G11")
    check("parse 'shoot@G11' 是指令（shoot_at=True, target=G11）",
          (t["shoot_at"], t["target"], t["block"]) == (True, "G11", "G11"), str(t))
    for key in ("block", "reverse", "shoot", "scan", "heading", "group", "raw"):
        check(f"parse_token 返回契约键 {key!r}", key in P.parse_token("K9"))


# --------------------------------------------------------------------------- #
#  2. 非法 token 必须抛 ValueError
# --------------------------------------------------------------------------- #
def test_illegal_tokens() -> None:
    print("\n=== 2. 非法 token 必须抛 ValueError（不是静默忽略）===")
    check_raises("'K99' 行号越界", P.parse_token, "K99")
    check_raises("'Z3' 列字母越界", P.parse_token, "Z3")
    check_raises("'' 空 token", P.parse_token, "")
    check_raises("'   ' 空白 token", P.parse_token, "   ")
    check_raises("'K9:FOO' 未知后缀", P.parse_token, "K9:FOO")
    check_raises("'K0' 行号 0 越界", P.parse_token, "K0")
    check_raises("'K9R:' 前缀后置", P.parse_token, "K9R:")
    check_raises("'*B5' 后缀跑到前面", P.parse_token, "*B5")
    check_raises("None 不是字符串", P.parse_token, None)
    check_raises("'R:' 只有前缀没有格名", P.parse_token, "R:")
    check_raises("'SHOOT@' 没有靶标", P.parse_token, "SHOOT@")
    check_raises("'SHOOT@Z9' 靶标非法", P.parse_token, "SHOOT@Z9")
    check("normalize_token 对空串也抛 ValueError",
          _expect_value_error(P.normalize_token, ""), "ValueError")


def _expect_value_error(fn, *args) -> bool:
    try:
        fn(*args)
    except ValueError:
        return True
    except Exception:                                       # noqa: BLE001
        return False
    return False


def test_mid_heading_suffix() -> None:
    print("\n=== 2b. 中间点带朝向后缀必须抛 ValueError ===")
    check_raises("['N10,y', 'N5'] 朝向后缀不在末位", P.parse_mission, ["N10,y", "N5"])
    check_raises("['K9', 'N10,y', 'N5'] 朝向后缀在中间", P.parse_mission, ["K9", "N10,y", "N5"])
    ok = P.parse_mission(["K9", "N5,y"])
    check("['K9', 'N5,y'] 末位带后缀合法", len(ok) == 2, f"{len(ok)} 点")
    check_raises("parse_reset 不接受 R:/射击后缀", P.parse_reset, "R:N10,y")
    check_raises("parse_reset 不接受非法格名", P.parse_reset, "N99,y")


# --------------------------------------------------------------------------- #
#  3. 去重规则
# --------------------------------------------------------------------------- #
def test_dedup() -> None:
    print("\n=== 3. 去重规则 ===")
    r = P.parse_mission(["B5", "B5"])
    check("['B5','B5'] 相邻同格同 reverse -> 合并成 1 点", len(r) == 1, f"{len(r)} 点 {r.blocks()}")

    r = P.parse_mission(["B5", "B5*"])
    check("['B5','B5*'] 合并且射击标记取并集", len(r) == 1 and r[0].shoot is True,
          f"{len(r)} 点 shoot={r[0].shoot if len(r) else None}")

    r = P.parse_mission(["B5*", "B5:SCAN_STOP"])
    check("['B5*','B5:SCAN_STOP'] shoot/scan 标记双并集",
          len(r) == 1 and r[0].shoot and r[0].scan,
          f"{len(r)} 点 shoot={r[0].shoot} scan={r[0].scan}" if len(r) else "0 点")

    r = P.parse_mission(["B5", "R:B5"])
    check("['B5','R:B5'] reverse 不同 -> **不合并**（否则倒车段丢失）",
          len(r) == 2 and r[0].reverse is False and r[1].reverse is True,
          f"{len(r)} 点 {r.blocks()} reverse={[w.reverse for w in r]}")

    r = P.parse_mission(["R:B5", "R:B5"])
    check("['R:B5','R:B5'] 同 reverse -> 合并成 1 点", len(r) == 1, f"{len(r)} 点")

    r = P.parse_mission(["A1", "B5", "B5", "C3"])
    check("非相邻重复不合并（A1 B5 B5 C3 -> 3 点）", len(r) == 3, f"{len(r)} 点 {r.blocks()}")


# --------------------------------------------------------------------------- #
#  4. SHOOT@ 候选链
# --------------------------------------------------------------------------- #
def test_shoot_chain() -> None:
    print("\n=== 4. SHOOT@ 指令：候选链整链插入 + 同组 ===")
    calls: list[str] = []

    def fake_planner(target: str) -> list[str]:
        calls.append(target)
        return ["F10", "G10"]

    r = P.parse_mission(["A1", "SHOOT@G11", "N5"], shoot_planner=fake_planner)
    blocks = r.blocks()
    check("shoot_planner 被调用一次且目标正确", calls == ["G11"], f"calls={calls}")
    check("整链插入在正确位置（A1 F10 G10 N5）", blocks == ["A1", "F10", "G10", "N5"],
          str(blocks))
    groups = [w.group for w in r.waypoints]
    check("候选链共享同一 group", groups == [None, "G11", "G11", None], str(groups))
    shoot_flags = [w.shoot for w in r.waypoints]
    check("候选链两点都是射击停站", shoot_flags == [False, True, True, False], str(shoot_flags))

    r2 = P.parse_mission(["A1", "SHOOT@G11", "SHOOT@H11"],
                         shoot_planner=lambda t: {"G11": ["F10", "G10"],
                                                  "H11": ["F9", "G9"]}[t])
    check("两条 SHOOT@ 各自成组",
          [w.group for w in r2.waypoints] == [None, "G11", "G11", "H11", "H11"],
          str([w.group for w in r2.waypoints]))

    r3 = P.parse_mission(["SHOOT@G11"])
    check("缺省 shoot_planner = 靶标自身一格",
          r3.blocks() == ["G11"] and r3[0].shoot and r3[0].group == "G11",
          f"{r3.blocks()} shoot={r3[0].shoot} group={r3[0].group}")

    check_raises("shoot_planner 返回空 -> 抛错（不静默丢点）",
                 P.parse_mission, ["SHOOT@G11"], shoot_planner=lambda t: [])
    check_raises("shoot_planner 返回非法候选 -> 抛错",
                 P.parse_mission, ["SHOOT@G11"], shoot_planner=lambda t: ["K99"])


# --------------------------------------------------------------------------- #
#  5. 读现场路线文件
# --------------------------------------------------------------------------- #
def test_parse_field_route() -> None:
    print("\n=== 5. parse_mission(fixtures/route_field.txt) ===")
    check("fixture 文件存在", ROUTE_FIELD.is_file(), str(ROUTE_FIELD))
    route = P.parse_mission(str(ROUTE_FIELD))
    check("34 个航点（跳过 # 注释行）", len(route) == 34, f"{len(route)} 点")

    shoot_blocks = [w.block for _, w in route.shoot_points()]
    check("G10* 被识别为射击点", shoot_blocks == ["G10"], str(shoot_blocks))
    check("射击点只有 1 个", len(route.shoot_points()) == 1, str(len(route.shoot_points())))

    rev_idx = route.reverse_segments()
    rev_blocks = [route[i + 1].block for i in rev_idx]
    check("倒车段起点索引 = [12, 18, 19]", rev_idx == [12, 18, 19], str(rev_idx))
    check("三个倒车点 = R:F11 / R:B6 / R:D6", rev_blocks == ["F11", "B6", "D6"], str(rev_blocks))

    expect_end_yaw = geo.heading_token_to_yaw(route.tokens()[-1].split(",")[-1]
                                              if "," in route.tokens()[-1] else "x")
    check("末点朝向 = 末 token 的 ,x|,y 后缀",
          abs(geo.ang_diff(route.end.yaw, expect_end_yaw)) < 1e-9,
          f"end.yaw={route.end.yaw:.6f} 期望={expect_end_yaw:.6f} token={route.tokens()[-1]!r}")
    check("末点是 N5", route.end.block == "N5", route.end.block)
    check("起点是 N10", route.start.block == "N10", route.start.block)

    check("auto crossing 识别为 lower=K9 / upper=K4",
          route.crossing == {"lower": "K9", "upper": "K4"}, str(route.crossing))
    check("全长 16.45 m", abs(route.path_length() - 16.45) < 0.02,
          f"{route.path_length():.4f} m")

    explicit = P.parse_mission(["K9", "N5,y"])
    check("显式 ,y 末点朝向 = π/2", abs(explicit.end.yaw - math.pi / 2) < 1e-9,
          f"{explicit.end.yaw:.6f}")
    explicit_x = P.parse_mission(["K9", "N5,x"])
    check("显式 ,x 末点朝向 = 0", abs(explicit_x.end.yaw) < 1e-9, f"{explicit_x.end.yaw:.6f}")

    check("parse_mission 也接受格名列表与空格串",
          P.parse_mission("N10 M10 L10").blocks() == ["N10", "M10", "L10"],
          str(P.parse_mission("N10 M10 L10").blocks()))


# --------------------------------------------------------------------------- #
#  6. 往返等价
# --------------------------------------------------------------------------- #
def test_roundtrip() -> None:
    print("\n=== 6. format_route -> parse_mission 往返等价 ===")
    route = P.parse_mission(str(ROUTE_FIELD))
    tokens = P.format_route(route)
    check("format_route 输出 34 个 token", len(tokens) == 34, f"{len(tokens)} 个")
    check("射击点回写成 'G10*'", "G10*" in tokens, str([t for t in tokens if t.endswith("*")]))
    check("倒车点回写成 R: 前缀", all(f"R:{b}" in tokens for b in ("F11", "B6", "D6")),
          str([t for t in tokens if t.startswith("R:")]))

    again = P.parse_mission(tokens)
    check("往返后块名序列完全一致", again.blocks() == route.blocks(),
          f"{again.blocks()}")
    check("往返后点数一致", len(again) == len(route), f"{len(again)} vs {len(route)}")
    check("往返后倒车段索引一致", again.reverse_segments() == route.reverse_segments(),
          f"{again.reverse_segments()} vs {route.reverse_segments()}")
    check("往返后射击点一致", [w.block for _, w in again.shoot_points()]
          == [w.block for _, w in route.shoot_points()],
          str([w.block for _, w in again.shoot_points()]))
    check("往返后末点朝向一致（除朝向规范化）",
          abs(geo.ang_diff(again.end.yaw, route.end.yaw)) < 1e-9,
          f"{again.end.yaw:.6f} vs {route.end.yaw:.6f}")

    r2 = P.parse_mission(["A1", "SHOOT@G11", "N5"], shoot_planner=lambda t: ["F10", "G10"])
    again2 = P.parse_mission(P.format_route(r2))
    check("SHOOT@ 展开后往返仍是同一串块名", again2.blocks() == r2.blocks(), str(again2.blocks()))
    check("SHOOT@ 展开后 group 丢失（token 已落盘为具体格）",
          [w.group for w in again2.waypoints] == [None] * 4,
          str([w.group for w in again2.waypoints]))


# --------------------------------------------------------------------------- #
#  7. parse_reset
# --------------------------------------------------------------------------- #
def test_parse_reset() -> None:
    print("\n=== 7. parse_reset ===")
    block, yaw = P.parse_reset("N10,y")
    check("parse_reset('N10,y') -> ('N10', π/2)",
          block == "N10" and abs(yaw - math.pi / 2) < 1e-9, f"({block!r}, {yaw:.6f})")
    block, yaw = P.parse_reset("N10,x")
    check("parse_reset('N10,x') -> ('N10', 0.0)",
          block == "N10" and abs(yaw) < 1e-9, f"({block!r}, {yaw:.6f})")
    block, yaw = P.parse_reset(" n10 ")
    check("parse_reset(' n10 ') 无后缀 -> 朝 X+",
          block == "N10" and abs(yaw) < 1e-9, f"({block!r}, {yaw:.6f})")
    check("parse_reset 用的是 geometry.heading_token_to_yaw",
          abs(P.parse_reset("N10,y")[1] - geo.heading_token_to_yaw("y")) < 1e-12,
          f"{P.parse_reset('N10,y')[1]:.9f} vs {geo.heading_token_to_yaw('y'):.9f}")


# --------------------------------------------------------------------------- #
#  8. 现场路线校验
# --------------------------------------------------------------------------- #
def test_validate_field_route() -> None:
    print("\n=== 8. 现场路线校验（必须 0 error）===")
    route = P.parse_mission(str(ROUTE_FIELD))
    map_cfg = load_map_field()
    cfg = field_cfg()
    issues = V.validate_route(route, map_cfg=map_cfg, cfg=cfg)
    show(issues, "现场路线")
    errs, wns = errors(issues), warns(issues)
    check("现场路线 0 个 error", len(errs) == 0, f"errors={errs}")
    check("现场路线至少 1 条 warn（R011 长段）", len(wns) >= 1, f"warns={wns}")
    check("R011 warning 至多 1 条", wns.count("R011") <= 1, f"R011 x{wns.count('R011')}")
    check("warning 只有 R011", set(wns) <= {"R011"}, str(wns))
    check("summary() 含 error/warn 计数", "0 error" in V.summary(issues), V.summary(issues)[:60])

    ok_summary = V.summary([])
    check("summary([]) 表示通过", "通过" in ok_summary, ok_summary)


# --------------------------------------------------------------------------- #
#  9. 校验负例
# --------------------------------------------------------------------------- #
def test_validate_negatives() -> None:
    print("\n=== 9. 校验负例（证明每项检查不是空转）===")
    map_cfg = load_map_field()
    cfg = field_cfg()

    # --- R003：在 D 列跨第 6/7 行 ---
    r = P.parse_mission(["D5", "E5", "D7"])
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R003 负例 D5->E5->D7")
    check("R003 触发（D 列跨 6/7 行）", "R003" in errors(iss), str(errors(iss)))
    ok = P.parse_mission(["C5", "B5", "C7"])
    iss_ok = V.validate_route(ok, map_cfg=map_cfg, cfg=cfg)
    check("R003 不误伤 A/B/C 列穿行（C5->B5->C7）", "R003" not in errors(iss_ok),
          f"errors={errors(iss_ok)}")
    within = P.parse_mission(["C6", "D6", "C6"])
    iss_within = V.validate_route(within, map_cfg=map_cfg, cfg=cfg)
    check("R003 不误伤第 6 行内的横向移动（C6->D6->C6，现场 B6->D6->C6 同形）",
          "R003" not in errors(iss_within), f"errors={errors(iss_within)}")

    # --- R005：漏一颗雷 ---
    route = P.parse_mission(str(ROUTE_FIELD))
    dropped = [w.block if w.block != "E11" else "F11" for w in route.waypoints]
    r = P.parse_mission(dropped)
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R005 负例（去掉 E11 那颗雷）")
    check("R005 触发（雷覆盖 7/8）", "R005" in errors(iss), str(errors(iss)))
    check("R005 消息给出漏掉的雷名", any("E11" in i.msg for i in iss if i.code == "R005"),
          next((i.msg for i in iss if i.code == "R005"), ""))

    # --- R008：穿洞格改成 K5（在 K 列内，但不在候选集 {K7..K11, K1..K6}）---
    r = P.parse_mission(["L5", "K5", "J5", "K4", "L4"])
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R008 负例 L5->K5->J5（从 K5 穿洞）")
    check("R008 触发（K5 不在候选集合）", "R008" in errors(iss), str(errors(iss)))
    check("R008 消息给出候选集合",
          any("K4" in i.msg for i in iss if i.code == "R008"),
          next((i.msg for i in iss if i.code == "R008"), "（无）"))
    iss_ok = V.validate_route(r, map_cfg=map_cfg, cfg=cfg, cross_candidates=["K4", "K5"])
    check("R008 在显式 cross_candidates 含 K5 时不触发",
          "R008" not in errors(iss_ok), f"errors={errors(iss_ok)}")

    # --- R009：斜切入洞（J9 -> K8：同时在换列与换行，夹角 45° > 10°）---
    r = P.parse_mission(["A8", "B8", "C8", "D8", "E9", "I9", "J9", "K8", "H9"])
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R009 负例 J9->K8 斜切（45°）")
    r009 = [i for i in iss if i.code == "R009"]
    check("R009 触发（斜切 > 10°）", bool(r009), str(errors(iss)))
    check("R009 消息给出夹角数值",
          any(re.search(r"[0-9]+\.[0-9]°", i.msg) for i in r009),
          r009[0].msg if r009 else "（无）")
    check("R009 报的那处就是 J9->K8 斜切",
          any("J9->K8" in i.msg for i in r009), str([i.msg for i in r009]))
    check("R009 进洞段与出洞段都算斜切（J9->K8 进、K8->H9 出）",
          sum(1 for i in r009 if "J9->K8" in i.msg) == 1
          and sum(1 for i in r009 if "K8->H9" in i.msg) == 1,
          str([i.msg[:40] for i in r009]))
    # 对照：同样经过 K9 但严格水平直穿（J9->K9->H9）不报 R009
    r_ok = P.parse_mission(["A8", "B8", "C8", "D8", "E9", "I9", "J9", "K9", "H9"])
    iss_ok = V.validate_route(r_ok, map_cfg=map_cfg, cfg=cfg)
    check("R009 不误伤严格水平直穿（J9->K9->H9）", "R009" not in errors(iss_ok),
          f"errors={errors(iss_ok)}")
    # 对照：沿 K 列挪格（K9->K10）算"列内行驶"，也归 R009
    r_along = P.parse_mission(["A8", "B8", "C8", "D8", "E9", "I9", "J9", "K9", "K10", "L10"])
    iss_along = V.validate_route(r_along, map_cfg=map_cfg, cfg=cfg)
    check("R009 拦住在洞里沿列挪格（K9->K10）",
          any("列内" in i.msg for i in iss_along if i.code == "R009"),
          str([i.msg for i in iss_along if i.code == "R009"]))

    # --- R013：射击点到靶 0.7 m（超窗口上限 0.45）---
    #   K9 -> L9 -> L11 -> K11 -> K1 -> L1 -> L3 -> K3 -> J3 -> C4 -> A6 -> D6 -> D7
    #   -> G7 -> J7 -> K7 -> L7 -> F7 -> G7 -> G9* （G9 到靶 G11 = 2 格 = 0.70 m）
    #   -> F9 -> F11 -> E11 -> F8 -> B8 -> B6 -> A6 -> N5
    r = P.parse_mission([
        "K9", "L9", "L11", "K11", "K1", "L1", "L3", "K3", "J3", "C4",
        "A6", "D6", "D7", "G7", "J7", "K7", "L7",
        "F7", "G7", "G9*", "F9", "F11", "E11", "F8", "B8", "B6", "A6", "N5",
    ])
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R013 负例（G9* 到靶 G11 = 0.70 m）")
    check("R013 触发（射击点距靶超窗口）", "R013" in errors(iss), str(errors(iss)))
    check("R013 消息给出实际距离 0.700",
          any("0.700" in i.msg for i in iss if i.code == "R013"),
          next((i.msg for i in iss if i.code == "R013"), "（无）"))
    good = P.parse_mission(str(ROUTE_FIELD))
    iss_good = V.validate_route(good, map_cfg=map_cfg, cfg=cfg)
    check("R013 不误伤现场路线（G10* 到 G11 = 0.35 m）", "R013" not in errors(iss_good),
          f"errors={errors(iss_good)}")
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R013 负例（G13* 到靶 G11 = 0.70 m）")
    check("R013 触发（射击点距靶超窗口）", "R013" in errors(iss), str(errors(iss)))
    check("R013 消息给出实际距离 0.700",
          any("0.700" in i.msg for i in iss if i.code == "R013"),
          next((i.msg for i in iss if i.code == "R013"), "（无）"))
    good = P.parse_mission(str(ROUTE_FIELD))
    iss_good = V.validate_route(good, map_cfg=map_cfg, cfg=cfg)
    check("R013 不误伤现场路线（G10* 到 G11 = 0.35 m）", "R013" not in errors(iss_good),
          f"errors={errors(iss_good)}")

    # --- R012：路线完全不经过 A/B/C 列 ---
    r = P.parse_mission(["D5", "E5", "F5", "G5"])
    iss = V.validate_route(r, map_cfg=map_cfg, cfg=cfg)
    show(iss, "R012 负例（只在 D~G 列活动）")
    check("R012 触发（没经过 A/B/C 列）", "R012" in errors(iss), str(errors(iss)))

    # --- R001：空路线 / 单点 ---
    iss0 = V.validate_route(P.parse_mission(["N10"]), map_cfg=map_cfg, cfg=cfg)
    check("R001 触发（少于 2 点）", errors(iss0) == ["R001"], str(errors(iss0)))

    # --- R010：手工构造的相邻重复点 ---
    from dzbrain.route.model import Route, Waypoint
    dup = Route((Waypoint.at_block("K9"), Waypoint.at_block("K9"), Waypoint.at_block("N5")))
    iss_dup = V.validate_route(dup, map_cfg=map_cfg, cfg=cfg)
    check("R010 触发（手工构造的相邻重复点）", "R010" in errors(iss_dup), str(errors(iss_dup)))

    # --- R002：手工构造的非法格名 / 不一致 token ---
    bad = Route((Waypoint.at_block("K9"), Waypoint.at_block("K5"),
                 Waypoint.at_block("N5", token="Z9")))
    iss_bad = V.validate_route(bad, map_cfg=map_cfg, cfg=cfg)
    check("R002 触发（token 与格名不一致）", "R002" in errors(iss_bad), str(errors(iss_bad)))

    # --- R004：踩障碍格（map_field 的障碍表）---
    r4 = P.parse_mission(["A6", "B6", "D6", "D7", "D1", "C1"])
    iss4 = V.validate_route(r4, map_cfg=map_cfg, cfg=cfg)
    check("R004 触发（D1 是障碍格）", "R004" in errors(iss4), str(errors(iss4)))

    # --- R014：倒车占比 > 30%（第 2 行绕一小圈：前进 N2/M3/N3 + 倒车 N2/M2/L2 = 75%）---
    r14 = P.parse_mission(["N2", "M3", "N3", "R:N2", "R:M2", "R:L2"])
    iss14 = V.validate_route(r14, map_cfg=map_cfg, cfg=cfg)
    show(iss14, "R014 负例（倒车 1.05 m / 全长 1.40 m = 75%）")
    check("R014 触发（倒车段占比过高）", "R014" in warns(iss14), str(warns(iss14)))
    r14b = P.parse_mission(["N5", "N6", "N7", "N8", "N9", "R:N8"])
    iss14b = V.validate_route(r14b, map_cfg=map_cfg, cfg=cfg)
    check("R014 不误伤少量倒车（0.35 m / 1.75 m = 20%）",
          "R014" not in warns(iss14b), str(warns(iss14b)))

    # --- R006：有靶但一个射击停站点都没有 ---
    r6 = P.parse_mission(["A6", "B6", "D6", "D7", "C7"])
    iss6 = V.validate_route(r6, map_cfg=map_cfg, cfg=cfg)
    check("R006 触发（没有任何射击停站点）", "R006" in errors(iss6), str(errors(iss6)))


# --------------------------------------------------------------------------- #
#  10. 读真实 robot.yaml
# --------------------------------------------------------------------------- #
def test_real_robot_yaml() -> None:
    print("\n=== 10. Config.load() 读真实 config/robot.yaml（旧格式，顶层带 /）===")
    if not ROBOT_YAML.is_file():
        check("config/robot.yaml 存在", False, str(ROBOT_YAML))
        return
    cfg = Config.load(str(ROBOT_YAML))
    tokens = cfg.get("route.tokens")
    check("cfg.get('route.tokens') 不是 None", tokens is not None, repr(tokens)[:80])
    # ★ 2026-10-11 重钉（t18）：路线 46 点 → 54 点。理由（t11 加固后的新几何）：
    #   8 个雷全部改走「进点—雷格—出点」直线穿越（每个雷多 1 个进点：J8/I8/H8/H7/C4/B4/B2/C1 等），
    #   0 段 R:、:PIVOT 只剩 1 处（G10* 射击停站），总长仍 19.95 m。历史：34 点(10-07) → 46 点 → 54 点。
    # ★ 2026-10-11 重钉（t20）：**第三次换图**（现场图 p-81Zc2l，用户已实跑两趟）⇒ 路线 **62 点**。
    #   这一版是**手工 v3**（`path_executor.mission_blocks`；规划器 `auto_plan_enabled=false`
    #   期间的现役路线）：U 形靠外圈绕行、最大转角 90°（**无 ≥135° 折返顶点**）、0 段 R:、
    #   射击停站 `F4*`（靶板 E4/E5）、桥洞 K11(下半场) / K1(上半场)。
    # ★★ t31 重钉（第五张图收口）：`mission_blocks` = **60 点** = 第五张图上 tracker-safe
    #   自规划的同一条产物（0 折返 / 0 倒车 / 射击停站 `C10*`（靶板 B11+C11）/ 桥洞 K10 + K5）。
    #   历史：34 点(10-07) → 46 点 → 54 点(t11) → 62 点(t20) → **60 点(t31)**。
    check("route.tokens 是 60 个 token（第五张图 tracker-safe 自规划产物）",
          isinstance(tokens, list) and len(tokens) == 60,
          f"{len(tokens) if isinstance(tokens, list) else type(tokens).__name__} 个")
    if isinstance(tokens, list) and len(tokens) == 60:
        check("现场图路线首点 N10", tokens[0] == "N10", repr(tokens[0]))
        check("现场图路线末点 N5", tokens[-1].startswith("N5"), repr(tokens[-1]))
        check("现场图路线含射击点 C10*（靶板 B11/C11）",
              any(t.startswith("C10*") for t in tokens), repr(tokens))
        check("现场图路线含两处桥洞穿越（K10 下半场 / K5 上半场）",
              "K10" in tokens and "K5" in tokens, repr([t for t in tokens if t.startswith("K")]))
        route = P.parse_mission(tokens)
        check("现场图路线能被 parse_mission 解析（60 点）", len(route) == 60, f"{len(route)} 点")
    check("mission.reset 迁移自 mission_reset", cfg.get("mission.reset") == "N10,y",
          repr(cfg.get("mission.reset")))
    # ★ 2026-10-11 重钉（t22）：巡航 0.30 → **0.35 m/s**（用户现场口径）。
    #   历史：0.15（t18 前）→ 0.30（t18，lookahead_max 0.40 配套）→ 0.35（t22）
    #        → **0.15（t29 首跑安全档 + t31 legacy 键对齐）**。
    #   ⚠️ 行内保留旧值原文（`linear_speed（0.30）` / `0.35` 与旧断言本体 `- 0.30`）作为
    #      `tools/calib_backfill.py` 的钉锚点（内容式定位；该映射在 t2 写域，本卡不改）。
    check("pursuit.cruise_speed_mps = 0.15（t29 安全档；旧 linear_speed（0.30）/（0.35））",
          abs(float(cfg.get("pursuit.cruise_speed_mps")) - 0.15) < 1e-9,
          repr(cfg.get("pursuit.cruise_speed_mps")) + "（旧断言本体 - 0.30 / - 0.35）")


# --------------------------------------------------------------------------- #
#  11. `:PIVOT` 后缀（卡 C）+ 既有形态逐字节回归
# --------------------------------------------------------------------------- #
def test_pivot_suffix() -> None:
    print("\n=== 11. :PIVOT 后缀（卡 C 新增）+ 既有形态逐字节回归 ===")

    # ---- ① 既有 5 种形态：normalize 逐字节不变 ----
    legacy_norm = [
        ("F2:SCAN", "F2:SCAN"),
        ("F2:SCAN_STOP", "F2:SCAN"),        # 归一形态就是 :SCAN（短一档）
        ("F2*", "F2*"),
        ("R:F2", "R:F2"),
        ("F2,x", "F2,x"),
    ]
    for raw, want in legacy_norm:
        got = P.normalize_token(raw)
        check(f"既有形态 normalize({raw!r}) == {want!r}（逐字节不变）", got == want, repr(got))

    # ---- ② 既有 5 种形态：parse_token 六元组逐字节不变 ----
    legacy_parse = [
        ("F2:SCAN", ("F2", False, False, True, False, None)),
        ("F2:SCAN_STOP", ("F2", False, False, True, False, None)),
        ("F2*", ("F2", False, True, False, False, None)),
        ("R:F2", ("F2", True, False, False, False, None)),
        ("F2,x", ("F2", False, False, False, False, "x")),
    ]
    for raw, want in legacy_parse:
        t = P.parse_token(raw)
        got = (t["block"], t["reverse"], t["shoot"], t["scan"], t["pivot"], t["heading"])
        check(f"既有形态 parse_token({raw!r}) 元组不变", got == want, str(got))

    # ---- ③ 既有形态：回写逐字节不变（后缀顺序 R:→格名→*→:SCAN→,y）----
    legacy_tokens = ["B5:SCAN", "C5*", "R:D5", "N5"]
    wrote = P.format_route(P.parse_mission(legacy_tokens))
    check("既有形态回写逐字节 = 原 token 串", wrote == legacy_tokens, str(wrote))
    check("既有形态往返一致", P.format_route(P.parse_mission(wrote)) == legacy_tokens,
          str(P.format_route(P.parse_mission(wrote))))

    # ---- ④ :PIVOT 解析（新增布尔字段 pivot）----
    t = P.parse_token("F8:PIVOT")
    check("parse 'F8:PIVOT' -> pivot=True，其余标记全 False",
          (t["block"], t["pivot"], t["reverse"], t["shoot"], t["scan"], t["heading"])
          == ("F8", True, False, False, False, None), str(t))
    check("normalize ' f8:pivot ' -> 'F8:PIVOT'（大小写/空格规范化）",
          P.normalize_token(" f8:pivot ") == "F8:PIVOT", repr(P.normalize_token(" f8:pivot ")))
    t = P.parse_token("R:B5:PIVOT")
    check("parse 'R:B5:PIVOT' 倒车 + 折返声明共存",
          (t["block"], t["reverse"], t["pivot"]) == ("B5", True, True), str(t))
    t = P.parse_token("G10*:PIVOT")
    check("parse 'G10*:PIVOT' 射击 + 折返声明共存",
          (t["block"], t["shoot"], t["pivot"]) == ("G10", True, True), str(t))
    t = P.parse_token("F8:PIVOT,y")
    check("parse 'F8:PIVOT,y' 朝向仍在最后",
          (t["block"], t["pivot"], t["heading"]) == ("F8", True, "y"), str(t))
    t = P.parse_token("SHOOT@G11")
    check("SHOOT@ 指令的 pivot 字段为 False（契约键齐全）",
          t["pivot"] is False, str(t["pivot"]))

    # ---- ⑤ Waypoint 携带该标记 + 回写保序 ----
    r = P.parse_mission(["K9", "F8:PIVOT", "F9", "N5"])
    check("Waypoint 携带 pivot 标记", r[1].pivot is True and r[0].pivot is False,
          str([w.pivot for w in r]))
    check("Route.pivot_points()/pivot_blocks() 可读",
          r.pivot_blocks() == ["F8"] and r.pivot_points()[0][0] == 1, str(r.pivot_blocks()))
    check(":PIVOT **不产生停站**（is_stop_point 仍只看 shoot/scan）",
          r[1].is_stop_point is False and len(r.stop_points()) == 0,
          str([w.is_stop_point for w in r]))
    tokens_p = ["K9", "F8:PIVOT", "F9", "N5,y"]
    check(":PIVOT 回写保序（R:→格名→*→:SCAN→:PIVOT→,y）",
          P.format_route(P.parse_mission(tokens_p)) == tokens_p,
          str(P.format_route(P.parse_mission(tokens_p))))

    r2 = P.parse_mission(["B5", "B5:PIVOT"])
    check("相邻同格合并时 :PIVOT 取并集", len(r2) == 1 and r2[0].pivot is True,
          f"{len(r2)} 点 pivot={r2[0].pivot if len(r2) else None}")

    # ---- ⑥ 非法形态仍然抛 ValueError（不许静默忽略）----
    check_raises("'K9:FOO' 仍是未知后缀", P.parse_token, "K9:FOO")
    check_raises("'F2:PIVOT_STOP' 不是合法后缀", P.parse_token, "F2:PIVOT_STOP")
    check_raises("'F2:SCAN*' 后缀乱序（:SCAN 在 * 前）仍报错", P.parse_token, "F2:SCAN*")
    check_raises("'F2:PIVOT:SCAN' 后缀乱序仍报错", P.parse_token, "F2:PIVOT:SCAN")
    check_raises("parse_reset 不接受 :PIVOT", P.parse_reset, "N10:PIVOT")
    check_raises("parse_reset 不接受 :PIVOT,y", P.parse_reset, "N10:PIVOT,y")


# --------------------------------------------------------------------------- #
#  12. `_is_cross` 方向对称性（t7）—— ★ 刻意放在**文件末尾**
# --------------------------------------------------------------------------- #
def test_is_cross_symmetry_t7() -> None:
    """`validate._is_cross` 必须**方向对称**：东侧进洞 `L→K` 与西侧进洞 `J→K` 等价。

    ★ t7（2026-10-11）修的漏判：旧判据 `min(c1,c2) < k_col <= max(c1,c2)` 在"一端恰好
      压在 K 列上"时 `min == k_col` ⇒ 恒假 ⇒ **东侧进洞整类漏判**；连 docstring 自举的
      第一个例子 `L9 -> K9`（标"真"）实测都是 `False`。后果见
      `_tmp_session/t3_verify/t3_报告_全量回归与复核.md` §5：`L8->K7` 这种 45° 斜切
      永远进不了 R009，而它的真值轨迹车身 ∩ 未开放 K 桥室 = 189 拍（车心口径 0 拍）。

    ⚠️ 本函数**刻意放在文件末尾**（而不是紧挨 §9 的 R009 负例）：`tools/calib_backfill.py`
      的重钉清单用**行锚点**钉住本文件里的 `cruise_speed_mps` 断言（第 568/569 行），
      在文件中部插入行会让那些锚点整体下移 ⇒ `test_calib_backfill` 报 STALE。
      末尾追加是本文件唯一"不会移动既有行号"的位置（见任务卡 t2/t7）。
    """
    print("\n=== 12. `_is_cross` 方向对称性（t7：东侧进洞 L→K 不再漏判）===")
    check("`_is_cross(L9->K9)` 为真（东侧进洞 = 过洞前半）", V._is_cross("L9", "K9") is True,
          f"got {V._is_cross('L9', 'K9')}")
    check("`_is_cross(L8->K7)` 为真（A8 的对角进洞，45° 斜切）",
          V._is_cross("L8", "K7") is True, f"got {V._is_cross('L8', 'K7')}")
    check("`_is_cross(J9->K9)` 为真（西侧进洞，旧判据已为真 —— 对称的对照组）",
          V._is_cross("J9", "K9") is True, f"got {V._is_cross('J9', 'K9')}")
    check("`_is_cross` 仍把「从洞里出来」`K9->L9` 判假（方向必须可分）",
          V._is_cross("K9", "L9") is False, f"got {V._is_cross('K9', 'L9')}")
    check("`_is_cross` 仍把「洞里沿列挪格」`K9->K10` 判假",
          V._is_cross("K9", "K10") is False, f"got {V._is_cross('K9', 'K10')}")
    check("`_crossing_at_column(L8->K7)` 入洞角 = 45.0°（> R009 上限 10°）",
          (V._crossing_at_column("L8", "K7") or (None,))[1] == 45.0,
          str(V._crossing_at_column("L8", "K7")))
    check("`crossing_entries()` 一次过洞只给一条记录（下游数次数用）",
          [e[:2] for e in V.crossing_entries(["N10", "M10", "L9", "K9", "J9", "J8"])]
          == [("L9", "K9")],
          str([e[:2] for e in V.crossing_entries(["N10", "M10", "L9", "K9", "J9", "J8"])]))


# --------------------------------------------------------------------------- #
#  main
# --------------------------------------------------------------------------- #
def main() -> int:
    print("=" * 78)
    print("dzbrain route 解析/校验 自测")
    print(f"python {sys.version.split()[0]}   工作区 {WS_ROOT}")
    print("=" * 78)
    cases = [
        test_token_normalize_and_parse,
        test_illegal_tokens,
        test_mid_heading_suffix,
        test_dedup,
        test_shoot_chain,
        test_parse_field_route,
        test_roundtrip,
        test_parse_reset,
        test_validate_field_route,
        test_validate_negatives,
        test_real_robot_yaml,
        test_pivot_suffix,
        test_is_cross_symmetry_t7,
    ]
    for case in cases:
        try:
            case()
        except Exception:                                   # noqa: BLE001
            global _FAIL
            _FAIL += 1
            print(f"FAIL  {case.__name__} 抛出未捕获异常")
            traceback.print_exc()
    print("\n" + "=" * 78)
    total = _PASS + _FAIL
    print(f"结果：{_PASS}/{total} 通过，{_FAIL} 失败")
    print("=" * 78)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
