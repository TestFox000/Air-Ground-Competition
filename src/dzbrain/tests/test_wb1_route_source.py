# -*- coding: utf-8 -*-
"""WB-1 路线真源回归套件（卡 `WB-1-ROUTE-20261031`）。

运行：``python src/dzbrain/tests/test_wb1_route_source.py``
全过 exit 0；任一 FAIL exit 1。风格与 `test_route.py` 一致：纯脚本 + 极简 check()，本机不需要 pytest。

这一张卡钉的是什么（一句话）：**文件模式必须和列表模式解析出完全相同的 token，
且修复不得放宽后缀的严格规范序。**

覆盖：
 1  现场第七图夹具：文件模式 == 列表模式（68 token，末点带 `,y`）
 2  多形态合成路线：`R:` / `*` / `:SCAN` / `:PIVOT` / `,y` 混排 + 空白/逗号/分号三种分隔符
 3  负例（≥2，实际 8 条）：乱序后缀 / 朝向不在末尾 / 空文件 / 只有注释 / 光秃秃 `,y` / 重复 R:
 4  具名来源登记 `fixtures/route_sources.json`：field7/sixth/carloop/field 可解析、路径全在 fixtures 内、
    绝不指向 `config/map.yaml`（D-054 裁决4 / D-062：仓库不存现役地图，禁止默默兜底）
 5  夹具逐字快照保护：`route_field7_t24.txt` 的 sha256 钉死 ⇒ 本卡**没改夹具**
 6  `parse_mission(<路径字符串>)` 直调（文档用法）与 `_read_token_file` 等价
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys
import tempfile
import traceback

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from dzbrain.route import parser as P                      # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
FIXTURES = HERE / "fixtures"
ROUTE_FIELD7 = FIXTURES / "route_field7_t24.txt"
SOURCES_JSON = FIXTURES / "route_sources.json"
# 逐字快照（D-062）：夹具内容不许被本卡改动 ⇒ 钉哈希，动了立刻红
ROUTE_FIELD7_SHA256 = "6d029b7cfd713272aa47054de607309a268305acebf2b9668d865ba82732768e"

_PASS = 0
_FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> bool:
    global _PASS, _FAIL
    if cond:
        _PASS += 1
        print(f"PASS  {name}" + (f"   [{detail}]" if detail else ""))
    else:
        _FAIL += 1
        print(f"FAIL  {name}" + (f"   [{detail}]" if detail else ""))
    return bool(cond)


def expect_error(name: str, fn, needle: str = "") -> bool:
    """必须抛 ValueError 才算过；不抛 = 放水，抛但类型不对也算 FAIL。"""
    global _PASS, _FAIL
    try:
        fn()
    except ValueError as exc:
        if needle and needle not in str(exc):
            _FAIL += 1
            print(f"FAIL  {name}   [抛了但信息里没有 {needle!r}: {exc}]")
            return False
        _PASS += 1
        print(f"PASS  {name}   [{str(exc)[:70]}]")
        return True
    except Exception as exc:                                # noqa: BLE001
        _FAIL += 1
        print(f"FAIL  {name}   [抛的是 {type(exc).__name__}: {exc}]")
        return False
    _FAIL += 1
    print(f"FAIL  {name}   [没抛异常 —— 修复把严格规范序放通了]")
    return False


def write_tmp_route(text: str) -> str:
    """把一段路线文本落成临时 .txt（LF），返回路径。"""
    fh = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8",
                                     newline="\n")
    fh.write(text)
    fh.close()
    return fh.name


# --------------------------------------------------------------------------- #
#  1. 现场夹具：文件模式 == 列表模式
# --------------------------------------------------------------------------- #
def test_field7_file_equals_list() -> None:
    print("\n=== 1. 第七图夹具 route_field7_t24.txt：文件模式 == 列表模式 ===")
    check("夹具存在", ROUTE_FIELD7.is_file(), str(ROUTE_FIELD7))
    if not ROUTE_FIELD7.is_file():
        return
    from_file = P._read_token_file(str(ROUTE_FIELD7))
    from_list = [t for t in ROUTE_FIELD7.read_text(encoding="utf-8").split() if t]
    check("夹具是 68 个 token", len(from_list) == 68, f"{len(from_list)} 个")
    check("末点保留朝向后缀 `N5,y`（不再被撕成 `N5`+`y`）",
          from_file[-1] == "N5,y", repr(from_file[-1]))
    check("文件模式 == 列表模式（逐 token 相同）", from_file == from_list,
          f"文件 {len(from_file)} / 列表 {len(from_list)}；首个差异 "
          f"{next((i for i, (a, b) in enumerate(zip(from_file, from_list)) if a != b), '无')}")
    r_file = P.parse_mission(from_file)
    r_list = P.parse_mission(from_list)
    check("两种来源 parse_mission 出同一条路线（68 点）",
          len(r_file) == 68 and r_file.blocks() == r_list.blocks(),
          f"{len(r_file)} 点")
    check("末点朝向 y 落到 final yaw（fix_final_yaw 生效）",
          abs(r_file.waypoints[-1].yaw - _geo_y()) < 1e-9,
          f"yaw={r_file.waypoints[-1].yaw!r}")


def _geo_y() -> float:
    from dzbrain.core import geometry as geo
    return geo.HEADING_Y


# --------------------------------------------------------------------------- #
#  2. 多形态合成：混排后缀 + 三种分隔符
# --------------------------------------------------------------------------- #
COMBO = ["N10", "M10", "R:B5*", "F2:SCAN", "I4:PIVOT", "B4*", "G4:PIVOT", "N5,y"]


def test_combo_file_equals_list() -> None:
    print("\n=== 2. 合成路线：`R:`/`*`/`:SCAN`/`:PIVOT`/`,y` 混排 + 空白/逗号/分号分隔 ===")
    variants = {
        "一行空格分隔": " ".join(COMBO) + "\n",
        "一行逗号分隔": ",".join(COMBO) + "\n",
        "一行分号分隔": ";".join(COMBO) + "\n",
        "一行一 token": "\n".join(COMBO) + "\n",
        "多行 + 注释 + 尾注": ("# 起步\n" + " ".join(COMBO[:4]) +
                          "\n  " + ",".join(COMBO[4:7]) + "\nN5,y   # 终点朝 Y+\n"),
    }
    want = P.parse_mission(list(COMBO))
    for label, text in variants.items():
        path = write_tmp_route(text)
        try:
            got_file = P._read_token_file(path)
            check(f"{label} -> token 序列 == 列表模式", got_file == list(COMBO),
                  repr(got_file)[:90])
            got_route = P.parse_mission(path)          # 文档用法：直接把路径喂进去
            check(f"{label} -> parse_mission(路径) 路线等价",
                  got_route.blocks() == want.blocks() and got_route.blocks(),
                  f"{len(got_route)} 点")
        finally:
            pathlib.Path(path).unlink(missing_ok=True)


# --------------------------------------------------------------------------- #
#  3. 负例：修复不得放宽松
# --------------------------------------------------------------------------- #
def test_negatives() -> None:
    print("\n=== 3. 负例（乱序后缀 / 朝向不在末尾 / 空文件 / 非法形态仍必须报错）===")
    # 列表模式：既有严格性回归（本卡没碰 normalize_token / parse_mission，这几条是护栏）
    expect_error("列表模式 `F2:SCAN*`（乱序：SCAN 在 * 前）仍报错",
                 lambda: P.parse_mission(["F2:SCAN*"]))
    expect_error("列表模式 `N5,y*`（乱序：朝向后又跟 *）仍报错",
                 lambda: P.parse_mission(["N5,y*"]))
    expect_error("列表模式中间点带朝向 `K9,y M10` 仍报错",
                 lambda: P.parse_mission(["K9,y", "M10"]), needle="最后一个 token")
    expect_error("列表模式 `R:R:B5`（重复倒车前缀）仍报错", lambda: P.parse_mission(["R:R:B5"]))
    expect_error("列表模式空 token `''` 仍报错", lambda: P.parse_mission([""]), needle="空")

    # 文件模式：同一批严格性必须原样成立（这是本卡最容易悄悄放水的方向）
    for label, text in (
        ("文件 `F2:SCAN*`", "F2:SCAN*\n"),
        ("文件 `N5,y*`", "N5,y*\n"),
        ("文件 `K9,y M10`（朝向不在末尾）", "K9,y M10\n"),
        ("文件光秃秃 `,y`", ",y\n"),
        ("文件 `N5,y,M10`（朝向后又接逗号点）", "N5,y,M10\n"),
    ):
        path = write_tmp_route(text)
        try:
            expect_error(f"{label} 仍报错", lambda: P.parse_mission(path))
        finally:
            pathlib.Path(path).unlink(missing_ok=True)

    # 空 / 只有注释 的文件：不是「0 个 token 静默通过」，而是显式报错
    p_empty = write_tmp_route("")
    p_comments = write_tmp_route("# 只有注释\n\n   # 又是注释\n")
    try:
        expect_error("纯空文件报错", lambda: P._read_token_file(p_empty), needle="没有任何 token")
        expect_error("只有注释的文件报错", lambda: P._read_token_file(p_comments),
                     needle="没有任何 token")
    finally:
        pathlib.Path(p_empty).unlink(missing_ok=True)
        pathlib.Path(p_comments).unlink(missing_ok=True)
    expect_error("不存在的路线文件报错", lambda: P._read_token_file(str(FIXTURES / "nope_route.txt")),
                 needle="不存在")


# --------------------------------------------------------------------------- #
#  4. 具名来源登记（field7 / sixth / carloop / field）
# --------------------------------------------------------------------------- #
def test_named_sources() -> None:
    print("\n=== 4. 具名来源 fixtures/route_sources.json：命名解析，无 config/map.yaml 兜底 ===")
    check("来源登记文件存在", SOURCES_JSON.is_file(), str(SOURCES_JSON))
    if not SOURCES_JSON.is_file():
        return
    data = json.loads(SOURCES_JSON.read_text(encoding="utf-8"))
    sources = data.get("sources", {})
    for name in ("field7", "sixth", "carloop", "field"):
        check(f"具名来源 `{name}` 已登记", name in sources, repr(sorted(sources)))
    for name, entry in sources.items():
        for kind, rel in (("map", entry.get("map")), ("route", entry.get("route"))):
            if rel is None:
                continue
            target = (FIXTURES / rel).resolve()
            check(f"{name}.{kind} 指向真实文件", target.is_file(), rel)
            check(f"{name}.{kind} 在 fixtures 目录内（不越界）",
                  FIXTURES.resolve() in target.parents or target.parent == FIXTURES.resolve(), rel)
            check(f"{name}.{kind} 不是现役地图 `config/map.yaml`",
                  target.name != "map.yaml", rel)
    check("登记表里没有绝对路径（来源只能在 fixtures 内）",
          not any(pathlib.Path(str(v)).is_absolute()
                  for e in sources.values() for v in (e.get("map"), e.get("route")) if v))
    cfg_map = pathlib.Path(__file__).resolve().parents[3] / "config" / "map.yaml"
    check("仓库里 `config/map.yaml` 确实不存在（D-062 删除后不得恢复）", not cfg_map.exists(),
          str(cfg_map))
    # 现场夹具本身可解析成路线（name -> 路径 -> parse_mission 一条命令跑通）
    f7 = sources.get("field7", {}).get("route")
    if f7:
        r = P.parse_mission(str(FIXTURES / f7))
        check("field7 具名解析后 parse_mission 直接成功（68 点）", len(r) == 68, f"{len(r)} 点")


# --------------------------------------------------------------------------- #
#  5. 夹具逐字快照保护（本卡只改实现，不改夹具）
# --------------------------------------------------------------------------- #
def test_fixture_snapshot_pinned() -> None:
    print("\n=== 5. 夹具逐字快照：sha256 钉死（本卡没动夹具内容）===")
    check("route_field7_t24.txt 存在", ROUTE_FIELD7.is_file(), str(ROUTE_FIELD7))
    if not ROUTE_FIELD7.is_file():
        return
    digest = hashlib.sha256(ROUTE_FIELD7.read_bytes()).hexdigest()
    check("route_field7_t24.txt sha256 == 卡前基线", digest == ROUTE_FIELD7_SHA256,
          f"{digest[:16]} vs {ROUTE_FIELD7_SHA256[:16]}")


# --------------------------------------------------------------------------- #
#  6. 列表模式字符串（逗号分隔）旧行为不变 —— 修复只作用于文件模式
# --------------------------------------------------------------------------- #
def test_list_mode_string_unchanged() -> None:
    print("\n=== 6. 字符串列表模式旧语义不变（逗号仍当分隔符）===")
    check("parse_mission('N10,M10,L10') == 3 点",
          P.parse_mission("N10,M10,L10").blocks() == ["N10", "M10", "L10"],
          str(P.parse_mission("N10,M10,L10").blocks()))
    check("parse_mission('N10 M10 L10') == 3 点",
          P.parse_mission("N10 M10 L10").blocks() == ["N10", "M10", "L10"],
          str(P.parse_mission("N10 M10 L10").blocks()))
    check("列表 ['N5,y']（真列表模式）单点朝向合法",
          len(P.parse_mission(["N5,y"])) == 1, str(P.parse_mission(["N5,y"]).blocks()))
    # 字符串 'N5,y' 走的是 `_as_token_list` 的非文件分支（逗号=分隔符）：这是**卡前既有语义**，
    # 本卡豁免只覆盖 `_read_token_file`，故这里钉「行为不变」而不是顺手放宽字符串模式。
    expect_error("字符串模式 'N5,y'（非文件、非列表）行为不变：逗号仍当分隔符 -> 报错",
                 lambda: P.parse_mission("N5,y"))
    expect_error("字符串模式同样严格：`K9,y M10`（朝向不在末尾）报错",
                 lambda: P.parse_mission("K9,y M10"))


def main() -> int:
    cases = [
        test_field7_file_equals_list,
        test_combo_file_equals_list,
        test_negatives,
        test_named_sources,
        test_fixture_snapshot_pinned,
        test_list_mode_string_unchanged,
    ]
    for case in cases:
        try:
            case()
        except Exception:                                    # noqa: BLE001
            _FAIL_inc()
            print(f"FAIL  {case.__name__} 抛出未捕获异常")
            traceback.print_exc()
    print("\n" + "=" * 78)
    total = _PASS + _FAIL
    print(f"结果：{_PASS}/{total} 通过，{_FAIL} 失败")
    print("=" * 78)
    return 0 if _FAIL == 0 else 1


def _FAIL_inc() -> None:
    global _FAIL
    _FAIL += 1


if __name__ == "__main__":
    sys.exit(main())
