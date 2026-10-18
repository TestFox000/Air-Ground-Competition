"""车端全链路烟雾测试：UDP帧→解析→态势→路径→命令→禁火（13 类协议）。

13 类协议（canonical，2026-10-16 D 方案）：雷 = 非 K 列 PAPER（落格判
语义），桥洞 = K 列 PAPER，障碍/靶仍是独立类别。校验一律用
`if not ...: raise ValueError(...)`——assert 在 `python -O` 下会被整条
删掉，比赛当天现场常带 -O 跑，冒烟工具必须仍然拦得住（治理点：
tests/test_planner_degradation.py::test_no_assert_statements_in_production_code）。

不依赖 ROS/硬件，可在 Windows 直接运行，验证车端软件链路完整性。
用法：python tools/smoke_test_chain.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "ag_integrate"))

from ag_integrate.intel_protocol import parse, IntelFrame, IntelObject, ObjectType, build
from ag_integrate.field_frame import CellRef, cell_to_world, LETTERS
from ag_integrate.world_model import WorldModel
from ag_integrate.mission_router import plan_route
from ag_integrate.car_commands import route_command, reset_command, RoutePlan
from ag_integrate.fire_guard import decide_fire


def cell(s):
    return CellRef(LETTERS.index(s[0]), int(s[1:]) - 1)


def obj(cell_s, t):
    x, y = cell_to_world(cell(cell_s))
    return IntelObject(t, x, y, 0.35, 0.35)


def check(cond, msg):
    """显式校验（-O 不受影响）；失败即抛，绝不静默带病通过。"""
    if not cond:
        raise ValueError(msg)


def main():
    # 场景（13 类）：2 雷（非 K 列 PAPER）+ K9 桥洞（K 列 PAPER）+ 1 障碍 + 1 蓝靶
    objs = [
        obj("D11", ObjectType.PAPER),        # 非 K 列 PAPER -> 雷
        obj("B7", ObjectType.PAPER),         # 非 K 列 PAPER -> 雷
        obj("K9", ObjectType.PAPER),         # K 列 PAPER -> 桥洞箭头（可通行）
        obj("F6", ObjectType.OBSTACLE),
        obj("D2", ObjectType.TARGET_BLUE),
    ]
    frame = IntelFrame(timestamp_ms=1000, objects=tuple(objs))
    data = build(frame)

    # 1. 车端解析
    parsed = parse(data)
    check(len(parsed.objects) == 5, f"解析目标数错误: {len(parsed.objects)}")
    print(f"[1/5 UDP解析] OK, {len(parsed.objects)}目标, {len(data)}字节")

    # 2. 世界模型（连喂两帧：默认障碍多帧确认 N=2，单帧幻影不进态势）
    wm = WorldModel()
    wm.update(parsed)
    snap = wm.update(parsed)
    check(len(snap.mines) == 2, f"雷数错误: 期望2, 实得{len(snap.mines)}")
    check(len(snap.passable) == 1, f"桥洞数错误: 期望1, 实得{len(snap.passable)}")
    check(len(snap.obstacles) == 1, f"障碍数错误: 期望1, 实得{len(snap.obstacles)}")
    n_blue = len([t for t in snap.targets if t.is_blue])
    check(n_blue == 1, f"蓝靶数错误: 期望1, 实得{n_blue}")
    print(f"[2/5 态势归并] OK, 雷={len(snap.mines)} 桥洞={len(snap.passable)} "
          f"障碍={len(snap.obstacles)} 蓝靶={n_blue}")

    # 3. 路径规划
    route = plan_route(snap, cell("N10"), cell("N11"))
    check(route[0] == "N10" and route[-1] == "N11",
          f"路线起终点错误: {route[0]}..{route[-1]}")
    check("D11" in route and "B7" in route, "雷必经: D11/B7 未全部在路线中")
    check("F6" not in route, "障碍不踩: F6 出现在路线中")
    check("K9" in route, "桥洞必经: K9 未在路线中（passable 是必经点）")
    check("D2" not in route, "靶格不可踩: D2 出现在路线中")
    print(f"[3/5 路径规划] OK, {len(route)}点, 经K9桥洞")

    # 4. 车端命令（朝向 token 已翻译成车端坐标架：本机 'y' -> 车端 'x'）
    reset_cmd = reset_command("N10", "y")
    route_cmd = route_command(RoutePlan(cells=route, final_heading="y"))
    check(reset_cmd == "reset:N10,x", f"reset 命令错误: {reset_cmd}")
    check(route_cmd.endswith(",x"), f"路线缺终点朝向(车端x): ...{route_cmd[-12:]}")
    print(f"[4/5 命令编码] OK, reset={reset_cmd}")

    # 5. 禁火保险
    blue_targets = [t for t in snap.targets if t.is_blue]
    check(bool(blue_targets), "场景里应有蓝靶，实得0")
    decision = decide_fire(snap, blue_targets[0].cell.name)
    # 靶格本身不是停靠格（正交邻格停靠开火），应拒绝
    check(decision.allowed is False, "靶格本身不是停靠格，应拒绝开火")
    print(f"[5/5 禁火保险] OK, 靶格本身拒绝开火")

    # 是 GBK 控制台编码不了的字符，直接用 ASCII（Windows 车端/笔记本默认 GBK）
    print("\n[OK] 车端全链路烟雾测试通过")


if __name__ == "__main__":
    main()
