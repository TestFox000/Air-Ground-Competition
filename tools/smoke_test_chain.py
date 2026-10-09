"""车端全链路烟雾测试：UDP帧→解析→态势→路径→命令→禁火。

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


def main():
    # 场景：2雷 + 1桥洞 + 1障碍 + 1蓝靶
    objs = [
        obj("D11", ObjectType.MINE),
        obj("B7", ObjectType.MINE),
        obj("K9", ObjectType.ARROW),
        obj("F6", ObjectType.OBSTACLE),
        obj("D2", ObjectType.TARGET_BLUE),
    ]
    frame = IntelFrame(timestamp_ms=1000, objects=tuple(objs))
    data = build(frame)

    # 1. 车端解析
    parsed = parse(data)
    assert len(parsed.objects) == 5, f"解析目标数错误: {len(parsed.objects)}"
    print(f"[1/5 UDP解析] OK, {len(parsed.objects)}目标, {len(data)}字节")

    # 2. 世界模型
    wm = WorldModel()
    snap = wm.update(parsed)
    assert len(snap.mines) == 2
    assert len(snap.passable) == 1
    print(f"[2/5 态势归并] OK, 雷={len(snap.mines)} 蓝靶={len([t for t in snap.targets if t.is_blue])}")

    # 3. 路径规划
    route = plan_route(snap, cell("N10"), cell("N11"))
    assert route[0] == "N10" and route[-1] == "N11"
    assert "D11" in route and "B7" in route  # 雷必经
    assert "F6" not in route  # 障碍不踩
    print(f"[3/5 路径规划] OK, {len(route)}点, 经L9桥洞")

    # 4. 车端命令
    reset_cmd = reset_command("N10", "y")
    route_cmd = route_command(RoutePlan(cells=route, final_heading="y"))
    assert reset_cmd == "reset:N10,y"
    assert route_cmd.endswith(",y")
    print(f"[4/5 命令编码] OK, reset={reset_cmd}")

    # 5. 禁火保险
    blue_targets = [t for t in snap.targets if t.is_blue]
    if blue_targets:
        decision = decide_fire(snap, blue_targets[0].cell.name)
        # 靶格本身不是停靠格，应拒绝
        assert decision.allowed is False
        print(f"[5/5 禁火保险] OK, 靶格本身拒绝开火")

    print("\n✅ 车端全链路烟雾测试通过")


if __name__ == "__main__":
    main()
