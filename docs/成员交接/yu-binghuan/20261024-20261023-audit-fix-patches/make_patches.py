# -*- coding: utf-8 -*-
"""生成 D1/D4/C2 三份审查修复补丁（只出 patch，不部署）。

D1/D4：基于"当前无人机部署态"（work/ 今晨靶向对齐后、D1/D4 前）的最小 diff；
C2  ：mjpg 13 类链 MapLocalizer.cpp 内参纵横比安全化。
"""
import difflib
import pathlib
from datetime import date

BASE = pathlib.Path(__file__).resolve().parent
WORK = BASE / "work"
PATCHES = BASE / "patches"
PATCHES.mkdir(exist_ok=True)

TODAY = date.today().isoformat()

# ---------------------------------------------------------------- D1
mr_now = (WORK / "ag_integrate" / "mission_router.py").read_text(encoding="utf-8")
D1_NEW = """        if t.track is not None:
            if t.blue_face not in _FACE_OPP:
                # 2026-10-23 审查#2：track 有值而 blue_face 缺/错时，旧代码在
                # 下面抛裸 KeyError、被上层 except 吞成无信息日志、整局无路线。
                # 显式报错点名靶格与非法值——现场一眼知道该改哪个配置。
                raise ValueError(
                    f"移动靶 {t.cell.name} 带轨道但蓝面朝向非法: {t.blue_face!r}"
                    f"（须 S/N/E/W，检查 target_blue_face 配置）")
            cands = _shoot_candidates_moving(t, blocked)
            faces = {s.name: _FACE_OPP[t.blue_face] for s in cands}"""
D1_OLD = """        if t.track is not None:
            cands = _shoot_candidates_moving(t, blocked)
            faces = {s.name: _FACE_OPP[t.blue_face] for s in cands}"""
assert D1_NEW in mr_now, "D1 hunk 不在当前 work 状态中"
mr_before = mr_now.replace(D1_NEW, D1_OLD)

# ---------------------------------------------------------------- D4
cc_now = (WORK / "ag_integrate" / "car_commands.py").read_text(encoding="utf-8")
D4_NEW = """    # 2026-10-23 审查#3：射击格只标首次出现。路线回环重访同一格时再标 '*'
    # 会让执行器二次停车开火（10 发弹量下纯浪费；契约＝顺路再过不重复开火）。
    starred: set[str] = set()
    tokens: list[str] = []
    for c in ordered:
        if c in shoot and c not in starred:
            tokens.append(f"{c}*")
            starred.add(c)
        else:
            tokens.append(c)
    if plan.final_heading:"""
D4_OLD = """    tokens = [f"{c}*" if c in shoot else c for c in ordered]
    if plan.final_heading:"""
assert D4_NEW in cc_now, "D4 hunk 不在当前 work 状态中"
cc_before = cc_now.replace(D4_NEW, D4_OLD)


def write_patch(name, relpath, before, after, subject):
    diff = difflib.unified_diff(
        before.splitlines(keepends=True), after.splitlines(keepends=True),
        fromfile=f"a/{relpath}", tofile=f"b/{relpath}", n=3)
    body = "".join(diff)
    header = (f"# {subject}\n"
              f"# 日期: {TODAY}  审查复核: docs/成员交接/yu-binghuan/"
              f"20261023-20261023-bug-audit-verification\n"
              f"# 底本: 无人机 ~/dzuav_deploy/air_ground @ 310558e（靶向对齐后）\n"
              f"# 验证: tests/test_audit_fixes.py（本地 11/11 过，全量 109 过/"
              f"15 既有失败零新增）\n"
              f"# 应用: cd ~/dzuav_deploy/air_ground && git apply <patch> 或人工核对\n\n")
    (PATCHES / name).write_text(header + body, encoding="utf-8")
    print(f"OK {name}  ({len(body.splitlines())} diff 行)")

write_patch("D1-blueface-validation.patch",
            "src/ag_integrate/ag_integrate/mission_router.py",
            mr_before, mr_now,
            "D1 移动靶 blue_face 显式校验（审查#2：裸 KeyError→可行动报错）")
write_patch("D4-shoot-star-once.patch",
            "src/ag_integrate/ag_integrate/car_commands.py",
            cc_before, cc_now,
            "D4 射击格只标首次出现（审查#3：回环重访不重复开火）")

# ---------------------------------------------------------------- C2
C2_SRC = pathlib.Path(
    r"c:\Users\29212\Documents\Trae-CN\default\20261019-机载自主规划\src\MapLocalizer.cpp")
c2_before = C2_SRC.read_text(encoding="utf-8")
OLD_FN = """namespace
{
// 按帧分辨率缩放基准内参（基准 1920x1080）。
cv::Mat scaledIntrinsics(const cv::Mat& base, cv::Size sz)
{
    cv::Mat K = base.clone();
    K.row(0) *= double(sz.width) / 1920;
    K.row(1) *= double(sz.height) / 1080;
    K.at<double>(2, 2) = 1;
    return K;
}
}  // namespace"""
NEW_FN = """namespace
{
// 按帧分辨率缩放基准内参（基准 1920x1080）。
// 2026-10-23 审查#1（blocker 级病灶）：旧版按轴独立缩放——纵横比与基准
// 一致（16:9）时无恙；一旦不是，fx/fy 缩放系数不同 = 像素非方形 = PnP
// 内参系统性失真、定位恒废退低精度单应兜底。修正：纵横比改变时 fx/fy
// 同比缩放（保持方形像素），主点按新画幅中心重定位。16:9 时与旧公式
// 逐位等价（W/2+(cx-960)*s == cx*sx 当 sx==sy==s）。
// 注：同比系数取 sy（纵向）——横向裁切与纵向裁切无法从分辨率分辨，
// 统一取纵向并在此注明；真非 16:9 相机应重新标定而非依赖缩放。
cv::Mat scaledIntrinsics(const cv::Mat& base, cv::Size sz)
{
    cv::Mat K = base.clone();
    const double sx = double(sz.width) / 1920;
    const double sy = double(sz.height) / 1080;
    if (std::abs(sx - sy) > 1e-9)
    {
        const double s = sy;
        K.at<double>(0, 0) = K.at<double>(0, 0) * s;   // fx
        K.at<double>(1, 1) = K.at<double>(1, 1) * s;   // fy
        K.at<double>(0, 2) = sz.width / 2.0 + (K.at<double>(0, 2) - 960.0) * s;
        K.at<double>(1, 2) = sz.height / 2.0 + (K.at<double>(1, 2) - 540.0) * s;
    }
    else
    {
        K.row(0) *= sx;
        K.row(1) *= sy;
    }
    K.at<double>(2, 2) = 1;
    return K;
}
}  // namespace"""
assert OLD_FN in c2_before, "C2 原函数文本不匹配"
c2_after = c2_before.replace(OLD_FN, NEW_FN)

diff = difflib.unified_diff(
    c2_before.splitlines(keepends=True), c2_after.splitlines(keepends=True),
    fromfile="a/onboard/dzuav_mjpg/src/MapLocalizer.cpp",
    tofile="b/onboard/dzuav_mjpg/src/MapLocalizer.cpp", n=3)
body = "".join(diff)
header = (f"# C2 内参纵横比安全化（审查#1 blocker 级病灶）\n"
          f"# 日期: {TODAY}  审查复核: 同 D1/D4\n"
          f"# 适用文件（同款函数三处，逐一应用）:\n"
          f"#   无人机 13 类链  onboard/dzuav_mjpg/src/MapLocalizer.cpp（本 diff 路径）\n"
          f"#   无人机 7 类链   ~/dzuav/src/MapLocalizer.cpp\n"
          f"#   仓库 main       onboard/dzuav/src/MapLocalizer.cpp\n"
          f"# ⚠️ C++ 改动：合入前必须过机上编译验证（链路恢复后执行）——"
          f"硬约束：未过编译验证不得部署机载硬件\n"
          f"# 16:9（现链 720p/1080p）行为逐位不变；仅非 16:9 生效\n\n")
(PATCHES / "C2-intrinsics-aspect-safe.patch").write_text(
    header + body, encoding="utf-8")
print(f"OK C2-intrinsics-aspect-safe.patch  ({len(body.splitlines())} diff 行)")
