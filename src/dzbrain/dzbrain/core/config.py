# -*- coding: utf-8 -*-
"""配置中心（新版）—— 一元化读取 + **显式默认值** + 启动期校验。

吸取旧代码的教训（readme【十一】第 34/35 条）：
- 旧 `path_executor._load_config_center()` 读失败会**静默回退内置值**，参数改了不生效却毫无提示；
- 键名靠 `globals()[k.upper()]` 批量注入，**没登记的键永远读不到**。

本模块的对策：
1. 所有键集中在 `SCHEMA`，带类型/范围/单位/说明；未登记的键**记录告警**而不是默默忽略。
2. 缺键 → 用 SCHEMA 里的显式默认值，并在 `issues` 里留一条 `default-used` 记录（可查）。
3. 关键键（critical=True）缺失 → **报错**（`require()` 抛异常），预检门禁会拦住发车。
4. 兼容旧结构：`path_executor.ros__parameters.*`、`/dzactuator.ros__parameters.*`（旧 YAML 顶层带斜杠）
   一律**展平**成 `path_executor.linear_speed` 这样的点号键，新旧代码读同一份 YAML。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Iterable

try:  # 车上与本地都有 PyYAML
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

DEFAULT_CONFIG_PATH = os.path.expanduser("~/dzrobot/config/robot.yaml")


@dataclass(frozen=True)
class Spec:
    default: Any
    kind: type | tuple[type, ...]
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    desc: str = ""
    critical: bool = False

    def check(self, value: Any) -> str | None:
        """返回错误说明，None 表示通过。"""
        if isinstance(value, bool) and self.kind is not bool:
            return f"类型不符：期望 {self.kind}，收到 bool"
        if not isinstance(value, self.kind):
            return f"类型不符：期望 {self.kind}，收到 {type(value).__name__}"
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if self.lo is not None and value < self.lo:
                return f"低于下限 {self.lo}{self.unit}"
            if self.hi is not None and value > self.hi:
                return f"高于上限 {self.hi}{self.unit}"
        return None


_F = float
_I = int
_S = str
_B = bool

# --------------------------------------------------------------------------- #
#  新版 SCHEMA（P0 范围；P1/P2/P3 继续追加）
# --------------------------------------------------------------------------- #
SCHEMA: dict[str, Spec] = {
    # ---------------- 循迹几何与限幅（旧代码里全是硬编码，现在全部可配） ----
    "pursuit.wheelbase_m": Spec(0.157, _F, "m", 0.05, 0.60,
                                "轴距（自行车模型）。旧代码硬编码 0.157"),
    "pursuit.max_steer_deg": Spec(30.0, _F, "°", 5.0, 45.0,
                                  "舵角硬限幅。底盘侧再 clamp 到 [25,95]（含 +60 偏置）"),
    "pursuit.max_steer_rate_deg_s": Spec(120.0, _F, "°/s", 10.0, 720.0,
                                         "★新增：舵角速率限幅，防一帧跳 60°（旧代码无此限制）"),
    "pursuit.curvature_gain": Spec(1.0, _F, "", 0.2, 2.0,
                                   "曲率系数：1.0 = 教科书式 δ=atan(2L·sinα/ld)；0.5 = 旧行为等价"),
    "pursuit.lookahead_min_m": Spec(0.18, _F, "m", 0.05, 1.0, "最小前视距离"),
    "pursuit.lookahead_max_m": Spec(0.45, _F, "m", 0.10, 2.0, "最大前视距离"),
    "pursuit.lookahead_k_v": Spec(1.2, _F, "s", 0.0, 5.0, "前视 = k_v × 速度"),
    "pursuit.lat_acc_max_mps2": Spec(0.35, _F, "m/s²", 0.05, 3.0,
                                     "横向加速度上限（曲率限速 v=sqrt(a/|κ|)）"),
    "pursuit.min_speed_mps": Spec(0.08, _F, "m/s", 0.01, 0.30, "最低前进速度（蠕动）"),
    "pursuit.cruise_speed_mps": Spec(0.15, _F, "m/s", 0.02, 0.60, "巡航速度"),
    # ---------------- ★PIVOT 原地转向（并入 10-05 核心的已验证修复）----------------
    #  为什么需要：纯追踪在**大角度换向**（现场图 F11 那个 180° 折返）时会画一条大弧，
    #  仿真实测该处横向偏差 0.37~0.41 m，而那里正是**压雷段**（E11/F8 是雷）——
    #  偏差 0.4 m 就可能漏压雷。9 组参数扫描证明这是几何结构性的（最好只降 0.7%）。
    #  10-05 核心的做法：接近顶点时**停下来 + 低速蠕动转向**，直到航向对上再放行。
    "pursuit.pivot_enabled": Spec(True, _B, "", None, None, "是否启用 PIVOT 原地转向"),
    "pursuit.pivot_angle_deg": Spec(135.0, _F, "°", 60.0, 180.0,
                                    "换向角超过此值才 PIVOT（90° 直角弯仍走纯追踪）"),
    "pursuit.pivot_dist_m": Spec(0.25, _F, "m", 0.05, 1.0, "距顶点多近开始 PIVOT"),
    #  ★G1 速度自适应触发距离（史实：0.30 巡航曾致 PIVOT 到达判定失控，每拍位移 6 mm、
    #    固定 0.25 m 窗口太紧）。eff = clamp(dist + lead×(v−v_ref), [dist, dist_max])：
    #    - 0.15 标定速度下 lead×(0.15−0.15)=0 ⇒ 行为逐位不变（不干扰标定）；
    #    - 0.30 巡航 ⇒ 0.25+1.0×0.15=0.40 → 钳到 0.32；触发更早、刹车余量更大；
    #    - 硬上限保护：PIVOT 顶点常是压雷点，触发点离格心太远会漏压雷（t8 新图实测
    #      盖雷对 pivot 行为敏感），故钳 pivot_dist_max_m。
    "pursuit.pivot_speed_lead_s": Spec(1.0, _F, "s", 0.3, 3.0,
                                        "PIVOT 触发距离随巡航速度加长的系数（秒）"),
    "pursuit.pivot_speed_ref_mps": Spec(0.15, _F, "m/s", 0.05, 0.40,
                                        "低于此速度不加长触发距离（0.15 标定行为逐位不变）"),
    "pursuit.pivot_dist_max_m": Spec(0.32, _F, "m", 0.25, 0.60,
                                     "PIVOT 触发距离硬上限（压雷折返点保护）"),
    "pursuit.pivot_wait_s": Spec(0.5, _F, "s", 0.0, 5.0, "PIVOT 前先刹车等待（让底盘停稳）"),
    "pursuit.pivot_speed_mps": Spec(0.08, _F, "m/s", 0.02, 0.30, "PIVOT 转向时的蠕动速度"),
    "pursuit.pivot_yaw_tol_deg": Spec(15.0, _F, "°", 3.0, 45.0, "航向对上多少度算完成"),
    #  ★超时值不是随手填的，必须覆盖**物理转弯时间**（否则触发即超时 = 这功能等于没实现）：
    #    偏航率上限 = v/L·tan(舵角) = 0.08/0.157·tan(30°) = 0.294 rad/s = 16.9°/s
    #    ⇒ 135°（触发阈值）需 8.01s、180°（现场 F11 折返）需 10.68s，再加 0.5s 刹车等待。
    #    原来的 8.0s **比触发阈值的 8.01s 还短** —— 实闭环 bench 实测：E11 处 PIVOT 触发 →
    #    8.0s 超时 → 半路放行 → 车在顶点来回摆、13/34 点后停滞。14.0s 留约 25% 余量。
    #    （test_pivot.py 第 5 组按这几个参数现算一遍并卡住，改参数改错立刻红。）
    "pursuit.pivot_timeout_s": Spec(14.0, _F, "s", 1.0, 30.0,
                                    "★PIVOT 硬超时（旧版**没有**超时，是本项目标注的缺口，"
                                    "必须兜底；且必须 > 物理转弯时间，见上）"),
    "pursuit.pivot_kp": Spec(1.0, _F, "°/°", 0.2, 5.0, "PIVOT 航向→舵角增益（舵角=误差×kp）"),
    #  ★行进方向裁决的门限：TrackerTask 用**实际位移**判断"车正在往哪边走"（用于裁决
    #    折返压线的重叠航段）。位移小于此值时方向是噪声（车没动/只有定位抖动）→
    #    锁存上一次可信方向，**绝不**退回"上拍命令的符号"（那是自指环，见 tracker.py）。
    #    取值依据：50 Hz 下 0.08 m/s 蠕动每拍 1.6 mm，定位 20 Hz 下每次更新 4 mm；
    #    1 mm 既能滤掉抖动，又不会漏掉真实的蠕动方向。
    "pursuit.motion_heading_eps_m": Spec(0.001, _F, "m", 0.0002, 0.05,
                                         "★行进方向裁决的最小位移（小于它=方向不可信，锁存上次）"),
    #  ★单拍内投影点允许向后跳的最大距离（防"折返压线时 s 在两条共线段之间倒带"）。
    #    现场 F10→F11→E11→R:F11→F10 与 B6→A6→R:B6 都是折返压线：去程段与回程段共线，
    #    两条候选的 s 相差整整一段（0.0 vs 0.70），位置完全裁决不了。限住"一拍最多倒退
    #    15 cm"即可，正常运动每拍只走 2.4 mm（0.12 m/s × 0.02 s）。
    "pursuit.projection_rewind_m": Spec(0.15, _F, "m", 0.0, 0.60,
                                        "★投影单拍最大回退量（防共线折返段之间来回倒带）"),
    "pursuit.speed_quant_comp": Spec(1.0, _F, "×", 0.8, 1.3,
                                     "底盘速度量化补偿：旧侧 |v·23/0.33| 截断丢约 4.3%，可填 1.043"),
    "pursuit.reverse_speed_ratio": Spec(0.6, _F, "×", 0.1, 1.0, "倒车速度比例"),
    #  ★倒车段入口的"推一把"距离：车贴到顶点这么近、且下一段是倒车段时，
    #    直接给一个小倒车速度。没有它就会"鸡生蛋"卡死（换段靠实测位移，
    #    而实测位移靠先发 v<0，而 v<0 靠先换段）—— bench 实测 s 冻在顶点不动。
    "pursuit.reverse_entry_m": Spec(0.06, _F, "m", 0.0, 0.30,
                                    "★倒车段入口触发距离（贴到顶点这么近就开始倒车）"),
    #  ★弧长地板：投影点贴到顶点这么近，就把"该顶点已走过"记进地板，之后不再允许投影退回
    #    到它之前（与仿真 `_project_with_floor` 同语义）。防的是**倒车段↔正向段交界处**的
    #    自指环振荡：探针实测车在 F11 处 30s 内 715 次命令符号翻转、位置冻在毫米级抖动。
    "pursuit.vertex_floor_m": Spec(0.05, _F, "m", 0.0, 0.30,
                                   "★贴到顶点多近算「已走过」（弧长地板，防交界处来回振荡）"),
    #  ★投影"跟丢"出口：窗口/地板把真实最近点排除掉之后，窗口内的点会离线越来越远，
    #    而 hint 每拍跟随上一拍的 s ⇒ 错误的 s 变成自洽不动点，车走多远都拉不回来。
    #    bench 实测：tracker 的 s 冻在 7.70 而车实际在 16.30（差 8.6 m）→ 车追幻影目标
    #    开出路线（最大横向偏差 4.101 m）。超过这个距离就丢掉 hint 与地板、全路线重投影。
    #    取值：正常跟踪误差 ≤0.2 m，取 0.30 m 留余量、又远小于"跟丢"时的 0.45+ m。
    "pursuit.projection_release_m": Spec(0.30, _F, "m", 0.10, 1.00,
                                         "★窗口内最近点离线超过它 → 判定跟丢，全路线重投影"),
    "pursuit.reverse_steer_sign": Spec(1.0, _F, "", -1.0, 1.0,
                                       "倒车舵向系数（旧注释：无实车依据，偏了改 -1）"),
    "pursuit.arrival_m": Spec(0.12, _F, "m", 0.03, 0.35, "到位判定半径"),
    "pursuit.approach_m": Spec(0.60, _F, "m", 0.10, 2.0, "终点/停站前接近段长度"),
    "pursuit.stop_radius_m": Spec(0.17, _F, "m", 0.05, 0.35, "停站触发半径"),
    "pursuit.overshoot_tol_m": Spec(0.30, _F, "m", 0.05, 0.60, "开过头补触发容差"),
    "pursuit.vertex_clamp_m": Spec(0.04, _F, "m", 0.0, 0.20,
                                   "距顶点多近时把前视点钳到顶点（防切角）"),
    "pursuit.lookahead_floor_m": Spec(0.10, _F, "m", 0.02, 0.50,
                                      "前视分母下限（防目标贴脸增益爆炸）"),
    "pursuit.corner_angle_deg": Spec(45.0, _F, "°", 10.0, 120.0, "转角减速/原地转向判据"),
    "pursuit.corner_slow_dist_m": Spec(0.45, _F, "m", 0.05, 1.5, "转角减速距离"),
    "pursuit.corner_speed_mps": Spec(0.12, _F, "m/s", 0.02, 0.40, "转角通过速度"),
    "pursuit.lookahead_corner_angle_deg": Spec(20.0, _F, "°", 5.0, 90.0,
                                               "前视点钳制的转角阈值（旧 20°）"),
    "pursuit.stop_approach_speed_gain": Spec(0.30, _F, "1/s", 0.05, 2.0,
                                             "停站减速：v = min(v, max(min_speed, gain×d))"),
    # 注：旧代码对"终点减速(0.35)"与"射击点减速(0.30)"用了两个键；两者数值几乎相同，
    #     新版统一为上面一个键（避免死键）。若将来确需区分，再加 `pursuit.finish_approach_gain`
    #     并在控制器里按"下一个停站点类型"选择增益。
    "pursuit.map_safety_margin_m": Spec(0.60, _F, "m", 0.0, 1.5, "越界急停余量（顶层强制生效）"),
    "pursuit.drift_warn_m": Spec(0.30, _F, "m", 0.05, 1.0, "单段漂移告警阈值"),
    "pursuit.drift_seg_gate_m": Spec(0.18, _F, "m", 0.02, 1.0,
                                     "★漂移判定独立于吸附：能吸附的偏差(<arrival_m)永远低于告警阈值，"
                                     "所以漂移用弧长窗口判据，不能用横向距离当门槛"),

    # ---------------- 桥洞保护（配置驱动，不硬编码 K 列） ----
    "pursuit.bridge_blocks": Spec(None, list, "", None, None,
                                  "整列保护点（现场为 K1~K11）；缺失则不做桥洞保护，必须显式配置"),
    "pursuit.align_straight_before": Spec(2.0, _F, "格", 0.5, 5.0, "洞前多少格开始锁航向"),
    "pursuit.align_straight_after": Spec(1.0, _F, "格", 0.5, 5.0, "过洞后多少格解除"),
    "pursuit.align_straight_speed_mps": Spec(0.10, _F, "m/s", 0.02, 0.30, "直行段速度"),
    "pursuit.align_kp": Spec(12.0, _F, "", 1.0, 40.0, "航向保持增益（度误差 → 度舵角 /10）"),
    "pursuit.align_steer_limit_deg": Spec(8.0, _F, "°", 1.0, 20.0, "直行段舵角限幅"),

    # ---------------- ★物理几何（t22：用户 2026-10-11 现场口径）----------------
    #  为什么单独一组键：车体/障碍/雷垫片/挡板的**尺寸**以前散落在各模块的硬编码里
    #  （`route_check` 的 BARRIER_MIN_MARGIN=0.10 格带口径、雷覆盖用"到格心 0.21 m"），
    #  现场一量尺寸就得改多处。现在全部收口到这里，公式放 `route/physics.py`。
    "vehicle.length_m": Spec(0.28, _F, "m", 0.10, 1.00,
                             "车体长（沿车头方向；现场实测 0.28）"),
    "vehicle.width_m": Spec(0.135, _F, "m", 0.05, 0.60,
                            "车体宽（现场实测 0.135；扫掠/挡板判据都用它的一半）"),
    "vehicle.coverage_radius_m": Spec(0.035, _F, "m", 0.005, 0.20,
                                      "★车心假设圆半径（压雷判据：车心到垫片 ≤ 此值算压到）"),
    "field.obstacle_radius_m": Spec(0.1275, _F, "m", 0.01, 0.50,
                                    "障碍圆柱半径（柱心在障碍格格心）"),
    "field.mine_pad_m": Spec(0.12, _F, "m", 0.02, 0.35,
                             "雷垫片边长（0.12×0.12 正方形，中心在雷格格心）"),
    "field.barrier_width_m": Spec(0.02, _F, "m", 0.002, 0.20,
                                  "挡板厚度（第 6/7 行之间的细条；半厚 0.01 进触线判据）"),
    "field.barrier_cols": Spec("D..N", _S, "", None, None,
                               "挡板覆盖的列（含两端；现场 D..N = 11 列 = 3.85 m）"),
    "planner.obstacle_margin_m": Spec(0.02, _F, "m", 0.0, 0.30,
                                      "障碍净空余量（规划/校验共用；判据 = r + 车半宽 + 此值）"),

    # ---------------- 路线与任务 ----
    "route.tokens": Spec(None, list, "", None, None, "mission_blocks（格名 token 列表）", True),
    "route.cross_lower_candidates": Spec([f"K{r}" for r in range(7, 12)], list, "",
                                         None, None, "下半场穿洞候选格（U 形赛道第一处）"),
    "route.cross_upper_candidates": Spec([f"K{r}" for r in range(1, 7)], list, "",
                                         None, None, "上半场穿洞候选格（U 形赛道第二处）"),
    "route.cross_lower_default": Spec("K9", _S, "", None, None, "未收到无人机情报时的兜底穿洞格"),
    "route.cross_upper_default": Spec("K4", _S, "", None, None, "未收到无人机情报时的兜底穿洞格"),
    "mission.auto_start": Spec(True, _B, "", None, None, "起栈后自动出发"),
    "mission.auto_start_delay_s": Spec(10.0, _F, "s", 0.0, 120.0, "出发前初始化等待"),
    "mission.reset": Spec("N10,y", _S, "", None, None, "起步复位（格,航向）"),
    "mission.scan_mode": Spec("continuous", _S, "", None, None,
                              "continuous 边走边扫 / waypoint 到点扫描 / none"),
    "mission.bridge_info_wait_s": Spec(20.0, _F, "s", 0.0, 120.0,
                                       "等待无人机桥洞情报的最长时间，超时用兜底格"),
    "mission.shots_per_stop": Spec(3, _I, "发", 1, 10, "每个射击停站点要打几发（旧 shoot_count_target）"),
    "mission.shoot_timeout_s": Spec(30.0, _F, "s", 1.0, 120.0, "单个停站点硬超时（绝不卡死）"),
    "mission.lock_timeout_s": Spec(18.0, _F, "s", 1.0, 120.0, "等云台锁定的最长时间"),
    "mission.settle_time_s": Spec(0.8, _F, "s", 0.0, 10.0, "底盘停稳判定时长"),
    "mission.settle_eps_m": Spec(0.02, _F, "m", 0.001, 0.20, "停稳判定的位置变化容差"),
    "mission.settle_speed_mps": Spec(0.02, _F, "m/s", 0.001, 0.20, "停稳判定的速度容差"),
    "mission.scan_stop_timeout_s": Spec(8.0, _F, "s", 0.5, 60.0,
                                        "纯扫描停站点（:SCAN_STOP）的等待上限"),
    "mission.stable_timeout_s": Spec(6.0, _F, "s", 0.5, 30.0, "等瞄准稳定的最长时间（超时降级放行）"),
    "mission.stable_offset_px": Spec(40.0, _F, "px", 5.0, 200.0, "瞄准稳定容差"),
    "mission.candidate_retry": Spec(3, _I, "个", 1, 10, "单个靶最多尝试几个候选射击点"),
    "mission.finish_voice_id": Spec(0, _I, "", 0, 255, "完成提示音 ID（0 = 不播）"),
    # ---- 拐角绕圈兜底（2026-10-07；仿真实测噪声 0.01m/拍会让车在拐角原地转圈）----
    "mission.stall_timeout_s": Spec(12.0, _F, "s", 2.0, 120.0,
                                    "★航点推进停滞判定：投影弧长 s 超过这么久不前进就判'不可达'，"
                                    "强制跳过该点（宁可漏一个点，不可卡死——readme【八】27）"),
    "mission.stall_yaw_deg": Spec(360.0, _F, "°", 90.0, 1440.0,
                                  "★绕圈判定的累计航向转动阈值：停滞期间车累计转满这么多度 = 原地绕圈"),

    # ---------------- 瞄准与打击（P2；键先就位，读旧 dzrknn 段） ----
    "aim.scan_request_on_start": Spec(2, _I, "", 0, 2,
                                      "任务开始发出的扫描请求：2=持续（readme【十】：绝不在到点发 1 降级）"),
    "aim.lead_time_s": Spec(0.35, _F, "s", 0.0, 2.0,
                            "★提前量补偿时间（视觉流水线 0.26s + 伺服滞后）；旧版只有 0.5 帧≈16ms"),
    "aim.fire_policy": Spec("moving_lead", _S, "", None, None,
                            "moving_lead（本靶）/ sweep / turnaround（无停顿靶不适用）/ static"),
    "aim.rail_span_m": Spec(0.70, _F, "m", 0.05, 3.0, "靶轨道行程（G11+H11 = 2 格 = 0.70 m）"),
    "aim.target_speed_mps": Spec(0.04, _F, "m/s", 0.001, 1.0, "靶标移速（现场实测 4 cm/s）"),
    "aim.max_shots_total": Spec(10, _I, "发", 1, 10, "全场弹药上限"),
    "aim.max_shots_per_target": Spec(3, _I, "发", 1, 10, "单靶发数上限"),
    "aim.shot_cooldown_ms": Spec(500, _I, "ms", 100, 5000, "两发最小间隔"),
    "aim.fire_box_far_px": Spec(14.0, _F, "px", 2.0, 100.0, "远处开火框（须 > 云台机械精度约 12px）"),
    "aim.fire_box_near_px": Spec(26.0, _F, "px", 2.0, 200.0, "近处开火框"),
    "aim.lost_frames_to_rescan": Spec(180, _I, "帧", 10, 600,
                                      "★丢帧回扫阈值；旧版 180 帧≈6s 会让云台在丢失期间冻结"),
    "aim.scan_pitch_offset_deg": Spec(6.0, _F, "°", -60.0, 30.0,
                                      "扫描俯仰基准（★旧注释与实测值长期冲突，现场必须核对）"),
    # ---- 移动靶估计器（RailEstimator，2026-10-07 从 α-β 拆出的长基线端点差分）----
    # ⚠️ 本段默认值/范围必须与 `tasks/aim.py` 的 `g(key, default)` **实现内置值一致**
    #   （那是运行期真正生效的值；SCHEMA 只是给 validate()/未知键告警用的登记）。
    "aim.box_w_far_px": Spec(30.0, _F, "px", 5.0, 200.0, "远处靶框宽标定（框宽→距离映射端点）"),
    "aim.box_w_near_px": Spec(120.0, _F, "px", 10.0, 400.0, "近处靶框宽标定"),
    "aim.cls_switch_confirm_frames": Spec(5, _I, "帧", 1, 60, "类别改锁需连续多少帧（抗抖动）"),
    "aim.est_dt_max_s": Spec(0.50, _F, "s", 0.01, 2.0, "估计器两帧时间差上限（超过视为断流；10fps 正常帧≈0.1s）"),
    "aim.est_dt_min_s": Spec(0.002, _F, "s", 0.0, 0.1, "估计器两帧时间差下限（防 dt=0 爆增益）"),
    "aim.est_flip_min_speed_px_s": Spec(2.0, _F, "px/s", 0.5, 20.0,
                                        "★折返判定：|v| 低于此值不算折返。必须 < 真实靶速(4cm/s≈5.6px/s)，"
                                        "否则折返闭锁被整个关掉（旧误写 8.0 就是这种事故）"),
    "aim.est_gate_base_px": Spec(3.0, _F, "px", 1.0, 20.0, "跳变门基础值"),
    "aim.est_gate_k": Spec(3.0, _F, "s", 0.0, 20.0, "跳变门速度系数"),
    "aim.est_gate_vel_scale": Spec(0.10, _F, "×", 0.0, 2.0, "跳变门随速度缩放"),
    "aim.est_jump_floor_px": Spec(25.0, _F, "px", 5.0, 100.0, "跳变门下限（gate_base 与 jump_floor 别写反）"),
    "aim.est_max_speed_px_s": Spec(300.0, _F, "px/s", 50.0, 3000.0,
                                   "速度估计上限夹紧（防荒谬值兜底，取 ≥300 且 5×靶速）"),
    "aim.est_vel_window_s": Spec(0.40, _F, "s", 0.05, 2.0, "速度估计长基线窗口（端点差分；随视觉帧率整定）"),
    "aim.est_xi": Spec(3.5, _F, "", 0.5, 10.0,
                       "跟踪指数 ξ（α=2(2ξ−1)/(ξ(ξ+1))，**不是** α 系数本身，勿按 0~1 理解）"),
    "aim.fire_only_blue": Spec(True, _B, "", None, None, "仅攻击蓝靶(cls==1)"),
    "aim.hit_hold_s": Spec(0.5, _F, "s", 0.0, 5.0, "命中后锁存时长"),
    "aim.hold_min_s": Spec(0.30, _F, "s", 0.0, 2.0, "许可迟滞（只对冷却这一条，防电平只亮一拍被漏采）"),
    "aim.lead_eps_px": Spec(0.5, _F, "px", 0.0, 20.0, "提前量收敛容差（sweep 策略用）"),
    "aim.max_lead_factor": Spec(0.45, _F, "×", 0.0, 2.0, "提前量限幅 = 该值 × 轨道行程 × px_per_m"),
    "aim.px_per_m_rail": Spec(140.0, _F, "px/m", 10.0, 1000.0,
                              "★轨道单位换算：0.70m 行程对应多少 px（**现场必须实测标定**）"),
    "aim.rearm_after_lost_s": Spec(1.0, _F, "s", 0.0, 10.0,
                                   "目标丢失多久后允许重新武装（实现取 max(0.3, lost_s/2)）"),
    "aim.rescan_on_lost": Spec(False, _B, "", None, None, "丢靶超阈值时是否发回扫请求"),
    "aim.reset_confirm_frames": Spec(5, _I, "帧", 1, 60, "估计器重置确认帧数（warmup 闭锁拍数）"),
    "aim.vision_nominal_fps": Spec(30.0, _F, "fps", 1.0, 120.0, "视觉名义帧率（丢帧计数换算）"),

    # ---------------- 误差修正（L2：位置） ----
    "fix.anchor_at_waypoint": Spec(True, _B, "", None, None, "到位后把位姿锚回格心"),
    "fix.anchor_max_offset_m": Spec(0.08, _F, "m", 0.01, 0.30,
                                    "★吸附的最大允许偏移：只有信念位姿本来就在格心附近才锚，"
                                    "否则'吸附'会自己制造误差（仿真实测：零误差下锚定把 belief 误差"
                                    "从 0.000 抬到 0.119 m）"),
    "fix.anchor_gain": Spec(1.0, _F, "×", 0.0, 1.0,
                            "吸附强度：1.0=完全锚到格心；<1 为部分吸附（保守，防制造误差）"),
    "fix.scale_est_enabled": Spec(True, _B, "", None, None, "系统性比例误差在线估计"),
    "fix.scale_step_max": Spec(0.005, _F, "×/段", 0.0, 0.05, "每段允许的最大比例修正步长"),
    "fix.scale_limit": Spec(0.10, _F, "×", 0.0, 0.5, "比例修正总量夹紧（±）"),
    # ---- FixTask 估计器（2026-10-07；默认值必须与 tasks/fix.py 实现一致）----
    "fix.drift_cull_m": Spec(0.30, _F, "m", 0.0, 1.0,
                             "漂移剔除阈值（默认回落到 pursuit.drift_warn_m）"),
    "fix.seg_min_m": Spec(0.05, _F, "m", 0.0, 0.5, "段长下限（短于此值不出估计）"),
    "fix.scale_min_samples": Spec(3, _I, "段", 1, 20, "最小有效样本数（低于不出估计值）"),
    "fix.ratio_bounds": Spec((0.75, 1.35), tuple, "×", None, None,
                             "单段比值合法区间（里程多报 18% 时上界需 ≥1.18，见 fix 段警告）"),
    "fix.ratio_jump_max": Spec(0.15, _F, "×", 0.0, 1.0, "比值突跳容差（与窗口均值比）"),
    "fix.ratio_jump_min_prior": Spec(3, _I, "段", 1, 20, "突跳判定需要的历史样本数"),
    "fix.scale_window": Spec(5, _I, "段", 1, 50, "比例估计滑动窗口"),
    #  ★ 单拍位移守卫（FixTask._accumulate）：`P_ROUTE_S` 有**三个写入者**
    #    （Localization / Tracker / Mission），各维护自己的投影链；任一处冻结或跳变，
    #    相邻两拍的 `ds` 就会出现物理上不可能的跳（实测 P_ROUTE_S 在 3.73↔4.20 之间翻），
    #    而 `ds>0` 只累加不判别 ⇒ 段比值被推成 0.12~0.23 并**挤掉**正常样本。
    "fix.ds_max_ratio": Spec(2.0, _F, "×", 1.0, 10.0,
                             "★单拍路线弧长增量上限 = 位姿位移 × 该系数（超过即不可能，丢弃并计数）"),
    "fix.ds_floor_m": Spec(0.05, _F, "m", 0.0, 0.30,
                           "★单拍弧长增量的绝对下限（位姿位移很小时用）。0.05 m ≈ 0.5 m/s × 0.1 s "
                           "的物理上限；低于它才允许按位移比例判定"),
    "fix.ds_reject_max_run": Spec(3, _I, "拍", 1, 50,
                                  "★连续多少拍 ds 都不可信就重新基线（那是换了写入者的坐标系，"
                                  "不是噪声；不重置会一直补账、卡死在拒绝里）"),

    # ---------------- 定位投影自愈（LocalizationTask._heal_stuck，2026-10-07） ----
    #  ★ 背景（bench 实证）：`Path.project(allow_backward=False)` 把窗口下界夹在 `s_hint`
    #    上，车的垂足一旦落到窗口之前，"窗口内最近点"就永远是下界点本身 ⇒ 返回的 s
    #    恰好等于上一拍 hint ⇒ 不动点：车继续走，而 P_ROUTE_S 永久停在原地
    #    （实测上报投影点离线 0.5~3.5 m、占 20~37% 拍，段比值被污染成 0.12~0.23）。
    #    以下三条判据**同时**成立才做一次全路线重投影（正常每拍仍是窗口投影）。
    "localization.stuck_ticks": Spec(5, _I, "拍", 2, 50,
                                      "★投影卡死：连续这么多拍 s 前进不足 stuck_eps_m"),
    "localization.stuck_eps_m": Spec(0.01, _F, "m", 0.001, 0.10,
                                      "★卡死判定的 s 前进量上限。最低速 0.08 m/s @20 Hz "
                                      "五拍也前进 20 mm → 正常行驶（含蠕动）不会被误判"),
    "localization.stuck_dist_m": Spec(0.20, _F, "m", 0.05, 1.0,
                                       "★卡死判定的投影点离线距离：上报点离车这么远就说明它"
                                       "已不是车的位置。取 0.20 ≈ 仿真实测跟踪 RMS，"
                                       "既放过正常横向偏差、又不放过 >0.5 m 的冻结"),

    # ---------------- 无人机链路 ----
    "drone.enabled": Spec(True, _B, "", None, None, "是否启用无人机链路"),
    "drone.topic_in": Spec("/dzlink/drone_to_car", _S, "", None, None, "无人机→车 话题"),
    "drone.topic_out": Spec("/dzlink/car_to_drone", _S, "", None, None, "车→无人机 话题"),
    "drone.heartbeat_timeout_s": Spec(3.0, _F, "s", 0.5, 30.0, "心跳丢失判定"),
    "drone.info_stale_s": Spec(30.0, _F, "s", 1.0, 300.0, "情报过期判定"),
    "drone.car_state_period_s": Spec(0.7, _F, "s", 0.1, 10.0, "车状态周期上报间隔"),
    "drone.ack_enabled": Spec(True, _B, "", None, None, "收到需确认消息时回 ack"),
    "drone.field_diff_max": Spec(8, _I, "条", 0, 100, "靶区/雷障核对差异条数上限"),
    "drone.map_path": Spec("", _S, "", None, None,
                           "本地地图路径（空 = 自动定位 <repo>/config/map.yaml）"),

    # ---------------- 安全 ----
    "safety.pose_timeout_s": Spec(1.0, _F, "s", 0.1, 10.0, "定位失联判定（超时即刹车）"),
    "safety.startup_grace_s": Spec(3.0, _F, "s", 0.0, 30.0,
                                   "★启动宽限：区分'还没收到数据（不刹）'与'跑着跑着断流（立刻刹）'，"
                                   "否则冷启动第一拍就会因 P_ODOM_AGE 缺失而永久锁死"),
    "safety.task_stall_s": Spec(1.0, _F, "s", 0.1, 10.0, "任务线程心跳丢失判定"),
    "safety.brake_on_fault": Spec(True, _B, "", None, None, "故障时强制发零速"),

    # ---------------- 日志/遥测 ----
    "logging.rate_limit_s": Spec(1.0, _F, "s", 0.0, 60.0, "同类日志限流间隔"),
    "telemetry.period_s": Spec(0.5, _F, "s", 0.1, 10.0, "遥测快照周期"),
}

#: 旧 YAML 里的段落名 → 新版点号前缀（保证同一份 robot.yaml 新旧代码都能读）
#: ★ `dzbrain` 段映射为**空前缀**：其 ros__parameters 下的 `pursuit:`/`route:`/… 会直接展平成
#:   `pursuit.wheelbase_m` 这样的点号键；而 YAML 那边仍符合 ROS 的 `<节点名>: ros__parameters:`
#:   结构 —— 顶层多一个**裸键**会让 --params-file 解析失败、相机/视觉/云台三节点同时 abort
#:   （readme【项目结构提示】踩过这个坑），所以新参数必须挂在节点段里。
SECTION_PREFIX = {
    "/dzbrain": "",
    "dzbrain": "",
    "path_executor": "path_executor",
    "/dzactuator": "dzactuator",
    "dzactuator": "dzactuator",
    "/dzrknn_dataprocess": "dzrknn_dataprocess",
    "dzrknn_dataprocess": "dzrknn_dataprocess",
    "/det_node": "det_node",
    "det_node": "det_node",
    "/usb_cam_node": "usb_cam_node",
    "usb_cam_node": "usb_cam_node",
    "/dzjudgment": "dzjudgment",
    "dzjudgment": "dzjudgment",
}


@dataclass(frozen=True)
class Issue:
    level: str      # "error" | "warn" | "info"
    key: str
    msg: str

    def __str__(self) -> str:
        mark = {"error": "❌", "warn": "⚠️", "info": "ℹ️"}.get(self.level, "·")
        return f"{mark} {self.key}: {self.msg}"


def _flatten(node: Any, prefix: str = "") -> dict[str, Any]:
    """把嵌套 YAML 展平成点号键；`ros__parameters` 层自动穿透。"""
    out: dict[str, Any] = {}
    if isinstance(node, dict):
        for k, v in node.items():
            key = str(k)
            if key == "ros__parameters":
                out.update(_flatten(v, prefix))
                continue
            out.update(_flatten(v, f"{prefix}{key}."))
    else:
        out[prefix.rstrip(".")] = node
    return out


class ConfigError(RuntimeError):
    pass


class Config:
    """点号键访问的配置对象。`cfg.get("pursuit.wheelbase_m")`。"""

    def __init__(self, flat: dict[str, Any], source: str = "<memory>",
                 raw: dict | None = None) -> None:
        self._flat = dict(flat)
        self.source = source
        self.raw = raw or {}
        self.issues: list[Issue] = []
        self._schema_hits: set[str] = set()

    # ------------------------------ 加载 ------------------------------ #
    @classmethod
    def load(cls, path: str | None = None) -> "Config":
        if yaml is None:
            raise ConfigError("缺少 PyYAML，无法读取配置")
        path = os.path.expanduser(path or os.environ.get("ROBOT_CONFIG") or DEFAULT_CONFIG_PATH)
        if not os.path.isfile(path):
            raise ConfigError(f"配置文件不存在: {path}")
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
        flat: dict[str, Any] = {}
        for top, body in raw.items():
            prefix = SECTION_PREFIX.get(str(top), str(top).lstrip("/"))
            child_prefix = f"{prefix}." if prefix else ""
            flat.update(_flatten(body, child_prefix))
            if isinstance(body, dict) and "ros__parameters" in body:
                flat.update(_flatten(body["ros__parameters"], child_prefix))
        cfg = cls(flat, source=path, raw=raw)
        cfg._migrate_legacy()
        return cfg

    @classmethod
    def from_dict(cls, data: dict[str, Any], source: str = "<dict>") -> "Config":
        flat = _flatten(data)
        cfg = cls(flat, source=source, raw=data)
        cfg._migrate_legacy()
        return cfg

    def _migrate_legacy(self) -> None:
        """把旧键映射到新版键（不删旧键，只补新键，保证双向兼容）。"""
        legacy_map = {
            "route.tokens": ["path_executor.mission_blocks"],
            "mission.auto_start": ["path_executor.auto_start"],
            "mission.auto_start_delay_s": ["path_executor.auto_start_delay"],
            "mission.reset": ["path_executor.mission_reset"],
            "mission.scan_mode": ["path_executor.mission_scan_mode"],
            "pursuit.cruise_speed_mps": ["path_executor.linear_speed"],
            "pursuit.reverse_speed_ratio": ["path_executor.reverse_speed_ratio"],
            "pursuit.reverse_steer_sign": ["path_executor.reverse_steer_sign"],
            "pursuit.map_safety_margin_m": ["path_executor.map_safety_margin"],
            "pursuit.drift_warn_m": ["path_executor.drift_warn_m"],
            "pursuit.bridge_blocks": ["path_executor.bridge_blocks"],
            "pursuit.align_straight_before": ["path_executor.align_straight_before"],
            "pursuit.align_straight_after": ["path_executor.align_straight_after"],
            "pursuit.align_straight_speed_mps": ["path_executor.align_straight_speed"],
            "pursuit.align_kp": ["path_executor.align_kp"],
            "pursuit.align_steer_limit_deg": ["path_executor.align_steer_limit"],
            "pursuit.stop_radius_m": ["pursuit.shoot_stop_radius_m"],
            # ---- 打靶/弹药（旧 path_executor 段与 dzrknn 段都有真源） ----
            "mission.shots_per_stop": ["path_executor.shoot_count_target"],
            "mission.shoot_timeout_s": ["path_executor.shoot_timeout"],
            "mission.lock_timeout_s": ["path_executor.shoot_lock_timeout"],
            "mission.settle_time_s": ["path_executor.shoot_settle_time"],
            "mission.settle_eps_m": ["path_executor.shoot_settle_speed"],
            "mission.stable_timeout_s": ["path_executor.shoot_stable_timeout"],
            "mission.stable_offset_px": ["path_executor.shoot_stable_offset_px"],
            "mission.candidate_retry": ["path_executor.shoot_max_retry"],
            "mission.finish_voice_id": ["path_executor.finish_voice_id"],
            "aim.max_shots_total": ["dzrknn_dataprocess.max_shots"],
            "aim.max_shots_per_target": ["dzrknn_dataprocess.max_shots_per_target"],
            "aim.shot_cooldown_ms": ["dzrknn_dataprocess.shot_cooldown_ms"],
            "aim.fire_box_far_px": ["dzrknn_dataprocess.fire_dz_far_px"],
            "aim.fire_box_near_px": ["dzrknn_dataprocess.fire_dz_near_px"],
            "aim.lost_frames_to_rescan": ["dzrknn_dataprocess.lost_frames_to_rescan"],
            "aim.scan_pitch_offset_deg": ["dzrknn_dataprocess.scan_pitch_offset_deg"],
        }
        for new_key, olds in legacy_map.items():
            if new_key in self._flat:
                continue
            for old in olds:
                if old in self._flat:
                    self._flat[new_key] = self._flat[old]
                    self.issues.append(Issue("info", new_key, f"由旧键 {old} 迁移"))
                    break

    # ------------------------------ 读 ------------------------------ #
    def get(self, key: str, default: Any = None) -> Any:
        if key in self._flat:
            value = self._flat[key]
            self._check_against_schema(key, value)
            return value
        spec = SCHEMA.get(key)
        if spec is not None:
            self._schema_hits.add(key)
            if spec.critical:
                raise ConfigError(f"关键配置缺失: {key}（{spec.desc}）")
            if default is None:
                return spec.default
        return default

    def require(self, key: str) -> Any:
        value = self.get(key, default=None)
        if value is None:
            raise ConfigError(f"缺少必需配置: {key}")
        return value

    def _check_against_schema(self, key: str, value: Any) -> None:
        spec = SCHEMA.get(key)
        if spec is None or key in self._schema_hits:
            return
        self._schema_hits.add(key)
        err = spec.check(value)
        if err:
            self.issues.append(Issue("error", key, err))

    def section(self, prefix: str) -> dict[str, Any]:
        p = prefix.rstrip(".") + "."
        return {k[len(p):]: v for k, v in self._flat.items() if k.startswith(p)}

    def keys(self) -> Iterable[str]:
        return self._flat.keys()

    def override(self, key: str, value: Any) -> "Config":
        """就地覆盖一个键（**只给 bench/测试用**，不允许在车端运行路径上调用）。

        为什么单独开一个方法而不是让调用方改 `_flat`：让"运行期改配置"这件事
        有唯一的、可被 grep 到的入口（旧代码到处 `globals()[...] = ...` 是反面教材）。
        """
        self._flat[key] = value
        return self

    # ------------------------------ 校验 ------------------------------ #
    def validate(self, strict: bool = True) -> list[Issue]:
        """启动期校验：范围/类型/互斥/一致性。返回问题列表（去重）。"""
        def note(level: str, key: str, msg: str) -> None:
            # get() 的即时类型检查与本节的范围检查可能对同一键各记一次 → 以"已存在签名"为准去重
            # （issues 列表很小，每次重算集合比维护增量状态更不容易出错）
            existing = {(i.level, i.key, i.msg) for i in self.issues}
            if (level, key, msg) in existing:
                return
            self.issues.append(Issue(level, key, msg))

        for key, spec in SCHEMA.items():
            try:
                value = self.get(key)
            except ConfigError as exc:
                note("error", key, str(exc))
                continue
            if value is None:
                level = "error" if spec.critical else "warn"
                note(level, key, "未设置且无默认值（新版必须显式配置）")
                continue
            err = spec.check(value)
            if err:
                note("error", key, err)

        # 已知键之外的路径段（打错键名会被发现，而不是静默忽略）
        known_prefixes = ("pursuit.", "route.", "mission.", "fix.", "drone.", "safety.",
                          "logging.", "telemetry.", "aim.", "localization.", "referee.",
                          "path_executor.", "dzactuator.", "dzrknn_dataprocess.", "det_node.",
                          "usb_cam_node.", "dzjudgment.", "ackermann_sim.")
        for key in self._flat:
            if not key.startswith(known_prefixes):
                note("warn", key, "未知配置段（拼写错误？新版本未登记？）")

        # 一致性断言
        try:
            if self.get("pursuit.lookahead_min_m") > self.get("pursuit.lookahead_max_m"):
                note("error", "pursuit.lookahead_min_m", "前视下限大于上限")
            tokens = self.get("route.tokens") or []
            for tok in tokens:
                if not isinstance(tok, str):
                    note("error", "route.tokens", f"token 必须是字符串，收到 {tok!r}")
            low = self.get("route.cross_lower_default")
            if low not in (self.get("route.cross_lower_candidates") or []):
                note("warn", "route.cross_lower_default", f"兜底穿洞格 {low} 不在候选集合里")
            up = self.get("route.cross_upper_default")
            if up not in (self.get("route.cross_upper_candidates") or []):
                note("warn", "route.cross_upper_default", f"兜底穿洞格 {up} 不在候选集合里")
            if self.get("pursuit.align_straight_speed_mps") > self.get("pursuit.cruise_speed_mps"):
                note("warn", "pursuit.align_straight_speed_mps",
                     "桥洞直行速度高于巡航速度（通常应更低）")
        except ConfigError:
            pass

        errors = [i for i in self.issues if i.level == "error"]
        if strict and errors:
            raise ConfigError("配置校验失败：\n" + "\n".join(f"  {i}" for i in errors))
        return self.issues

    def _signatures(self) -> list[tuple[str, str, str]]:
        return [(i.level, i.key, i.msg) for i in self.issues]

    def summary(self) -> str:
        lines = [f"配置来源: {self.source}",
                 f"键数: {len(self._flat)}"]
        for key in ("route.tokens", "pursuit.cruise_speed_mps", "pursuit.lookahead_min_m",
                    "route.cross_lower_default", "route.cross_upper_default"):
            lines.append(f"  {key} = {self.get(key)!r}")
        return "\n".join(lines)


def lookahead_distance(speed: float, cfg: "Config") -> float:
    """前视距离 = clamp(k_v·|v|, [min, max])。"""
    ld = cfg.get("pursuit.lookahead_k_v") * abs(speed)
    lo = cfg.get("pursuit.lookahead_min_m")
    hi = cfg.get("pursuit.lookahead_max_m")
    return max(lo, min(hi, ld))


def curve_speed_limit(curvature: float, cfg: "Config") -> float:
    """曲率限速 v = sqrt(a_lat / |κ|)。"""
    a = cfg.get("pursuit.lat_acc_max_mps2")
    k = abs(curvature)
    if k < 1e-6:
        return float("inf")
    return math.sqrt(a / k)


def pivot_dist_eff(cfg: "Config", speed_mps: float) -> float:
    """PIVOT 触发距离（速度自适应）：`clamp(dist + lead×max(0, v−v_ref), [dist, dist_max])`。

    ★ 为什么：0.30 巡航每拍 6 mm，固定 0.25 m 窗口太紧（G1 史实：0.3 曾致 PIVOT
    到达判定失控）。0.15 标定速度下逐位不变。硬上限保护压雷折返点（见 Spec 注释）。
    """
    base = cfg.get("pursuit.pivot_dist_m")
    lead = cfg.get("pursuit.pivot_speed_lead_s")
    ref = cfg.get("pursuit.pivot_speed_ref_mps")
    cap = cfg.get("pursuit.pivot_dist_max_m")
    eff = base + lead * max(0.0, abs(speed_mps) - ref)
    return max(base, min(cap, eff))
