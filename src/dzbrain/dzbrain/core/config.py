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
    # 速度剖面第 5 层「加减速斜坡」的两键（真源 = `pursuit/controller.py:speed_profile()`）
    #   `default=0.0` 是**硬要求**：缺省即"整层不参与" ⇒ 与旧版逐位一致（回归安全）。
    "pursuit.accel_up_mps2": Spec(0.0, _F, "m/s²", 0.0, 3.0,
                                  "★升速上限（每拍速度增量 ≤ 该值×dt）；0=不启用斜坡"),
    "pursuit.accel_down_mps2": Spec(0.0, _F, "m/s²", 0.0, 3.0,
                                    "★降速上限（每拍速度降幅 ≤ 该值×dt）；0=不启用斜坡"),
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
    #    - 硬上限保护：PIVOT 顶点常是压雷点，触发点离格心太远会漏压雷（新图实测
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
    #    旧值 8.0 s **比触发阈值的 8.01 s 还短** —— 实闭环 bench 实测：PIVOT 触发后
    #    超时半路放行 ⇒ 车在顶点来回摆、随后停滞。14.0 s 留约 25% 余量。
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

    # ---------------- ★物理几何（用户 2026-10-11 现场口径）----------------
    #  为什么单独一组键：车体/障碍/雷垫片/挡板的**尺寸**以前散落在各模块的硬编码里
    #  （`route_check` 的 BARRIER_MIN_MARGIN=0.10 格带口径、雷覆盖用"到格心 0.21 m"），
    #  现场一量尺寸就得改多处。现在全部收口到这里，公式放 `route/physics.py`。
    "vehicle.length_m": Spec(0.28, _F, "m", 0.10, 1.00,
                             "车体长（沿车头方向；现场实测 0.28）"),
    "vehicle.width_m": Spec(0.135, _F, "m", 0.05, 0.60,
                            "车体宽（现场实测 0.135；扫掠/挡板判据都用它的一半）"),
    "vehicle.coverage_radius_m": Spec(0.035, _F, "m", 0.005, 0.20,
                                      "★车心假设圆半径（压雷判据：车心到垫片 ≤ 此值算压到）"),
    # 参考点偏移（现场卷尺实测 2026-10-11）
    #   后轴中心→车尾沿 6.5 cm、后轴中心→车头沿 21.5 cm、总长 28 cm
    #   ⇒ 车体几何中心在参考点（= 里程计/仿真积分点 = 后轴中心）**前方 (21.5−6.5)/2 = +7.5 cm**。
    #   正值 = 车体中心在参考点前方；0.0 = 老假设（中心与参考点重合，**与实测矛盾**）。
    #   消费者：`route/physics.py` 的 `body_corners()/body_*_clearance_m()`（净空唯一真源）、
    #   `tasks/mission.py` 的规划门禁足迹诊断、离线足迹链（t5lib）。
    "vehicle.ref_offset_m": Spec(0.075, _F, "m", -0.30, 0.30,
                                 "参考点(后轴中心)→车体几何中心的纵向偏移（正=中心在参考点前方）"),
    "field.obstacle_radius_m": Spec(0.1275, _F, "m", 0.01, 0.50,
                                    "障碍圆柱半径（柱心在障碍格格心）"),
    "field.mine_pad_m": Spec(0.12, _F, "m", 0.02, 0.35,
                             "雷垫片边长（0.12×0.12 正方形，中心在雷格格心）"),
    "field.barrier_width_m": Spec(0.02, _F, "m", 0.002, 0.20,
                                  "挡板厚度（第 6/7 行之间的细条；半厚 0.01 进触线判据）"),
    "field.barrier_cols": Spec("D..N", _S, "", None, None,
                               "挡板覆盖的列（含两端；现场 D..N = 11 列 = 3.85 m）"),
    # 桥洞**进洞侧**冗余长度（虚拟墙延长段）。
    #   设计规格（用户 2026-10-16）：只加进洞侧，长度 = **1/4 格** = 0.35 × 0.25 = 0.0875 m。
    #   为什么给上限 0.35：超过一整格时"冗余墙"会伸进邻格、与邻格语义打架（1/4 格是设计值，
    #   上限留到一整格只为现场微调；真的需要更长就该改桥洞几何而不是抬这个键）。
    #   消费者：`route/physics.py: bridge_entry_walls()`（几何）与
    #   `route/planner.py: _TSGrid`（tracker-safe 引擎的进洞侧判据，兜底常量
    #   `planner.BRIDGE_ENTRY_REDUNDANCY_M` 与本默认值同值）。
    #   ⚠️ 这段墙**现实不存在**（设计冗余），**不得**加进 `tools/route_check.py` 的用户侧判据。
    "field.bridge_entry_redundancy_m": Spec(0.0875, _F, "m", 0.0, 0.35,
                                            "桥洞进洞侧冗余长度（= 1/4 格，虚拟墙延长）"),
    "planner.obstacle_margin_m": Spec(0.02, _F, "m", 0.0, 0.30,
                                      "障碍净空余量（规划/校验共用；判据 = r + 车半宽 + 此值）"),

    # ---------------- ★ 车上自动规划（`planner.*`）10 键登记 ----------------
    #  为什么补这一组：这 10 个键**一直在 robot.yaml 里生效**（`tasks/mission.py` 读它们），
    #  却从没进过 SCHEMA ⇒ ① 没有类型/范围校验；② `validate()` 只能靠"段前缀"笼统放行，
    #  段内拼错（`planner.pivot_penality`）抓不到。登记后：类型/范围可查，拼错也能报。
    #  ⚠️ `default` 取的是**代码里的兜底值**（= 键缺失时规划器的真实行为），不是现场调过的
    #  值：`pivot_penalty` 现场 5.0 而代码兜底 0.0、`auto_plan_enabled` 现场 true 而兜底 False。
    #  两者含义不同（"缺键时怎么走" vs "现在怎么走"），所以**不要**把 default 改成现场值。
    "planner.engine": Spec("tour", _S, "", None, None,
                           "规划引擎：tour(历史，逐字节不变) / tracker-safe(新引擎，跑双门禁)；"
                           "缺省 tour = 与历史一致"),
    "planner.footprint_gate": Spec("warn", _S, "", None, None,
                                   "足迹诊断严重级：warn=只报不拦（现役）/ error=有侵入即门禁不过"),
    "planner.tracker_max_seconds": Spec(10.0, _F, "s", 1.0, 120.0,
                                        "tracker-safe 引擎时间预算（车端实测中位 2.79 s，取 ×3 ⇒ 10 s）"),
    "planner.auto_plan_enabled": Spec(False, _B, "", None, None,
                                      "启动即自己算路线；false = 只用 mission_blocks"),
    "planner.map_path": Spec("", _S, "", None, None,
                             "自动规划读哪张地图（空 = 按主流程顺序找 config/map.yaml）"),
    "planner.fallback_to_tokens": Spec(True, _B, "", None, None,
                                       "自规划失败时回退 mission_blocks；false = 停 IDLE（不带病出车）"),
    "planner.allow_reverse": Spec(False, _B, "", None, None,
                                  "允许倒车段（真车底盘不支持倒车 ⇒ 缺省 False，折返交 :PIVOT）"),
    "planner.max_seconds": Spec(2.0, _F, "s", 0.1, 60.0,
                                "tour 引擎时间预算（与历史同值 2.0 s；超时抛 PlannerError 走回退）"),
    "planner.pivot_penalty": Spec(0.0, _F, "", 0.0, 100.0,
                                  "折返惩罚/处（tour 引擎；代码兜底 0.0 = 历史基线，现场调成 5.0）"),
    "planner.mine_turn_penalty": Spec(2.0, _F, "", 0.0, 100.0,
                                      "雷格不得作拐点的惩罚（车心必须压过雷心；0 = 关闭）"),

    # ---------------- ★ 定位/吸附/漂移门禁 12 键登记 ----------------
    #  为什么也要补：检查器改成"段内未登记键要报"后，这 12 个 `localization.*`
    #  立刻现形 —— 它们**一直在 robot.yaml 里生效**（`tasks/localization.py` 读），
    #  只是从没进 SCHEMA。不补的话，要么白报 12 条，要么把两级判据退回"段内一律放行"
    #  （= 拼写错误又抓不到了）。补完这一段，SCHEMA 对"现役 robot.yaml 用到的每一段"都是完备的。
    #  ⚠️ 唯独 `localization.yaw_trim_deg` **故意不登记**（见 `INTENTIONALLY_UNREGISTERED`）：
    #    它的 ±10° 护栏由 `tools/calib_backfill.py` 提供，`tests/test_calib_backfill.py:2.3`
    #    把"SCHEMA 里没有它"当成设计约束钉住（避免两套判据各说各话）。
    #  ⚠️ `default` 取代码兜底值；现场某些值被特意改大（`drift_slow_after/stop_after = 999`
    #   = 2026-10-11 快速采集时**解除**降速/停车门禁）⇒ 上限必须容得下 999，不要按"常理"限制为 10。
    "localization.anchor_max_pull_m": Spec(0.12, _F, "m", 0.0, 0.50,
                                          "单次吸附位移上限（缺省 = pursuit.arrival_m，双保险）"),
    "localization.anchor_reject_eps_m": Spec(0.005, _F, "m", 0.0, 0.10,
                                            "『吸附后离折线不变远』的容差"),
    "localization.anchor_scale_max": Spec(0.08, _F, "", 0.0, 0.50,
                                          "隐含尺度误差判别上限（仅判别，不改标定值）"),
    "localization.implied_scale_window": Spec(4, _I, "拍", 1, 100,
                                              "尺度/沿程修正量统计窗口"),
    "localization.reanchor_pull_max_m": Spec(0.35, _F, "m", 0.0, 1.00,
                                             "横向重锚单次上限（且只在顶点窗口内）"),
    "localization.drift_slow_after": Spec(3, _I, "段", 1, 10000,
                                          "连续漂移多少段后降速；999 = 解除该门禁（快速采集档）"),
    "localization.drift_slow_speed_mps": Spec(0.08, _F, "m/s", 0.0, 0.40,
                                              "漂移降速目标速度"),
    "localization.drift_stop_after": Spec(8, _I, "段", 1, 10000,
                                          "连续漂移多少段后请求停车；999 = 解除该门禁（快速采集档）"),
    "localization.drift_clear_ticks": Spec(5, _I, "拍", 1, 1000,
                                           "连续干净 N 拍 ⇒ 撤销降速/停车"),
    "localization.boundary_guard_m": Spec(0.10, _F, "m", 0.0, 1.00,
                                          "离安全边界多近开始限速"),
    "localization.boundary_guard_speed_mps": Spec(0.08, _F, "m/s", 0.0, 0.40,
                                                  "边界限速值"),
    "localization.boundary_stop_m": Spec(0.03, _F, "m", 0.0, 1.00,
                                         "离安全边界多近请求停车"),

    # ---------------- 路线与任务 ----
    "route.tokens": Spec(None, list, "", None, None, "mission_blocks（格名 token 列表）", True),
    "route.cross_lower_candidates": Spec([f"K{r}" for r in range(7, 12)], list, "",
                                         None, None, "下半场穿洞候选格（U 形赛道第一处）"),
    "route.cross_upper_candidates": Spec([f"K{r}" for r in range(1, 7)], list, "",
                                         None, None, "上半场穿洞候选格（U 形赛道第二处）"),
    # 兜底值 = K10(下) / K5(上)。现役 `config/robot.yaml` 已按第五张图覆盖为 K9/K5；
    # 兜底值必须 ∈ 候选集合，否则"yaml 漏键"会静默退回失效的洞，并让 `validate()`
    # 多报一条 warn。`validate()` 一致性断言逻辑与候选集合宽口径缺省（K7~K11 / K1~K6）不在此处。
    "route.cross_lower_default": Spec("K10", _S, "", None, None,
                                      "未收到无人机情报时的兜底穿洞格（第五张图下半场 = K10）"),
    "route.cross_upper_default": Spec("K5", _S, "", None, None,
                                      "未收到无人机情报时的兜底穿洞格（第五张图上半场 = K5）"),
    # ---- SHOOT 停站点的**靶板中心距离窗口**（取代"靶格 1 格邻居"口径）----
    #   新口径：候选 = 到**靶板中心**距离 ∈ [shoot_distance_m − tol, + tol] 的可通行格，
    #           再按"离轨道垂直平分线最近 + 距离误差最小 + 格名序"定序取首选。
    #           现役 `config/robot.yaml` = 0.90 / 0.12 ⇒ [0.78, 1.02]，命中双停站 D3/G3。
    #   为什么用两个键：`shoot_distance_m` 是**现场标定值**（靶板到停站点的目标距离），
    #   `_tol_m` 是**窗口半宽**（靶会摆动/标定有误差）。改其一即可（只改常数）。
    #   ⚠️ 与 `path_executor.shoot_distance_target/min/max`（0.35 / 0.20 / 0.45）**语义不同**：
    #      那三键是**视觉侧**"哪一帧算命中"的口径（`route/validate.py` 与 `tools/route_check.py`
    #      的射击点距离判据），本两键管的是"**车停哪儿**"（规划期挑停站点）。两套不要互相抄。
    #   消费者：`route/planner.py` 的 `shoot_distance_window()`；缺键时 planner 侧常量兜底
    #      0.90 / 0.12（SCHEMA 兜底仍为历史默认 0.80 / 0.10，yaml 正常提供本两键时不触发）。
    "route.shoot_distance_m": Spec(0.80, _F, "m", 0.0, 5.0,
                                   "SHOOT 停站点到**靶板中心**的目标距离（靶板中心 = 各靶格中心之中点）"),
    "route.shoot_distance_tol_m": Spec(0.10, _F, "m", 0.0, 2.0,
                                       "上述距离的窗口半宽（候选区间 = 目标距离 ± 本值 ⇒ 默认 [0.70, 0.90]）"),
    "mission.auto_start": Spec(True, _B, "", None, None, "起栈后自动出发"),
    "mission.auto_start_delay_s": Spec(10.0, _F, "s", 0.0, 120.0, "出发前初始化等待"),
    "mission.reset": Spec("N10,y", _S, "", None, None, "起步复位（格,航向）"),
    "mission.scan_mode": Spec("continuous", _S, "", None, None,
                              "continuous 边走边扫 / waypoint 到点扫描 / none"),
    "mission.bridge_info_wait_s": Spec(20.0, _F, "s", 0.0, 120.0,
                                       "等待无人机桥洞情报的最长时间，超时用兜底格"),
    # ---- 桥前等待（L10 决策点）+ 赛事预算 + 迟到改道上限 ----
    "mission.bridge_wait_at_start": Spec(False, _B, "", None, None,
                                         "起点是否仍为情报推迟发车（默认 false = 等移到 L10 决策点）"),
    "mission.bridge_wait_max_s": Spec(12.0, _F, "s", 0.0, 120.0,
                                      "L10 桥前等待的上限（与预算余量取 min；到点必发车）"),
    "mission.race_budget_s": Spec(180.0, _F, "s", 10.0, 600.0,
                                  "本局总预算（3 分钟口径）= 桥前等待余量的数据源"),
    "mission.bridge_reroute_max": Spec(1, _I, "次", 0, 5,
                                       "迟到情报在决策点前允许的换洞改道次数（全程上限）"),
    "mission.shots_per_stop": Spec(3, _I, "发", 1, 10, "每个射击停站点要打几发（旧 shoot_count_target）"),
    "mission.shoot_timeout_s": Spec(30.0, _F, "s", 1.0, 120.0, "单个停站点硬超时（绝不卡死）"),
    "mission.lock_timeout_s": Spec(18.0, _F, "s", 1.0, 120.0, "等云台锁定的最长时间"),
    "mission.settle_time_s": Spec(0.8, _F, "s", 0.0, 10.0, "底盘停稳判定时长"),
    "mission.settle_eps_m": Spec(0.02, _F, "m", 0.001, 0.20, "停稳判定的位置变化容差"),
    "mission.settle_speed_mps": Spec(0.02, _F, "m/s", 0.001, 0.20, "停稳判定的速度容差"),
    "mission.scan_stop_timeout_s": Spec(8.0, _F, "s", 0.5, 60.0,
                                        "纯扫描停站点（:SCAN_STOP）的等待上限"),
    # ---- 动态档（按桥洞远近在近/远两档间选巡航速度；**默认关闭**）----
    #   闸门 = `pursuit.dynamic_gear_enabled`（false ⇒ `pursuit.controller.dynamic_gear_speed()`
    #   返回 None ⇒ tracker 不给 `compute_command` 传覆盖值 ⇒ 与历史逐位一致）。
    #   判据 = 车到**下一个桥洞格（K 列）**的距离：`≤ near_dist_m` 用 `near_mps`，否则 `far_mps`；
    #   前方没有桥洞（None）= `far_mps`（离洞远 ⇒ 放开跑）。
    "pursuit.dynamic_gear_enabled": Spec(False, _B, "", None, None,
                                         "动态档总闸（默认 false；开启后按桥洞远近选档）"),
    "pursuit.dynamic_gear_near_dist_m": Spec(1.05, _F, "m", 0.0, 20.0,
                                             "近洞阈值（默认 1.05m = 3 格 × 0.35m）"),
    "pursuit.dynamic_gear_near_mps": Spec(0.35, _F, "m/s", 0.02, 0.60,
                                          "近洞档巡航速度（默认 0.35）"),
    "pursuit.dynamic_gear_far_mps": Spec(0.40, _F, "m/s", 0.02, 0.60,
                                         "远洞档巡航速度（默认 0.40）"),
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
    "aim.lead_time_s": Spec(0.0, _F, "s", 0.0, 2.0,
                            "打靶修复：提前量先归零（pred=dx 直接瞄准）；旧 0.35 未对准 0.5~0.9s 真实链路延迟"),
    "aim.fire_policy": Spec("moving_lead", _S, "", None, None,
                            "moving_lead（本靶）/ sweep / turnaround（无停顿靶不适用）/ static"),
    "aim.rail_span_m": Spec(0.70, _F, "m", 0.05, 3.0, "靶轨道行程（G11+H11 = 2 格 = 0.70 m）"),
    "aim.target_speed_mps": Spec(0.04, _F, "m/s", 0.001, 1.0, "靶标移速（现场实测 4 cm/s）"),
    "aim.max_shots_total": Spec(10, _I, "发", 1, 10, "全场弹药上限"),
    "aim.max_shots_per_target": Spec(3, _I, "发", 1, 10, "单靶发数上限"),
    "aim.shot_cooldown_ms": Spec(500, _I, "ms", 100, 5000, "两发最小间隔"),
    "aim.fire_box_far_px": Spec(14.0, _F, "px", 2.0, 100.0, "远处开火框（> 云台机械精度约 12px；放宽回 14）"),
    "aim.fire_box_near_px": Spec(26.0, _F, "px", 2.0, 200.0, "近处开火框（放宽回 26）"),
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
    # 红面早退。靶板一面蓝一面红（蓝面朝向每局随机但恒定），只打蓝、打红扣分
    #   ⇒ 若本停站点只能看到红面，再等也是白等（30s 停站硬超时）。锁定红持续到这个秒数
    #   就置 `P_AIM_RED`，Mission 据此提前离站去下一个射击停站（两个停站分居板面两侧，
    #   对侧必见另一面）。缺省 3.0 s = 用户口径「红面 ≥3s 早退」。
    "aim.red_early_leave_s": Spec(3.0, _F, "s", 1.0, 10.0,
                                  "锁定红持续多久 ⇒ 置红面确认标志（Mission 提前离站）"),
    # 第一停站已经打出去了，就不必再去第二个射击停站（省一整段绕行 + 停站）。
    #   判据 = 本停站点**发数增量达标**（与 `mission.shots_per_stop` 同一账本），不是"锁定蓝"：
    #   只见蓝但一直没打出去（开火框/冷却/门禁）时仍会去第二停站，避免"以为打了其实没打"。
    "mission.skip_second_stop_after_hit": Spec(True, _B, "", None, None,
                                               "第一射击停站发数达标 ⇒ 跳过后续射击停站"),
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
    # ---- 云台扫描单写者 / 巡迹锁止（四把键，此前只在 AimTask 里读默认值）----
    "aim.gimbal_lock_enabled": Spec(True, _B, "", None, None,
                                    "★云台扫描**单写者**开关：true（默认）= 只有 AimTask 写 "
                                    "/nav/scan_request（巡迹段 0 锁止、最佳射击点停站 2 起转、"
                                    "离开/FINISH 回 0），MissionTask 一个值都不发；"
                                    "false = 旧行为（Mission 按 mission.scan_mode 下发）"),
    "aim.gimbal_lock_scan_request": Spec(0, _I, "", 0, 2,
                                         "锁止态下发的 scan_request 值（0=停扫；车端 C++ 收到 0 "
                                         "进 AIM_IDLE 并每帧归位到车头正前）"),
    "aim.gimbal_seek_scan_request": Spec(2, _I, "", 0, 2,
                                         "起转寻靶下发的 scan_request 值（2=持续扫描；"
                                         "绝不发 1，会被降级成单趟）"),
    "aim.gimbal_seek_on_shoot_stop_only": Spec(True, _B, "", None, None,
                                               "只在 StopKind.SHOOT 停站起转（false 时 SCAN 类停站也可起转）"),

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

    # ---------------- ★★ 航向完整性守卫（LocalizationTask._update_yaw_guard，2026-10-17） ----
    #  背景（_tmp_session/adjudication_t1_t4/裁决书.md §3.4/§4）：14:59:31.8 K9 处 IMU 地图
    #  yaw 90°→−78°（−168°/1.8 s，峰值 120.3°/s @ 轮速 0.112 m/s = Ackermann 上限的 5.08 倍），
    #  比投影跳变早 5.4~7.4 s；停车后静止期还有 169.2°/s 的 yaw 跳变。当时代码对"物理上
    #  不可能的航向变化"没有任何判据 ⇒ 下游一路用坏航向。以下 10 键即该判据的阈值。
    #  ⚠️ 全部 default = 代码兜底值（tasks/localization.py 模块常量），现场可在 robot.yaml 覆盖。
    "localization.yaw_guard": Spec(True, _B, "", None, None,
                                   "★航向完整性守卫总开关（关掉 = 行为回到 2026-10-17 之前）"),
    "localization.yaw_kin_margin": Spec(1.25, _F, "", 1.0, 3.0,
                                        "★运动学上限安全系数：ω_max = margin×v×tan(δmax)/L"
                                        "（v=0.35 时 73.7°/s → 92°/s）"),
    "localization.yaw_rate_floor_dps": Spec(8.0, _F, "°/s", 0.0, 60.0,
                                            "★加在运动学上限上的绝对余量（量化噪声/舵角超调）"),
    "localization.yaw_abs_floor_dps": Spec(20.0, _F, "°/s", 0.0, 90.0,
                                           "★动态判据的绝对下限：低速蠕动时也允许的 yaw 率"),
    "localization.yaw_static_dps": Spec(45.0, _F, "°/s", 5.0, 180.0,
                                        "★静止期（v<yaw_min_speed）允许的 yaw 率上限：事故静止段"
                                        "实测 169.2°/s，正常停车漂移 <5°/s"),
    "localization.yaw_min_speed_mps": Spec(0.03, _F, "m/s", 0.0, 0.10,
                                           "★判定『车没在动』的线速度阈值（帧间位移÷Δt）"),
    "localization.yaw_bad_ticks": Spec(3, _I, "拍", 1, 20,
                                       "★连续多少个**新帧**超限才判故障（单帧毛刺不刹；"
                                       "事故的超限持续 1.8 s）"),
    "localization.yaw_plausible_speed_mps": Spec(0.70, _F, "m/s", 0.20, 2.0,
                                                 "★位姿连续性闸：帧间位移/Δt 超过它就不是"
                                                 "『车在动』的样本（重定位/测试传送），跳过不判"),
    "localization.yaw_max_dt_s": Spec(0.60, _F, "s", 0.10, 2.0,
                                      "★帧间隔上限：超过它角速度不可信（丢帧/卡顿），跳过不判"),
    "localization.yaw_imu_mismatch_ratio": Spec(0.35, _F, "", 0.0, 1.0,
                                                "★有 IMU 时：|gyro z| 低于位姿角速度的这个比例 ⇒ "
                                                "判『位姿-航向链内部不一致』（只改 reason 文案）"),

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

#: **故意**不登记进 SCHEMA 的键（白名单，逐条有出处）。
#:   `localization.yaw_trim_deg` —— 它的 ±10° 护栏由 `tools/calib_backfill.py` 提供，
#:   且 `tests/test_calib_backfill.py:2.3` 明确把「SCHEMA 里没有它」当作设计约束钉住
#:   （理由：同一个量的护栏只该有一处，否则两套判据会各说各话）。登记它会让那条测试变红，
#:   也违背原裁决 ⇒ 这里只做**豁免**，不做登记。
#:   ⚠️ 往这里加键必须写明"护栏在哪儿"，否则等于给拼写错误开后门。
INTENTIONALLY_UNREGISTERED = frozenset({
    "localization.yaw_trim_deg",
})

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
            # ---- ★ 物理几何真源迁移（真 bug 修复，2026-10-16）----
            #   真源在 `/dzactuator` 段（vehicle.* / field.*，见 robot.yaml:749-769），
            #   而 planner.py / sim.py / route_check 读**规范名** `vehicle.*` / `field.*`。
            #   缺这组映射时规范名恒吃 SCHEMA 默认（0.28/0.135/0.1275…）⇒ 现场改尺寸被静默忽略（P0）。
            #   仅 mission.py 曾用 `_f((dzactuator.*, vehicle.*))` 双键兜底读对，其余消费者全断。
            #   迁移后规范名从 dzactuator 段取真值，两端一致，零行为变化（现值恰等于默认）。
            "vehicle.length_m": ["dzactuator.vehicle.length_m"],
            "vehicle.width_m": ["dzactuator.vehicle.width_m"],
            "vehicle.coverage_radius_m": ["dzactuator.vehicle.coverage_radius_m"],
            "vehicle.ref_offset_m": ["dzactuator.vehicle.ref_offset_m"],
            "field.obstacle_radius_m": ["dzactuator.field.obstacle_radius_m"],
            "field.mine_pad_m": ["dzactuator.field.mine_pad_m"],
            "field.barrier_width_m": ["dzactuator.field.barrier_width_m"],
            "field.barrier_cols": ["dzactuator.field.barrier_cols"],
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
        # 改成**两级判据**，真源 = SCHEMA。
        #   旧写法把段前缀手抄成常量表，漏了 `planner.` / `vehicle.` / `field.` 三段 ⇒
        #   这三段里**早已登记**的键被白报"未知配置段"，config_selfcheck 每次刷 20 条，
        #   把真正该看的告警淹掉（复核点名）。
        #   两级判据（缺一不可）：
        #     ① 键在 SCHEMA 里 ⇒ 合法（**登记即生效**，不再依赖手抄表）；
        #     ② 键的**段**在 SCHEMA 里但键本身没登记 ⇒ 报"未登记的配置键"（拼错仍能抓到）；
        #     ③ 段也不认识（ROS 节点段：`path_executor.` / `dzactuator.` …）⇒ 透传，不报。
        #   为什么不能只做 ①+③（即"段来自 SCHEMA 就放行"）：那样 `planner.pivot_penality`
        #   这类段内拼写错误会**静默通过** ⇒ 现场改了参数却不生效（本项目踩过同类坑：
        #   `robot.yaml` 里写 `planner.corner_cut` 之前，"声明了但没人读"的键就是这么来的）。
        node_prefixes = ("path_executor.", "dzactuator.", "dzrknn_dataprocess.", "det_node.",
                         "usb_cam_node.", "dzjudgment.", "ackermann_sim.")
        schema_sections = {key.split(".", 1)[0] + "." for key in SCHEMA if "." in key}
        for key in self._flat:
            if key in SCHEMA or key in INTENTIONALLY_UNREGISTERED:
                continue
            if key.startswith(node_prefixes):
                continue
            section = key.split(".", 1)[0] + "." if "." in key else ""
            if section and section in schema_sections:
                note("warn", key,
                     "未登记的配置键（该段已登记 ⇒ 多半是拼写错误，或新键没写进 SCHEMA）")
            else:
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
