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
    "pursuit.speed_quant_comp": Spec(1.0, _F, "×", 0.8, 1.3,
                                     "底盘速度量化补偿：旧侧 |v·23/0.33| 截断丢约 4.3%，可填 1.043"),
    "pursuit.reverse_speed_ratio": Spec(0.6, _F, "×", 0.1, 1.0, "倒车速度比例"),
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
    "pursuit.finish_approach_gain": Spec(0.35, _F, "1/s", 0.05, 2.0, "终点减速增益"),
    "pursuit.map_safety_margin_m": Spec(0.60, _F, "m", 0.0, 1.5, "越界急停余量（顶层强制生效）"),
    "pursuit.drift_warn_m": Spec(0.30, _F, "m", 0.05, 1.0, "单段漂移告警阈值"),

    # ---------------- 桥洞保护（配置驱动，不硬编码 K 列） ----
    "pursuit.bridge_blocks": Spec(None, list, "", None, None,
                                  "整列保护点（现场为 K1~K11）；缺失则不做桥洞保护，必须显式配置"),
    "pursuit.align_straight_before": Spec(2.0, _F, "格", 0.5, 5.0, "洞前多少格开始锁航向"),
    "pursuit.align_straight_after": Spec(1.0, _F, "格", 0.5, 5.0, "过洞后多少格解除"),
    "pursuit.align_straight_speed_mps": Spec(0.10, _F, "m/s", 0.02, 0.30, "直行段速度"),
    "pursuit.align_kp": Spec(12.0, _F, "", 1.0, 40.0, "航向保持增益（度误差 → 度舵角 /10）"),
    "pursuit.align_steer_limit_deg": Spec(8.0, _F, "°", 1.0, 20.0, "直行段舵角限幅"),

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
    "mission.stable_timeout_s": Spec(6.0, _F, "s", 0.5, 30.0, "等瞄准稳定的最长时间（超时降级放行）"),
    "mission.stable_offset_px": Spec(40.0, _F, "px", 5.0, 200.0, "瞄准稳定容差"),
    "mission.candidate_retry": Spec(3, _I, "个", 1, 10, "单个靶最多尝试几个候选射击点"),
    "mission.finish_voice_id": Spec(0, _I, "", 0, 255, "完成提示音 ID（0 = 不播）"),

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

    # ---------------- 误差修正（L2：位置） ----
    "fix.anchor_at_waypoint": Spec(True, _B, "", None, None, "到位后把位姿锚回格心"),
    "fix.scale_est_enabled": Spec(True, _B, "", None, None, "系统性比例误差在线估计"),
    "fix.scale_step_max": Spec(0.005, _F, "×/段", 0.0, 0.05, "每段允许的最大比例修正步长"),
    "fix.scale_limit": Spec(0.10, _F, "×", 0.0, 0.5, "比例修正总量夹紧（±）"),

    # ---------------- 无人机链路 ----
    "drone.enabled": Spec(True, _B, "", None, None, "是否启用无人机链路"),
    "drone.topic_in": Spec("/dzlink/drone_to_car", _S, "", None, None, "无人机→车 话题"),
    "drone.topic_out": Spec("/dzlink/car_to_drone", _S, "", None, None, "车→无人机 话题"),
    "drone.heartbeat_timeout_s": Spec(3.0, _F, "s", 0.5, 30.0, "心跳丢失判定"),
    "drone.info_stale_s": Spec(30.0, _F, "s", 1.0, 300.0, "情报过期判定"),

    # ---------------- 安全 ----
    "safety.pose_timeout_s": Spec(1.0, _F, "s", 0.1, 10.0, "定位失联判定（超时即刹车）"),
    "safety.task_stall_s": Spec(1.0, _F, "s", 0.1, 10.0, "任务线程心跳丢失判定"),
    "safety.brake_on_fault": Spec(True, _B, "", None, None, "故障时强制发零速"),

    # ---------------- 日志/遥测 ----
    "logging.rate_limit_s": Spec(1.0, _F, "s", 0.0, 60.0, "同类日志限流间隔"),
    "telemetry.period_s": Spec(0.5, _F, "s", 0.1, 10.0, "遥测快照周期"),
}

#: 旧 YAML 里的段落名 → 新版点号前缀（保证同一份 robot.yaml 新旧代码都能读）
SECTION_PREFIX = {
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
            flat.update(_flatten(body, f"{prefix}."))
            if isinstance(body, dict) and "ros__parameters" in body:
                flat.update(_flatten(body["ros__parameters"], f"{prefix}."))
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
                          "logging.", "telemetry.", "aim.", "referee.", "path_executor.",
                          "dzactuator.", "dzrknn_dataprocess.", "det_node.", "usb_cam_node.",
                          "dzjudgment.", "ackermann_sim.")
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
