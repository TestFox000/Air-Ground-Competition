"""敌我识别与射击决策。

规则硬约束（《15空地协同侦排挑战赛》）：
    * 红色面 = 友方人员目标  -> 禁止打击；误击一次计数 +1，扣 5 分，计到 3 判误击毁
    * 蓝色面 = 敌方人员目标  -> 允许打击；命中一次计数 +1，得 5 分，计到 3 判击毁
    * 每累计 2 次有效命中，受击侧计数器 +1
    * 激光子弹共 10 发；击毁一个目标需 6 次有效命中
    * 靶标双面共用同一实体，任一侧计到 3 后整体失效，后续命中不计分也不扣分

模型类别（best_new.rknn / YOLOv8n-pose 元数据）：
    {0: 'blue glowing target', 1: 'red glowing target'}
    -> 因此 CLS_BLUE=0 才是可打击目标，CLS_RED=1 必须放过。切勿按字面直觉反推。

由于误击与得分完全对称（-5 / +5），本策略默认要求 **连续多帧一致** 才允许开火。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

from .postprocess import Detection

__all__ = ["TargetPolicy", "TargetDecision", "FireAction", "CLASS_BLUE", "CLASS_RED"]

#: 默认值对应 best_new 模型元数据 {0: 'blue glowing target', 1: 'red glowing target'}。
#: 重新训练后若类别顺序变了，必须同步 `target_node` 的 class_blue / class_red 参数，
#: 否则会把友方当敌方打（每次 −5，最多 −15）。
CLASS_BLUE = 0  # 蓝色面 = 敌方 = 可打击
CLASS_RED = 1   # 红色面 = 友方 = 禁止打击


class FireAction(str, Enum):
    FIRE = "fire"
    HOLD = "hold"
    AVOID = "avoid"    # 识别为友方，明确不打
    NO_TARGET = "no_target"


@dataclass
class TargetDecision:
    action: FireAction
    cls: Optional[int] = None
    score: float = 0.0
    aim_px: Optional[Tuple[float, float]] = None   # 瞄准点（关键点优先）
    yaw_delta: float = 0.0
    pitch_delta: float = 0.0
    reason: str = ""
    consensus: int = 0


@dataclass
class TargetPolicy:
    conf_thres: float = 0.60
    kpt_conf_thres: float = 0.40
    consensus_frames: int = 3
    aim_tolerance_px: float = 25.0
    # 类别 id 与「蓝=敌方/红=友方」的对应关系，必须与训练时的类别顺序一致
    class_blue: int = CLASS_BLUE
    class_red: int = CLASS_RED
    ammo_total: int = 10
    hits_per_count: int = 2
    max_count: int = 3
    img_w: int = 640
    img_h: int = 480
    fov_h_deg: float = 60.0

    shots_fired: int = 0
    enemy_hits: int = 0
    friend_hits: int = 0
    _history: List[Tuple[int, float, float]] = field(default_factory=list, repr=False)

    # ---------------- 计数换算 ----------------
    @property
    def ammo_left(self) -> int:
        return max(0, self.ammo_total - self.shots_fired)

    @property
    def enemy_count(self) -> int:
        return min(self.max_count, self.enemy_hits // self.hits_per_count)

    @property
    def friend_count(self) -> int:
        return min(self.max_count, self.friend_hits // self.hits_per_count)

    @property
    def target_destroyed(self) -> bool:
        return self.enemy_count >= self.max_count or self.friend_count >= self.max_count

    @property
    def expected_score(self) -> int:
        return 5 * self.enemy_count - 5 * self.friend_count

    def on_shot_result(self, hit: bool, on_enemy: bool) -> None:
        """由裁判系统反馈回填命中结果。"""
        self.shots_fired += 1
        if hit:
            if on_enemy:
                if not self.target_destroyed:
                    self.enemy_hits += 1
            else:
                if not self.target_destroyed:
                    self.friend_hits += 1

    def reset_episode(self) -> None:
        self.shots_fired = 0
        self.enemy_hits = 0
        self.friend_hits = 0
        self._history.clear()

    # ---------------- 决策 ----------------
    def update(self, dets: List[Detection]) -> TargetDecision:
        if self.target_destroyed:
            return TargetDecision(FireAction.HOLD, reason="靶标已失效，后续命中不计分")
        if self.ammo_left <= 0:
            return TargetDecision(FireAction.HOLD, reason="弹药耗尽")

        best: Optional[Detection] = None
        for d in dets:
            if d.score < self.conf_thres:
                continue
            if best is None or d.score > best.score:
                best = d
        if best is None:
            self._history.clear()
            return TargetDecision(FireAction.NO_TARGET, reason="无满足置信度的目标")

        aim = self._aim_point(best)
        if aim is None:
            self._history.clear()
            return TargetDecision(FireAction.HOLD, cls=best.cls, score=best.score,
                                  reason="关键点置信度不足，放弃瞄准")

        ax, ay = aim
        yaw, pitch = self._delta(ax, ay)

        # 类别一致性投票
        self._history.append((best.cls, ax, ay))
        if len(self._history) > self.consensus_frames:
            self._history.pop(0)
        same_cls = all(h[0] == best.cls for h in self._history)
        drift = max(abs(h[1] - ax) for h in self._history) + max(abs(h[2] - ay) for h in self._history)
        consensus = len(self._history)

        if best.cls == self.class_red:
            self._history.clear()
            return TargetDecision(
                FireAction.AVOID, cls=best.cls, score=best.score, aim_px=aim,
                yaw_delta=yaw, pitch_delta=pitch,
                reason="识别为友方（红面），禁止打击",
            )

        if not same_cls or consensus < self.consensus_frames:
            return TargetDecision(
                FireAction.HOLD, cls=best.cls, score=best.score, aim_px=aim,
                yaw_delta=yaw, pitch_delta=pitch, consensus=consensus,
                reason=f"等待类别一致投票 {consensus}/{self.consensus_frames}",
            )
        if drift > self.aim_tolerance_px:
            return TargetDecision(
                FireAction.HOLD, cls=best.cls, score=best.score, aim_px=aim,
                yaw_delta=yaw, pitch_delta=pitch, consensus=consensus,
                reason=f"目标漂移 {drift:.0f}px 超过容差",
            )
        if max(abs(yaw), abs(pitch)) > self._max_angle():
            return TargetDecision(
                FireAction.HOLD, cls=best.cls, score=best.score, aim_px=aim,
                yaw_delta=yaw, pitch_delta=pitch, consensus=consensus,
                reason="偏角过大，先转云台",
            )

        return TargetDecision(
            FireAction.FIRE, cls=best.cls, score=best.score, aim_px=aim,
            yaw_delta=yaw, pitch_delta=pitch, consensus=consensus,
            reason="蓝色面（敌方）+ 多帧一致 + 已在容差内",
        )

    # ---------------- 内部 ----------------
    def _aim_point(self, d: Detection) -> Optional[Tuple[float, float]]:
        """优先用关键点（发光中心），否则退回框中心。"""
        if d.kpts is not None and len(d.kpts) > 0:
            x, y, c = d.kpts[0]
            if c >= self.kpt_conf_thres:
                return float(x), float(y)
            return None
        return d.center()

    def _delta(self, ax: float, ay: float) -> Tuple[float, float]:
        from .geometry import pixel_to_gimbal_delta  # 局部导入避免循环

        return pixel_to_gimbal_delta(ax, ay, self.img_w, self.img_h, self.fov_h_deg)

    def _max_angle(self) -> float:
        from .geometry import focal_pixels
        import math

        f = focal_pixels(self.img_w, self.fov_h_deg)
        return math.degrees(math.atan2(self.aim_tolerance_px, f))
