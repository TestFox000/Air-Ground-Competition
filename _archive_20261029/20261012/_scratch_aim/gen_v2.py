# -*- coding: utf-8 -*-
"""从 aim.py 生成 aim_v2.py 原型（只用于迭代，最终结论回写 aim.py）。"""
from __future__ import annotations

import pathlib

SRC = pathlib.Path(__file__).resolve().parents[1] / "src" / "dzbrain" / "dzbrain" / "tasks" / "aim.py"
DST = pathlib.Path(__file__).resolve().parent / "aim_v2.py"

text = SRC.read_text(encoding="utf-8")


def sub(old: str, new: str, count: int = 1) -> None:
    global text
    assert text.count(old) == count, f"pattern x{text.count(old)} (want {count}): {old[:80]!r}"
    text = text.replace(old, new)


# ---------------------------------------------------------------- 1) 估计器增益/字段
sub(
    """    xi: float = 3.5
    gate_k: float = 3.0
    gate_base_px: float = 3.0
    gate_vel_scale: float = 0.10
    jump_floor_px: float = 25.0
    reset_confirm_frames: int = 5
    dt_min_s: float = 0.002
    dt_max_s: float = 0.50
    max_speed_px_s: float = 420.0
""",
    """    xi: float = 3.5
    gate_k: float = 3.0
    gate_base_px: float = 3.0
    gate_vel_scale: float = 0.10
    jump_floor_px: float = 25.0
    reset_confirm_frames: int = 5
    dt_min_s: float = 0.002
    dt_max_s: float = 0.50
    max_speed_px_s: float = 420.0
    #: ★ 估计速度的物理加速度上限（px/s²）。默认 1.6 m/s² × 140 px/m ≈ 220 px/s²。
    #:   作用：把"单帧残差 → 速度"的增益 1/dt 变成**有界**的 ——
    #:   折返那一拍的残差天生是错的（旧速度方向 + 新测量方向相反），无上限时
    #:   30px/s 的靶会被算成 −61px/s（过冲 2 倍），既污染提前量又拖长收敛。
    max_accel_px_s2: float = 220.0
    #: 折返判据的死区：|v| 小于它就认为"停在零点附近"，不算方向翻转
    #: （否则噪声会让 v 每帧在 ±1px/s 之间抖，判据会被反复触发）
    turn_deadband_px_s: float = 2.0
""",
)

# ---------------------------------------------------------------- 2) 新增字段
sub(
    """    jump_rejections: int = 0
    warmup: int = 0
    flips: int = 0
""",
    """    jump_rejections: int = 0
    warmup: int = 0
    flips: int = 0
    #: 折返（方向翻转）计数。★ 与 `resets` **分开**：折返只清零速度、不重置位置，
    #: 不是"重新初始化"（`resets` 的语义= 估计器被重新初始化，测试按它断言）
    turns: int = 0
    #: 上一拍速度的符号（含死区），用于"折返"判据 + 复位后的方向确认
    prev_sign: int = 0
    #: `prev_sign` 的有效性（复位后要等 |v| 走出死区才重新确认方向）
    sign_valid: bool = False
    #: 最近一次残差的符号（连续两拍反号 = 目标已反向）
    prev_res_sign: int = 0
""",
)

# ---------------------------------------------------------------- 3) _block_warmup → 折返复位
sub(
    '''        self.v = 0.0
        self.warmup = max(0, int(self.reset_confirm_frames))
        self.armed_run = 0
        self.flips += 1
        self.last_reason = reason
''',
    '''        self.v = 0.0
        self.warmup = max(0, int(self.reset_confirm_frames))
        self.armed_run = 0
        self.flips += 1
        self.turns += 1
        self.sign_valid = False
        self.prev_res_sign = 0
        self.last_reason = reason
''',
)

# ---------------------------------------------------------------- 4) update(): 折返判据 + 加速度限幅
sub(
    """        # ---- 校正 ----
        v_prev = self.v
        self.x = x_pred + self.alpha * res
        v_new = v_prev + self.beta * res / dt

        # ---- ★ 折返：速度符号翻转 → 清零 + 重新确认（见 `_block_warmup` 的说明） ----
        if v_prev * v_new < 0.0:
            self._block_warmup(f"turnaround v {v_prev:+.1f}→{v_new:+.1f}px/s")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        # ---- 荒谬速度（噪声/误检）：宁可清零重估，也不外推一个不可能的速度 ----
        if abs(v_new) > self.max_speed_px_s:
            self._reinit(self.x, f"speed_clamp(|v|>{self.max_speed_px_s:.0f}px/s)")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        self.v = v_new
""",
    """        # ---- 校正 ----
        v_prev = self.v
        self.x = x_pred + self.alpha * res
        v_raw = v_prev + self.beta * res / dt

        res_sign = 0 if abs(res) <= 1e-12 else (1 if res > 0.0 else -1)

        # ---- ★ 折返：速度符号翻转 → 清零 + 重新确认（见 `_block_warmup` 的说明） ----
        #  判据：`v_raw` 把符号翻过去了（或连续两拍残差反号 = 目标已经反向），
        #        且翻转前速度确实走出了死区（否则只是停在零点附近的噪声抖动）。
        flip = False
        if abs(v_prev) > self.turn_deadband_px_s:
            if v_prev * v_raw < 0.0:
                flip = True
            elif (self.prev_res_sign and res_sign
                  and res_sign != self.prev_res_sign and abs(v_prev) > 3.0 * self.turn_deadband_px_s):
                flip = True
        self.prev_res_sign = res_sign
        if flip:
            self._block_warmup(f"turnaround v {v_prev:+.1f}→{v_raw:+.1f}px/s")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        # ---- ★ 加速度限幅：单帧速度变化不许超过物理加速度上限（px/s² × dt） ----
        #  这是 1/dt 增益的**有界化**：dt 越小（帧率越高）单帧允许的变化越小，
        #  折返/重捕那一拍的错误残差最多只能把速度拉动 accel×dt，不会一步冲到 −61px/s。
        dv_lim = max(1.0, self.max_accel_px_s2 * dt)
        dv = v_raw - v_prev
        if dv > dv_lim:
            v_new = v_prev + dv_lim
        elif dv < -dv_lim:
            v_new = v_prev - dv_lim
        else:
            v_new = v_raw
        if v_prev * v_new < 0.0:                 # 限幅后仍越过零点 → 停在零点并复位
            self._block_warmup(f"turnaround v {v_prev:+.1f}→{v_new:+.1f}px/s")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        # ---- 荒谬速度（噪声/误检）：宁可清零重估，也不外推一个不可能的速度 ----
        if abs(v_new) > self.max_speed_px_s:
            self._reinit(self.x, f"speed_clamp(|v|>{self.max_speed_px_s:.0f}px/s)")
            self.last_ts = float(ts)
            self.last_dt = dt
            self.samples += 1
            return True

        self.v = v_new
""",
)

# ---------------------------------------------------------------- 5) 方向确认计数（按原逻辑保留）
# 无需改动：smooth 判据用 res 与 v_new 同向

# ---------------------------------------------------------------- 6) AimTask 构造：新参数
sub(
    """            max_speed_px_s=float(g("aim.est_max_speed_px_s",
                                   max(300.0, 5.0 * self._v_expected))),
        )""",
    """            max_speed_px_s=float(g("aim.est_max_speed_px_s",
                                   max(300.0, 5.0 * self._v_expected))),
            max_accel_px_s2=float(g("aim.est_max_accel_px_s2", 220.0)),
        )""",
)

# ---------------------------------------------------------------- 7) 丢靶计时：连续不可见时长
sub(
    """        self._seen_prev = False
        self._lost_at: float | None = None
""",
    """        self._seen_prev = False
        self._lost_at: float | None = None
        #: ★ 目标"连续不可见"的起点（与 `_lost_at` 分开）：
        #:   `_lost_at` 是**判定丢失那一刻**（观测过期后），`_invisible_since`
        #:   是**最后一帧真实视觉的时间戳**。重新武装要看"真的消失了多久"，
        #:   用 `_lost_at` 会把观测过期的 0.5s 丢掉，现场表现=第二块靶永远不重新武装。
        self._invisible_since: float | None = None
""",
)

sub(
    """        # ---- 4. 可见性台账（seen / lost_since / 事件） ----
        self._track_visibility(obs, has_target, now)
""",
    """        # ---- 4. 可见性台账（seen / lost_since / 事件） ----
        self._track_visibility(obs, has_target, now, self._vision_absent(now))
""",
)

sub(
    """    def _track_visibility(self, obs: OffsetObservation | None, has_target: bool,
                          now: float) -> None:
        if has_target and obs is not None:
""",
    """    def _vision_absent(self, now: float) -> bool:
        \"\"\"视觉通道是否已经"断流"（帧被清掉、或最新帧已过期）。

        ★ 为什么要单独判：`FakeBridge.vision()` 在缓冲被 `clear_vision()` 清空后返回 None，
          但**重新喂帧**时 `ts` 会被刷成当前时刻 —— 观测对象本身永远不会"过期"，
          过期只体现在 `vision(max_age)` 这一层。所以"目标消失了多久"必须由这里给。
        \"\"\"
        raw = self.bridge.vision_raw()
        if raw is None:
            return True
        ts = float(getattr(raw, "ts", 0.0) or 0.0)
        return (now - ts) > VISION_MAX_AGE_S

    def _track_visibility(self, obs: OffsetObservation | None, has_target: bool,
                          now: float, absent: bool = False) -> None:
        if has_target and obs is not None:
""",
)

sub(
    """            self._seen_prev = True
            return

        # ---- 不可见：立刻闭锁（旧版这里会冻结云台最长 6s，survey §10 第 7 条） ----
        if self._seen_prev:
            self._lost_at = now
            self._seen_prev = False
            self._rescan_requested = False
""",
    """            self._seen_prev = True
            self._invisible_since = None
            return

        # ---- 不可见：立刻闭锁（旧版这里会冻结云台最长 6s，survey §10 第 7 条） ----
        if self._seen_prev or absent:
            if self._invisible_since is None:
                self._invisible_since = self._last_frame_ts if self._last_frame_ts is not None else now
        if self._seen_prev:
            self._lost_at = now
            self._seen_prev = False
            self._rescan_requested = False
""",
)

# ---------------------------------------------------------------- 8) 重新武装：用真实消失时长
sub(
    """                lost_for = (now - self._lost_at) if self._lost_at is not None else 0.0
                self._lost_at = None""",
    """                lost_for = (self._last_frame_ts - self._invisible_since
                            if self._invisible_since is not None else 0.0)
                self._lost_at = None
                self._invisible_since = None""",
)

sub(
    """        p = self._permit
        if not p.hit_latched:
            return
        if (now - p.hit_at) < self._hit_hold_s:
            return
        if lost_for < self._lost_arm_s:
            return
        if p.window >= self._max_per_target:
            return
""",
    """        p = self._permit
        if not p.hit_latched:
            return
        if (now - p.hit_at) < self._hit_hold_s:
            return
        if lost_for < self._lost_arm_s:
            return
        # ⚠️ 注意：这里**不**再要求 `p.window < max_per_target`。
        #   命中锁存已经说明"本靶打完了"，窗口许可数是**本靶**的账；重新武装的语义是
        #   "换了一块新靶 / 目标重新出现"，新靶当然要重新拿到自己的额度。
        #   全场账由 `_max_total` 与 `bridge.shoot_count()` 独立把关（不在这里拦）。
""",
)

# ---------------------------------------------------------------- 9) 预算：独立的全场许可上限
sub(
    """        # ---- 硬门 6：弹药 / 预算 ----
        if self._permit.window >= self._max_per_target:
            return False, REASON_BUDGET_PER_TARGET
        if self._shots_seen >= self._max_total:
            return False, REASON_BUDGET_TOTAL
""",
    """        # ---- 硬门 6：弹药 / 预算（★ 单靶与全场**两套独立的账**，两个都要拦） ----
        if self._permit.window >= self._max_per_target:
            return False, REASON_BUDGET_PER_TARGET
        #  全场上限同时看两路：① 已**许可**的累计数（预算必须先于实发被拦住，
        #  否则 max_shots_total=1 会先发 2 次许可再被 shoot_count 追上）；
        #  ② 云台/裁判回传的实发数（真机上许可与实发可能不同步，两路都要对）。
        if self._permit.total >= self._max_total:
            return False, REASON_BUDGET_TOTAL
        if self._shots_seen >= self._max_total:
            return False, REASON_BUDGET_TOTAL
""",
)

# ---------------------------------------------------------------- 10) 冷却：许可跳变沿计数
sub(
    """        # ---- 硬门 8：冷却 ----
        if not self._permit.cool_ok(now, self._cooldown_s):
            return False, REASON_COOLDOWN
        return True, REASON_OK
""",
    """        return True, REASON_OK
""",
)

sub(
    '''    def _set_fire_ready(self, ready: bool, now: float, reason: str) -> None:
        """★ 每拍都发（锁存 + 保活）：云台补丁重启/丢帧后不会卡在旧的许可上。

        ⚠️ 这里**只**碰 `fire_ready`。`fire_gate` 是 MissionTask 的钥匙，本任务绝不写。

        ★★ 许可计数发生在**冷却到点那一拍**，而不是"False→True 跳变那一拍"。

        为什么必须这样（否则单靶只能打 1 发）：本任务每拍都在发 `fire_ready`，
        云台侧只要收到 1 就会在条件满足时开火。如果许可只在跳变沿计数，
        第一发之后 `fire_ready` 一直是 1（只要目标还在框内），`_permit.window` 就永远停在 1 ——
        `max_shots_per_target=3` 形同虚设、且**第二发永远不会被授权**。
        改成"冷却到点即记一次许可"后：1 发 = 1 个 `shot_cooldown_ms` 窗口，
        该窗口内本任务不再重复计数，云台自己的冷却也正好用掉一个窗口。
        不变量：**一拍最多记一次许可**（记完 `cool_ok` 立即为假）。
        """
        ready = bool(ready)
        if not ready:
            self._blocked_reason = reason
            if self._fire_ready:
                self._fire_ready = False
                self.bus.publish(EV_FIRE_BLOCKED, source=self.name, reason=reason,
                                 lead_px=self._last_lead, at=now)
                self._log(f"闭锁 fire_ready=0：{_REASON_TEXT.get(reason, reason)}")
            elif self._blocked_reason != REASON_OK:
                if self._log_throttle.allow("blocked:" + reason, now):
                    self._log(f"不开火：{_REASON_TEXT.get(reason, reason)}")
            self._emit_ready(False)
            return

        # ---- ready=True ----
        first_permit = self._permit.last_allow_at <= 0.0
        grant = first_permit or self._permit.cool_ok(now, self._cooldown_s)
        if grant:
            first_for_this_target = first_permit
            self._permit.allowed(now)
            if first_for_this_target:
                # 本靶的**第一发**：一个独立事件（EV_FIRE_ALLOWED），便于复盘时间线
                self.bus.publish(EV_FIRE_ALLOWED, source=self.name, lead_px=self._last_lead,
                                 permits=self._permit.window, at=now)
            self._log(f"★ 瞄准合格 → fire_ready=1（许可 {self._permit.window}"
                      f"/{self._max_per_target}，累计 {self._permit.total}，"
                      f"lead {self._last_lead:+.1f}px）")
        elif not self._fire_ready:
            self._log(f"瞄准合格但冷却未过（剩 {self._permit.cooldown_left(now, self._cooldown_s):.2f}s）"
                      f"→ 保持 ready，不重复计数")
        self._fire_ready = True
        self._blocked_reason = REASON_OK
        self._emit_ready(True)
''',
    '''    def _set_fire_ready(self, ready: bool, now: float, reason: str) -> None:
        """★ 每拍都发（锁存 + 保活）：云台补丁重启/丢帧后不会卡在旧的许可上。

        ⚠️ 这里**只**碰 `fire_ready`。`fire_gate` 是 MissionTask 的钥匙，本任务绝不写。

        ★★ 许可记账：**上升沿记一次 + 冷却到点续记一次**，其余时间闭锁。

        为什么是"上升沿 + 续记"（这是本文件唯一一处需要小心的地方，两条约束互相拉扯）：
          · 只按上升沿记 → 第一发之后 `fire_ready` 一直是 1（目标还在框内），
            `window` 永远停在 1：`max_shots_per_target=3` 形同虚设、第二发永不授权；
          · 只按"冷却到点"记 → 一整段持续 ready 会每隔 `shot_cooldown_ms` 记一次，
            而"停车射击"到站只给一次发射窗口，弹药会在一个窗口里被记满。

        所以正确的语义是"**一个冷却窗口 = 一发**"：
          · 许可那一刻 = 上升沿 → 记一次，随即**本轮闭锁**；
          · 冷却到点后若目标仍合格 → 又一次上升沿 → 再记一次（这就是连发）；
          · 冷却期内一律 `fire_ready=0`（云台侧看到 0 就不会留上一拍的 1）。

        不变量：**一个冷却窗口最多记一次许可**（记完 `cool_ok` 立即为假）；
                **每一拍都发 `publish_fire_ready`**（闭锁期发 0，占位保活）。
        """
        ready = bool(ready)
        if ready and not self._permit.cool_ok(now, self._cooldown_s):
            ready, reason = False, REASON_COOLDOWN

        if not ready:
            self._blocked_reason = reason
            if self._fire_ready:
                self._fire_ready = False
                self.bus.publish(EV_FIRE_BLOCKED, source=self.name, reason=reason,
                                 lead_px=self._last_lead, at=now)
                self._log(f"闭锁 fire_ready=0：{_REASON_TEXT.get(reason, reason)}")
            elif reason != REASON_OK:
                if self._log_throttle.allow("blocked:" + reason, now):
                    self._log(f"不开火：{_REASON_TEXT.get(reason, reason)}")
            self._emit_ready(False)
            return

        # ---- ready=True（冷却已过，或本靶第一发）----
        first_permit = self._permit.last_allow_at <= 0.0
        self._permit.allowed(now)
        if first_permit:
            # 本靶的**第一发**：一个独立事件（EV_FIRE_ALLOWED），便于复盘时间线
            self.bus.publish(EV_FIRE_ALLOWED, source=self.name, lead_px=self._last_lead,
                             permits=self._permit.window, at=now)
        self._log(f"★ 瞄准合格 → fire_ready=1（许可 {self._permit.window}"
                  f"/{self._max_per_target}，累计 {self._permit.total}，"
                  f"lead {self._last_lead:+.1f}px）")
        self._fire_ready = True
        self._blocked_reason = REASON_OK
        self._emit_ready(True)
''',
)

# ---------------------------------------------------------------- 11) 回扫标志：与 visible 解耦
sub(
    """        # ---- 丢靶回扫：默认**关**（理由见下），开着才发 ----
        if (self._lost_at is not None and not self._rescan_requested
                and (now - self._lost_at) >= self._lost_s):
            self._rescan_requested = True
""",
    """        # ---- 丢靶回扫：默认**关**（理由见下），开着才发 ----
        #  ★ 计时用**最后一帧真实视觉**算（`_invisible_since`），不是"判定丢失那一刻"，
        #    否则丢帧阈值里会凭空多出 0.5s 的观测过期时间。
        if (self._invisible_since is not None and not self._rescan_requested
                and (now - self._invisible_since) >= self._lost_s):
            self._rescan_requested = True
""",
)

sub(
    """                    self._log(f"丢靶 {now - (self._lost_at or now):.2f}s ≥ {self._lost_s:.2f}s"
                              f" → 重发 scan_request=2（持续扫描，不发 1）")""",
    """                    self._log(f"丢靶 {now - (self._invisible_since or now):.2f}s ≥ "
                              f"{self._lost_s:.2f}s → 重发 scan_request=2（持续扫描，不发 1）")""",
)

# ---------------------------------------------------------------- 12) 遥测补充
sub(
    """            "lost_s": (now - self._lost_at) if self._lost_at is not None else 0.0,""",
    """            "lost_s": (now - self._lost_at) if self._lost_at is not None else 0.0,
            "invisible_s": ((now - self._invisible_since)
                            if self._invisible_since is not None else 0.0),
            "est_turns": int(est.turns),""",
)

sub(
    """        "aim.rescan_on_lost", "aim.lead_eps_px", "aim.hold_min_s",
    )""",
    """        "aim.rescan_on_lost", "aim.lead_eps_px", "aim.hold_min_s",
        "aim.est_max_accel_px_s2", "aim.est_turn_deadband_px_s",
    )""",
)

sub(
    """            max_accel_px_s2=float(g("aim.est_max_accel_px_s2", 220.0)),
        )""",
    """            max_accel_px_s2=float(g("aim.est_max_accel_px_s2", 220.0)),
            turn_deadband_px_s=float(g("aim.est_turn_deadband_px_s", 2.0)),
        )""",
)

DST.write_text(text, encoding="utf-8")
print("wrote", DST)
