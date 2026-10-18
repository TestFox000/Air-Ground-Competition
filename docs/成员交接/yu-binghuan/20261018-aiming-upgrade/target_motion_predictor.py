#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
target_motion_predictor — 移动标靶运动模型预测器（air_ground 车端瞄准增强）
================================================================================

定位：挂载式任务层预测节点（★不改动 dzrknn_dataprocess 的任何 C++ 主干）。
      治的问题：标靶匀速往返移动时，旧方案的差分预瞄只提前 ~1 帧(33ms)，
      而链路延迟实测 ~0.26s —— 云台永远落后目标 22~44px（开火框才 3.5px），
      匀速段锁不死也打不中；端部 0.2s 停顿窗口又被 EMA 拖尾吃掉。

实测运动先验（2026-10-17 视频逐帧分析，577 帧 30fps，蓝色 LED 跟踪标定）：
  · 单程运动 3.40s ± 0.04s，两端各停 ~0.2s
  · 含停顿半周期 3.6s，完整往返周期 7.2s
  · 中段匀速 ~0.14 m/s，行程 ~0.46m（≈1.3 格），无变速段
  ⇒ 运动完全可预测：拟合"段内匀速 + 端点折返 + 端部停顿"三段式模型即可。

工作原理：
  1. 观测合成：θ_target(世界角) = 云台角(camemo_feedback) + 画面偏差(offset_center)
     × 像素转角（px_per_tick × ticks_per_deg，与 C++ 侧共用同一标定真源）
  2. 在线拟合：最近 fit_window_s 内最小二乘拟合角速度 v；|v| 跌破失速阈值
     → 判定进入端部停顿；v 符号翻转 → 记录折返点，滑动平均学习两端端点
  3. 模型外推：预测 latency_ms 之后靶的位置 —— 匀速段线性外推；越过已学习
     端点则按"到达端点→停 0.2s→反向"处理；停顿中则预测停顿结束后反向运动
  4. 输出：
     /aim/lead_x   (Float32, px, 正=目标向右) —— 超前瞄准量，C++ 侧由
                   lead_apply_gain 控制施加比例，开火判据 in_fire_zone 自动
                   用它判断"超前落点"是否进框
     /aim/fire_ready (UInt8) —— 目标已确认 + 运动模型可信才放行（两键开火的
                   第二把钥匙；第一把 /aim/fire_gate 由 MissionTask 掌握）

优雅之处：超前瞄准后云台角速度≈靶角速度，靶在画面里【相对静止】——
C++ 的"画面稳定 ≤1.5px/帧"开火判据自然满足，匀速段也能开火，
不再依赖 0.2s 停顿窗口苟活。

部署（车上）：
  python3 target_motion_predictor.py        # 需已 source ROS2 环境
  参数见 main() 的 declare 区，全部可用 ROS 参数覆盖；建议写入 robot.yaml
  与 C++ 侧联动的三个关键参数（见补丁说明文档 §六）：
    use_external_fire_gate: true   # C++ 开两键开火，fire_ready 才生效
    lead_apply_gain: 0.6 → 1.0     # 渐进施加提前量（先 0.6 看命中分布）
    predict_gain: 0.0 / vision_latency_ms: 0.0  # ★预测器接管后 C++ 差分
                                   预瞄必须归零，否则双重补偿过冲！

回退：enable=false（预测器静默）或直接停节点；C++ 把 use_external_fire_gate
      设回 false 即回到旧的画面静止判据。

依赖：rclpy + std_msgs（车端零新增依赖）。
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32MultiArray, UInt8, Float32

from target_ekf import TargetEkf


class TargetMotionPredictor(Node):
    """移动标靶运动模型预测器：观测→拟合→外推→超前瞄准指令。"""

    def __init__(self):
        super().__init__("target_motion_predictor")

        # ---------------- 参数（全部可被 robot.yaml 覆盖） ----------------
        self.declare_parameter("enable", True)                # 总开关
        self.declare_parameter("px_per_tick", 2.0)            # 像素/刻度（与 C++ 共用标定真源）
        self.declare_parameter("ticks_per_deg", 11.378)       # 刻度/度 = 4096/360
        self.declare_parameter("gimbal_center_tick", 2047)    # 云台零位刻度
        self.declare_parameter("gimbal_sign", 1.0)             # 云台反馈刻度->角度符号（反了改-1）
        self.declare_parameter("latency_ms", 260.0)           # 总链路延迟：曝光→推理→发布→伺服拍
        self.declare_parameter("fit_window_s", 0.6)           # 速度拟合窗口
        self.declare_parameter("fit_min_points", 8)           # 拟合最少观测点
        self.declare_parameter("stall_v_deg_s", 2.0)          # 失速阈值(度/s)：短窗噪声斜率
        # ↑ 2026-10-17 仿真踩坑：0.3 压不住 5 点短窗的噪声斜率（±1°/s），
        #   停顿中状态会抖动导致端点被污染；应设为 > 噪声斜率上界(~1°/s)
        #   且 << 真实运动速度(~12°/s @0.5m 打击距离)，默认 2.0
        self.declare_parameter("dwell_s", 0.20)               # 端部停顿时长先验（实测 0.2s）
        self.declare_parameter("dwell_judge_lag_s", 0.15)     # DWELL 判定迟滞补偿（短窗 0.17s 的近似）
        self.declare_parameter("confirm_frames", 6)           # 连续见到多少帧才算目标确认
        self.declare_parameter("max_lead_px", 120.0)          # 超前量限幅(px)：模型抽风的最后防线
        self.declare_parameter("lead_sign", 1.0)              # 超前方向符号（现场反了改-1）
        self.declare_parameter("speed_match_tol", 0.30)       # 相邻拟合速度一致判据（±30%）
        self.declare_parameter("model_stable_n", 3)           # 连续 N 拍速度一致→模型可信
        self.declare_parameter("endpoints_to_trust", 4)       # 学到第几个折返点后端点可信
        self.declare_parameter("diag_period_s", 2.0)         # 状态日志限流周期
        self.declare_parameter("half_period_expect_s", 3.6)   # 期望半周期（仅诊断对照用）
        # ---- EKF 状态估计参数（★吸收 RM 自瞄做法；默认值由 aim_ekf_sim.py 扫掠定标）----
        self.declare_parameter("ekf_q_theta", 0.1)            # 角度过程噪声
        self.declare_parameter("ekf_q_omega", 200.0)          # 角速度过程噪声
        self.declare_parameter("ekf_r_theta", 0.30)           # 观测噪声方差(度²)

        p = self.get_parameter
        self.enable = bool(p("enable").value)
        self.px_per_tick = float(p("px_per_tick").value)
        self.ticks_per_deg = float(p("ticks_per_deg").value)
        self.gimbal_center_tick = float(p("gimbal_center_tick").value)
        self.gimbal_sign = float(p("gimbal_sign").value)
        self.latency_ms = float(p("latency_ms").value)
        self.fit_window_s = float(p("fit_window_s").value)
        self.fit_min_points = int(p("fit_min_points").value)
        self.stall_v_deg_s = float(p("stall_v_deg_s").value)
        self.dwell_s = float(p("dwell_s").value)
        self.confirm_frames = int(p("confirm_frames").value)
        self.max_lead_px = float(p("max_lead_px").value)
        self.lead_sign = float(p("lead_sign").value)
        self.speed_match_tol = float(p("speed_match_tol").value)
        self.model_stable_n = int(p("model_stable_n").value)
        self.endpoints_to_trust = int(p("endpoints_to_trust").value)
        self.diag_period_s = float(p("diag_period_s").value)
        self.half_period_expect_s = float(p("half_period_expect_s").value)
        self.dwell_judge_lag_s = float(p("dwell_judge_lag_s").value)

        # 派生换算
        self.px_per_deg = self.px_per_tick * self.ticks_per_deg  # 像素/度
        self.t_ahead = self.latency_ms / 1000.0

        # ---------------- ★EKF 状态估计（吸收：RM 自瞄标准做法）----------------
        # 用 EKF(匀速模型) 取代原 0.6s 长窗最小二乘做角速度/角度估计：
        #   · LS 长窗对匀速段有 ≈0.3s 等效滞后，几乎吃掉全部链路延迟预算；
        #   · EKF 递推滞后小、丢帧只预测不重置。
        # A/B（aim_ekf_sim.py，合成实测运动+噪声+丢帧）：匀速段超前落点误差
        #   14.09px → 5.09px，ω误差 4.78→3.09°/s。三段式端点/停顿逻辑完全保留。
        self.ekf = TargetEkf(
            q_theta=float(p("ekf_q_theta").value),
            q_omega=float(p("ekf_q_omega").value),
            r_theta=float(p("ekf_r_theta").value))
        self._last_obs_t = None
        self._innov = deque(maxlen=20)   # 一步预测残差(像素口径)，供 _model_ok 早期判据

        # ---------------- ROS I/O ----------------
        qos1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int32MultiArray, "offset_center",
                                 self._cb_vision, qos1)
        self.create_subscription(Int32MultiArray, "camemo_feedback",
                                 self._cb_gimbal, qos1)
        self.pub_lead_x = self.create_publisher(Float32, "/aim/lead_x", qos1)
        self.pub_fire_ready = self.create_publisher(UInt8, "/aim/fire_ready", qos1)

        # ---------------- 运行时状态 ----------------
        self.gimbal_deg = None          # 最近云台角(度)
        self.obs = deque(maxlen=240)    # (t, θ_deg) 观测环形缓冲（~8s @30Hz）
        self.seen_streak = 0            # 连续见到靶的帧数
        self.v_fit = None               # 当前拟合角速度(度/s)
        self.v_hist = deque(maxlen=10)  # 最近若干拍拟合速度（一致性判据）
        self.fit_rms = None            # 拟合残差
        self.model_ok = False          # 模型可信标志
        self.state = "INIT"            # INIT / MOVING / DWELL
        self.dwell_t0 = None           # 进入停顿的时刻
        self.dwell_pos = None          # 停顿位置(度)
        self.dwell_sign = 0            # 停顿前的运动方向(+1/-1)
        self.endpoints = {"up": None, "down": None}  # 学习到的端点(度)
        self.reversal_cnt = 0          # 已观测折返次数
        self.last_rev_t = None         # 上次折返时刻（周期诊断）
        self.lead_decay = 1.0          # 丢靶时超前量的缓释系数
        self.last_diag_t = None        # 日志限流
        self._last_lead = 0.0          # 最近一次发布的超前量（丢靶缓释用）
        self._v_last_signed = 0.0      # 上一拍拟合速度符号（折返兜底检测用）

        self.get_logger().info(
            f"预测器启动: enable={self.enable} px/tick={self.px_per_tick} "
            f"px/deg={self.px_per_deg:.2f} latency={self.latency_ms:.0f}ms "
            f"dwell先验={self.dwell_s}s 半周期先验={self.half_period_expect_s}s")

    # ---------------- 回调：云台反馈 ----------------
    def _cb_gimbal(self, msg: Int32MultiArray):
        # data[1] = Position_1（水平轴刻度）；刻度→角度（相对零位）
        if len(msg.data) >= 2:
            tick = float(msg.data[1])
            self.gimbal_deg = (tick - self.gimbal_center_tick) \
                / self.ticks_per_deg * self.gimbal_sign

    # ---------------- 回调：视觉帧（主驱动 ~30Hz） ----------------
    def _cb_vision(self, msg: Int32MultiArray):
        if not self.enable:
            return
        # 字段契约与 C++ 相同：[dx, dy, cls, box_w, box_h]，cls: 1=蓝 2=红
        if len(msg.data) < 5:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dx_px = float(msg.data[0])
        cls = int(msg.data[2])

        is_target = cls in (1, 2)
        if not is_target:
            self.seen_streak = 0
            # 丢靶缓释：超前量逐帧衰减到 0（比瞬间归零温和，云台不抽）
            self.lead_decay *= 0.7
            if self.lead_decay < 0.02:
                self.lead_decay = 0.0
                self.state = "INIT"
                self._publish(lead_px=0.0, ready=False)
            else:
                self._publish(lead_px=self._last_lead * self.lead_decay,
                              ready=False)
            return

        self.lead_decay = 1.0
        self.seen_streak += 1

        # ---- 观测合成：靶的世界角 = 云台角 + 画面偏差角 ----
        if self.gimbal_deg is None:
            return  # 云台反馈还没来过，无法合成世界角，等一拍
        dx_deg = dx_px / self.px_per_deg
        theta = self.gimbal_deg + dx_deg
        self.obs.append((now, theta))
        # 缓冲里只保留最近 ~2 个周期的数据
        while len(self.obs) > 4 and now - self.obs[0][0] > 2.0 * self.half_period_expect_s:
            self.obs.popleft()

        # ---- 在线拟合 ----
        self._fit(now)
        self._update_state(now)

        # ---- 模型外推 → 超前量 ----
        lead_px = self._predict_lead(now)
        ready = self._fire_ready_ok()

        self._publish(lead_px, ready)
        self._diag(now, lead_px, ready)

    # ---------------- 速度/位置估计（EKF，取代原长窗最小二乘） ----------------
    def _fit(self, now):
        """EKF(匀速模型) 递推角度/角速度。

        吸收自 RM 自瞄生态（rm_gimbal_controllers / rm_auto_aim 均用 EKF 做状态估计）：
        原 0.6s 长窗 LS 对匀速段有 ≈0.3s 等效滞后、丢靶 >0.15s 整段作废；
        EKF 递推滞后小、丢帧只预测不重置。v_hist 一致性判据与三段式状态机均保留。
        """
        theta = self.obs[-1][1]
        if self._last_obs_t is not None:
            self.ekf.predict(now - self._last_obs_t)
        pred_theta = self.ekf.theta if self.ekf.inited else theta
        self.ekf.update(theta)
        self._last_obs_t = now

        b = self.ekf.omega if self.ekf.inited else None
        self.v_fit = b

        # 残差诊断（一步预测 vs 实测，像素口径）——供 _model_ok 早期判据使用
        self._innov.append((theta - pred_theta) * self.px_per_deg)
        n = len(self._innov)
        self.fit_rms = math.sqrt(sum(x * x for x in self._innov) / n) if n else None

        # 速度一致性历史：只记【有效运动拍】（停顿期噪声符号不得污染外推速度）
        if b is not None and abs(b) > self.stall_v_deg_s:
            self.v_hist.append(abs(b))

    # ---------------- 状态机：MOVING / DWELL + 折返学习 ----------------
    def _short_v(self):
        """短窗斜率（最近 5 点最小二乘）—— 专用于停顿/起步判定。

        为什么不用 v_fit（0.6s 长窗）判停顿：端部停顿只有 0.2s（6 帧），
        长窗要等停顿数据占满全窗口才会跌破失速阈值 ⇒ 判定迟滞 ~0.6s，
        DWELL 永远追不上。短窗 5 帧（0.17s）让停顿判定延迟压到 ~0.17s。
        长窗继续负责匀速段的速度估计（抗噪优先）。
        """
        if len(self.obs) < 5:
            return None
        seg = list(self.obs)[-5:]
        n = 5
        t0 = seg[0][0]
        ts = [t - t0 for t, _ in seg]
        ths = [th for _, th in seg]
        st = sum(ts); sh = sum(ths)
        stt = sum(t * t for t in ts); sth = sum(ts[i] * ths[i] for i in range(n))
        det = n * stt - st * st
        if abs(det) < 1e-9:
            return None
        return (n * sth - st * sh) / det

    def _update_state(self, now):
        v = self.v_fit
        v_s = self._short_v()
        prev_state = self.state

        if v_s is None:
            return  # 观测不足，保持现状态

        if abs(v_s) < self.stall_v_deg_s:
            # 接近停顿（短窗判据，延迟 ~0.17s）
            if self.state == "MOVING":
                # 从运动跌入失速 → 到达端部
                self.state = "DWELL"
                # 停顿起点回退判定迟滞：短窗从"失速到被察觉"平均晚 ~0.15s，
                # 不回退的话 dwell_left 会被高估 0.15s、反向外推晚 0.15s
                self.dwell_t0 = now - self.dwell_judge_lag_s
                self.dwell_pos = self.obs[-1][1]
                # 停顿前运动方向：用最近一次【有效运动拍】的带符号速度
                # （v_hist 存的是绝对值，不能用于判方向 —— 2026-10-17 仿真踩坑）
                self.dwell_sign = 1 if self._v_last_signed >= 0 else -1
                # 清零带符号速度：停顿结束反向起步时，兜底的符号翻转检测
                # 不会把【已由 DWELL 路径记账的这次折返】再数一遍（双计数坑）
                self._v_last_signed = 0.0
                self._learn_endpoint(self.dwell_pos)
                if self.last_rev_t is not None:
                    half_p = now - self.last_rev_t
                    self.get_logger().info(
                        f"[折返] 端点={self.dwell_pos:.2f}° 半周期实测={half_p:.2f}s"
                        f"（先验 {self.half_period_expect_s}s）")
                self.last_rev_t = now
            # 已在 DWELL / INIT：无事
        else:
            # 有效运动速度（短窗）
            if self.state == "DWELL":
                self.state = "MOVING"  # 停顿结束反向起步
            elif self.state == "INIT":
                self.state = "MOVING"
            # 符号翻转兜底（极短停顿没触发 DWELL 就直接翻了）
            if self._v_last_signed * v_s < 0 \
                    and abs(self._v_last_signed) > self.stall_v_deg_s:
                self.reversal_cnt += 1
                if self.last_rev_t is not None:
                    half_p = now - self.last_rev_t
                    self.get_logger().info(
                        f"[折返-兜底] 半周期实测={half_p:.2f}s")
                self.last_rev_t = now
            # 只在有效运动拍更新带符号速度（失速拍的符号是噪声，不能污染）
            self._v_last_signed = v_s

    def _learn_endpoint(self, pos_deg):
        """折返点滑动平均学习端点（上/下端分开）。

        方向判定：dwell_sign = 停顿前角速度符号 —— +1（角度增大方向）
        到达的端点记为 up 端，-1 记为 down 端。
        """
        self.reversal_cnt += 1
        key = "up" if self.dwell_sign > 0 else "down"
        cur = self.endpoints[key]
        if cur is None:
            self.endpoints[key] = pos_deg
        else:
            # 滑动平均，抗单次检测噪声
            self.endpoints[key] = cur * 0.6 + pos_deg * 0.4
        self.get_logger().info(
            f"[端点学习] {key}端 → {self.endpoints[key]:.2f}° "
            f"（第 {self.reversal_cnt} 次折返，"
            f"可信={'是' if self.reversal_cnt >= self.endpoints_to_trust else '否'}）")

    # ---------------- 模型外推 ----------------
    def _predict_theta(self, now, t_ahead):
        """预测 t_ahead 秒后靶的世界角（三段式模型：匀速→端停→反向）。"""
        t_now = now
        # 当前角优先取 EKF 估计（低滞后、低噪声）；EKF 未初始化时退回最近观测
        th_now = self.ekf.theta if self.ekf.inited else self.obs[-1][1]

        if self.state == "DWELL" and self.dwell_t0 is not None:
            dwell_left = self.dwell_s - (now - self.dwell_t0)
            if dwell_left < 0:
                dwell_left = 0.0
            if t_ahead <= dwell_left:
                return self.dwell_pos
            # 停顿结束后反向匀速。速度用历史中位数（长窗 v_fit 在停顿期
            # 被运动尾巴污染成半速值，不能用 —— 2026-10-17 仿真踩坑）
            v_back = self._v_nominal()
            return self.dwell_pos - self.dwell_sign * v_back * (t_ahead - dwell_left)

        # MOVING：匀速外推
        v = self.v_fit
        if v is None:
            return th_now
        pred = th_now + v * t_ahead

        # 端点折返处理（端点已学习才启用）
        if self.reversal_cnt >= self.endpoints_to_trust:
            if v > 0:
                end = self.endpoints.get("up")
            else:
                end = self.endpoints.get("down")
            if end is not None:
                toward = (pred - end) * (1 if v > 0 else -1)
                if toward > 0:
                    # 外推越端：到达端点→停顿→反向
                    t_to_end = (end - th_now) / v
                    t_rest = t_ahead - t_to_end - self.dwell_s
                    if t_rest <= 0:
                        return end
                    return end - v * t_rest
        return pred

    def _v_nominal(self):
        """速度先验兜底：取历史拟合绝对速度的中位数。"""
        if self.v_hist:
            s = sorted(self.v_hist)
            return s[len(s) // 2]
        return 0.0

    # ---------------- 超前量输出 ----------------
    def _predict_lead(self, now):
        th_now = self.ekf.theta if self.ekf.inited else self.obs[-1][1]
        th_pred = self._predict_theta(now, self.t_ahead)
        lead_deg = th_pred - th_now
        lead_px = lead_deg * self.px_per_deg * self.lead_sign
        # 限幅：模型异常时的最后防线
        lead_px = max(-self.max_lead_px, min(self.max_lead_px, lead_px))
        self._last_lead = lead_px
        return lead_px

    # ---------------- 开火第二把钥匙 ----------------
    def _fire_ready_ok(self):
        """fire_ready 语义 = 目标已确认 + 靶的未来位置可预测。

        分状态判据（停顿窗口恰恰是最佳开火时机，绝不能被"速度一致性"
        挡掉 —— 靶停着不动时预测误差最小、最难打丢）：
          · DWELL：连续可见 + 已学到端点（dwell_pos 即实测端点，预测完全可信）
          · MOVING：连续可见 + 速度拟合连续多拍一致（匀速模型成立）
          · INIT：不放行（模型还没建立）
        """
        if self.seen_streak < self.confirm_frames:
            return False
        if self.state == "INIT":
            return False
        if self.state == "DWELL":
            # 停顿中靶不动，预测=dwell_pos；至少学过一次折返（排除偶然减速）
            return self.reversal_cnt >= 1
        return self._model_ok()

    def _model_ok(self):
        """MOVING 状态的模型可信判据：速度拟合连续多拍一致（匀速性成立）。"""
        if len(self.v_hist) < self.model_stable_n:
            return False
        recent = list(self.v_hist)[-self.model_stable_n:]
        if min(recent) < self.stall_v_deg_s * 0.5:
            return False  # 混入失速拍（加减速瞬间），匀速性不成立
        mid = sorted(recent)[len(recent) // 2]
        if mid < 1e-4:
            return False
        for v in recent:
            # 量纲：|v - mid| ≤ tol × mid（tol 为相对容差）
            if abs(v - mid) > self.speed_match_tol * mid:
                return False
        # 端点未学满前，用"拟合残差小"替代端点判据（早期也能放行，
        # 落点是否进框由 C++ 的 in_fire_zone + lead 判据兜底）
        if self.reversal_cnt < self.endpoints_to_trust:
            return self.fit_rms is not None and self.fit_rms < 6.0  # px 口径
        return True

    # ---------------- 发布 ----------------
    def _publish(self, lead_px, ready):
        m = Float32()
        m.data = float(lead_px)
        self.pub_lead_x.publish(m)
        r = UInt8()
        r.data = 1 if ready else 0
        self.pub_fire_ready.publish(r)

    # ---------------- 诊断日志（限流） ----------------
    def _diag(self, now, lead_px, ready):
        if self.last_diag_t is None or now - self.last_diag_t >= self.diag_period_s:
            self.last_diag_t = now
            v = self.v_fit if self.v_fit is not None else 0.0
            self.get_logger().info(
                f"[状态] {self.state} v={v:+.2f}°/s "
                f"v_px={abs(v) * self.px_per_deg:.0f}px/s "
                f"rms={self.fit_rms if self.fit_rms is not None else -1:.1f}px "
                f"seen={self.seen_streak} 折返={self.reversal_cnt} "
                f"端点={self.endpoints} lead={lead_px:+.0f}px ready={1 if ready else 0}")

def main(args=None):
    rclpy.init(args=args)
    node = TargetMotionPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
