# -*- coding: utf-8 -*-
"""执行器链（卡 `WB-2-RIG-20261031` 四项齐）：指令 → **底盘实际执行量**。

模型的唯一真源是 `src/dzactuator/src/dzactuator.cpp`（下位机本尊），不是"我们的想象"。
逐行对应关系见下表；改动任何一项都要回去核 `dzactuator.cpp` 的行号。

| 环节 | 真源行为 | 本模块实现 |
|---|---|---|
| ① 速度平滑 | `smoothValueAdaptive(speed_history_, v, smooth_window_speed)`（`dzactuator.cpp:149`），`smooth_window_speed=20`（`:1129`） | `_adaptive_window()` —— **自适应窗**：速度绝对值 <0.3 → 20 拍、<0.5 → 25、<1.0 → 30、否则 35；缓存上限 `window+15`；取最近 `temp_window` 个的算术均值 |
| ② 速度量化 | `TargetSpeed = abs(clamp(v*23/0.33, -255, 255))`（`:151`），`TargetSpeed` 是 **int**（`dzactuator.h:163`） | `Actuator._apply()` —— 截断成整数计数；执行速度反算 `计数*0.33/23`（1 计数 ≈ 0.014348 m/s），符号走 `SpeedDirection`（`:165-170`） |
| ③ 舵机关节 | `TargetAngle = w + 60`（`:152`）再夹 `[25,95]`（`:156-162`） | `servo_raw = clamp(度+60, 25, 95)` ⇒ 机械舵角 ∈ `[-35,+35]`（配置 `max_steer_deg=30` 更严，两道都在） |
| ④ 舵角平滑 | `smoothValue(angle_history_, TargetAngle, smooth_window_angle)`（`:388`），`smooth_window_angle=5`（`:1130`） | `_angle_hist` —— 5 拍定窗算术均值（**对 servo 值**平滑，与真源同序） |
| ⑤ 发帧取整 | `angle = static_cast<int16_t>(TargetAngle * 10)`（`:396`） | `servo_final = round(servo_smoothed / 0.1) * 0.1` —— 0.1° 量化 |
| ⑥ 无指令看门狗 | 底盘 1 s 无指令强制停（`io/contract.py:186` 注释 + 现场口径） | `watchdog_s=1.0` —— 超时**速度**归零、舵角保持（真源只停速度） |

★★ 为什么必须建模这一层（而不是"直接拿 cmd 积分"）：
   `bench --drive` 就是直接拿 cmd 积分，于是"负速度被吃掉"这类现场案例永远测不出来
   （`tracker.py` 里 H8 卡死那段注释写明：底盘平滑窗 ~0.4 s，旧"推一把"只推 2 拍，
   负速度还没填满平滑窗车就继续前冲 0.44 m）。本卡的 16 条案例里有这样一整类。

★ 刻意**不建**（仓内零实测，建了就是拟合噪声；卡面 §注入默认档「不建」行）：
   舵机回差/死区、电压-速度曲线、轮胎滑移。上行帧里没有电流，这三项没有可标定的观测量。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
#  真源常量（全部来自 dzactuator.cpp / dzactuator.h；改这里必须同时核 C++ 行号）
# --------------------------------------------------------------------------- #
#: 速度平滑基础窗（拍）；`dzactuator.cpp:1129 declare_parameter("smooth_window_speed", 20)`
SMOOTH_WINDOW_SPEED = 20
#: 速度平滑缓存上限的**额外**长度：`max_window = window + 15`（`dzactuator.cpp:340`）
SMOOTH_MAX_EXTRA = 15
#: 速度量化的换算系数：`TargetSpeed = abs(clamp(v*23/0.33, -255, 255))`（`:151`）
SPEED_COUNTS_PER_MPS = 23.0 / 0.33
#: 量化上限（真源 `clamp(..., -255, 255)` 后再取绝对值）
SPEED_COUNT_CAP = 255.0
#: 舵机零点偏移：`TargetAngle += 60`（`:152`）
SERVO_ZERO_DEG = 60.0
#: 舵机行程夹取（`:156-162`）⇒ 机械舵角 ∈ [-35, +35]
SERVO_MIN, SERVO_MAX = 25.0, 95.0
#: 舵角平滑窗（`smooth_window_angle = 5`；`dzactuator.cpp:1130` + `:388`）
SMOOTH_WINDOW_ANGLE = 5
#: 发帧舵角量化 0.1°（`angle = static_cast<int16_t>(TargetAngle * 10)`；`:396`）
SERVO_QUANT_DEG = 0.1
#: 无指令看门狗（s）：底盘 1 s 无指令强制停（`io/contract.py:186` 现场口径）
WATCHDOG_S = 1.0
#: 指令→执行延迟（s）：链路 + 下位机解析。与 `tools/sim.py::SimConfig.cmd_latency_s` 同源同值
DEFAULT_LATENCY_S = 0.06

#: 自适应平滑窗的分档（`dzactuator.cpp:345-352`）：(|v| 上界, 附加拍数)
_ADAPTIVE_BANDS: tuple[tuple[float, int], ...] = ((0.3, 0), (0.5, 5), (1.0, 10))


def _adaptive_window(speed_abs: float, window: int) -> int:
    """`smoothValueAdaptive` 的动态窗口：速度越快窗越长（抑制加速冲击）。

    逐字照抄 `dzactuator.cpp:344-352`（含 `<` 的严格边界与"最后 else 取 +15"）。
    """
    for bound, extra in _ADAPTIVE_BANDS:
        if speed_abs < bound:
            return window + extra
    return window + SMOOTH_MAX_EXTRA


@dataclass
class ActuatorState:
    """底盘执行侧状态（**全部**是"已经下发/已发出帧"的量，不是控制器的意图）。"""

    #: 路过延迟队列后**到达底盘**的指令（本拍真正进入执行链的那一条）
    cmd_v_in: float = 0.0
    cmd_steer_in: float = 0.0
    #: 平滑后的速度（量化前的浮点）
    v_smoothed: float = 0.0
    #: 量化后的整数计数（**真源里 TargetSpeed 就是这个类型**；0 = 死区内、车不动）
    speed_counts: int = 0
    #: 符号（`SpeedDirection`：0=正/零，1=负）
    speed_negative: bool = False
    #: 实际执行速度（计数反算 + 符号；这是**唯一**进积分的速度）
    v_exec: float = 0.0
    #: servo 值（未平滑/已平滑/已量化各一份，便于定位是哪一环吃掉的变化）
    servo_raw: float = SERVO_ZERO_DEG
    servo_smoothed: float = SERVO_ZERO_DEG
    servo_final: float = SERVO_ZERO_DEG
    #: 实际执行舵角（度；= servo_final − 60）
    steer_exec: float = 0.0
    #: 距上一条指令的时长（s）；≥ `watchdog_s` ⇒ 本拍被判"无指令"
    cmd_age_s: float = 0.0
    watchdog_tripped: bool = False
    #: 死区标记：|cmd| 非零但量化成 0 计数 ⇒ "发过指令，车不动"（H8 类案例的直接观测量）
    dead_zone: bool = False


@dataclass
class Actuator:
    """执行器链状态机（**无业务决策**：只把指令变换成可执行量）。"""

    smooth_window_speed: int = SMOOTH_WINDOW_SPEED
    smooth_window_angle: int = SMOOTH_WINDOW_ANGLE
    latency_s: float = DEFAULT_LATENCY_S
    watchdog_s: float = WATCHDOG_S
    #: 平滑窗为 0 ⇒ **关闭速度平滑**（负例 1 用：验证"H8 类案例不复现"说明注入真起作用）
    enable_speed_smooth: bool = True
    _speed_hist: deque = field(default_factory=lambda: deque(maxlen=SMOOTH_WINDOW_SPEED
                                                             + SMOOTH_MAX_EXTRA))
    _angle_hist: deque = field(default_factory=lambda: deque(maxlen=SMOOTH_WINDOW_ANGLE))
    _queue: list = field(default_factory=list)     # [(due_t, v, steer)]
    _last_cmd_t: float | None = None
    state: ActuatorState = field(default_factory=ActuatorState)
    #: 累计观测量（口径在 `metrics.transfer` 里分级）
    steps: int = 0
    dead_zone_steps: int = 0
    watchdog_steps: int = 0

    # ------------------------------------------------------------------ #
    #  输入
    # ------------------------------------------------------------------ #
    def command(self, now: float, v: float, steer_deg: float) -> None:
        """收一条已下发指令（= 上一条 `bridge.commands`，真机就是 DDS 到帧那一刻）。"""
        self._last_cmd_t = float(now)
        if self.latency_s <= 0.0:
            self._apply(now, v, steer_deg)
        else:
            self._queue.append((float(now) + float(self.latency_s), float(v), float(steer_deg)))

    def tick(self, now: float) -> ActuatorState:
        """推进一拍：到期的指令进链 → 平滑 → 量化 → 夹舵 → 看门狗判定。"""
        self.steps += 1
        # ---- ① 延迟队列：队列空时**不**回零（保持上一条已生效指令，与 sim.py 同语义） ----
        while self._queue and self._queue[0][0] <= now:
            _, v_q, s_q = self._queue.pop(0)
            self._apply(now, v_q, s_q)
        age = float("inf") if self._last_cmd_t is None else (now - self._last_cmd_t)
        st = self.state
        st.cmd_age_s = age
        # ---- ② 看门狗：超过 `watchdog_s` 没有新指令 ⇒ 速度强制归零（舵角保持） ----
        #   ★ 真源只停**速度**：`dzactuator` 的 1 s 兜底是"没有新 Twist 就 TargetSpeed=0"，
        #     舵角保持最后一条（机械上也不会自己回中）。所以这里不动 `steer_exec`。
        tripped = (self.watchdog_s > 0.0 and age >= self.watchdog_s)
        st.watchdog_tripped = tripped
        if tripped:
            self.watchdog_steps += 1
            st.v_exec = 0.0
            st.speed_counts = 0
        # ---- ③ 死区判定放在看门狗**之后**：被看门狗归零的那一拍不是"死区"，
        #     而是"根本没指令"。两者混在一处会让 H8 类案例的归因错到量化环节上。
        st.dead_zone = (not tripped) and st.speed_counts == 0 and abs(st.cmd_v_in) > 1e-12
        if st.dead_zone:
            self.dead_zone_steps += 1
        return st

    # ------------------------------------------------------------------ #
    #  内部：四项变换（顺序与真源调用序一致）
    # ------------------------------------------------------------------ #
    def _apply(self, now: float, v: float, steer_deg: float) -> None:
        st = self.state
        st.cmd_v_in, st.cmd_steer_in = float(v), float(steer_deg)

        # ---- ① 速度平滑（`smoothValueAdaptive`）----
        if self.enable_speed_smooth and self.smooth_window_speed > 0:
            self._speed_hist.append(float(v))
            # 缓存上限 window+15（真源 `while(history.size() > max_window) pop_front()`）
            cap = self.smooth_window_speed + SMOOTH_MAX_EXTRA
            while len(self._speed_hist) > cap:
                self._speed_hist.popleft()
            win = _adaptive_window(abs(float(v)), self.smooth_window_speed)
            tail = list(self._speed_hist)[-win:]
            st.v_smoothed = sum(tail) / len(tail) if tail else float(v)
        else:
            st.v_smoothed = float(v)

        # ---- ② 速度量化（截断成整数计数；符号单独走 SpeedDirection）----
        counts_f = min(abs(st.v_smoothed * SPEED_COUNTS_PER_MPS), SPEED_COUNT_CAP)
        st.speed_counts = int(counts_f)                      # ← 截断，不是四舍五入
        st.speed_negative = st.v_smoothed < 0.0
        v_exec = st.speed_counts / SPEED_COUNTS_PER_MPS
        st.v_exec = -v_exec if st.speed_negative else v_exec

        # ---- ③ 舵机关节：servo = 度 + 60，夹 [25, 95] ----
        st.servo_raw = max(SERVO_MIN, min(SERVO_MAX, float(steer_deg) + SERVO_ZERO_DEG))
        # ---- ④ 舵角平滑（窗口 5，作用在 servo 值上 —— 与真源同序）----
        self._angle_hist.append(st.servo_raw)
        while len(self._angle_hist) > max(1, self.smooth_window_angle):
            self._angle_hist.popleft()
        st.servo_smoothed = sum(self._angle_hist) / len(self._angle_hist)
        # ---- ⑤ 发帧量化 0.1°（`int16_t(TargetAngle * 10)`）----
        st.servo_final = round(st.servo_smoothed / SERVO_QUANT_DEG) * SERVO_QUANT_DEG
        st.steer_exec = st.servo_final - SERVO_ZERO_DEG

    # ------------------------------------------------------------------ #
    #  观测
    # ------------------------------------------------------------------ #
    def reset_histories(self) -> None:
        """清平滑历史（换臂/换场景时用；不清的话上一档的尾巴会污染下一档首秒）。"""
        self._speed_hist.clear()
        self._angle_hist.clear()
        self._queue.clear()
        self._last_cmd_t = None
        self.state = ActuatorState()


def steer_limits_deg() -> tuple[float, float]:
    """由 servo 行程反推的**机械**舵角可达区间（度）。配置限幅比它更严时会先被配置夹住。"""
    return (SERVO_MIN - SERVO_ZERO_DEG, SERVO_MAX - SERVO_ZERO_DEG)


def describe() -> str:
    """一行横幅文字（契约行与 `--pretty` 共用，保证人读与机器读同一份口径）。"""
    lo, hi = steer_limits_deg()
    return (f"执行器链：速度平滑窗 {SMOOTH_WINDOW_SPEED} 拍(自适应+0/5/10/15) "
            f"舵角平滑窗 {SMOOTH_WINDOW_ANGLE} 拍 | 量化 {SPEED_COUNTS_PER_MPS:.2f} 计数/(m/s) "
            f"截断≤{int(SPEED_COUNT_CAP)} → {1.0/SPEED_COUNTS_PER_MPS:.6f} m/s/计数 | "
            f"servo=度{SERVO_ZERO_DEG:+.0f} 夹[{SERVO_MIN:.0f},{SERVO_MAX:.0f}] ⇒ 舵角∈[{lo:+.0f},{hi:+.0f}]° | "
            f"看门狗 {WATCHDOG_S:.1f}s | 延迟 {DEFAULT_LATENCY_S:.2f}s")


__all__ = [
    "Actuator", "ActuatorState", "SMOOTH_WINDOW_SPEED", "SMOOTH_WINDOW_ANGLE",
    "SPEED_COUNTS_PER_MPS", "SPEED_COUNT_CAP", "SERVO_ZERO_DEG", "SERVO_MIN", "SERVO_MAX",
    "SERVO_QUANT_DEG", "WATCHDOG_S", "DEFAULT_LATENCY_S", "steer_limits_deg", "describe",
    "math",
]
