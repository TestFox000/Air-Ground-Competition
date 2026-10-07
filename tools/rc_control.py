#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""浏览器遥控器的服务端（跑在 cam_viewer 进程内）。

=============================================================================
  为什么要"接管仲裁"，而不是直接发指令
-----------------------------------------------------------------------------
  车上同时有两个自动发布者，频率都比遥控高：
    · path_executor   以 20Hz 持续发 pursuitAngle（追求角）
    · dzrknn_dataprocess 以 50~100Hz 持续发 CameMo_Conl（云台位置）

  遥控器如果直接往这两个话题发，下位机拿到的会是【互相打架】的指令 ——
  车一顿一顿地抽、云台来回甩。所以遥控必须先"接管"：

      接管(on)  ->  path_executor 与云台节点收到 /nav/manual_override(1)
                    它们停止下发自动指令，遥控独占控制权
      释放(off) ->  两边恢复自动下发，接力无缝

  这两个开关是在节点里新加的（见 path_executor.py 的 _GatedCmdPub
  与 dzrknn_dataprocess.cpp 的 manual_override_）。

=============================================================================
  四层失效保护（任何一层单独失效都不会失控）
-----------------------------------------------------------------------------
  ① 按住才动    前端是 press-and-hold，松开立刻发停车；不存在"点一下就冲出去"
  ② 数值钳制    速度/转向/云台刻度都夹在安全包络内，与自动控制共用同一套限位
  ③ 接管看门狗  本模块每 0.5s 检查：接管中且 2 秒没收到遥控指令
                -> 自动刹车并释放接管（防浏览器崩溃/网络断/人跑了）
  ④ 底盘看门狗  dzactuator 自身有 1.0s 无指令强制停车（既有机制，最底层兜底）
"""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Int32MultiArray, UInt8, UInt8MultiArray


class RemoteControl:
    """遥控器的服务端：发布指令 + 接管仲裁 + 失效保护。"""

    # ---------- 安全包络（与自动控制共用同一套硬限位）----------
    CENTER = 2047            # 云台中心刻度
    TPD = 11.378             # 刻度/度
    # 水平：由 scan_yaw_min/max_deg = ±90° 推出
    YAW_MIN, YAW_MAX = 1022, 3071
    # 俯仰：config 里的 gimbal_pitch_tick_min/max
    PITCH_MIN, PITCH_MAX = 950, 2400

    MAX_SPEED = 0.30         # m/s 速度硬上限（巡航 0.15，留一倍余量）
    MAX_STEER = 40.0         # 度 转向硬上限（下位机再夹到 [25,95] 字节）
    GIMBAL_SPEED = 1400      # 遥控云台速度（比扫描 1295 略快，手感跟手）

    WATCHDOG_S = 2.0         # 遥控静默多久自动释放接管
    SHOOT_COOLDOWN_S = 0.8   # 两发之间的最小间隔
    LASER_ON_S = 3.0         # 开火时激光常亮持续秒数（脉冲本身肉眼不可见，靠这个确认）

    def __init__(self, node: Node, pitch_offset_deg: float = -26.0):
        self.node = node
        self._lock = threading.Lock()

        self._override = False       # 是否处于接管状态
        self._estopped = False       # ★急停粘性标志（看门狗不得自动解除）
        self._last_cmd_t = 0.0       # 最后一次收到遥控指令的时间
        self._last_shoot = 0.0
        self._shots = 0              # 遥控开火计数（本会话）
        self._laser_timer = None     # 激光自动熄灭定时器

        # 云台当前目标刻度：初值 = 基准俯仰 + 正前方，与自动模式起步姿态一致
        self._pitch = self._clamp_pitch(self.CENTER + pitch_offset_deg * self.TPD)
        self._yaw = self.CENTER

        # best_effort + depth1，与 dzactuator 的订阅者 QoS 对齐
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_drive = node.create_publisher(Twist, "pursuitAngle", qos)
        self.pub_gimbal = node.create_publisher(Int32MultiArray, "CameMo_Conl", qos)
        self.pub_shoot = node.create_publisher(UInt8, "LaserShot_Command", 10)
        # 激光【常亮】开关：走底盘下行帧 buf[7] 的 LaserGunSwitch
        self.pub_laser = node.create_publisher(UInt8, "/laser_switch", 10)
        # 射击计数重置（把云台节点的 shots_fired_ 归零，等于"弹药加满"）
        self.pub_shoot_reset = node.create_publisher(UInt8, "/shoot_reset", 10)
        # 裁判系统【通用指令】：原样下发任意协议帧（开启/复位等未知指令靠它试验）
        self.pub_referee = node.create_publisher(UInt8MultiArray, "/referee_cmd", 10)

        # 裁判系统状态回传：HpAndHitmsg = [敌方HP, 己方HP, 被击打方位, 累计射击数, 剩余弹药]
        self._referee = {"targethp": None, "self_hp": None, "hit": None,
                         "shots": None, "ammo": None, "ts": 0.0}
        node.create_subscription(UInt8MultiArray, "/HpAndHitmsg", self._on_referee, 10)
        # 接管开关
        self.pub_ovr_car = node.create_publisher(UInt8, "/nav/manual_override", 10)
        self.pub_ovr_gim = node.create_publisher(UInt8, "/nav/manual_gimbal", 10)

        # 镜像云台的实际指令位置：接管时以它为起点，云台不会"跳"一下
        node.create_subscription(Int32MultiArray, "CameMo_Conl", self._on_gimbal, qos)

        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    # ==================================================================
    #  内部工具
    # ==================================================================
    @classmethod
    def _clamp_pitch(cls, t) -> int:
        return int(max(cls.PITCH_MIN, min(cls.PITCH_MAX, round(t))))

    @classmethod
    def _clamp_yaw(cls, t) -> int:
        return int(max(cls.YAW_MIN, min(cls.YAW_MAX, round(t))))

    def _on_gimbal(self, msg) -> None:
        """镜像云台指令位置。

        ★ 只在【未接管】时更新 —— 接管期间这条话题上的消息就是我们自己发的，
          再镜像回来虽然没有数值错误，但会让"起点"随回声漂移，不利于排查。
        """
        if len(msg.data) < 2:
            return
        with self._lock:
            if not self._override:
                self._pitch = self._clamp_pitch(msg.data[0])
                self._yaw = self._clamp_yaw(msg.data[1])

    def _touch(self) -> None:
        self._last_cmd_t = time.time()

    def _publish_stop(self, times: int = 3) -> None:
        """发零 Twist 刹车。多发几次是因为下位机订阅是 best_effort，可能丢。"""
        z = Twist()
        for _ in range(times):
            self.pub_drive.publish(z)

    # ==================================================================
    #  接管 / 释放
    # ==================================================================
    def takeover(self, on: bool) -> dict:
        on = bool(on)
        with self._lock:
            if on == self._override:
                if on:
                    self._touch()
                return {"ok": True, "override": self._override, "note": "状态未变"}

            self._override = on
            self.pub_ovr_car.publish(UInt8(data=1 if on else 0))
            self.pub_ovr_gim.publish(UInt8(data=1 if on else 0))

            if on:
                # 接管瞬间先刹一脚，避免循迹最后一条速度指令继续生效
                self._publish_stop()
                self._touch()
                self.node.get_logger().warn(
                    "★ 遥控接管：循迹与云台自动下发已暂停（%s）" % self._src)
            else:
                # ★释放是解除急停的唯一途径
                was_estop = self._estopped
                self._estopped = False
                self._publish_stop()
                self.node.get_logger().warn(
                    "★ 遥控释放：恢复自动控制" + ("（同时解除急停）" if was_estop else ""))

        return {"ok": True, "override": self._override,
                "estopped": self._estopped}

    _src = "浏览器"

    # ==================================================================
    #  小车
    # ==================================================================
    def drive(self, v: float, w: float) -> dict:
        with self._lock:
            if self._estopped:
                return {"ok": False, "error": "急停中，请先「解除急停」再操作"}
            if not self._override:
                return {"ok": False, "error": "未接管，拒绝下发速度指令"}
            self._touch()
            v = max(-self.MAX_SPEED, min(self.MAX_SPEED, float(v)))
            w = max(-self.MAX_STEER, min(self.MAX_STEER, float(w)))
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.pub_drive.publish(cmd)
        return {"ok": True, "v": v, "w": w}

    # ==================================================================
    #  云台
    # ==================================================================
    def gimbal(self, pitch=None, yaw=None) -> dict:
        """设置云台绝对刻度（None = 该轴不变）。"""
        with self._lock:
            p = self._pitch if pitch is None else float(pitch)
            y = self._yaw if yaw is None else float(yaw)
            return self.gimbal_raw(p, y)

    def gimbal_step(self, dp: float = 0.0, dy: float = 0.0) -> dict:
        """按增量挪云台（前端只用发"往上挪 N 刻度"，不必维护绝对位置）。

        ★ 为什么用增量而不是让前端算绝对位置：
          绝对位置要求前端持有云台当前值，一旦页面刷新/多标签页/丢包就会失同步，
          表现为"一按就跳到奇怪的角度"。增量由服务端累加，天然免疫。
        """
        with self._lock:
            return self.gimbal_raw(self._pitch + dp, self._yaw + dy)

    def gimbal_raw(self, pitch, yaw) -> dict:
        """已持有锁的内部实现（避免 gimbal_step -> gimbal 二次加锁死锁）。"""
        if self._estopped:
            return {"ok": False, "error": "急停中，请先「解除急停」再操作"}
        if not self._override:
            return {"ok": False, "error": "未接管，拒绝下发云台指令"}
        self._touch()
        self._pitch = self._clamp_pitch(pitch)
        self._yaw = self._clamp_yaw(yaw)
        msg = Int32MultiArray()
        msg.data = [self._pitch, self._yaw, self.GIMBAL_SPEED, self.GIMBAL_SPEED]
        self.pub_gimbal.publish(msg)
        return {"ok": True, "pitch": self._pitch, "yaw": self._yaw}

    def gimbal_home(self) -> dict:
        """云台回正（正前方 + 基准俯仰）。

        ★ 注意：这里不能自己拿锁再调 gimbal() ——
          threading.Lock 不可重入，会把自己锁死。gimbal() 内部已经拿锁了。
        """
        return self.gimbal(pitch=self.CENTER + self._home_deg * self.TPD,
                           yaw=self.CENTER)

    _home_deg = -26.0

    # ==================================================================
    #  开火
    # ==================================================================
    # ==================================================================
    #  激光【常亮】开关
    # ==================================================================
    #  ★ 为什么需要它：
    #    LaserShot_Command 走的是判罚板的【单次脉冲】帧 B5 5B 03 21 01 25，
    #    脉宽极短（0.3~0.5ms），肉眼和 30fps 相机都捕捉不到 —— 所以按了开火
    #    "什么反应都看不到"。真正能让激光亮起来的是【底盘下行帧 buf[7]】的
    #    LaserGunSwitch（struct 第 4 个字段），那是常亮使能位；
    #    原来被硬编码成 0x00，全项目从未赋值，所以激光永远打不开。
    def laser(self, on: bool, seconds: float = 0.0) -> dict:
        """on=True 点亮；seconds>0 则在 N 秒后自动熄灭。"""
        with self._lock:
            if self._estopped:
                return {"ok": False, "error": "急停中"}
            self._set_laser(on)
            if on and seconds > 0:
                self._arm_laser_off(seconds)
        return {"ok": True, "laser": bool(on)}

    def _on_referee(self, msg) -> None:
        """裁判系统状态回传（judgment 节点 1Hz 上报 + 0x07 帧实时上报）"""
        d = list(msg.data)
        if len(d) < 5:
            return
        self._referee = {
            "targethp": d[0],
            "self_hp": d[1],
            "hit": d[2],          # 0x10 = 未被击打
            "shots": d[3],        # 硬件侧累计射击次数
            "ammo": d[4],         # 剩余弹药
            "ts": time.time(),
        }

    def referee_state(self) -> dict:
        """给网页用的裁判状态（含 'stale' 标志：超过 3s 没更新 = 裁判未开启/未连接）"""
        r = dict(self._referee)
        r["stale"] = (time.time() - self._referee.get("ts", 0.0)) > 3.0
        return r

    def reset_shots(self) -> dict:
        """重置射击计数（软件侧），让 10 发弹药重新可用。"""
        with self._lock:
            self.pub_shoot_reset.publish(UInt8(data=1))
            self._shots = 0
            self.node.get_logger().warn("★ 射击计数已重置（shots_fired_=0，弹药恢复满额）")
        return {"ok": True}

    def referee_cmd(self, hexstr: str) -> dict:
        """下发任意裁判协议帧。hexstr 形如 'B5 5B 03 21 01 25'。"""
        try:
            data = [int(x, 16) for x in hexstr.replace(",", " ").split()]
        except ValueError:
            return {"ok": False, "error": "十六进制格式不对，例如: B5 5B 03 21 01 25"}
        if not data:
            return {"ok": False, "error": "空帧"}
        if any(b < 0 or b > 255 for b in data):
            return {"ok": False, "error": "字节超出 0~FF"}
        m = UInt8MultiArray()
        m.data = data
        self.pub_referee.publish(m)
        self.node.get_logger().warn("★ 下发裁判帧: %s" % " ".join("%02X" % b for b in data))
        return {"ok": True, "bytes": data}

    def _set_laser(self, on: bool) -> None:
        self.pub_laser.publish(UInt8(data=1 if on else 0))
        self.node.get_logger().warn("★ 激光常亮 -> %s" % ("开" if on else "关"))

    def _arm_laser_off(self, seconds: float) -> None:
        if self._laser_timer is not None:
            self._laser_timer.cancel()
        t = threading.Timer(seconds, self._laser_off_cb)
        t.daemon = True
        self._laser_timer = t
        t.start()

    def _laser_off_cb(self) -> None:
        with self._lock:
            if not self._estopped:
                self._set_laser(False)

    def shoot(self) -> dict:
        with self._lock:
            if self._estopped:
                return {"ok": False, "error": "急停中，请先「解除急停」再操作"}
            if not self._override:
                return {"ok": False, "error": "未接管，拒绝开火"}
            now = time.time()
            wait = self.SHOOT_COOLDOWN_S - (now - self._last_shoot)
            if wait > 0:
                return {"ok": False, "error": "冷却中，还需 %.1fs" % wait}
            self._last_shoot = now
            self._shots += 1
            self.pub_shoot.publish(UInt8(data=1))          # 判罚板脉冲（真实击发，肉眼不可见）
            self._set_laser(True)                          # ★ 同时点亮常亮，让人眼能看到激光
            self._arm_laser_off(self.LASER_ON_S)
            self.node.get_logger().warn(
                "★ 遥控开火（本会话第 %d 发）+ 激光常亮 %.1fs" % (self._shots, self.LASER_ON_S))
        return {"ok": True, "shots": self._shots}

    # ==================================================================
    #  紧急停止 —— 任何时候都生效
    # ==================================================================
    def estop(self) -> dict:
        """全部静默 + 刹车，并【粘住】停止状态。

        ★ 为什么是粘性的（这是踩过的一个真坑）：
          最初实现只做了"静默 + 刹车"，没有把急停记成独立状态 ——
          于是 2 秒后接管看门狗判定"遥控静默"就把接管释放了，
          自动模式立刻接管，车按原路线继续跑。
          那根本不是"急停"，而是"停了 2 秒"。
          现在 _estopped 一旦置位：
            · 看门狗不会释放接管
            · 持续补发零速度，车保持刹车
            · 只有显式「解除急停/释放控制」才会恢复
          代价：浏览器崩了会一直停着 —— 但"停着"本来就是最安全的失败方向。
        """
        with self._lock:
            if not self._override:
                self._override = True
                self.pub_ovr_car.publish(UInt8(data=1))
                self.pub_ovr_gim.publish(UInt8(data=1))
            self._estopped = True
            self._publish_stop(times=6)
            self._touch()
            self.node.get_logger().error(
                "★★ 紧急停止：已静默全部自动指令并刹车（粘性，需显式解除）")
        return {"ok": True, "override": True, "estopped": True}

    # ==================================================================
    #  看门狗
    # ==================================================================
    def _watchdog_loop(self) -> None:
        """接管中若遥控静默超时 -> 自动刹车并释放。

        防的是：浏览器崩溃 / 网络断 / 人走开忘了释放 ——
        这三种情况都会让车停在"最后一条速度指令"上继续跑。
        （即使这一层也失效，dzactuator 自己的 1s 看门狗仍会兜底刹停。）
        """
        while True:
            time.sleep(0.5)
            try:
                with self._lock:
                    if not self._override:
                        continue
                    # ★急停状态优先：保持刹车、绝不自动释放
                    if self._estopped:
                        self._publish_stop(times=2)
                        continue
                    if (time.time() - self._last_cmd_t) <= self.WATCHDOG_S:
                        continue
                    self._publish_stop(times=3)
                    self._override = False
                    self.pub_ovr_car.publish(UInt8(data=0))
                    self.pub_ovr_gim.publish(UInt8(data=0))
                self.node.get_logger().error(
                    "★ 遥控静默超过 %.0fs —— 已自动刹车并释放接管" % self.WATCHDOG_S)
            except Exception:
                pass      # 看门狗自己绝不能把主进程带崩

    # ==================================================================
    #  状态
    # ==================================================================
    def state(self) -> dict:
        with self._lock:
            return {
                "ok": True,
                "override": self._override,
                "estopped": self._estopped,
                "pitch": self._pitch,
                "yaw": self._yaw,
                "shots": self._shots,
                "limits": {
                    "max_speed": self.MAX_SPEED,
                    "max_steer": self.MAX_STEER,
                    "pitch_min": self.PITCH_MIN,
                    "pitch_max": self.PITCH_MAX,
                    "yaw_min": self.YAW_MIN,
                    "yaw_max": self.YAW_MAX,
                    "center": self.CENTER,
                    "tpd": self.TPD,
                },
            }
