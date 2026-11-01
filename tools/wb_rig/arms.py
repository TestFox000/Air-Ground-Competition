# -*- coding: utf-8 -*-
"""双臂注入档案：`ideal` / `realistic`（卡 `WB-2-RIG-20261031`）。

★ 两档必须**同参数同场地同种子**，唯一差别就是下表的注入量。这条是本卡
  "双臂对照"成立的前提：任何一个别的键变了，diff 出来就不是"注入的代价"。

| 量 | realistic 值 | 出处 | 状态 |
|---|---|---|---|
| 里程比例虚增 | +17.9% | `tasks/localization.py:290,1014` 实测修正 2776→3280 | 文档值，待 wb-8 标定 |
| 静置 yaw 零偏 | −0.0705 °/s | 车端/04 §6；`dzactuator.cpp:591/534` | 文档值，待标定 |
| 位置白噪声 | 20 Hz 采样抖动 | 现场口径（见 `pos_noise_m`） | 文档值 |

★★ 为什么不能把它们"调成看起来漂亮"的值：注入量是**被控对象的属性**，
  调它等于调被控对象。台账 D-026 子约束「未校准扫参禁止」正是这条。
  所以本模块的值要么带出处，要么显式标 `PROVISIONAL`。

★ `ideal` 档的准确定义：**全部注入恒为 0**，于是信念 ≡ 真值（逐位相同的强不变量，
  不是"差很小"）。这条由 `--arm ideal` 的 `belief_error_rms_m == 0` 断言钉住，
  同时也钉住"注入确实没生效 vs 生效"这个最容易假绿的地方。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmProfile:
    """一档注入参数。字段名与 `tools/sim.py::SimConfig` 同名处**语义相同**。

    ★ `anchor_on_arrival` / `yaw_trim_deg` 也算在"注入"里，理由是**同一件事**：
      它们都是"让信念偏离真值"的来源。前者是外部把信念拽到格心（`FixTask` 的 L2 修正），
      后者是实车标定出来的横向漂移补偿（`robot.yaml localization.yaw_trim_deg=-0.6`）。
      `sim.py` 头注把前者明确列为"零误差不变量的唯一例外"，所以 `ideal` 档里
      **两者都必须关掉**，否则 `belief_error_rms_m ≡ 0` 这条判据永远不可能成立，
      而原因与"注入链有没有偷改物理"毫无关系 —— 那会把一条好判据变成噪声源。
    """

    name: str
    #: 里程计比例虚增（0.179 = 里程计报得比真实多 17.9%）
    speed_scale_err: float
    #: 陀螺积分零偏（°/s；配置与文档习惯是 °/min，换算在 `SimConfig` 之外）
    yaw_drift_dps: float
    #: 位置白噪声（米/拍，20 Hz 重算）
    pos_noise_m: float
    #: 到位吸附（模拟 `FixTask.anchor_at_waypoint` 的 L2 格心修正）
    anchor_on_arrival: bool
    #: 信念 yaw 的横向漂移补偿（度；`None` = **沿用配置现场值**）
    yaw_trim_deg: float | None
    #: 说明（打在横幅与契约行里，让人一眼看懂"这一档假设了什么"）
    note: str

    def sim_kwargs(self) -> dict:
        """转成 `tools/sim.py::SimConfig` 可直接消费的 kwargs。"""
        return {
            "speed_scale_err": self.speed_scale_err,
            "yaw_drift_deg_per_min": self.yaw_drift_dps * 60.0,
            "pos_noise_m": self.pos_noise_m,
            "anchor_on_arrival": self.anchor_on_arrival,
        }


#: ★ ideal = 零注入 + 关掉两条"外部修正"。**不许**在这里"顺手加一点噪声让曲线更真实"——
#:   这档的全部价值就是"能证明注入链本身没在偷偷改物理"。
IDEAL = ArmProfile(
    name="ideal",
    speed_scale_err=0.0,
    yaw_drift_dps=0.0,
    pos_noise_m=0.0,
    anchor_on_arrival=False,
    yaw_trim_deg=0.0,
    note="零注入：信念≡真值（逐位同拍相同）。到位吸附与 yaw_trim 一并关闭 —— "
         "两者都是「外部让信念偏离真值」的来源，不是被控对象特性。",
)

#: ★ realistic = **现场全量**：实车量级注入 + 现场配置的修正项（吸附 ON / trim −0.6°）。
REALISTIC = ArmProfile(
    name="realistic",
    speed_scale_err=0.179,
    #: −0.0705 °/s = −4.23 °/min，静止时车头"自己"在转的速率（陀螺零偏）
    yaw_drift_dps=-0.0705,
    #: 位置白噪声：20 Hz 重算时 3σ ≈ 2 cm 量级（够把吸附/漂移闸的门限搅动起来，
    #: 又不至于把车甩出走廊 —— 后者是"模型崩了"不是"噪声大"）
    pos_noise_m=0.006,
    anchor_on_arrival=True,
    yaw_trim_deg=None,       # 沿用 `config/robot.yaml` 的现场值（-0.6）
    note="实车量级：里程 +17.9% / 陀螺 −0.0705 °/s / 位置白噪声 6 mm(σ) / "
         "到位吸附 ON / yaw_trim 取现场 −0.6°。"
         "①②为文档值待 wb-8 标定；③为现场口径估值。",
)

ARMS: dict[str, ArmProfile] = {"ideal": IDEAL, "realistic": REALISTIC}

#: 未经实车标定、只有文档/推导来源的量（UI 须标黄，8 号卡负责回填）
PROVISIONAL: tuple[str, ...] = ("speed_scale_err", "yaw_drift_dps", "pos_noise_m")


def get(name: str) -> ArmProfile:
    """名字 → 档案。未知名字抛 `KeyError`（CLI 转成退出码 2）。"""
    key = str(name or "").strip()
    if key not in ARMS:
        raise KeyError(f"未知注入档 {name!r}；可选 {sorted(ARMS)}")
    return ARMS[key]
