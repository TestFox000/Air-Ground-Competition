# -*- coding: utf-8 -*-
"""空地联合仿真（t11）：无人机天眼模型 × 车侧循迹仿真（dzbrain 只读 import）。

## 本包定位

把无人机建模为**移动情报源**：N8 起飞 → 与小车同向的 U 形航线（含 K 列直飞侦察腿）
→ M2 降落；每拍揭示以自身为中心半径 2.5 格（0.875 m）的圆形区域；情报按 t2 感知管线
协议 JSON（``bridge_passable`` / ``target_zone`` / ``field_verify``，dzlink 契约）经
0.06 s 量级延迟与可配丢包注入小车；小车侧采纳门与 t6 ``BridgeIntel`` 对齐
（缺/过期/离线 → 回落 ``route.cross_lower_default=K10 / cross_upper_default=K5``）。

车侧动力学**原样复用** ``dzbrain.tools.sim.run_sim``（只读 import，不写 dzrobot_v2 任何文件）；
无人机航线/感知/链路/采纳门/信息领先指标由本包建模。

## 写入域（交接包纪律）

- 本包目录（``dzsmartscout/tools/sim_airground/``）
- ``_tmp_session/sim_ag/``（每次运行的台账/轨迹/路线图）
- ``docs/空地联合仿真设计.md``（设计报告）

禁止：写 ``dzrobot_v2/src`` 与 ``config`` 落盘值、改 dzsmartscout 既有 tools、真机、git 提交。

## 判据口径

- 车完赛指标六项（跑完/时长/RMS/MAX/超调/越界）+ 吸附 + 物理在环两口径（盖雷几何
  ``mine_pressed_count`` vs 车体足迹 ``obstacle_hit_steps``/``board_overlap_steps``）
  全部取自 ``run_sim`` 契约指标（``docs/P0接口约定.md`` §5）。
- 信息领先契约：关键格揭示时刻（含链路延迟）vs 小车到达时刻；情报缺口拍数/缺口格数。
- 改道次数：相对"无情报基线"的**执行路线版本更换次数**（采纳点语义见 ``car_gate.py``）。
"""

__version__ = "1.0.0"
