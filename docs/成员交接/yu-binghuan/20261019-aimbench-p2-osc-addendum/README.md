# 补档：P2 证伪 + 振荡滞后分解 + 修订版优化方案

- 归档日期：2026-10-19
- 归档目录：`docs/成员交接/yu-binghuan/20261019-aimbench-p2-osc-addendum/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

车掉线后的离线深挖：①P2 des 连续推进台架证伪（7.33° vs 基线 6.49°，误差速度含云台运动分量的正反馈，源码已回滚待编译）；②rail@267 基线振荡定量——周期 1.44s（0.70Hz）对应总环路滞后 0.714s，账目分解严丝合缝（注入 267ms + EKF速度收敛 dt/β=165ms + 内环 1/k_sp=125ms + 执行器 90ms + EMA 40ms），证明注入延迟只占总账 37%，其余 430ms 全是可攻击的环路动力学；③三个推论：β 是失稳区的减滞后旋钮（0.4 优有机制解释）、原 α→0.45 计划撤销（降 α 加滞后）、修订版序列 = k_sp5+β0.4 先行（一键脚本已备）→ 残余振荡则 ff→0.7 → 稳后 lead 0.1 → det thread_num 2 → P2-v2 世界系速度。含 P2 证伪轮全量数据与振荡分析脚本。

## 文件清单

| 文件 | 大小 |
|---|---|
| `deliverable_addendum_20261019/osc_analysis.py` | 1,945 B |
| `deliverable_addendum_20261019/README.md` | 3,074 B |
| `deliverable_addendum_20261019/runs/p2_desadv_rail267_214347817/report.md` | 2,067 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。