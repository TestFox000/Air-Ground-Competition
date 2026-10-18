# 开源方案代码级对标 + 吸收落地结果

- 归档日期：2026-10-18
- 归档目录：`docs/成员交接/yu-binghuan/20261018-open-source-benchmark/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

开源对标与吸收落地四批次成果：①代码级对标 SpireCV/Line_Follower/pb2025_sentry_nav/JunLin_navigation2025（均宽松许可），P0-1 检测召回经 4 段真机视频 A/B 被否决（22%→25% 几乎不涨，误检 2.0→6.5 倍增），真瓶颈为网格拟合成功率 50~62%；P0-2 CTE 控制律已落地（超调 -83%、末段RMS -91%）。②G1 转弯扫掠对拍（Fields2Cover 吸收）：发现规划层弧（拐点起步 35°）与执行层弧（提前 0.294m 切角 30°）系统性几何失配——45°/90° 零真漏检，135° 全组合真漏检且 F5 拐点贴 G3 蓝靶仅 1.2cm 惊险通过，错杀空格 278 处（G9 死结强候选解释）。③G2 前瞻仿真：照搬 Nav2 RPP 无效，激进短前瞻 k=0.6 clamp[0.10,0.30] 有实质收益（残差 -44%、唯一收敛）但抖动 +66%，需台架定夺。④G1-α 已实施（mission_router.py，备份 .bak_20261018_turngeo）：R_MIN_M 35°→30° 对齐执行层口径、扫掠模型换切角弧几何、180° 掉头双侧并集（执行层掉头方向是 atan2 ±0.0 浮点偶然行为）、并顺手修复存量 bug（ensure_mine_center_pass 有靶分支漏传 turn_model，雷补点新拐点扫掠检查被跳过，205411 实测 I1 拐点扫撞 K1/K2 桥墙进入路线）。验证：148 项回归全绿、56 组合真漏检 0/56、8 条真实路线 148 拐点核查撞实体 0 机会损失 0。A/B 生产口径对比（官方 8 雷基线 + 靶先验裁决 + 三级可达阶梯）：4 段全持平或更短（-2.59 ~ -6.25 格），G9 死结在 205411 解除（旧版 L1 失败退 L2 丢视频避障保守性，新版 L1 直接成功）。上机前必须过 orin_build_verify.sh（该门槛在 Orin 机载侧执行，本 Windows 开发工作区无此脚本，本次全离线验证未触机载）。⑤执行层遗留项已处理：path_executor.py start_turn 掉头侧显式定义（cross+0.0 归一，备份 .bak_20261018_uturn）——修复前同格 180° 掉头从不同方向进入转不同侧（北→南/西→东右转，其余左转，-0.0 浮点偶然），修复后 8 组恒左转；非 180° 组合零变化（手术精确）；G1 对拍重跑 180° 真漏检仍 0（规划层双侧并集互补覆盖），148 回归全绿。

## 文件清单

| 文件 | 大小 |
|---|---|
| `开源方案代码级对标-20261018.md` | 10,720 B |
| `吸收落地-G1转弯对拍与G2前瞻仿真-20261018.md` | 9,883 B |
| `turn_sweep_crosscheck.py` | 15,461 B |
| `lookahead_sim.py` | 13,752 B |
| `turn_geo_ab.py` | 8,565 B |
| `G1_转弯扫掠对拍结果.json` | 10,858 B |
| `G2_前瞻策略对比.json` | 40,060 B |
| `G1_AB_路线对比.json` | 2,026 B |
| `mission_router.py` | 93,456 B |
| `uturn_fix_verify.py` | 2,936 B |
| `path_executor.py` | 41,889 B |
| `det_recall_ab.py` | 6,310 B |
| `cte_pid_sim.py` | 3,296 B |
| `ab_summary.json` | 4,025 B |
| `cte_pid.py` | 4,175 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。