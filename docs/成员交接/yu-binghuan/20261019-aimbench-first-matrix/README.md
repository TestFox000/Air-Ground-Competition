# AimBench 实车首跑：9 轮真值矩阵与打击精度定量基线

- 归档日期：2026-10-19
- 归档目录：`docs/成员交接/yu-binghuan/20261019-aimbench-first-matrix/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

今晚从「卡顿玄学」收口到「失稳裕度定量地图」：AimBench L2 HIL 实车首跑闭环（合成视觉注入+真云台+真值指标+台账），9 轮实验矩阵钉死三个定律级发现——①真实视觉延迟 267ms 下环路增益栈近失稳（rail 场景 RMSE 1.47°@0延迟 → 6.5°@267ms），这是「追踪不精准」的定量本质；②lead 双重补偿实锤：EKF 内建 267ms 前瞻已吸收 2/3 延迟，叠加 dzbrain lead 后滞后 455→156ms 但振幅不动，平滑律下 lead_time_s 应降至 ~0.1；③β 影响随延迟翻转（0ms 时 0.2 优、267ms 时 0.4 减半误差）。静止靶 0.18°（1.9px 正中）证明伺服本体优秀。新增 rail 真实口径场景（±25°@3.3°/s，修正原工具 12°/s 的换算错误）。下一步按杠杆排序：P0 det 延迟治理（每降 100ms 误差约减半）→ P1 k_sp/α 稳定裕度 A/B → P2 des 连续推进 → P3 lead 口径修正；laser_offset 标定并行待做。含完整 run 数据（data/meta/metrics/report/plots）与实跑版 aim_bench.py，次日复跑命令在 README。

## 文件清单

| 文件 | 大小 |
|---|---|
| `deliverable_bench_20261019/aim_bench.py` | 59,337 B |
| `deliverable_bench_20261019/bench_analyze.py` | 40,688 B |
| `deliverable_bench_20261019/benchctl.py` | 18,106 B |
| `deliverable_bench_20261019/experiments.csv` | 1,626 B |
| `deliverable_bench_20261019/README.md` | 4,787 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/data.csv` | 214,392 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/meta.json` | 2,717 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/metrics.json` | 1,783 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/plots.png` | 113,663 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/quick_metrics.json` | 319 B |
| `deliverable_bench_20261019/runs/rail_beta02_0ms_211914606/report.md` | 2,058 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/data.csv` | 240,823 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/meta.json` | 2,737 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/metrics.json` | 1,812 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/plots.png` | 139,810 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/quick_metrics.json` | 321 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_lead027_213009310/report.md` | 2,076 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/data.csv` | 219,547 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/meta.json` | 2,733 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/metrics.json` | 1,813 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/plots.png` | 144,341 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/quick_metrics.json` | 321 B |
| `deliverable_bench_20261019/runs/rail_beta02_267_nolead_213155728/report.md` | 2,078 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/data.csv` | 218,390 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/meta.json` | 2,719 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/metrics.json` | 1,788 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/plots.png` | 133,798 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/quick_metrics.json` | 320 B |
| `deliverable_bench_20261019/runs/rail_beta04_267_212444669/report.md` | 2,060 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/data.csv` | 144,653 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/meta.json` | 2,666 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/metrics.json` | 1,785 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/plots.png` | 141,300 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/quick_metrics.json` | 319 B |
| `deliverable_bench_20261019/runs/ramp_beta04_0ms_212248998/report.md` | 2,006 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/data.csv` | 182,498 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/meta.json` | 2,668 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/metrics.json` | 1,790 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/plots.png` | 143,192 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/quick_metrics.json` | 320 B |
| `deliverable_bench_20261019/runs/ramp_beta04_267_212351522/report.md` | 2,010 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/data.csv` | 160,841 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/meta.json` | 1,549 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/metrics.json` | 438 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/plots.png` | 42,275 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/quick_metrics.json` | 138 B |
| `deliverable_bench_20261019/runs/real_1019_pitchfix_baseline_205239804/report.md` | 367 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/data.csv` | 144,309 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/meta.json` | 2,656 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/metrics.json` | 1,769 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/plots.png` | 146,544 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/quick_metrics.json` | 318 B |
| `deliverable_bench_20261019/runs/smoke_ramp_211526855/report.md` | 1,995 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/data.csv` | 67,805 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/meta.json` | 2,652 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/metrics.json` | 1,662 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/plots.png` | 55,264 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/quick_metrics.json` | 319 B |
| `deliverable_bench_20261019/runs/smoke_static_211439248/report.md` | 1,935 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。