# 瞄准打击提升（EKF + 提前量启用/标定 + 扫描EMA定值 + 现场症状诊断）

- 归档日期：2026-10-18
- 归档目录：`docs/成员交接/yu-binghuan/20261018-aiming-upgrade/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

现场症状（各种转/放不在中心/打不到）诊断为【参数未落实】而非算法缺陷：px_per_tick=2.0未标定(真实≈0.94→每帧只修47%偏差→放不在中心)、gyro_ff_gain=0(车一转靶就飞→微找重扫循环=各种转)、开火框实际仅±3.5px且未开两键/未施加提前量(打不到)。另发现代码declare_parameter默认值与注释/文档口径不一致(开火框6/18→实际3.5/3.5、confirm8→4、扫描±90→±160、patrol0.35→0.3)，会误导现场整定。交付：aim_param_audit.py(对ros2 param dump一条命令出体检表)+现场症状诊断与落实清单(8步优先顺序，含px_per_tick标定后环路增益翻倍须重整kp/kd的关键警告)。另有EKF/提前量/标定脚本/扫描EMA定值三批成果。均离线/未上机；C++未改。

## 文件清单

| 文件 | 大小 |
|---|---|
| `瞄准打击提升-20261018.md` | 5,320 B |
| `提前量启用与标定流程.md` | 4,968 B |
| `扫描速度与EMA定值-20261018.md` | 3,850 B |
| `现场症状诊断与落实清单-20261018.md` | 10,962 B |
| `工具可用性与可靠性审查-20261018.md` | 7,128 B |
| `target_ekf.py` | 3,539 B |
| `target_motion_predictor.py` | 24,384 B |
| `gimbal_ff_calib.py` | 6,207 B |
| `aim_param_audit.py` | 7,007 B |
| `aim_tune_once.py` | 7,760 B |
| `laser_offset_calib.py` | 5,489 B |
| `fire_commit_sim.py` | 3,845 B |
| `shot_budget_sim.py` | 2,721 B |
| `aim_ekf_sim.py` | 5,307 B |
| `aim_ff_calib_sim.py` | 3,608 B |
| `scan_ema_tuning_sim.py` | 5,197 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。