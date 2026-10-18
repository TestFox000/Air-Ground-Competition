# 瞄准打击提升（EKF + 提前量启用/标定 + 扫描EMA定值 + 现场症状诊断）

- 归档日期：2026-10-18
- 归档目录：`docs/成员交接/yu-binghuan/20261018-aiming-upgrade/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

瞄准打击全链提升 + 车上实测逐项复核。车上实测(17:41 dump 92键+robot.yaml+日志)证实：车端为增量式PID版本(track_mode=pid, pid_kp=10/1/1.5)，与本地副本不同——px_per_tick 口径不适用，等价换算为修正比例=pid_kp/31.9(现31%,稳态滞后≈13.8px)；267ms链路延迟下 pid_kp 上限≈10~12 勿追死拍，滞后用 lead 消。P0：max_shots=1(试射遗留,打满即永久禁火须重启5_gimbal)。已确认未开：两键/lead(dzbrain /aim/lead_x 已发布,链路就绪)/gyro_ff。laser_offset=0 仍须标定(发10/中0隐藏根因)。车端开火框14/26合理(>机械抖动12px)。本会话3处建议被车端数据修正：px_per_tick口径、死拍、开火框6/18→14/26。aim_param_audit.py 已适配车端pid口径。另有EKF/提前量/标定脚本/扫描EMA定值/工具审查成果。均离线验证，上机件须过 orin_build_verify.sh。

## 文件清单

| 文件 | 大小 |
|---|---|
| `瞄准打击提升-20261018.md` | 5,320 B |
| `提前量启用与标定流程.md` | 4,968 B |
| `扫描速度与EMA定值-20261018.md` | 3,850 B |
| `现场症状诊断与落实清单-20261018.md` | 10,962 B |
| `工具可用性与可靠性审查-20261018.md` | 7,128 B |
| `车上实测复核-20261018.md` | 6,937 B |
| `车上实测_20261018/dump_aim_live.yaml` | 2,446 B |
| `车上实测_20261018/healthcheck.sh` | 22,044 B |
| `车上实测_20261018/robot_yaml_aim_section.yaml` | 17,573 B |
| `车上实测_20261018/robot_yaml_dzactuator.yaml` | 4,609 B |
| `车上实测_20261018/体检报告_20261018_1741.md` | 8,783 B |
| `target_ekf.py` | 3,539 B |
| `target_motion_predictor.py` | 24,384 B |
| `gimbal_ff_calib.py` | 6,207 B |
| `aim_param_audit.py` | 8,062 B |
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