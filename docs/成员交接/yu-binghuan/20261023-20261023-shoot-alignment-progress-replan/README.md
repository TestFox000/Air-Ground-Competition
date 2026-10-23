# 靶向对齐与最新进度重规划：停靠射击格车头正对标靶

- 归档日期：2026-10-23
- 归档目录：`docs/成员交接/yu-binghuan/20261023-20261023-shoot-alignment-progress-replan/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

用户需求：标靶打击时车头正对标靶（一次到位或逐渐调整），规划基于最新进度。改动：① mission_router 停靠段终点态约束对齐到达（一次到位优先），物理不可行才降级停靠转身并 RuntimeWarning 显式告警；② 新增 route_progress.py 纯算法进度推断，integrate_node/sidecar 中途重规划换到车当前格并剔除已扫雷（修静默漏必经点+时间预算高估两个故障面）；③ 以机上最新血缘为底（本地旧副本漂移已存证 baseline/）。测试：基线 81过/15败（全为旧套件既有失配），改后 98过/同样15败零新增，新增17项专项测试全绿。无人机已部署并机上冒烟三连过（备份.bak_align_orig可回滚）；车端离线待部署。历史坑逐条对照：不手工插航点、未辨靶禁火不动、守卫签名防重规划风暴、reset不重发。

## 文件清单

| 文件 | 大小 |
|---|---|
| `20261023-靶向对齐/.pytest_cache/README.md` | 302 B |
| `20261023-靶向对齐/baseline/ag_integrate/__init__.py` | 99 B |
| `20261023-靶向对齐/baseline/ag_integrate/anchored_registration.py` | 8,871 B |
| `20261023-靶向对齐/baseline/ag_integrate/car_adapter.py` | 3,388 B |
| `20261023-靶向对齐/baseline/ag_integrate/car_commands.py` | 4,656 B |
| `20261023-靶向对齐/baseline/ag_integrate/cjk_font.py` | 1,912 B |
| `20261023-靶向对齐/baseline/ag_integrate/dzbrain_bridge.py` | 19,522 B |
| `20261023-靶向对齐/baseline/ag_integrate/evidential_map.py` | 10,408 B |
| `20261023-靶向对齐/baseline/ag_integrate/field_frame.py` | 2,656 B |
| `20261023-靶向对齐/baseline/ag_integrate/field_matrix.py` | 11,066 B |
| `20261023-靶向对齐/baseline/ag_integrate/field_rules.py` | 3,587 B |
| `20261023-靶向对齐/baseline/ag_integrate/fire_guard.py` | 3,903 B |
| `20261023-靶向对齐/baseline/ag_integrate/ground_projection.py` | 9,786 B |
| `20261023-靶向对齐/baseline/ag_integrate/heading_calibration.py` | 3,861 B |
| `20261023-靶向对齐/baseline/ag_integrate/integrate_node.py` | 50,860 B |
| `20261023-靶向对齐/baseline/ag_integrate/intel_protocol.py` | 14,395 B |
| `20261023-靶向对齐/baseline/ag_integrate/mission_router.py` | 83,392 B |
| `20261023-靶向对齐/baseline/ag_integrate/ortools_solver.py` | 14,197 B |
| `20261023-靶向对齐/baseline/ag_integrate/pipeline.py` | 19,193 B |
| `20261023-靶向对齐/baseline/ag_integrate/route_time.py` | 5,631 B |
| `20261023-靶向对齐/baseline/ag_integrate/turn_feasibility_v2.py` | 8,900 B |
| `20261023-靶向对齐/baseline/ag_integrate/video_recon.py` | 82,085 B |
| `20261023-靶向对齐/baseline/ag_integrate/world_model.py` | 9,106 B |
| `20261023-靶向对齐/baseline/tools/_router_baseline.py` | 3,921 B |
| `20261023-靶向对齐/baseline/tools/build_v2_dataset.py` | 5,154 B |
| `20261023-靶向对齐/baseline/tools/calibrate_camera.py` | 16,207 B |
| `20261023-靶向对齐/baseline/tools/capture_dataset_local.py` | 6,229 B |
| `20261023-靶向对齐/baseline/tools/car_deploy.py` | 11,825 B |
| `20261023-靶向对齐/baseline/tools/check_stack_health.py` | 10,267 B |
| `20261023-靶向对齐/baseline/tools/convert_capture_to_xanylabeling.py` | 3,014 B |
| `20261023-靶向对齐/baseline/tools/dznav_deploy.py` | 14,709 B |
| `20261023-靶向对齐/baseline/tools/factory_flight_diag.py` | 9,016 B |
| `20261023-靶向对齐/baseline/tools/fake_udp_sender.py` | 4,987 B |
| `20261023-靶向对齐/baseline/tools/flight_console.py` | 32,445 B |
| `20261023-靶向对齐/baseline/tools/hover_drift_forensics.py` | 11,452 B |
| `20261023-靶向对齐/baseline/tools/hover_report.py` | 7,631 B |
| `20261023-靶向对齐/baseline/tools/log_report.py` | 16,089 B |
| `20261023-靶向对齐/baseline/tools/matrix_report.py` | 3,401 B |
| `20261023-靶向对齐/baseline/tools/monitor_link.py` | 16,988 B |
| `20261023-靶向对齐/baseline/tools/orin_agent_watchdog.py` | 8,400 B |
| `20261023-靶向对齐/baseline/tools/orin_broadcast_route.py` | 6,497 B |
| `20261023-靶向对齐/baseline/tools/orin_capture_dataset.py` | 5,670 B |
| `20261023-靶向对齐/baseline/tools/orin_chain.py` | 13,616 B |
| `20261023-靶向对齐/baseline/tools/orin_deploy.py` | 13,902 B |
| `20261023-靶向对齐/baseline/tools/orin_flight_blackbox.py` | 21,179 B |
| `20261023-靶向对齐/baseline/tools/orin_log_capture.py` | 10,459 B |
| `20261023-靶向对齐/baseline/tools/orin_plan_sidecar.py` | 30,942 B |
| `20261023-靶向对齐/baseline/tools/orin_print_summary.py` | 3,377 B |
| `20261023-靶向对齐/baseline/tools/orin_probe_cells.py` | 2,127 B |
| `20261023-靶向对齐/baseline/tools/orin_replan_from_state.py` | 2,257 B |
| `20261023-靶向对齐/baseline/tools/orin_watchdog.py` | 21,657 B |
| `20261023-靶向对齐/baseline/tools/patch_deploy.py` | 16,240 B |
| `20261023-靶向对齐/baseline/tools/preflight.py` | 18,651 B |
| `20261023-靶向对齐/baseline/tools/release_models.py` | 8,188 B |
| `20261023-靶向对齐/baseline/tools/remote_run.py` | 18,667 B |
| `20261023-靶向对齐/baseline/tools/render_field_matrix.py` | 22,585 B |
| `20261023-靶向对齐/baseline/tools/render_referee_route.py` | 9,198 B |
| `20261023-靶向对齐/baseline/tools/render_route_plan.py` | 6,217 B |
| `20261023-靶向对齐/baseline/tools/repair_web_upload.py` | 8,268 B |
| `20261023-靶向对齐/baseline/tools/sim_match.py` | 19,631 B |
| `20261023-靶向对齐/baseline/tools/smoke_test_chain.py` | 4,611 B |
| `20261023-靶向对齐/baseline/tools/speed_envelope_report.py` | 6,610 B |
| `20261023-靶向对齐/baseline/tools/sync_progress.py` | 4,642 B |
| `20261023-靶向对齐/baseline/tools/target_speed_calib.py` | 34,419 B |
| `20261023-靶向对齐/baseline/tools/track_diag.py` | 52,968 B |
| `20261023-靶向对齐/baseline/tools/train_on_5090.py` | 5,895 B |
| `20261023-靶向对齐/baseline/tools/uav_link.py` | 114,616 B |
| `20261023-靶向对齐/baseline/tools/ulog_salvage.py` | 14,297 B |
| `20261023-靶向对齐/baseline/tools/uxrce_link_guard.py` | 7,609 B |
| `20261023-靶向对齐/deploy_align.py` | 7,105 B |
| `20261023-靶向对齐/diag_compare.py` | 2,476 B |
| `20261023-靶向对齐/README.md` | 5,402 B |
| `20261023-靶向对齐/tests/test_blackbox_health_v2.py` | 17,849 B |
| `20261023-靶向对齐/tests/test_field_config_consistency.py` | 3,984 B |
| `20261023-靶向对齐/tests/test_integrate_pipeline.py` | 6,636 B |
| `20261023-靶向对齐/tests/test_integrate_router.py` | 6,196 B |
| `20261023-靶向对齐/tests/test_jump_guard_v2.py` | 20,280 B |
| `20261023-靶向对齐/tests/test_localization_watchdog.py` | 14,493 B |
| `20261023-靶向对齐/tests/test_planning_fixes.py` | 6,158 B |
| `20261023-靶向对齐/tests/test_planning_hardening.py` | 10,566 B |
| `20261023-靶向对齐/tests/test_route_progress.py` | 3,374 B |
| `20261023-靶向对齐/tests/test_route_time.py` | 4,340 B |
| `20261023-靶向对齐/tests/test_router_fuzz.py` | 6,734 B |
| `20261023-靶向对齐/tests/test_router_robustness.py` | 3,123 B |
| `20261023-靶向对齐/tests/test_router_zones.py` | 10,925 B |
| `20261023-靶向对齐/tests/test_shoot_alignment.py` | 6,781 B |
| `20261023-靶向对齐/tests/test_video_recon.py` | 10,652 B |
| `20261023-靶向对齐/tests/test_world_model_confirm.py` | 2,009 B |
| `20261023-靶向对齐/tests/test_zoned_planning.py` | 9,416 B |
| `20261023-靶向对齐/tests/yaml_lite.py` | 2,217 B |
| `20261023-靶向对齐/work/ag_integrate/__init__.py` | 99 B |
| `20261023-靶向对齐/work/ag_integrate/anchored_registration.py` | 8,871 B |
| `20261023-靶向对齐/work/ag_integrate/car_adapter.py` | 3,388 B |
| `20261023-靶向对齐/work/ag_integrate/car_commands.py` | 4,656 B |
| `20261023-靶向对齐/work/ag_integrate/cjk_font.py` | 1,912 B |
| `20261023-靶向对齐/work/ag_integrate/dzbrain_bridge.py` | 19,522 B |
| `20261023-靶向对齐/work/ag_integrate/evidential_map.py` | 10,408 B |
| `20261023-靶向对齐/work/ag_integrate/field_frame.py` | 2,656 B |
| `20261023-靶向对齐/work/ag_integrate/field_matrix.py` | 11,066 B |
| `20261023-靶向对齐/work/ag_integrate/field_rules.py` | 3,587 B |
| `20261023-靶向对齐/work/ag_integrate/fire_guard.py` | 3,903 B |
| `20261023-靶向对齐/work/ag_integrate/ground_projection.py` | 9,786 B |
| `20261023-靶向对齐/work/ag_integrate/heading_calibration.py` | 3,861 B |
| `20261023-靶向对齐/work/ag_integrate/integrate_node.py` | 53,474 B |
| `20261023-靶向对齐/work/ag_integrate/intel_protocol.py` | 14,395 B |
| `20261023-靶向对齐/work/ag_integrate/mission_router.py` | 85,781 B |
| `20261023-靶向对齐/work/ag_integrate/ortools_solver.py` | 14,197 B |
| `20261023-靶向对齐/work/ag_integrate/pipeline.py` | 19,193 B |
| `20261023-靶向对齐/work/ag_integrate/route_progress.py` | 5,010 B |
| `20261023-靶向对齐/work/ag_integrate/route_time.py` | 5,631 B |
| `20261023-靶向对齐/work/ag_integrate/turn_feasibility_v2.py` | 8,900 B |
| `20261023-靶向对齐/work/ag_integrate/video_recon.py` | 82,085 B |
| `20261023-靶向对齐/work/ag_integrate/world_model.py` | 9,106 B |
| `20261023-靶向对齐/work/tools/orin_plan_sidecar.py` | 32,999 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。