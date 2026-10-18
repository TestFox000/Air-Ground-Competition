# 无人机上机包 · 2026-10-17

> 无人机 Orin（`192.168.55.1`，SSH 用户 `duzhong`）一次性上机全部资产。
> 全部部署器遵循铁律：备份→推送→编译→重启→验证，失败可回滚（§五）。
> 包内为资产快照；本机执行部署时按 §三 命令从原位置运行（§六 注意绝对路径）。

## 一、包内清单

| 分节 | 内容 | 用途 |
|---|---|---|
| `01_定位栈v2.1/` | px4_odom_convert v2.1（三级防护+防双实例+健康行）+ localization_watchdog + patch_deploy / check_stack_health / uxrce_link_guard / factory_flight_diag / remote_run + 3 测试 | 韧性定位栈部署/体检/回滚 |
| `02_D13与v4d模型链/` | hbb_d13.onnx（38MB）+ cell_classifier_v4d ONNX（4.3MB）+ deploy_d13_board / verify_d13_l3 / l3b_with_v4d / l3b_field_frame + L3B_RUNBOOK.md | D13 13类检测引擎 + v4d 格号引擎 + L3b 闭环 |
| `03_机载代码与工具/` | air_ground 子集：onboard/dzuav（13类四件套）、src/ag_integrate（含 paper 落格判语义 + **新映射三模块**）、orin 工具链 10 件（remote_run / orin_deploy / orin_build_verify 等） | 机载代码推送与工具链 |
| `04_备份_视频侦察/` | orin_recon2.py + v4d ONNX（相对路径已对齐） | 视频侦察备份方案（无人机不便起飞时用） |
| `docs/` | 无人机代码体系改动说明-20261017.md | px4_odom_convert 三代演进/部署运维交接 |

**新映射三模块**（2026-10-17 晚实现，30 项专项测试全绿、全量回归 345 通过）：
`anchored_registration.py`（锚定配准 RANSAC+Kabsch，11% 格号误读零污染）、
`ground_projection.py`（检测框底边中心反投影落格+位姿插值）、
`evidential_map.py`（正/负证据格地图，禁火类粘性、雷负证据减半）。
随 ag_integrate 整包预置上机；**Orin 集成节点（订阅 SLAM 位姿+D13 检测流）尚未实现**，本次为库预置，不影响现有链路。

## 二、前置

```powershell
set AG_ORIN_PASS=duzhong
```

- 当前机上为**出厂代码**（10-17 曾部署 v2.1 后被替换），按步骤 0-2 恢复。
- 激光雷达 MID360 配置**已更新好**（机上），步骤 3 验证链路即可。

## 三、上机顺序（0→7，逐步判据过关再进下一步）

**0 · 链路复位**：飞控**断电 5 秒重上电**（整机重启无效），然后：
```powershell
python C:\Users\29212\Documents\Trae-CN\default\guard_v2_deploy_20261017\tools\uxrce_link_guard.py --host 192.168.55.1 --user duzhong
```
判据：`LINK_OK`；`FC_SIDE_STUCK` → 再做一次飞控断电。

**1 · 定位栈体检（30 秒）**：
```powershell
cd C:\Users\29212\Documents\Trae-CN\default\guard_v2_deploy_20261017
python tools\check_stack_health.py --host 192.168.55.1 --user duzhong --kill-dup
```
判据：livox/fastlio/odom/agent 各 1 进程、VIO/SLAM 发布者数=1、EKF xy_reset 10s 增长≤2、无 DUPLICATE/INSANE/STORM。定位栈未跑时加 `--start-cmd` 拉起并复检。

**2 · 定位栈 v2.1 确认/部署**：体检横幅已报 `Resilience layer v2` → 跳过；否则：
```powershell
python tools\patch_deploy.py --host 192.168.55.1 --user duzhong
```
判据：部署后自动体检全绿。

**3 · 雷达与 VIO 自检（起飞前铁律）**：
```powershell
python tools\remote_run.py --host 192.168.55.1 --user duzhong --cmd "ping -c 3 192.168.3.1 | tail -2; timeout 12 ros2 topic hz /fmu/in/vehicle_visual_odometry"
```
判据：雷达 0 丢包 + VIO **约 43Hz**。**无输出禁止起飞。**

**4 · 机载代码推送**：
```powershell
python C:\Users\29212\Documents\kimi\tasks\2026-10-14\18-59-27-5bee8702\air_ground\tools\orin_deploy.py --host 192.168.55.1 --user duzhong
```

**5 · D13 引擎（幂等）**：
```powershell
cd C:\Users\29212\Documents\Trae-CN\default\dk_train_pack_20261016
python deploy_d13_board.py
```
判据：`~/dzuav/engines/hbb_detector_d13_fp16.engine` 在位 + md5 一致 + L2 自检通过。

**6 · v4d 引擎 + L3b 闭环**：
```powershell
python l3b_with_v4d.py
```
前置 P0 与失败分支见 `L3B_RUNBOOK.md`。判据：`[done] L3 全链通过（v4d 格号引擎生效）`。

**7 · 悬停验证**：判据 **0.5m 悬停 10 秒位移 <0.3m、无突跳、高度稳定**。
不过关：复查 check_stack_health + uxrce_link_guard，再考虑 `patch_deploy.py --rollback` 排除我方代码因素。

**8 · live 读数修复·终局方案（2026-10-18，dzuav 带实时 HUD 替代第三方 live 程序）**：
现场物理核对定案：H9 显示 I6、H8 显示 G6、H7 显示 J7——无恒定偏移，
"+5 解码错位"假设作废；根因=第三方 live 程序预处理不符 v4d 契约致
**形近字符混淆**（本地实测：正确契约 106/106；ImageNet 归一化 69/106；
+无TTA 31/106 含读作 I6/G6/J7 的同值错例）。
**终局方案**：我方 dzuav（识别 106/106 全对）已加 live 同款 HUD
（main.cpp：相机模式 DISPLAY 存在即开 dets/cells/localized/fps + 画框 +
格号标注；DZUAV_VIEW=1/0 强制开/关；无头自动退回纯 stdout）——
VNC 画面体验不变，识别却是正确的。一键部署：
```powershell
python C:\Users\29212\Documents\Trae-CN\default\fix_live_hud.py --host 192.168.55.1 --user duzhong --deploy
```
自动完成：备份 main.cpp→推送→编译→二进制同步→停第三方 live 程序
（其启动命令存 ~/live_prog_backup.txt 可回滚）→tmux 窗口 dzlive 以
DISPLAY=:0 起 dzuav camera→验证 HUD 出现。判据：VNC 见 "dzuav live"
窗口，对准 H7/H8/H9 读数正确且 localized=1（--verify 自动统计）。
回滚：`--rollback`（恢复 main.cpp+原样重启第三方 live 程序）。
历史方案状态：移位引擎（fix_live_decode.py --deploy）作废；其 --probe
（探第三方程序真身）与 fix_localized_gate.py 的门槛加固仍备查。

## 四、备份链路（非必须）

无人机不便起飞时：`04_备份_视频侦察\orin_recon2.py 视频.mp4 --host 192.168.55.1`（用机上现役引擎，与比赛链路数值一致，端到端约 30-40 秒/187 帧）。

## 五、回滚

| 项 | 方法 |
|---|---|
| 定位栈 v2.1 | `patch_deploy.py --host ... --rollback` |
| D13/v4d 引擎 | 板端 `~/dzuav/backup_20261015/` 完整备份 `cp -a` 回滚 |
| 出厂代码 | 机上 `.bak_patch_*` 备份可回退 |

## 六、注意事项

1. `deploy_d13_board.py` / `l3b_with_v4d.py` 脚本头硬编码 kimi 主仓库与 dk_train_pack 绝对路径，**必须在本机原位置运行**；换机需改脚本头或 `set PYTHONPATH=<本包>\03_机载代码与工具\air_ground\tools`。
2. 双实例防线：patch_deploy 部署后自动体检 + check_stack_health `--kill-dup`；起飞前必须体检。
3. src/ag_integrate 内 anchored_registration / ground_projection / evidential_map 三模块为无人机端映射库（预置），车端大脑也用同一包源码仓库，互不影响。
