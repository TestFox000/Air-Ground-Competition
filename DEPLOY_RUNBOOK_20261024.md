# 无人机端部署运行手册（2026-10-24 · 缺雷修复 + 定位加固轮）

> 分支：`agents/zcode/drone-replan-fixes-20261024`（基线 = 机上快照 snapshot/orin-20261023 + 本轮修复）
> 本手册是本轮上机的**唯一入口**：按序执行，每步有验证门，失败即停。

## 0. 前置（一次性）

```bash
# 操作机（与 Orin 同网）：
git fetch origin && git checkout agents/zcode/drone-replan-fixes-20261024
ping <Orin IP>                       # 不通先解决链路（mDNS: dzsmart.local）
ssh duzhong@<Orin IP> 'date; date'   # 双机对时（偏差必须 <1s；历史实测偏过 ~3 天）
set AG_ORIN_PASS=******              # 部署脚本密码环境变量
```

## 1. 本轮变更清单（为什么上这趟机）

| # | 变更 | 文件 | 修复的问题 |
|---|---|---|---|
| 1 | **route_cells_of 认连写朝向后缀**（`N5,y`） | src/ag_integrate/ag_integrate/route_progress.py | 旧版抛 CommandError→静默退回摆位起点+全量雷，**"重规划基于最新进度"从未生效**（缺雷主因） |
| 2 | **replan_on_update 默认开**（integrate_node + sidecar 双线） | integrate_node.py / orin_plan_sidecar.py | 首发后识别到的新雷永不进路线 |
| 3 | ~~px4_odom_convert v2.2~~（⛔ 2026-10-30 废除,见 §5） | onboard/dzpx4_patch/（载荷留档不用） | 已定案弃用：机上为官方原版直通,dzpx4 零触碰 |
| 4 | D13 13类/17通道（**已在树内，无新增**，本轮确认） | onboard/dzuav/ | 就绪度报告 P0-1 按旧副本误报，机上源码已是 17 通道 + 形状契约校验（:72 换错引擎即抛错） |

## 2. 部署步骤

### ★ 一键入口（推荐，2026-10-24 起）

```bash
# 无人机端全链：预检(mDNS/历史IP自动解析+时钟)→四阶段→验证门 G1~G4→汇总报告
# dry-run 先看一遍：python tools/deploy_one_click.py --dry-run
python tools/deploy_one_click.py                      # IP 自动解析（mDNS→历史候选）
python tools/deploy_one_click.py --host <IP>          # 显式 IP
python tools/deploy_one_click.py --host <IP> --verify-only   # 部署后体检（只跑验证门）
# 阶段跳过：--skip code|px4|engines|bringup；回滚命令在失败报告尾部自动打印

# 车端全链（dzrobot_v2 仓库根）：probe→本地门禁→all(强制 auto_start=false)→verify→报告
python tools/deploy_car_one_click.py                  # 同款 --dry-run / --ip / --keep-autostart
```

一键做了什么（= 下面四步的自动化 + 前后置门禁）：
- 无人机：预检（SSH 探活/时钟偏差告警）→ 阶段①~④ → 静态验证门 G1~G4（缺雷修复标记/px4 进程/引擎在位/规划侧进程），任一 FAIL 给回滚命令；
- 车端：推前推后各跑一遍本地门禁（route_check 0 error / config_selfcheck / test_deploy 88/88）——**推上车的东西必须是绿的**。

#### 备选全链入口：orin_chain.py（2026-10-25 在册）

```bash
# 8 段链：探活→preflight→orin_deploy→patch_deploy→引擎转换→编译验证→bringup→健康检查+冒烟
python tools/orin_chain.py --dry-run   # 密码走 AG_ORIN_PASS 环境变量，不落盘
```

与 deploy_one_click 的分工：one_click = 四阶段+静态验证门（本轮推荐主线）；
orin_chain = 多了 patch_deploy 段、check_stack_health --kill-dup 与
smoke_test_chain 冒烟（首装/换机后值得跑全 8 段）。两者编排重叠部分的取代
关系待团队评审（2026-10-25 审计在案）；现场二选一即可，不要连跑。

### ⚠️ ft_v1 契约换装（2026-10-26 起，与本仓 dzuav 代码绑定）

C++ 契约已切 **14 类/18 通道**（types.hpp kObjectClassCount=14，含 target
整体板）。HbbDetector 构造期形状校验 `[1,18,33600]`：**17 通道 d13 引擎配
本代码会 fail-fast，侦察栈起不来**。上机编译本代码前必须同批完成：
1. `models/release/current_hbb_detector` 指针 → ft_v1 ONNX（best.onnx，
   `default\data_14_20261025\runs\...\weights\`）；
2. `orin_convert_engines.sh` 重转 `~/dzuav/engines/hbb_detector_*_fp16.engine`；
3. `tools/ml/verify_contract.py --model <ft_v1.onnx>` 通过（通道数从
   types.hpp 单点派生，18 不符会拒）。
反向回退同理：要继续用 d13 引擎，须回退到本契约变更之前的提交。
cell_classifier（v4d 格号）契约不变，无需动。

### 分步执行（fallback / 排障时用）

```bash
# ① 源码+工具链+规划包（含 ag_integrate，自动备份机上原副本）
python tools/orin_deploy.py --host <IP> --user duzhong --verify
#   验证门：远程 L1 编译 + L2 自检全过，输出含 ag_integrate 备份路径

# ② 定位补丁（px4_odom_convert v2.2：备份→推送→colcon→重启节点→验证横幅）
# ⛔ 2026-10-30 起本阶段废除：px4_odom_convert v2.2 已定案弃用（10-26/27 两次
#    飞控事故链根因=补丁 bump/重锚逻辑与 EKF 交互；机上已回退官方 7758B 原版并
#    整体对齐官方交付版）。**dzpx4 官方栈零触碰**：不运行 patch_deploy.py、不重编
#    dzpx4 下任何包、不改其 launch/config；机上 pip 严禁再动 numpy（官方栈硬依赖
#    1.26.x）。护栏需求若复活须新评审+系留验证后走新通道。
#    官方基线核查报告：桌面\无人机备份总库_20261028\10_官方nano板源码核查报告

# ③ 引擎（D13/v4d；一键版缺引擎时自动转换）
ssh duzhong@<IP> 'cd ~/dzuav_deploy/air_ground && bash tools/orin_check_engines.sh'
ssh duzhong@<IP> 'cd ~/dzuav_deploy/air_ground && bash tools/orin_convert_engines.sh'   # 仅缺时
#   验证门：~/dzuav/engines/ 下 hbb_detector_d13_fp16.engine + cell_classifier_v4d_fp16.engine 存在
#           （v4d 构建必须带 --min/opt/maxShapes 动态形状，命令在脚本内已固化）

# ④ 重启规划侧（sidecar 由 orin_bringup.sh / orin_menu.sh 管理）
ssh duzhong@<IP> 'cd ~/dzuav_deploy/air_ground && bash tools/orin_bringup.sh restart'
```

## 3. 上机验证清单（全部过才算部署完成）

- [ ] `pgrep -af orin_plan_sidecar` 存在，且日志含 `plan-mode=on-change`（重规划默认开）
- [ ] 起飞前推一架：dzuav 侦察流起来后，sidecar 日志出现 `重规划基于最新进度: 起点 <当前格>`——**这是修复 1+2 生效的直接证据**（修复前该行永远不会出现，只会退回摆位起点）
- [ ] 新雷进场（或回放含雷视频）：路线版本随态势签名变化重算（`replan` 计数 >0），新路线含该雷格
- [ ] px4 health 行 `status_age_s` 持续 <2s；拔飞控 USB 模拟断流 → 日志出现 `status=断流(按armed处理)` 且不重锚
- [ ] `orin_build_verify.sh` L3 推理自检过（D13 引擎 17 通道契约）

## 3.5 起飞前 30 秒体检（硬门，2026-10-26 起）

**每次起飞前必跑，13 项全绿（EXIT=0）才允许解锁**。本门收编自 2026-10-26 悬停失控事故教训（多实例撕扯 EKF / numpy 2.x / 墙钟 1970），随 `orin_*.sh` 通配自动上机：

```bash
# Orin 上，栈由 orin_bringup.sh up 启动并沉降 2~3 分钟后：
ssh duzhong@<IP> 'cd ~/dzuav_deploy/air_ground && bash tools/orin_preflight_check.sh'
# 退出码 0=可飞；1=有红项（每项 FAIL 行自带排查指引，按提示处理后再跑）
```

- 体检面：栈存活 / 关键节点单实例 / 定位→EKF 链路（aligned、ekf_valid、slam≥30Hz、复位风暴=0、跳变） / **EKF 复位增长**（10s 双采样 `ekf_reset_total` 零增长——不能用 `ekf_reset_60s`，那是陈旧队列假象，见 2026-10-26 冷启动实测）/ 飞控链路 / 墙钟与环境。
- `aligned=0` 且刚起栈：先等 2~3 分钟收敛再重跑，不要带红起飞。

## 4. 回滚（每步独立）

```bash
python tools/orin_deploy.py --host <IP> --user duzhong --restore    # ag_integrate 回备份
# ⛔ patch_deploy 已废除（2026-10-30）：px4_odom_convert 保持官方原版,无回滚需求
# sidecar 一次性规划回退：--plan-mode once 或 yaml replan_on_update: false
```

## 5. 车端（同轮，另一入口）

车端分支 `agents/dzhong/dzrobot-v2-20261024`；部署唯一入口 `tools/deploy_to_car.py`（见车端仓库 docs）。本轮车端变更：R005 增量契约 / 无解归因诊断 / auto_start 统一 false（等无人机 0x4003/`/nav/waypoint` 下发再出发）。

## 6. 已知事实（不要当故障修）

- 第六图 K7 桥洞贴隔板 = 结构性死锁（车端 tracker-safe 拒绝规划是**正确行为**，带归因诊断）；现役 68-token 路线 M7 转弯仿真隔板余量 1.2mm/113 拍穿渗——现场蹭板风险，处置见就绪度报告 P0。
- sidecar 与车端 integrate_node **不可同时消费** UDP:50000（双决策核心互斥）。
- v1 本地 `dzsmartscout/src/uav_camera_code/onboard/dzuav` 是 7 类旧参考副本，**勿部署**；机上以 `~/dzuav_deploy/air_ground/onboard/dzuav`（13 类）为准。

## 5. 2026-10-30 改版注记（部署口径更新）

- **部署源分支**:`agents/zcode/modularize-20261027`(完整包含部署线全部历史+模块化+走廊安全+模型候选;单线无分叉)。模块化新结构(mr_*/vr_* 扁平模块、tools/uavlink/ 包)随 orin_deploy 整目录同步上机即可,机载 sidecar 按路径直跑不受 setup.py 打包影响。
- **阶段②废除**(见上);三阶段部署:①源码+规划包 → ③引擎(cell 指针=v4f_baked,若机上引擎内容未变则跳过重转)→ ④起栈。
- **上机第一动作**:EKF2/PX4 参数全量 dump 建档(从未建档;官方交付无参数单)——`param dump > param_baseline_$(date).txt` 存档后才能动任何东西。
- **飞前硬门不变**:orin_preflight_check 13 项全绿 EXIT=0;首飞流程按事故后纪律(系留/低空悬停→复查→逐步放开)。
- 模型现役:hbb=ft_v1(候选 ft_v3b 已入库未切,切换=改指针一行+重转引擎);cell=v4f_baked。
