> ⚠️ **已过时（2026-10-01 归档）**：本文写于官方框架重构（commit `c5120b8`）之前，
> 描述的是已删除的旧自研模块（ag_bridge / ag_car 等 8 个包、car.py、JSON UDP、
> 单应矩阵 H 标定等），**不能作为当前开发依据**。当前架构见
> [README.md](../../README.md) 与
> [官方框架重构方案-20261001.md](../官方框架重构方案-20261001.md)。
> 本文保留仅作历史与排障参考，文中链接可能已失效。

# 上场前检查清单

按时间倒序排列。带 `[ ]` 的项现场逐条勾，任何一条不过就不要上电出发。

## -1. 让车先动起来（初赛选拔的最小路径）

```bash
python car.py restore <车IP>                                  # ① 官方栈（编译 dzactuator）
python car.py move    <车IP>                                  # ② 只验链路：车不动，但能看到完整路线
python car.py move    <车IP> --really-move --run-s 60          # ③ ★真发车★（有人在场、遥控器在手）
python car.py stop    <车IP>                                  # ④ 急停（任何时候都可跑）
python car.py logs    <车IP>                                  # ⑤ 取回日志出复盘
python tools/traj_report.py logs/car/ag_logs/run/traj.txt     # ⑥ ★必须做★ 看车是怎么走的
```

> ⚠️ **⑥ 不能省**：`car.py move` 只报一个「位移」数，而**本项目的路线终点 N11 就在起点
> N10 旁边**——跑完全程的净位移本来也就 0.35 m。光看位移既分不出"跑完了"，
> 也分不出"一直在原地画圆"。2026-09-30 就是靠 ⑥ 发现：位移 1.741 m 看着像动了，
> 实际是**走了 12.68 m 却只净移动 1.74 m、净转角 +936°、转向零次反转**——
> 车在半径 0.78 m 的圈里画了 2.6 圈，一段路线都没跑。
> 判据怎么看、三种根因怎么分，见 `tools/traj_report.py` 末尾的说明。

**车端满分 85 分里，光有车能跑只能拿 10 分**（驶离基地 5 + 回终点 5）：
排雷 40 分、桥洞 20 分、打击 15 分都要**无人机侦察**（桥洞标识在桥顶，车看不见）。

- [ ] 官方栈就绪：`car.py restore` 跑完五个包都 `[OK]`
      （`car_localization` / `dzactuator` / `dzjudgment` / `det_node` / `dzrknn_dataprocess`）
      —— 2026-09-28 之前这里一直是残缺的，原因与修法见
      [`docs/官方环境修复记录-20260928.md`](docs/官方环境修复记录-20260928.md)
      （根因：`~/dzrobot` 下 `src` 与 `src_4` 包名重复，colcon 拒绝构建）
- [ ] 先 `car.py move` 不带 `--really-move` 跑一遍，确认 `/car/route_status` 里的路线正确
- [ ] **SWC（通道 7）拨到低端** —— 这是「车辆 遥控 ↔ 串口程序驱动」模式开关，
      **车不动的头号原因**
      固件出处（`DZVCU程序/stm32code20260602/DZCONTROL/control.c`）：
        - 第 106 行 `operationMode = 1;` ← **默认就是遥控模式**，所以上电不动
        - 第 144 行 `Remote_CH[6] > 1500` → `operationMode = 1`（航模遥控模式，第 404 行分支）
        - 第 147 行 `Remote_CH[6] <  500` → `operationMode = 0`（**ROS 串口控制模式**，第 426 行分支）
      通道映射（`Remote_CH[i]` ↔ 通道 `i+1`）：
        - ch6 / **SWB** = `Remote_CH[5]` → `camera_mode`，**云台**那一路（遥控器左半边）
        - ch7 / **SWC** = `Remote_CH[6]` → `operationMode`，**车辆**那一路（遥控器右半边）
      ⚠️ **关掉遥控器 ≠ 切到串口模式**：关机后接收机无信号，`Remote_CH[6]` 没有有效值，
      底盘照样不进串口分支（2026-09-29 实测过，别再试）
      —— 要人工接管时把 SWC 拨回高端即可
- [ ] 真发车前：人在车边 + 车周围 2 m 净空（或架起）
- [ ] 收工：`car.py stop` → 取日志 → `tools/car_cleanup.sh`

## 0. 车端接口（已实测，见 `docs/实机接口对照.md`）

2026-09-27 已把车端摸清，**不再是未知项**：

> 车端 IP 会随 DHCP 变（当天从 `10.15.166.188` 变成 `172.23.116.188`）。
> 连不上先看自己笔记本在哪个网段，排查法见 `docs/实机接口对照.md` §9。
> 登录凭据出处：`docs/raw_txt/05_VNC远程桌面使用方法.txt`（用户名 `duzhong`）。

| 问题 | 结论 |
|---|---|
| 车端 ROS 版本 | **ROS2 Humble**（`/opt/ros/humble`），不需要 ROS1 bridge |
| 底盘怎么驱动 | 渡众 `dzactuator`（收 `cmd_vel`/`pursuitAngle`），**不要自己写底盘协议** |
| 怎么让车去某个格 | 向 `/nav/waypoint` 发路线字符串，见下 |
| 裁判怎么读/怎么开火 | 官方 `dzjudgment` 节点；收 `/HpAndHitmsg`，发 `/LaserShot_Command` |
| 云台/瞄准 | 官方 `dzrknn_dataprocess` 已实现，别抢 |
| 相机 | `usb_cam` → `/image_raw`（640×480 yuyv） |
| 场地网格 | **14 列 A~N × 11 行 1~11**，车起点 **N10** |

**每局开赛的固定动作**（`car_node` 自动做，手动调试时照抄）：

```bash
source /opt/ros/humble/setup.bash && source ~/dzrobot/install/setup.bash

# 1) 官方栈（裁判 + 底盘 + 相机 + 视觉）
ros2 launch dzactuator bringup_all.launch.py
ros2 launch dzactuator judgment.launch.py
ros2 launch dzactuator camera.launch.py
ros2 launch dzactuator det_node.launch.py
ros2 launch dzactuator dzrknn_dataprocess.launch.py

# 2) 车端决策（本项目的对接节点）：先 dry_run 看路线对不对
ros2 launch ag_bringup car.launch.py
ros2 topic echo /car/route_status        # 核对路线与情报

# 3) 确认无误后放开：car_node 参数 dry_run:=false
```

仍然需要现场量/确认的 6 件事，列在 `docs/实机接口对照.md` §8
（车终点格号、摆放朝向、桥洞具体占格、打击朝向、命中计数换算、激光偏置标定）。

**一局跑完的固定动作**（黑匣子日志默认就开着，见 `docs/调试日志.md`）：

```bash
python run.py car-fetch <车IP>          # 取回日志 + 出复盘报告（结论/指标/时间轴）
python run.py log --dir logs/car --out logs/car/report-第N局.md   # 归档这一局的证据
```

- [ ] 起步前 `export AG_LOG_DIR=/tmp/ag_logs`（车端；不设也行，会落到系统临时目录）
- [ ] 跑完**立刻**取回日志：车端 `/tmp` 重启即清，别等
- [ ] 报告里 `严重` 必须为 0；有 `注意` 就当场看清楚再跑下一局
- [ ] 车端干跑（不动车）先过一遍：`python run.py car-rehearse <车IP>`
- [ ] **相机单独验过**（凡是用视觉的联调都必须先做）：
      `python tools/remote_run.py --host <车IP> --user duzhong --script tools/car_camera_check.sh`
      —— 官方 `usb_cam` 实测会以 `exit code -6` 崩掉，崩了之后 `det_node` 只会无限刷
      `waiting for /image_raw`，**没有任何一处报错**，现场表现就是"什么都检测不到"

- [ ] 每局开始前发过 `reset:<起点>,<朝向>`（车是被搬过去的，里程计不知道自己在哪）
- [ ] `car_node` 的 `dry_run` 已按需要设置；上场前必须是 `false`
- [ ] 车终点格号已量取并写入 `params.yaml` 的 `goal_cell`

### 0.1.1 打击链（弹药只有 10 发，规则最紧的一环）

| 车端常量 | 值 | 我们必须做的事 |
|---|---|---|
| `SHOOT_COUNT_TARGET` | 10 | 路线里**只允许一个 `*` 停靠点**（`max_shoot_stops: 1`） |
| `SHOOT_WAIT_TIMEOUT` | 150 s | `shoot_wait_budget_s` 超预算就放弃打击保终点 |
| `AIM_MAX_SHOTS` | 10 | 打光就没有第二处了 |

- [ ] 用假情报演练过：**目标是红面（友方）时也必须发车**，只是路线里不带 `*`
      （可用 `tools/car_dryrun_test.sh` 改 `target_face` 试）
- [ ] `car_shoot_wait_timeout_s` 与车端 `SHOOT_WAIT_TIMEOUT` 一致（改了车端要同步）
- [ ] 确认过 `/shoot_count` 有数（`ros2 topic echo /shoot_count`），否则 `*` 会等满 150 s
- [ ] 若要把等待改短（例如按"2 命中=1 计数"改成 6 发），改车端
      `path_executor.py` 的 `SHOOT_COUNT_TARGET` **并同步本项参数**

## 0.2 规则红线（违反直接判负/取消资格，来自规则原文）

| 条款 | 出处 | 我们要注意什么 |
|---|---|---|
| 禁止公网通信 | 02 文件「参赛通信要求」 | 链路只能走局域网 |
| **允许**自带路由器 / 数传 / 蓝牙 / 图传 / Lora | 同上 | UDP 局域网方案合规 |
| **禁止**自行架设干扰性网络，**含笔记本电脑热点** | R2 | 现场必须用**自带路由器**，不能用笔记本开热点当 AP |
| 候场区不得进行无线调试、不得干扰比赛通信 | 流程 | 联调要在候场区外做完 |
| 禁止身着红/蓝色系衣物入场 | R12 | 会被当成靶标干扰视觉；队员着装注意 |
| 不允许开启闪光灯或强光 | R11 | 现场别用强光补光 |
| 激光光斑：1 m 处直径 ≤ 0.6 cm | 赛前检测 | 超标禁止上场，提前量 |
| 允许离线训练，**禁止在线推理** | 设备要求 | 模型必须跑在板端，不能云端推理 |
| 比赛中不允许人工干预 | 比赛阶段 | 全程自主，遥控器只能用于紧急接管 |

## 0.3 场地要点（决定格号与规划）

| 项 | 规则值 | 代码位置 |
|---|---|---|
| 场地尺寸 / 单元 | 4 m × 5 m，单元 35×35 cm | `FieldGrid(cols=14, rows=11, cell_size=0.35)`（A~N × 1~11） |
| 地雷 | 裁判随机选 **8 个方格**，直径 12 cm 图片置单元中央 | `max_mines=8` |
| 敌我识别区 | 随机生成 **相连的两个**横向或纵向单元，单局仅 1 个 | `target_cells` 取 2 格 |
| 障碍 | 随机 **8 个**，装满水的彩色带盖水桶，置单元中央 | `max_obstacles=8` |
| 桥洞 | 两处，各「5 选 1 / 6 选 1」随机指定可通行通道 | `bridge_a` / `bridge_b` |
| **桥洞通行标识在桥梁顶部** | 地面车视角看不到，**必须由无人机识别** | `intel_fusion_node` 必须在机载端 |
| 场地中间有黄色挡板 + 白色单面胶长条 | 会阻挡小车穿行 | 由无人机识别后作为 `obstacles` 下发 |

## 0.1 评分核对（已逐条比对规则原文，代码无误）

| 评分项 | 规则 | 代码 |
|---|---|---|
| 驶离基地 | 5 | `LEAVE_START = 5` |
| 回到终点 | 5 | `GOAL = 5` |
| 排除地雷 | 8 颗 × 5 | `MINE = 5`，`max_mines=8` |
| 桥洞通行 | 2 × 10 | `BRIDGE_A/B = 10` |
| 打击敌方 | 计数每 +1 得 5（2 次命中=1 计数，3 计数满） | `hits_per_count=2, max_count=3, TARGET=15` |
| 误击友方 | 计数每 +1 扣 5 | `expected_score = 5*敌 - 5*友` |
| 弹药 | 10 发 | `ammo_total=10` |
| 限时 | 3 分钟 | `budget_s=180` |

车端满分 **85**（另有无人机起飞 5 + 降落 10，全场 100）。

## 0.1 车端部署（⚠️ 车是公用的，规则如下）

- [ ] 只用 `/tmp/ag_ws` 编译，**不往家目录写东西**
- [ ] `~/dzrobot`、`~/src_save_backup_*`、`~/backup` 全程只读
- [ ] 收工前跑 `tools/car_cleanup.sh`，并核对输出里的体积/文件数/指纹与会话开始时一致
      （基线：`~/dzrobot` 2.8G/28367 文件、`path_executor.py` md5 `6d31d068c1`）
- [ ] 确认家目录顶层没有 `ag_*` / `car_*` 残留

## A. 赛前（有电脑就能做，不需要车）

- [ ] **`python run.py preflight` 通过**（自动查：单应是否已标定、dry_run 是否已改 false、
      广播地址是否还是默认值、模型路径是否为空、代码是否已提交）。
      剩下的"只能现场量"的项它列成提示给你，不自动判。
- [ ] `python run.py check` 环境体检通过
- [ ] `python -m pytest tests -q` 全绿（517 项）
- [ ] 模型契约：`python tools/ml/verify_contract.py --model <模型>` 通过（输出必须是 `[1,9,8400]`）
      —— 车端 5 个 .rknn 里只有 `light_det.rknn`(=light_det3) 符合，另 3 个是旧版多输出，别指错
- [ ] **别用 SSH/服务方式启动 det_node**：它内部 `cv::imshow`，无 `DISPLAY` 会一帧就崩
      （报 `Can't initialize GTK backend`）。用 VNC 桌面启动，或先 `export DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority`
- [ ] 车端干跑通过：`tools/car_dryrun_test.sh`，路线正确且 8 秒内 `/nav/waypoint` 无消息
- [ ] 契约校验通过：`tools/car_xcheck_constants.sh`（18 项全一致，防止车端被改而我们不知道）
- [ ] `python run.py demo` 跑出 `[PASS]`，且预估总分为 85/85
      （低于 85 说明有雷没排进计划，见 README 第 5 节的时间/速度参数）
- [ ] `python tools/sim_selfcheck.py` 输出 `[PASS]`，且预估得分符合预期（默认参数下应为 85）
  - 若排雷数 < 8：调 `--speed`（更保守反而更慢）或 `--budget` / `--margin`
  - 若出现误报格号：调高 `--min-votes` / `--ratio`
- [ ] 模型解码参数已用 `tools/verify_decode.py` 在真机校准，写入 `config/params.yaml`
  - `cls_activation`：`cls` 输出是否已在 [0,1]
  - `kpt_unit`：关键点量级是像素还是网格
  - `input_layout`：RKNN 导出常为 `nhwc`
- [ ] `input_topic` 与相机实际发布的话题一致（先 `ros2 topic list` 确认）
- [ ] 单应矩阵已标定：`tools/calib_homography.py` 残差 < 5 cm，9 个数填入 `params.yaml`

## B. 赛前联调（车 + 无人机 + 裁判板）

- [ ] 车端与机载端在同一局域网，能互相 ping 通（禁止走公网）
- [ ] `params.yaml` 里 `peer_ip` 改成实际网段广播地址（不是 `255.255.255.255`）
- [ ] 机载端：`ros2 launch ag_bringup air.launch.py model_path:=...`
  - `/perception/detections` 有数据：`ros2 topic hz /perception/detections`
  - `/air/field_intel` 有输出，且 `mines` 格号与目视一致
- [ ] 车端：`ros2 launch ag_bringup ground.launch.py`
  - `/link/field_intel` 有数据
  - `/link/link_stats` 里 `bad_crc` 长期为 0（不为 0 说明同频干扰，换信道/端口）
  - `/mission/plan` 非空且 `time_s <= 180`
- [ ] 底盘：`chassis_node` 能消费 `/mission/waypoints`，走完一段发 `/mission/task_done`
  - 未对接底盘协议时用 `simulate:=true` 先验证逻辑，再实现 `chassis_link._send_path`
  - 实机注意把 `time_scale` 改回 1.0（仿真快进倍速会让车"瞬移"）
- [ ] 裁判板：`/referee/state` 的 `ammo` 随实机变化（否则串口参数或帧格式不对）
- [ ] 开火链路：`enable_fire:=false` 跑一轮确认瞄准正常，再打开实弹
  - **必须先确认红面不会开火**：拿红面靶试，日志里应出现「红面目标禁止开火」

## C. 上场前 5 分钟

- [ ] 电池：车、无人机、机载电脑、图传/数传
- [ ] 相机镜头干净，云台零位正常
- [ ] 场地格号与 `params.yaml` 的 `cols` / `rows` / `start_cell` / `goal_cell` 一致
- [ ] 单应矩阵是**当前场地**标的（换场地必须重标）
- [ ] `mission_node` 日志里 `start -> goal` 与实际发车方向一致
- [ ] 计时器归零，裁判系统已复位

## D. 故障排查

| 现象 | 可能原因 | 检查 |
|---|---|---|
| **命令发了、执行器也算出了速度，但车纹丝不动** | **遥控器还在「遥控」档**——底盘有「车辆遥控/串口程序驱动」模式开关，在遥控档时完全忽略串口速度指令（`docs/raw_txt/04_遥控器控制.txt`） | 把遥控器拨到「串口程序驱动」档再发一次。**关遥控器没用**（已实测：关机后接收机无信号，底盘照样不动） |
| **位移有数、但车其实在原地画圆** | 只看"位移"看不出来（终点 N11 就在起点 N10 旁，跑完净位移也才 0.35 m）；⚠️ `/odom/localization` 是 **block 单位**，按米读会把半径/速度放大 2.86 倍 | `python tools/traj_report.py <traj.txt>`：**净转角 >360° 且转向反转 ≈0 次 = 单向画圆**；根因分析与证据链见 [`docs/画圆根因分析-20260930.md`](docs/画圆根因分析-20260930.md)（结论：转向环从发车即发散，是符号/坐标架问题，不是调参） |
| 画圆了，想知道是哪一层的问题 | 三个候选：angular.z 符号接反 / 定位 yaw 漂 / 单位（度-弧度、格-米） | `python tools/traj_geometry.py <traj.txt> --first <第一路点>` 先判"有没有朝目标走"；再 `tools/car_probe_circle.sh`（默认不动车测静息 yaw；发命令加 `AG_SEND=1`） |
| 同上，且想确认"到底卡在哪一层" | 逐级看：决策层→执行器→`pursuitAngle`→底盘 | `tools/car_probe_velstate.sh`：看 `/current_vel_state` 有没有回吐 `linear.y=9.0`（D 档）。**有 = 速度已进下发分支，断点在底盘固件；没有 = 还在 ROS 层** |
| 一直以为"执行器没发速度" | **判据用错了**：执行器发的是 **`pursuitAngle`**，不是 `/cmd_vel`。`/cmd_vel` 上"发布者 0"是**正常现象** | 别拿 `/cmd_vel` 判断执行器死活（2026-09-29 踩过，白查一轮） |
| 执行器启动后立刻没了、车不动 | `path_executor.py` 缺 `map_utils`（只在源码目录里有）→ `ModuleNotFoundError` 启动即崩，`/nav/waypoint` 订阅者变 0 | `car.py preflight` 已会拦；修法 `export PYTHONPATH=$HOME/dzrobot/src/car_localization:$PYTHONPATH` |
| 复盘时执行器日志是空的 | stdout 块缓冲，脚本 `cp` 时文件还是 0 字节 | 执行器必须用 `python3 -u` 起（已改） |
| 检测一直是空的，**且看不出原因** | **相机崩了（链路静默）** | 先跑 `tools/car_camera_check.sh`；复盘报告里看 `no-image` / `vision-stale` 结论；`ros2 node list` 里还有没有 `usb_cam_node` |
| 有框但瞄准点乱飘 | `kpt_unit` 猜错 | `verify_decode.py` 看 kpt 量级 |
| 置信度全在 0.5 附近 | `cls_activation` 猜错 | 同上，看 cls 是否在 [0,1] |
| 格号明显偏一格 | 单应没标 / 残差大 | `calib_homography.py --check` 验证单点 |
| `/mission/plan` 一直为空 | 情报未达投票阈值 | 看 `/mission/status` 的 `frames` 与 `confident` |
| 情报里有假格号 | 噪声太强 | `replay_intel.py --sweep` 调 `min_votes` / `ratio` |
| 车不走 / 不回报 | 底盘协议未对接或 time_scale 没改回 | `simulate:=true` 先验证；实机 `time_scale:=1.0` |
| 停在起点不动 | 第一段 eta=0 被判为已到位 | 检查 `/chassis/state` 的 status 是否 arrived |
| 车端收不到情报 | 广播地址/端口/网段 | 两端 `bind_port` 与 `peer_port` 是否交叉对应；`link_stats` 是否递增 |
| `bad_crc` 持续增长 | 同频干扰 | 换路由器信道或 UDP 端口 |
| 裁判状态不更新 | 串口号/波特率/帧格式 | `dmesg | grep ttyUSB`；`referee_protocol` 字段偏移待实机校准 |
| 开了火但没计分 | 未命中或靶标已失效 | `target_policy` 的 `enemy_count` 是否已达 3 |
| 误击友方 | 类别映射反了 | 确认模型元数据：`0=blue(敌)`、`1=red(友)`，不能按字面直觉反推 |
