---
来源: docs/20_资料索引库/车端/05_配置与部署.md、docs/00_项目总览/交接_20261024_打靶闭环.md、docs/00_项目总览/交接_20261023.md、docs/20_资料索引库/赛事与治理/04_决策台账摘编.md、dzrobot_v2/docs/决策台账.md D-007/D-019/D-011/D-020
取证: 实+档
更新: 2026-10-24
取代: 无（车端/05 §3/§4 的部署段在本篇展开为可执行 runbook；原文保留）
status: 现行
---

# 起栈与发车 runbook

> 面向「今晚要跑车」的人。按 §0→§7 顺序走，每步带判据；失败就停，别硬跑。
> 本文是**提炼**：命令与判据的真源是 `车端/05_配置与部署.md`、`交接_20261024_打靶闭环.md` 与决策台账；原文保留，不搬整段。

## TL;DR（给 agent 看的）

- **起栈前先做三件事**：① 域号 `source dzros.env`（否则 ROS 静默失联）；② `cam_viewer` 杀掉（否则车收到指令不动）；③ 确认 `~/dzrobot/config/map.yaml` 的状态（第七图才是现役）。
- **部署唯一入口** `tools/deploy_to_car.py`；`all` 会整包推送并**覆盖 `robot.yaml`**，单独 push 不自动 backup，且**必须人工执行**（D-007）。
- **刻意不推 `config/map.yaml`**：仓库是第六图、车上是第七图，谁推谁把车弄瘸（D-019）。
- **发车方式**：`mission.auto_start: false`（`robot.yaml:703`）⇒ 车停在起点等无人机 `/nav/waypoint` 下发；台架自测用 `ros2 topic pub /nav/waypoint std_msgs/String "data: reset:N10,y"` 触发。
- **最大坑**：`robot.yaml` 同名键散在两个配置段，改一处另一处不变，且值相同**看不出没改**。

## 0. 起栈前先做的三件事（缺一不可）

| # | 动作 | 判据 / 命令 |
|---|---|---|
| 1 | 域号对齐 | `source /home/duzhong/dzrobot/dzros.env`（或 `export ROS_DOMAIN_ID=118`）⇒ 启动横幅必须出现 `ROS_DOMAIN_ID=118`；不出现则跨域，互相看不见且**零报错** |
| 2 | 杀掉 `cam_viewer` | `pkill -f cam_viewer`；否则它以 20 Hz 发全零 `/pursuitAngle`，底盘每帧更新 ⇒ 车「收到了指令但不动」（`车端/05` §5 坑1） |
| 3 | 确认地图状态 | 车端 `~/dzrobot/config/map.yaml` 是**第七图**（hash `b5629d05…59bc`，桥 K10/K4、start N10/end N5）；仓库 `dzrobot_v2/config/map.yaml` 是**第六图**（`385e0718…d20d`，桥 K5/K7）。两者**不许混用** |

## 1. 域号与 shell 环境

| 项 | 值 | 出处 |
|---|---|---|
| 域号真源 | `dzros.env`：`ROS_DOMAIN_ID=118`、`ROS_LOCALHOST_ONLY=0` | `车端/05` §1 |
| 车端 DDS 端口 | 118 = `7400+250×118` = **36900**，组播 239.255.0.1 | `空地链路/02` / t4 报告 |
| 非交互 SSH 默认域号 | **0** | `车端/05` §1 |
| 症状 | ping/SSH/网页全通、ROS 互相看不见、**零报错** | `车端/05` §5 坑3 |

**每条 SSH 命令前先 source**：

    `source /home/duzhong/dzrobot/dzros.env`
    # 或显式：ROS_DOMAIN_ID=118 ros2 ...

> 坑：`ssh dzrobot` 被本机别名强制密码（`C:\Users\TestFox\.ssh\config:1-6` 设了 `PreferredAuthentications password` + `PubkeyAuthentication no`），BatchMode 会报 `Permission denied`；显式 `ssh duzhong@<当前IP>` 已验证公钥可用。`duzhong.local` 可能被旧 `hosts` 条目劫持（`C:\Windows\System32\drivers\etc\hosts:24` 仍写 `192.168.137.79`），先用 `probe` 或 `hostname -I` 核实当前 IP 再直连。

## 2. 部署（唯一正确姿势）

`powershell
cd D:\User\Desktop\比赛存档\v1\dzrobot_v2
$env:PYTHONIOENCODING='utf-8'
python tools\deploy_to_car.py probe          # 解析车 IP/状态
python tools\deploy_to_car.py all --dry-run  # 先看将执行什么
python tools\deploy_to_car.py all            # backup -> push -> build（不推 map）
`

- **必须人工执行**（D-007）；`deploy_car.py` 只作只读诊断，其 `--write` 仅现场应急且需 Lead 授权（D-019）。
- 部署后要**复采哈希**才能宣称生效（「推过去了」≠「车端跑的是它」）。
- ⚠️ **2026-10-23 起车端有两套工作区**：`~/dzrobot`（10-03 血统原版，云台 cpp 98 KB）与 `~/dzrobot2`（10-21 精简重写栈，云台 cpp 26 KB，归属未确认）。**部署前先确认目标栈**，别默认 `~/dzrobot`。
- 冻结区清单（D-008）：`src/dzbrain/dzbrain/**`、`src/dzrknn_dataprocess/**`、`config/**`、`tools/deploy_to_car.py`；改动须登记 `tools/wf/frozen.json` 的 in_flight（带 owner/卡号/解冻日）。

## 3. 地图与路线的版本矩阵（必须并列，不许二选一）

| 版本 | 洞（上/下） | 停站/终点 | 雷/障（节选） | 出处 |
|---|---|---|---|---|
| **A 仓库第六图** | 上 K5 / 下 K7 | 单停站 `B4*` | 68 token / 24.85 m | `dzrobot_v2/config/map.yaml`；压测报告按此图 |
| **B 实车第七图** | 下 K10 / 上 K4 | `9:F9` + 终点 `54:N5` | 55 点 / 20.65 m | `车端/05_配置与部署.md §2` |
| **C 车端手写第七图** | K10 / K4 | F9 / F5 / N5 | **50 token** | `dzrobot_v2/docs/使用手册.md:178`；车端 `robot.yaml` |
| **D 第七图第二版识读** | K10 / K4 | — | 雷 B1 A2 I2 C3 E3 G5 I8 H9；障 D5 I6 A7 F8 B9 F10 H10 A11 | `车端/05_配置与部署.md §2` |

- **C 版与 D 版不是同一份**：D 比 C 多 3 雷 4 障，拿 D 的雷表去核 C 的路线必然对不上。
- 有效期：A 版是仓库口径（回归夹具按它钉）；B/C/D 都是第七图家族，只对 10-18 之后的车端有效。

## 4. 起栈

`bash
cd ~/dzrobot && ./start_all.sh restart   # 7 步；start/stop/restart
`

| 项 | 内容 | 出处 |
|---|---|---|
| 起栈脚本 | `~/dzrobot/start_all.sh`，7 步；`start`/`stop`/`restart` | `车端/05` §1 |
| 启动判据 | 横幅打印 `ROS_DOMAIN_ID=118`；各节点 1 实例 | `交接_20261024_打靶闭环` §2 |
| 布局门 | **30 s**，超时 **exit 3 且不出发** | `空地链路/02` §8 |
| `healthcheck` | **不是起栈门禁**（自检已收敛到发车前 10 s 窗口） | `车端/05` §5 坑6 |
| `restart_node.sh path` | 指向**旧核**；改路线/速度必须 `start_all.sh restart` | `车端/05` §5 坑5 |

**起栈后逐项核对**：`ros2 topic info /LaserShot_Command -v`（订阅数 ≥1）、`/aim/status` 有值、`/pursuitAngle` 有发布者且非全零（`cam_viewer` 已杀）。

## 5. auto_start 与发车

| 键 | 现役值 | 语义 | 出处 |
|---|---|---|---|
| `mission.auto_start` | **false** | 车停在起点等外部 `/nav/waypoint` 下发，**不自己发车** | `robot.yaml:703` |
| `mission.auto_start_delay_s` | 10.0（车端现役）/ 45（R3 实验） | 若开启，起栈后等 N 秒自动发车 | `robot.yaml` / RUN_CARD |
| `path_executor.auto_start` | false（车端）/ true（本地） | 另一路自动起跑开关，两处不同步 | `交接_20261024_打靶闭环` §6.4 |
| `planner.auto_plan_enabled` | **true**（车端现役） | 启动时自动规划并（失败时）回退 token 路线；**不是故障**，但必须在上车日志里记录行为 | RUN_CARD §B |

**发车 = 解除接管或下发路线**，二选一：

1. **完整赛制（无人机情报）**：车停在 N10，无人机下发 `/nav/waypoint` 首段后车才走（`mission.auto_start=false`）。
2. **台架自测**：`ros2 topic pub --once /nav/waypoint std_msgs/String "data: reset:N10,y"`，或多点 `"data: B1,y C2,y ..."`（命令语法见 `交接_20261024_打靶闭环` §8）。
3. **安全口径（R3 实验）**：r3_preflight.sh 先接管 `/nav/manual_override=1` 并断言 `pursuitAngle.linear.x==0`，装路线需显式 `--route`；未架空时只有「场地净空 + 停车人到位 + 接管后 v=0 已确认」三项同时成立才可加 `--route`，否则不得装路线。

> ⚠️ 旧版「预检不发车」的说法**已作废**：旧脚本 `:28` 直接发「多点」会触发 `mission._start_mission()`（`mission.py:3937-3943 → :2722-2730 DRIVE`），`auto_start=false` 拦不住。

## 6. D11 摆位与射击站距

| 项 | 值 | 出处 |
|---|---|---|
| 射击站距 | 第七图 **D11 / D3** 都是 **1.4109 m** | `交接_20261024_打靶闭环` §1.4（设计包 T3_standoff） |
| 反向反推 | 1.41 m 反推 k≈**280~296**，比配置 `px_per_m_rail=140` 大一倍 | 同上 |
| 起点摆位 | 车摆 **N10 格心**，车头朝图左（+y，列号减小） | `技术全景_A_车端核心.md:1307` |
| 摆位判据 | 规划器把起点当**设计输入**，不按 `mission.reset` 反推，也不允许「摆错就拒绝发车」的校验 | `技术全景_A_车端核心.md:1307` |
| 容差 | 位置 ≤2 cm、航向 ≤5°（R3 更严：≤1.5 cm / ≤3°） | RUN_CARD §D/§G |

**现场摆位三步**：
1. 车压 N10 格心、车头朝 +y；
2. 核对三源一致：实物摆位 / `3_localization` 的 Start/pos 与 belief / 地图 `start=N10`；
3. 若现场只能做到 ≤5°，只跑短段（0.35 m）并记录降级原因。

> **D11 ≠ D-011**：D11 是第七图的一个格子（射击站）；D-011 是决策台账「30+10 扩量未批准」。两者别混。

## 7. GO 前置门（全部满足才发车）

1. 车在线，SSH 通，域号 `118` 已核；
2. 现役配置/地图/import 身份已采（hash 对账）；
3. 现场 N10/+y 摆位确认，实物-raw-belief 三源一致；
4. `cam_viewer` 已杀，`/pursuitAngle` 有非零发布者；
5. 无人机布局 `known` 含车位姿所在格（recon 129 格漏了 N10 ⇒ 车第一拍就 SAFE_STOP）；
6. 停车人与 GO/停口令就位，路线 ±0.3 m 净空。

## 8. 收工与回滚

`bash
cd ~/dzrobot && ./start_all.sh stop
`

- 收工顺序：先停车栈 → 停 bridge/dzuav → 还原 `map.yaml` 与 `auto_plan_enabled=false`（`空地链路/02` §8）。
- 代码回滚：备份在 `~/_backups/dzbrain_pre_step_20261018_144521`。
- 实验配置只在 `~/r3_field_20261020/`，删除即回现役；现役 `robot.yaml`/`map`/`start_all` 不动；不 commit。

## 9. 起栈失败对照表

| 现象 | 先看 | 处置 |
|---|---|---|
| 车收到指令但不动 | `/pursuitAngle` 发布者是不是 `cam_viewer` | kill cam_viewer |
| ping/SSH 通但 ROS 互相看不见、零报错 | 域号 | `source dzros.env`；确认 `ROS_DOMAIN_ID=118` |
| 起栈后相机/视觉/云台三节点同时 abort | `robot.yaml` 顶层裸键（如 `camera:` 写在根） | 地图放 `config/map.yaml`，参数进正确段 |
| 传了 `--map` 却不进布局模式 | 传的路径**存在** | 必须传**不存在**的路径 |
| 车半路停死必须重启栈 | `SAFE_STOP` / `yaw_untrusted` 闩锁 | `车端/04_定位与安全.md` §6 |
| 第一拍就 SAFE_STOP | `known` 是否含车位姿格 | 补 layout，别改护栏 |

## 相关文件

- `docs/20_资料索引库/车端/05_配置与部署.md`（真源）
- `docs/20_资料索引库/车端/00_索引.md`、`docs/20_资料索引库/车端/02_循迹与控制.md`
- `docs/00_项目总览/交接_20261024_打靶闭环.md`（发车方式、D11 站距、auto_start）
- `docs/00_项目总览/现状快照.md`、`docs/20_资料索引库/赛事与治理/04_决策台账摘编.md`
- `dzrobot_v2/tools/deploy_to_car.py`、`dzrobot_v2/config/robot.yaml`、`dzrobot_v2/config/map.yaml`
