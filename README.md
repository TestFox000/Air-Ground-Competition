# air_ground —— 空地协同侦排挑战赛

无人机侦察 + 地面车执行的完整软件栈（ROS2 Humble / Python）。
算法核心与 ROS 解耦：**不出机房就能跑通全链路**，上车前先在笔记本验证闭环。

> 📌 **实机（渡众 SmartMaster）接口已经全部实测清楚**，见
> **[`docs/实机接口对照.md`](docs/实机接口对照.md)**：车端自带
> `dzactuator`(底盘) / `dzjudgment`(裁判) / `dzrknn_dataprocess`(瞄准开火) /
> `car_localization`(任务执行器)，**我们只负责"报点"** —— 把格号序列写成
> `/nav/waypoint` 命令。场地是 **14 列 A~N × 11 行 1~11**，车起点 **N10**。

## 1. 架构

### 1.1 实机（渡众 SmartMaster）—— 用 `ag_car`

```
        ┌──────────── 机载端（无人机 / 机载电脑）────────────┐
 相机 ─▶ vision_node ─▶ /perception/detections ─▶ intel_fusion_node ─▶ air_link_node
                              （像素→格号，需现场标定单应 H）        │
        └──────────────────────────────┬────────────────────────┘
                             UDP 广播（局域网，禁公网）
        ┌──────────────────────────────┴────────────────────────┐
        │               车端（Orange Pi 5 / RK3588S）            │
 ground_link_node ─▶ /link/field_intel ─▶ car_node ─▶ /nav/waypoint
                                            │              │
                    IntelVoter（抗误码）     │              ▼
                    MissionPlanner（排序）   │  渡众 path_executor（车端自带）
                    RouteBuilder（拼命令）   │              │
                                            │              ├─▶ dzrknn_dataprocess ─▶ 瞄准/开火
 referee_node(dz) ◀─ /HpAndHitmsg           │              ├─▶ dzactuator ─▶ 底盘/云台（串口）
        └─▶ /LaserShot_Command ─────────────┘              └─▶ dzjudgment ─▶ 裁判板（串口）
        └────────────────────────────────────────────────────────┘
```

### 1.2 自研 / 仿真底盘 —— 用 `mission_node` + `chassis_node`

没有渡众那套栈时（或用 `run.py demo` 在笔记本上验证算法）走这条路：
`mission_node` 输出 `/mission/waypoints`(JSON)，`chassis_node` 消费并回报 `task_done`，
`target_node` 负责瞄准/开火，`referee_node(source=serial)` 直接解裁判串口。

| 包 | 作用 | 依赖 ROS |
|---|---|---|
| `ag_core` | 纯算法：栅格地图、任务规划、后处理、敌我策略、裁判协议（自研 + **实机 dz 帧**）、链路协议、推理后端、单应标定 | 否（可单测） |
| `ag_interfaces` | 自定义消息 | 是 |
| `ag_perception` | `vision_node`（图像→检测）、`intel_fusion_node`（检测→格号情报） | 是 |
| **`ag_car`** | **实机对接层**：`car_node`（情报→规划→`/nav/waypoint`）、`route_builder`（格号→车端路线串） | 是 |
| `ag_mission` | `mission_node`（情报→规划→路点）、`target_node`（检测→云台/开火）、`field_fusion`（车端多帧投票） | 是 |
| `ag_referee` | `referee_node`（`source=dz` 接官方话题 / `serial` 自研串口）、`serial_io` | 是 |
| `ag_chassis` | `chassis_node`（消费路点 → 行进 → 回报）、`chassis_link`（底盘协议对接位） | 是 |
| `ag_bridge` | `air_link_node` / `ground_link_node` / `fake_intel_node`、`udp_link` | 是 |
| `ag_bringup` | launch 文件 + 统一参数 | 是 |

两级情报确认是刻意设计：机载端 `IntelAccumulator` 抗**视觉误检**，车端 `IntelVoter` 抗**链路误码与口径不一致**。
车端路线下发前还会三重校验：格号范围（14×11）、打击点数量、路线是否变化（避免重复触发）。

## 2. 一键运行

不想记命令就双击 **`run.bat`**（Windows）或 `./run.sh`（Linux/macOS），等价于：

```bash
python run.py demo
```

`demo` 会在本机跑通 **全栈**：`fake_intel → air_link → UDP → ground_link → mission → 底盘推进 → target → referee`，
跑的是真实节点代码（用 `tests/ros_stub.py` 顶替 rclpy），**不需要 ROS / 无人机 / 串口 / 模型**。默认结果：

```
[14] 出发 leave_start  N10  +5   eta=0.0s
[15] 出发 mine         I9   +5   eta=3.3s
...
[27] 出发 goal         N11  +5   eta=7.4s
规划任务数 13，已完成 13
行驶耗时 61.1 s / 180 s   行驶得分 85
开火 6 发，敌计数 3/3，友计数 0，余弹 4
预估总分 85 / 85
[PASS] 全流程跑通
```

其它子命令：

```bash
python run.py all       # test + demo + 复盘报告（改完代码跑这一个就够）
python run.py log       # 复盘报告：哪一步卡住、为什么、下一步做什么
python run.py preflight # 上场前体检：配置/环境里最容易忘的项逐条查（有 FAIL 就别上场）
python run.py test     # 单元测试（-k 可过滤）
python run.py check    # 环境体检：依赖 / ROS / 参数一致性
python run.py sim      # 有 ROS2 时启动 sim.launch.py
python run.py demo --mode full --noise-ratio 0.4 --confirm-frames 5   # 换场景
```

**跑完一圈一定要看复盘**。所有节点都带黑匣子日志，`all` / `demo` 结束后会自动出报告：

```bash
python run.py log                                  # 看最近一批日志（默认 $AG_LOG_DIR）
python run.py log --dir logs/car --out report.md    # 指定目录 / 存文件
python run.py log --json | python -m json.tool      # 机器可读（接 CI 就加 --strict）
```

结论长这样（每条都带"为什么会这样"+"下一步做什么"）：

```
[严重] 情报已收敛，但没有发出任何路线
        被拦下的原因分布：face_unknown×3
        -> 敌我尚未判决（蓝/红都没认出来）……
[正常] 打击停靠点：D2
```

车端也能一条命令走完（需要 `AG_CAR_PASS` 环境变量，或按提示输入密码）：

```bash
python run.py car-rehearse 172.23.116.188            # 上传+编译+干跑一圈+取回日志+出报告（不动车）
python run.py car-rehearse 172.23.116.188 --build    # 连编译一起做（改过源码时用）
python run.py car-fetch 172.23.116.188               # 只取回日志并出报告
```

原理、事件表、规则表、踩过的坑见 [`docs/调试日志.md`](docs/调试日志.md)。

## 2.1 版本控制

代码已在 Git 仓库里（首次提交 `a1ea04b`）。想看改动与回滚：

```bash
git status / git diff              # 改了什么
git log --oneline -5               # 最近提交
git restore <文件>                 # 撤销未提交的改动
```

⚠️ 仓库里有 `.gitattributes`，**强制 `*.sh` 用 LF**：这些脚本要上传到 Linux 车端执行，
一旦被 Git 转成 CRLF，shebang 会变成 `#!/bin/bash\r`，车端直接 `bad interpreter`。
所以不要改 `.gitattributes` 里 `*.sh` 那一行，也不要开 `core.autocrlf=true` 覆盖它。

## 3. 快速开始（手动）

```bash
pip install -r requirements.txt        # 核心：numpy + pytest
python -m pytest tests -q              # 456 项
python tools/sim_selfcheck.py          # 离线闭环自检（无需 ROS）
```

可选依赖（PC 调试 / 板端推理 / 串口）见 `requirements-extra.txt`。

### 2.1 上车前全链路仿真（无无人机、无串口、无模型）

```bash
ros2 launch ag_bringup sim.launch.py
```

`fake_intel_node` 逐帧揭示情报（带噪声），可观察 `/mission/plan`、`/mission/waypoints`、`/mission/status`。
推进任务只需发一次完成信号：

```bash
ros2 topic pub --once /mission/task_done std_msgs/msg/String "{data: C3}"
```

### 3.2 实机（渡众 SmartMaster）

> ⚠️ **这台车是公用的**（别人在 `~/dzrobot` 等目录有数据）。我们的做法是：
> 编译工作空间放 **`/tmp/ag_ws`**（重启即清、不占家目录），
> 厂家目录全程只读，收工跑 `tools/car_cleanup.sh` 把痕迹删净并核对指纹。
> 详见 [`docs/实机接口对照.md`](docs/实机接口对照.md) §10。

```bash
# ① 传到车上并在车端编译（本机执行；自动建目录，不碰家目录）
python tools/remote_run.py --host 172.23.116.188 --user duzhong --put src --dest /tmp/ag_ws/src
python tools/remote_run.py --host 172.23.116.188 --user duzhong --script tools/car_build.sh --timeout 900
python tools/remote_run.py --host 172.23.116.188 --user duzhong --script tools/car_verify.sh
#   实测：9 包 18.5s 编译通过，9 入口可发现、4 个 launch 可解析、11 模块可导入

# ② 上车前先干跑一遍全链路（不动车），确认"日志 → 取回 → 复盘"是通的
python tools/remote_run.py --host 172.23.116.188 --user duzhong --script tools/car_log_rehearsal.sh

# ③ 机载端：视觉推理 + 情报融合 + 空地链路
ros2 launch ag_bringup air.launch.py model_path:=/home/duzhong/dzrobot/src/rknn_pt/model/light_det.rknn

# ④ 车端：先起渡众官方栈（底盘/裁判/相机/检测/瞄准）
source /opt/ros/humble/setup.bash && source ~/dzrobot/install/setup.bash
ros2 launch dzactuator bringup_all.launch.py
ros2 launch dzactuator judgment.launch.py
ros2 launch dzactuator camera.launch.py
ros2 launch dzactuator det_node.launch.py
ros2 launch dzactuator dzrknn_dataprocess.launch.py

# ⑤ 车端：我们的决策层 —— 先 dry_run 核对路线，再放开
export AG_LOG_DIR=/tmp/ag_logs                           # 黑匣子落这里（默认就开着）
ros2 launch ag_bringup car.launch.py                     # dry_run 默认 true
ros2 topic echo /car/route_status                        # 看路线与情报对不对
ros2 launch ag_bringup car.launch.py dry_run:=false start_cell:=N10

# ⑥ 跑完一圈，把日志取回来出复盘报告（本机执行，一条命令）
python run.py car-fetch 172.23.116.188    # 取回 /tmp/ag_logs → logs/car 并直接出报告
python run.py log --dir logs/car --out report.md --strict
```

**跑完一定要看复盘报告**：它会直接告诉你"哪一步卡住、为什么"，比如
"情报已收敛却没有发路线，原因分布：`face_unknown` ×1"。
细节见 [`docs/调试日志.md`](docs/调试日志.md)。

发车/收队的固定动作：车是**被人搬到起点**的，里程计不知道自己在哪，
所以每局必须先发 `reset:<起点>,<朝向>`（`car_node` 自动发，手动调试时用
`ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'reset:N10,y'}"`）。

若现场不用我们的 `ag_car`，只要往同一个话题发路线字符串也一样能让车跑：

```bash
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'G8 A10 D2* N11,y'}"
```

## 4. 车端接口

**实机（渡众车）**：只需往 `/nav/waypoint` 发命令，详见
[`docs/实机接口对照.md`](docs/实机接口对照.md) §4：

```
reset:N10,y                  复位里程计（每局第一件事）
G8 A10 D2* E2,y              路线：'*' = 停靠射击，',y' = 终点朝 Y+
M10,y                        单点
```

`ag_car/route_builder.py` 负责拼装与校验（格号范围、连续重复、逗号位置）。

**自研底盘**：`chassis_node` 订阅 `/mission/waypoints`，按 `path` 逐段走，
到位后向 `/mission/task_done` 发布格号；底盘驱动协议接
`chassis_link._send_path` / `_poll_arrived`，未对接时自动回退为计时推进。

```json
{"done": false, "index": 2, "type": "mine", "cell": "C3",
 "path": ["B2","B3","C3"], "score": 5, "eta_s": 4.7}
```

走完一段后向 `/mission/task_done` 发布格号（`"C3"`）或任务类型（`"MINE"`），
`mission_node` 自动推进；收到新情报时只对**剩余**任务重排，已执行段不动。

## 5. 评分策略（180 s 单限）

| 项目 | 分值 | 策略 |
|---|---|---|
| 驶离起点 / 回到终点 | 5 / 5 | 始终预留 `goal_margin_s` 回终点 |
| 排雷 | 5 × 8 | 按「分值 / 耗时」贪心，时间不够时优先放弃远雷 |
| 桥洞 | 10 × 2 | 分值高、优先排 |
| 打击敌方 | 5 × 3 | 需连续 `consensus_frames` 帧类别一致才开火 |
| 误击友方 | −5 × 3 | 红面一律 `AVOID`，节点层还有一道硬拦截 |

满分 85（5+5+8×5+2×10+15）。`tools/sim_selfcheck.py` 会给出当前参数下的预估得分。

## 6. 现场必须先校准的三件事

1. **单应矩阵**（`tools/calib_homography.py`）— 没有它，像素落不到格号
   ```bash
   python tools/calib_homography.py --corners "120,80 980,95 965,700 105,690" --check "500,400"
   ```
   四角顺序固定为 **A1 → N1 → N11 → A11**（14 列 A~N × 11 行 1~11）。残差 > 5 cm 要重标。
   输出的 9 个数填进 `config/params.yaml` 的 `homography`。

2. **模型解码参数**（`tools/verify_decode.py`）— `cls_activation` / `kpt_unit`
   Ultralytics 导出到 RKNN 时行为不一致，猜错表现为「框准但瞄准点乱飘」。
   ```bash
   python tools/verify_decode.py --model models/best_new.rknn --image samples/frame.jpg --nc 2 --nkpt 1
   ```

3. **裁判系统帧格式**（`src/ag_core/referee_protocol.py`）
   官方未公开帧格式，当前是自洽的参考实现（AA 55 + cmd + len + payload + crc8）。
   拿到实机抓包后只改 `RefereeState.from_bytes` / `to_bytes` 的字段偏移。

## 7. 工具

| 工具 | 用途 |
|---|---|
| `tools/verify_decode.py` | 模型输出体检，校准 `cls_activation` / `kpt_unit` |
| `tools/calib_homography.py` | 四角像素 → 单应矩阵，输出可直接粘贴的 yaml |
| `tools/replay_intel.py` | 回放情报帧（JSONL），`--sweep` 扫描 `min_votes` / `min_ratio` / `ttl_s` |
| `tools/sim_selfcheck.py` | 离线闭环自检：情报→规划→执行→打击，输出预估得分与耗时 |
| `tools/log_report.py` | **一局跑完的复盘报告**：结论（按严重度）+ 指标 + 跨节点时间轴 |
| `tools/car_log_rehearsal.sh` | 车端全链路干跑，验证"日志 → 取回 → 复盘"通路（不动车） |
| `tools/car_camera_check.sh` | **相机健康检查**：有视觉的联调前先跑（官方 `usb_cam` 崩了整条链是静默的） |
| `tools/preflight_check.py` | **上场前体检**：自动查"最容易忘"的项（单应、dry_run、广播地址、模型、版本） |
| `tools/build_dataset.py` | 数据集加工：标注转换 / 自动补发光中心关键点 / 清洗 / 划分 / 生成 data.yaml |

```bash
python tools/replay_intel.py --input tools/samples/intel_sample.jsonl --sweep
python tools/sim_selfcheck.py --speed 0.45 --noise-ratio 0.4 --min-votes 3
python tools/log_report.py --dir logs/car --out report.md --strict   # 跑完一圈的复盘
```

## 8. 测试与 CI

- `tests/` 共 477 项，覆盖：算法核心、协议编解码、情报投票、串口帧、UDP 链路、单应标定、图像解码、底盘状态机、桥洞占格、**黑匣子日志与复盘规则**（含"日志写不进去也不能崩主流程"）、**相机断流看门狗**、**单应未标定不再误诊为链路故障**、**上场前体检**、**一键入口 CLI**（子命令字段契约 —— `run.py all` 曾因少一个参数崩掉）、**打包结构**（真机才暴露的两个坑）、**工具脚本冒烟**（`--help` 能跑）、**模型契约校验**
- `tests/ros_stub.py` 提供极简 ROS2 替身，使**节点层**（参数、话题、回调状态机、端到端链路）也能在没有 ROS 的机器上测试
- `tests/test_params_consistency.py` 校验 `params.yaml` 的参数名与节点声明一致，防止拼写错误静默失效
- `.github/workflows/ci.yml`：Python 3.10/3.12 跑单测 + 语法检查 + 工具冒烟；另有一个 ROS2 Humble 容器做 `colcon build` 与可执行文件检查

## 8. 链路可靠性

- 禁止公网；自带路由器组局域网，UDP 广播，端口见 `params.yaml`
- 每帧带 `seq` + `crc32`：车端丢弃乱序/重复帧，损坏帧计数发在 `/link/link_stats`
- 无新情报时发心跳，便于判断链路存活

## 10. 目录

```
air_ground/
├── src/
│   ├── ag_core/          纯算法核心（无 ROS）：栅格/规划/后处理/策略/协议/标定
│   ├── ag_interfaces/    自定义 msg
│   ├── ag_perception/    vision_node / intel_fusion_node / intel_accumulator
│   ├── ag_car/           实机对接层：car_node / route_builder（→ /nav/waypoint）
│   ├── ag_mission/       mission_node / target_node / field_fusion（自研底盘用）
│   ├── ag_chassis/       chassis_node / chassis_link（自研底盘用）
│   ├── ag_referee/       referee_node / serial_io
│   ├── ag_bridge/        air_link / ground_link / fake_intel / udp_link
│   └── ag_bringup/       launch（air / ground / sim / car）+ config
├── docs/                 规则原文(raw_txt) + 速查手册 + 模型报告 + 实机接口对照
│   ├── 调试日志.md       黑匣子：打开方式 / 事件表 / 复盘用法 / 规则表 / 踩过的坑
│   └── probe/            车端探测原始输出（可复现）
├── reference/car_src/    从车上取回的官方包源码（只作接口参照）
├── run.py / run.bat / run.sh   一键入口
├── tests/                pytest（含 ROS 替身，Windows 可直接跑）
├── tools/                校准 / 回放 / 自检 / 车端部署（car_build · car_verify · car_cleanup · remote_run）
└── .github/workflows/    CI
```

## 11. 项目状态

**结构已经定型**（9 个包、384 项测试、一键 demo 跑出 85/85、
**真机 colcon build / 决策干跑 / 视觉链联调均已通过**）。
距离"能上场比赛"还差什么，有一张按四档（L0~L4）划分的核对表：
[`docs/实机接口对照.md` §8](docs/实机接口对照.md)。

已经闭环、可离线验证的：

| 环节 | 状态 |
|---|---|
| 检测结果 → 格号情报（单应标定 + 多帧累积） | 完成，待现场标定 H |
| 情报 → 任务排序 → **车端 `/nav/waypoint` 路线** | 完成，`ag_car` 已按实机语法实现+校验 |
| 敌我识别 → 瞄准 → 开火 → 裁判计分 | 实机由官方 `dzrknn_dataprocess` 承担，我们只做策略闸门与状态订阅 |
| 空地链路（UDP + crc32 + seq 去重） | 完成，含损坏帧统计 |
| 裁判协议 | **实机帧格式已拿到**（`ag_core/dz_referee.py`），实机走官方 `dzjudgment` 话题 |

已经过去的两大关：

1. **真机 `colcon build` 通过**（9 包 18.5s 零错误）—— 并因此揪出两个本地永远测不到的坑
   （`ag_core` 扁平布局、缺 `setup.cfg` 导致 `ros2 run` 找不到入口），
   已修好并固化成 `tests/test_packaging.py`。
2. **车端干跑联调通过** —— `car_node` 在实机上吃假情报吐出真实路线
   `K9 K4 D2* B2 C3 G8 N11,y`，且确认 dry_run 下 8 秒内**一条 `/nav/waypoint` 都没发**（车不会动）。

顺带修掉一个会丢分的隐患：早期用「至少 1 个打击点」当出发条件，
**靶是友方（红面）时会永远不发车**，白丢排雷 40 + 桥洞 20 + 终点 5 分。
现在改为按「情报收敛」出发，并加了三条打击硬约束（单停靠点、红面不标 `*`、等待超预算就放弃）。
详见 `docs/实机接口对照.md` §6.1 / §10。

剩下的是「只能现场量/标」的参数，不是结构问题（清单见 `docs/实机接口对照.md` §8）：

1. **智能车终点格号** — 规则只说"固定位置"，需现场量取后填入 `params.yaml`
2. **摆放朝向** — `car_node.start_heading` 必须与裁判要求的摆放方向一致
3. **单应矩阵 H** — 无人机视角像素→格号，不需要模型，先标
4. **模型文件** — 门面 `models/` 未入库；车端现有的 `light_det*.rknn` 在
   `~/dzrobot/src/rknn_pt/model/`。训练标签是 YOLO-pose（`cls x y w h kx ky 2`），
   导出后 `kpt_unit` / `cls_activation` 必须用 `tools/verify_decode.py` 实测。
5. **相机与激光不同轴偏置** — 官方有 `laser_offset_x/y` 参数要现场标

资料位置：`docs/raw_txt/` 规则原文、`docs/空地协同侦排挑战赛_速查手册.md` 提炼版、
`docs/实机接口对照.md` 车端实测接口、`docs/probe/` 探测原始输出、
`reference/car_src/` 从车上取回的官方源码（只作接口参照，不是我们的代码）。

上场前的逐项确认见 `CHECKLIST.md`。
