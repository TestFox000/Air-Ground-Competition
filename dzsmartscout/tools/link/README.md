# dzlink_supervisor —— 无人机侧空地链路常驻节点（操作手册）

> 一架无人机只要挂着**这一个进程**，就完成了「感知 → 下发 → 闭环」三件事。
> 文件：`tools/link/dzlink_supervisor.py`（单文件，纯 stdlib + rclpy，**不需要编译**）
> 部署位置：无人机 `~/dzlink_tools/dzlink_supervisor.py`

| 项 | 值 |
|---|---|
| 目标机 | 无人机 `duzhong@10.29.122.104`（SSH 免密） |
| 域号 | `ROS_DOMAIN_ID=118`（**两端唯一真相源**：机 `~/dzlink_tools/dzlink.env` ↔ 车 `~/dzrobot/dzros.env`） |
| 依赖 | ROS 2 Humble 自带（`rclpy` + `std_msgs`）。**不装任何包** |
| 本地验证 | 70 个单测 + `selftest`，全绿（`python -m unittest discover -s tests`） |
| 是否在真机跑过 | 逻辑层本机验证；**rclpy/DDS 层由 `smoke_ros.sh` 在无人机上验证**（见 §5） |

---

## 1. 它到底做什么

```
        ┌────────────── 感知侧（t2 的管线 / dzuav） ──────────────┐
        │  文件 JSONL   /   ROS 话题 String   /   UDP JSON 包      │
        └───────────────────────┬────────────────────────────────┘
                                ▼  decide()  ← 纯函数，认不出就丢，绝不猜
   ┌──────────────────── dzlink_supervisor ───────────────────┐
   │ ① 1 Hz 心跳        → /dzlink/drone_to_car   BEST_EFFORT   │
   │ ② 情报转发         → /dzlink/drone_to_car   BEST_EFFORT   │
   │ ③ 订阅车回传       ← /dzlink/car_to_drone   BEST_EFFORT   │
   │ ④ 路线/复位下发    → /nav/waypoint          RELIABLE      │
   └───────────────┬──────────────────────────┬───────────────┘
                   ▼                          ▼
           小车 dzbrain DroneTask        车 path_executor / dzbrain
           （JSON 情报 + ack 回执）        （裸文本路线，收到即出发）
```

### QoS 为什么写死（改错＝完全收不到且不报错）

| 话题 | 类型 | QoS | 车侧订阅者（读源码核实） |
|---|---|---|---|
| `/nav/waypoint` | `std_msgs/String` | **RELIABLE** / VOLATILE / depth10 | `path_executor_node`、`serial_reader_node` |
| `/dzlink/drone_to_car` | `std_msgs/String`(JSON) | **BEST_EFFORT** / VOLATILE / depth10 | dzbrain `DroneTask` |
| `/dzlink/car_to_drone` | `std_msgs/String`(JSON) | BEST_EFFORT | 车往这里发 `car_state`/`ack` |

### 时序（决定你什么时候发情报）

| 车侧参数 | 值（车侧真值） | 含义 |
|---|---|---|
| `drone.info_stale_s` | 30 s | 情报超过 30 s 车侧当"无情报" |
| `mission.bridge_info_wait_s` | **2 s** | Mission 最多等 2 s，等不到用 `route.cross_lower_default`=**K10**(下) / `cross_upper_default`=**K5**(上) |
| `mission.auto_start_delay_s` | **45 s** | `auto_start=true` 时起栈 45 s 后车就出发 |
| `drone.heartbeat_offline_s` | 3 s | 3 s 没心跳车侧判无人机离线 |

> ★★ **等情报窗口只有 2 s，而且它只管「两半场相隔 ≤ 2 s 到达」**（2026-10-12 t16 对齐；
> 旧口径 20 s 已作废）。超过 2 s 才到的那个半场只进遥测，**路线已冻结**（车侧不换洞）。
> ⇒ **两半场必须成对发出**：`lower`/`upper` 要在同一轮里先后发完。
> 本节点已保证这一点：**任一新鲜桥洞半场到点，就把所有新鲜桥洞半场一起发**
> （`tick()` 里的 `bridge_round_due`），所以即使两半场是**分两次**从感知层进来的
> （相隔多久都行），车侧拿到的也是紧挨着的一对。
>
> 真机实测（车侧接收时刻，见 [`reports/t16-evidence/`](../reports/t16-evidence/)）：
>
> | 场景 | lower→upper 间隔 | 相邻两条桥洞报文的**最大**间隔 |
> |---|---|---|
> | `run --bridge lower=K10 --bridge upper=K5`（同时注入） | 中位 0.016 s / 最大 0.023 s | 1.13 s |
> | `--cmd-file` 一次写入两行 | 中位 0.016 s / 最大 0.022 s | 1.12 s |
> | **两半场相隔 5 s 分两次注入** | 中位 0.017 s / 最大 0.027 s | 1.13 s |
>
> 三种场景都远小于 2 s 窗口。⇒ 情报**必须在车起栈前备好**（`--cmd-file` 预置 / `once`），
> 不能指望发车后再抢窗口。

⇒ 本节点 `bridge` 默认**每 2 s 重发一次**，且**内容一变立即重发**，就是为了卡住上面这些窗口。
查当前值：`python3 dzlink_supervisor.py contract`

---

## 2. 部署（本地一条命令）

```powershell
# 在本地（Windows）工作区根目录
bash dzsmartscout/tools/link/deploy_supervisor.sh              # 上传 + md5 核对 + 远端自检 + 真机冒烟
bash dzsmartscout/tools/link/deploy_supervisor.sh --no-smoke    # 只想上传，不跑 ROS
bash dzsmartscout/tools/link/deploy_supervisor.sh --dry-run     # 只打印将执行的命令
```

脚本做的事（每一步都会打印证据）：
1. SSH 连通性；
2. `mkdir -p ~/dzlink_tools/{perception/out,logs,routes}`（**只新建，不覆盖**）；
3. scp 上传 `dzlink_supervisor.py` / `smoke_ros.sh` / 单测 / 本手册；
4. **本地 md5 vs 远端 md5 逐文件核对**（不一致就停）；
5. 远端 `py_compile` + `selftest` + `contract`；
6. 远端跑一遍单测（纯逻辑，不需要 ROS）；
7. 真机 ROS 冒烟（见 §5）。

> ⚠️ 两个历史坑（脚本已内置规避）：
> ① PowerShell 5.1 给 `ssh.exe` 传参会吃双引号 → **要跑的东西写成脚本 scp 过去**；
> ② 复制粘贴可能带 `\r` → 上传后 `sed -i 's/\r$//'` 归一化。

---

## 3. 四种用法（无人机上，先 source 两句）

```bash
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env          # ★ 域号唯一真相源 = 118
```

### 3.1 常驻（比赛主用）

```bash
python3 ~/dzlink_tools/dzlink_supervisor.py run \
    --perception-glob '~/dzlink_tools/perception/out/*.jsonl' \
    --cmd-file       ~/dzlink_tools/cmd.jsonl \
    --log-file       ~/dzlink_tools/logs/supervisor.jsonl
```

后台挂着（脱离 SSH 会话）：

```bash
setsid nohup python3 ~/dzlink_tools/dzlink_supervisor.py run \
    --perception-glob '~/dzlink_tools/perception/out/*.jsonl' \
    --cmd-file ~/dzlink_tools/cmd.jsonl \
    --log-file ~/dzlink_tools/logs/supervisor.jsonl \
    > ~/dzlink_tools/logs/supervisor.stdout 2>&1 < /dev/null &
```

### 3.2 只发一次（验证报文，不常驻）

```bash
python3 ~/dzlink_tools/dzlink_supervisor.py once --bridge lower=K9 --bridge upper=K4
python3 ~/dzlink_tools/dzlink_supervisor.py once --target G10,H10 --mines A3 --obstacles K9
```

> ★ **`run` 也能预置情报了**（2026-10-12 t12 修复 t4 真机缺陷 D2）：
> ```bash
> python3 ~/dzlink_tools/dzlink_supervisor.py run \
>     --bridge lower=K9 --bridge upper=K4 --target G10,H10 --mines A3 --obstacles K7
> ```
> `run` 的 `--bridge/--target/--mines/--obstacles` 在 build_argparser 里一直存在，但**修复前
> 真跑时从不生效**（argparse 照收、`intel_emitted=0`、零提示 —— "以为发了其实没发"），
> 只有 `--dry-run` / `once` 那条路径生效。现在三条路径行为一致（`--dry-run` 仍只注入一次，不重复）。
> 常驻更稳的注入方式仍是 `--cmd-file`（见 §6，运行中追加即生效）。

### 3.3 下发路线 / 复位

```bash
python3 ~/dzlink_tools/dzlink_supervisor.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"
python3 ~/dzlink_tools/dzlink_supervisor.py reset N10,y
```

### 3.4 诊断 / 纯本地

```bash
python3 ~/dzlink_tools/dzlink_supervisor.py contract                 # 契约一页纸（含当前域号）
python3 ~/dzlink_tools/dzlink_supervisor.py selftest                 # 纯逻辑自检（不需要 ROS）
python3 ~/dzlink_tools/dzlink_supervisor.py dry-run --bridge lower=K9 # 只在本地拼报文，不发
python3 ~/dzlink_tools/dzlink_supervisor.py listen --duration 30      # 只监听车回传
```

**退出码**：`0` 正常 / `2` 没有 ROS 环境 / `3` 发得出去但**没有任何订阅者**（车没起栈、域号不一致或组播被拦）/ `1` 参数错

---

## 4. 感知侧怎么接进来（三种输入，可叠加）

| 方式 | 参数 | 适用 |
|---|---|---|
| 文件尾随 | `--perception-glob '.../out/*.jsonl'` 或 `--perception-file x.jsonl` | 感知管线把结果落盘的**最省事**方式 |
| ROS 话题 | `--perception-topic /dzlink/perception` | 感知侧已经在 ROS 里 |
| UDP JSON | `--perception-udp 39042` | 感知侧是独立进程、只想发包 |

默认**从文件尾读**（只处理启动之后新写的行）；要把历史行也补上就加 `--perception-from-start`。
命令文件同理：`--cmd-file` 默认从尾读，`--cmd-from-start` 从头读。

**下发顺序固定**（不靠字典序）：`bridge_passable` → `target_zone` → `field_verify`；
桥洞排最前是因为车侧穿洞决策只等 **2 s**，且要求 lower/upper **同轮成对**送达（见 §1 时序表）。

### 4.1 接受的 JSON 形态（字段尽量宽容，认不出就丢）

```jsonc
{"type":"bridge_passable","half":"lower","block":"K9","confidence":0.9}
{"type":"bridge","direction":"left","hole":"K9"}        // 方向词自动映射（见下）
{"type":"target_zone","blocks":["G10","H10"]}
{"type":"field_verify","mines":["A3"],"obstacles":["K9"]}
{"type":"mines","blocks":["A3"]}                        // 拆开单发也行
{"type":"obstacles","blocks":["K9"]}
{"type":"heartbeat"}                                    // 触发一次立即心跳
{"payload":{"type":"target_zone","blocks":["G10"]}}     // 外层包裹也认
```

### 4.2 ★ 两条必须知道的边界（如实交代）

1. **不解析 dzuav 的私有 UDP 二进制帧**（端口 50000：`ef ef ef ef 10` + u64 ts + 25 B/目标 +
   CRC16 + `fe fe fe fe`）。帧格式属于感知层，本节点**只吃 JSON**——
   要么感知侧直接往 `--perception-udp` 发 JSON，要么加一个薄适配器转 JSON。
   这样做的好处：帧格式变动不会同时打坏两侧。
2. **`direction`→`half` 是"约定假设"，不是源码事实**：默认 `left→lower`、`right→upper`。
   写错的表现是**车穿错洞而且不报错**。所以：
   - 每次用到方向词都会打印 `[decide] 方向词 'left' → half='lower'（约定映射，非源码事实）`；
   - 可用 `--bridge-left-halves upper` 反转；
   - **最稳的做法**：让感知侧直接写 canonical 的 `"half"`，绕开这个假设。
3. 桥顶是 z=0.35 的**第二个平面**（见 `docs/感知可行性_摄像头评估.md`），地面格号单应对桥顶不适用。
   这个换算**由感知侧负责**；本节点只把 `{half,block}` 原样下发，不做任何像素→格运算。

---

## 5. 验证：三级判据

### L1 · 本机纯逻辑（不需要 ROS，天天可跑）

```bash
python -m unittest discover -s dzsmartscout/tools/link/tests -p "test_*.py"   # 70 项
python dzsmartscout/tools/link/dzlink_supervisor.py selftest                  # 精简版
```

覆盖：心跳节律 / 情报翻译与时效 / 报文契约字段 / 路线形状校验 / ack 台账 / 文件尾随（半行、截断）/ 命令文件 / UDP / 端到端（感知文件→报文→模拟 ack）。

### L2 · 真机 ROS（`smoke_ros.sh`，在无人机上，域 118）

```bash
bash ~/dzlink_tools/smoke_ros.sh
```

它**不依赖小车、也不碰小车**：用本机临时订阅者冒充小车，喂三种输入，再回一条 ack。
判据（脚本自己核对，全过才 `exit 0`）：

- 收到 ≥3 条心跳（1 Hz）
- 收到 `bridge_passable`，且 **lower / upper 两个半场都在**
- 收到 `target_zone`（命令文件生效）与 `field_verify`（文件尾随生效）
- 收到 `/nav/waypoint` 上的路线（RELIABLE 通道通）
- supervisor 日志里出现 `car_ack`（回传方向 + ack 台账都通）

### L3 · 与小车的端到端（车起栈时）

```bash
# 无人机侧
python3 ~/dzlink_tools/dzlink_supervisor.py once --bridge lower=K9 --bridge upper=K4
# 看到「订阅者匹配数 ≥1」才算真配上；=0 先查域号，再查车是否起栈
# 车侧（只读）
ROS_DOMAIN_ID=118 ros2 topic info /nav/waypoint -v     # Subscription 应为 ≥1
tail -f ~/robot_logs/7_path.log                        # 桥洞情报行 / 任务开始行
```

**反面判据（最先排的三种）**：匹配数 0 / 车没起栈 / 组播被 AP 拦。
域 d 的发现端口 = `7400 + 250·d`（域 118 → `36900`）；在 7400 上嗅探**永远**看不到域 118 的对端。

---

## 6. 命令文件（`--cmd-file`，JSONL 追加即生效）

```bash
echo '{"type":"bridge","half":"lower","block":"K9"}'      >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"target","blocks":["G10","H10"]}'           >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"route","data":"N10 M10 L10 L9 K9"}'        >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"route","file":"/home/duzhong/dzlink_tools/routes/r1.txt"}' >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"reset","block":"N10","yaw":"y"}'           >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"heartbeat"}'                               >> ~/dzlink_tools/cmd.jsonl
echo '{"type":"stop"}'                                    >> ~/dzlink_tools/cmd.jsonl
```

坏命令**只跳过不崩**（类型不认识、格名非法、缺 `data` 都会记进日志 `cmd_rejected`）。

---

## 7. 路线命令语法（`→ /nav/waypoint`，裸文本）

| 写法 | 含义 |
|---|---|
| `N10 M10 L10 L9 K9` | 裸 token 列表 = 多航点，**最常用** |
| `G10*` | `*` = 该点停车打靶 |
| `R:F11` | `R:` 前缀 = 该段倒车（顺序：`R:` 在前、`*` 在后） |
| `N5,y` / `N5,x` | 朝向后缀，**只允许挂最后一个 token** |
| `K9:SCAN_STOP` | 扫描后缀 |
| `SHOOT@G11` | 指令非坐标：车侧算候选链整链插入 |
| `reset:N10,y` | 复位信念位姿 + 清路线 → IDLE |
| `load:文件名` | 读路径文件并出发 |
| 分隔符 | 空格 / 分号 / 竖线 |

★ **10/11 列**（`K10` / `K11`）曾经被本地校验误判为非法 —— 已修为回归测试
（`test_grid_columns_10_and_11_are_accepted`），别再"顺手"改那两个正则。

---

## 8. 日志

默认落 `~/dzlink_logs/supervisor.jsonl`（Windows 本机默认落在当前目录），JSONL、每次写都 flush
（断电/被 kill 不丢上下文）。关键记录：

| `kind` | 含义 |
|---|---|
| `intel_in` / `intel_emit` / `intel_stale` | 收到/下发/过期（过期后**停止下发**，车侧回落默认） |
| `route_queued` / `route_emit` | 路线入队/下发（含格点个数） |
| `car_ack` / `car_state` / `car_other` | 车回传（`matched` 表示这个 seq 确实是我们发过的） |
| `sample_rejected` / `sample_bad_line` | 认不出/坏行的感知输入（不崩，只记） |
| `cmd_in` / `cmd_rejected` | 命令文件 |
| `subs` | 订阅者匹配数变化（=0 就是没配上） |
| `status` / `final_status` | 周期与退出快照 |

## 9. 停止 / 清理

```bash
pkill -f dzlink_supervisor          # 用 -f（脚本名较长，-x 匹配不到）
# 或往命令文件追加一条： echo '{"type":"stop"}' >> ~/dzlink_tools/cmd.jsonl
```

进程对 `SIGINT`/`SIGTERM` 都做了处理：会先写 `final_status` 再退，**日志不会丢尾部**。
