# 补丁 05 —— 无人机 → 小车：航点 / 情报链路

| 项 | 值 |
| --- | --- |
| 无人机目标路径 | `~/dzlink_tools/dz_send_waypoint.py`（新建目录，不覆盖任何现有文件） |
| 小车目标路径 | **不需要放任何文件** —— 小车的两个订阅入口是现成的 |
| 本地文件 | `files/dz_send_waypoint.py` |
| 是否要编译 | **不要**（纯 Python + rclpy，两端 Humble 自带） |
| 风险 | 极低（只发不收，不动两端源码/配置/环境变量） |

---

## 一、结论：不用写桥接，链路早就通了

2026-10-06 已经实测过两端 DDS 直连：双向信标 `LINK_OK`，且 200 Hz 的 `/livox/imu` 实时到达小车。
两端环境本来就一致（都是 Humble、`ROS_DOMAIN_ID` 未设=0、RMW 默认 FastDDS）。

**所以"无人机发航点、小车收航点"这件事，缺的不是代码，是三样东西对齐：话题名、QoS、报文格式。**
本补丁把这三样钉死，并给出一个发送器。

---

## 二、小车的两个入口（全部读车侧源码核实，不是猜的）

车侧源码在本地：`D:\User\Desktop\dzrobot_v2\`（即部署到车上 `~/dzrobot` 的那份）。
契约集中在 `src/dzbrain/dzbrain/io/contract.py`：

| 入口 | 话题 | 类型 | QoS | 用途 | 证据 |
| --- | --- | --- | --- | --- | --- |
| **A · 路线** | `/nav/waypoint` | `std_msgs/String` | **RELIABLE** / VOLATILE / depth 10 | 直接下发路线，车立即出发 | `contract.py:73` |
| **B · 情报** | `/dzlink/drone_to_car` | `std_msgs/String`（JSON） | **BEST_EFFORT** / VOLATILE / depth 10 | 桥洞/靶区/雷障情报 | `contract.py:94` |
| 回传 | `/dzlink/car_to_drone` | `std_msgs/String`（JSON） | BEST_EFFORT | 车状态 `car_state` / 回执 `ack` | `contract.py:96` |

> **QoS 必须对上**：B 通道车侧是 BEST_EFFORT。反过来（车 RELIABLE、机 BEST_EFFORT）会**完全收不到**，
> 而且不报错 —— 这是跨机 ROS 2 最经典的"静默失败"。发送器已按上表逐条设好。

另外两条有用的车侧事实（**2026-10-07 登真车核实后已更正**）：

- ⚠️ **更正**：车上**跑的是旧核**。实测 `~/dzrobot/start_all.sh:197` =
  `ros2 run car_localization path_executor_node`；而且车上**根本没有 dzbrain** ——
  `ros2 pkg prefix dzbrain` → `Package not found`、`import dzbrain` → `ModuleNotFoundError`、
  `~/dzrobot/src/` 下无 dzbrain 目录、脚本里也**没有** `LEGACY_PATH` 开关。
  本文档早先"第 7 步默认跑新核"的说法读的是本地 `D:\User\Desktop\dzrobot_v2` 源码树，**与车上的实际部署不符**。
- **A 通道：现役旧核就吃**（`path_executor.py:757` 订阅 `/nav/waypoint`）—— 2026-10-07 闭环实测已通。
  **B 通道：当前部署下没人吃** —— 旧核全文 `grep -i dzlink` = **0 命中**，JSON 情报车侧收不到。
  → **想让无人机影响穿洞选择，现在只能走 A 通道：把选定的洞直接写进路线 token**（如 `… L9 K9 J9 …`）。

---

## 三、A 通道：直接下发路线（建议先用这条）

报文就是**一段裸文本**，语法由 `route/parser.py` + `tasks/mission.py:870-943` 定义：

| 写法 | 含义 |
| --- | --- |
| `N10 M10 L10 L9 K9` | 裸 token 列表 = 多航点路线，**最常用** |
| `多点 N10 M10` / `multi …` / `route …` | 同上，前缀只为日志可读 |
| `单点 N10,y` / `single …` / `goto …` | 单点（裸的单个 `N10,y` 也当单点） |
| `G10*` | `*` 后缀 = 该点**停车打靶** |
| `R:F11` | `R:` 前缀 = 该段**倒车到位**（必须在最前） |
| `R:B5*` | 倒车 + 射击（**唯一合法顺序：`R:` 在前、`*` 在后**） |
| `SHOOT@G11` | **指令不是坐标**：调 shoot_planner 算候选链并整链插入，共用一个组号 |
| `:SCAN_STOP` | 扫描后缀（顺序恒定：`R:` → 格名 → `*` → `:SCAN_STOP` → `,x\|,y`） |
| `N5,y` / `N5,x` | 朝向后缀，**只允许挂在最后一个 token**（写中间会被自动挪到最后） |
| `reset:N10,y` | 复位信念位姿 + 清路线 + 回 IDLE（**不允许带 `R:`/`*`/扫描后缀**） |
| `load:<文件名>` | 读路径文件并出发 |
| 分隔符 | 空格 / 分号 / 竖线 都行 |

**行为**：收到即 `_start_mission()` 出发；未知命令只告警不崩（`mission.py:866-868`）。

---

## 四、B 通道：JSON 情报（决定每局穿哪个桥洞）

场景来自 `config/map.yaml:19-21`：
> "每次穿洞的具体格是**每局随机**的，只有无人机能看到桥顶标识 —— 由无人机情报选格。"

字段由 `tasks/drone.py:204-327` 定义（必须是 **JSON 对象**、`type` 字段选分支）：

| `type` | 字段 | 车侧动作 |
| --- | --- | --- |
| `heartbeat` | `ts`（可选 `seq`） | 刷新心跳；**3 s** 没有就判离线（不阻塞任务） |
| `bridge_passable` | `half`(`lower`/`upper`) + `block`(如 `K9`) + `confidence` | 写 `P_DRONE`；Mission 据此选穿洞格 |
| `target_zone` | `blocks: [...]` | 与本地 `map.yaml` 的 `targets` 核对 → `field_diff` |
| `field_verify` | `mines: [...]` / `obstacles: [...]` | 同上（雷/障差异） |
| 任意 | **带 `seq`** | 车回 `{"type":"ack","of":<seq>}` 到 `/dzlink/car_to_drone` ← **验收到的硬证据** |

**时效语义**（决定你什么时候发）：
> ★ **口径更新（2026-10-12 t16，以车侧真值为准）**：`bridge_info_wait_s` 已从 **20 s → 2.0 s**
> （`robot.yaml:546`），`auto_start_delay_s` 从 **10 s → 45.0 s**（`:525`），默认穿洞格从
> **K9/K4 → K10(下)/K5(上)**（`:520-521`）。下面三行是**当时（2026-10-07）**的口径，已过期，
> 请按新值执行；另外 2 s 窗口只管「两半场相隔 ≤2 s 到达」。

- 情报过期 `drone.info_stale_s = 30 s` → `bridge_of()` 返回 None；
- Mission 最多等 `mission.bridge_info_wait_s = 20 s`，等不到就用 `route.cross_lower_default`(K9) / `cross_upper_default`(K4)；
- **所以：要在发车（`auto_start_delay_s=10 s`）之前发，并且最好在 20 s 窗口内**。
  （★ 新值见上方口径更新：发车延时 **45 s**、等情报窗口 **2 s**。）

---

## 五、前提与当前现状（必须知道）

1. **两端要在同一个二层网络**、`ROS_DOMAIN_ID` 一致（历史都是未设=0）、RMW 一致（默认 FastDDS）。
   DDS 靠**组播发现**，**不需要 DNS/mDNS**。
2. **当前网络已变**（实测 2026-10-07）：
   - 小车 `duzhong.local` → **10.226.211.188**，可达 ✅
   - 无人机 `dzsmart.local` → **mDNS 解析失败**；`10.240.174.104` / `10.158.138.104` 均不通 ❌
   → **先把两台机器接到同一个 AP**，再谈发航点。
3. **新 AP 是否允许组播没有实测过**（老 AP 实测允许）。若发送器报"订阅者匹配数=0"，先判定组播：
   ```bash
   # 小车先跑（会等 15 秒）:
   source /opt/ros/humble/setup.bash && ros2 multicast receive
   # 无人机紧接着跑:
   ros2 multicast send
   ```
   收不到 → AP 开了客户端隔离，改用 `tools/link/fastdds-unicast-wifi.xml` 单播兜底。

---

## 六、落地步骤（用户手动执行）

```bash
# 1) 本地文件：fc-audit/fixes/05-drone-to-car-waypoint/files/dz_send_waypoint.py
#    用 scp / VNC 拖拽 / 复制粘贴送到无人机，放在（需要新建目录）：
mkdir -p ~/dzlink_tools

# 2) 无人机上（每个终端先 source 两句）
source /opt/ros/humble/setup.bash

# 3) 先干跑看报文格式（不发任何东西）
python3 ~/dzlink_tools/dz_send_waypoint.py contract
python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K9 upper=K4 --dry-run

# 4) 小车侧起栈（车上的事，别和无人机混）
cd ~/dzrobot && ./start_all.sh restart     # 第 7 步 = dzbrain
```

> 文件是 UTF-8、LF。若用复制粘贴导致 `\r`，先 `sed -i 's/\r$//' ~/dzlink_tools/dz_send_waypoint.py`。

---

## 七、用起来（无人机侧）

```bash
# ① 发桥洞情报（发 3 遍；车超过 30s 就过期）
python3 ~/dzlink_tools/dz_send_waypoint.py bridge lower=K9 upper=K4 --seq 1

# ② 直接下发一条完整路线（* = G10 停车打靶）
python3 ~/dzlink_tools/dz_send_waypoint.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"

# ③ 挂心跳（3s 不刷新就判离线）
python3 ~/dzlink_tools/dz_send_waypoint.py heartbeat --duration 120

# ④ 看小车回传（car_state/ack）—— 证明"车收到了且活着"
python3 ~/dzlink_tools/dz_send_waypoint.py listen --duration 30
```

**不想装脚本也行**（A 通道一行就够）：

```bash
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "{data: 'N10 M10 L10 L9 K9'}"
```

> ⚠️ B 通道不建议用 `ros2 topic pub` 手写：JSON 里嵌引号，在 PowerShell → SSH 链路上会被吞引号
> （本仓库有两次踩坑记录），用脚本安全得多。

---

## 八、验证（三级判据，缺一不可）

**L1 发现层**（两端各跑，看得到就行）

```bash
ros2 topic info /nav/waypoint -v            # 应有 Publisher（无人机）与 Subscription（车）
ros2 topic info /dzlink/drone_to_car -v     # 同上
ros2 node list | grep -i dzbrain            # 车侧应有 dzbrain
```
发送器自己也会打印 `订阅者匹配数=N`：**N≥1 才说明 DDS 真的配上了**（=0 时脚本退出码 3）。

**L2 收报层**（车侧日志）

```bash
tail -f ~/robot_logs/7_path.log
# A 通道应出现命令解析/出发相关日志；B 通道应出现：
#   桥洞情报：lower 半场 → K9（置信度 0.90，第 1 次）
# 若出现「⚠️ 外部命令无效（已忽略）」= 格式不对；「未知无人机消息类型」= type 写错
```

**L3 动作层**（真正的判据）
- A 通道：车**真的开始走**，且日志的航点数与你发的 token 数一致。
- B 通道：`ack` 回到无人机（用 `listen` 看得到），且车日志的穿洞格 = 你发的 `block`。

**反面判据（最先排的三种）**：匹配数 0 / 车没起栈（第 7 步没跑）/ 组播被 AP 拦。

---

## 九、回退

```bash
rm -rf ~/dzlink_tools        # 无人机侧
```
小车侧**本来就没改任何东西**，无需回退。

---

## 十、如实交代：未验证项

1. **发送器没有在真机上实测过** —— 无人机当前不在线（第五节的实测结果），
   所以只做了本地验证：`py_compile` 通过、4 个干跑分支输出正确、非法入参被拒（退出码 1）、
   无 ROS 环境时退出码 2 且给出 `source` 提示。
2. **没有登录小车核对在跑版本**。上面所有契约读的是本地 `dzrobot_v2` 源码。
   若车上实际跑的是 `LEGACY_PATH=1` 的旧 `path_executor`：
   A 通道照常可用（旧核 `path_executor.py:757` 订阅同一话题，语法兼容），**B 通道无效**。
3. **新核的 `load:` 是否接受绝对路径**没验证（`mission.py:882-888` 只做了非空检查，路径解析在 `_load_tokens` 里）。
4. 报文里的 `ts` 字段：车侧实际用的是**自己的本地时钟**（`drone.py:284`），
   我按文档把 `ts` 补上了，但它不参与判据；两端时钟没对齐也不会导致误判。
5. `bridge_passable` 的 `block` 车侧**不做地图内校验**（`drone.py:280-281` 明确注释"无人机才是权威"），
   写错格名不会立刻报错，会在 Mission 路线校验那一层兜住。

---

## 十一、2026-10-07 现场实测：链路已闭环打通 + 域号坑（★ 必读）

### 11.1 闭环实证（硬证据，两面都有输出）

两端 BSSID 相同（`7a:4b:aa:42:26:fc`，单路由器，网关 MAC = BSSID），小车 `wlan0` = `10.226.211.188`，
无人机 = `10.226.211.104`。车侧只挂一个临时订阅者（**不起栈、不动车**）：

```
[无人机] MATCHED_SUBSCRIPTIONS=1 after 0.2s (domain=118)
[无人机] PUBLISHED#1 'N10 M10 L10 L9 K9' subs=1
[小车  ] car probe listening: domain=118 topic=/nav/waypoint timeout=90s
[小车  ] RECEIVED: N10 M10 L10 L9 K9
[小车  ] RESULT received=1 waited=5.0s
```

→ **"无人机发航点、小车收航点"这件事本身已经完全通了**，不需要再写任何桥接代码。

### 11.2 ★ 域号坑：这是本轮"怎么都发现不了对方"的真正原因

| 事实 | 证据 |
| --- | --- |
| 小车**在 ROS 域 118** | 车侧 `echo $ROS_DOMAIN_ID` → `118` |
| 域号**唯一来源**是 `~/.bashrc:126` 的 `export ROS_DOMAIN_ID=118` | 全盘 `grep -rn ROS_DOMAIN_ID`（排除 install/build）**只有这一处命中** |
| `start_all.sh` 里**没有**域号 | `grep -n ROS_DOMAIN_ID ~/dzrobot/start_all.sh` → 空 |
| 无人机/PC 侧未设 = **域 0** | 两端 `ROS_DOMAIN_ID` unset |

**后果（赛场翻车点）**：
- 从**交互式终端** `./start_all.sh start` → 栈继承 **118**；
- 从 **systemd / cron / `ssh host 'cd ~/dzrobot && ./start_all.sh start'`** 非交互启动 → 域 **0**。
- 两种启动方式域号不同，**表现完全一样：ping 通、SSH 通、网页台通，ROS 就是看不见对方**，且**不报任何错**。
- 域 118 的组播发现端口是 `239.255.0.1:36900`（= 7400 + 250×118），域 0 是 `7400` ——
  所以在 7400 上嗅探**永远**看不到车主，会误导排查方向。

**命令行速查**：域 d 的发现端口 = `7400 + 250·d`（组播），用户端口 = `7401 + 250·d`。

### 11.3 建议的加固（二选一，别留第三种状态）

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| **A（推荐）保持 118** | 车侧在 `start_all.sh` 顶部加 `export ROS_DOMAIN_ID=118`（域号不再依赖 `.bashrc`）；无人机侧在发送器前统一 `export ROS_DOMAIN_ID=118`（建议落成 `~/dzlink_tools/dzlink.env` 并由发送器 source 并打印） | 改动最小；车/无人机 DDS 流量天然隔离，无人机的雷达话题不会跑进车的域 |
| **B 统一到 0** | 车侧注释掉 `.bashrc:126` 并在 `start_all.sh` 里 `export ROS_DOMAIN_ID=0` | 与历史文档一致，但要动车的环境，且需要重启栈 |

**无论选哪个，都要写进启动脚本**（不要只靠 `.bashrc`），否则换一种启动方式就静默失效。

### 11.4 本轮顺带更正的两个错误结论（都是我的错）

1. ❌ **"跨 AP 组播不互通"** —— 不成立。车/无人机/PC **同一个 BSSID**，且网关 MAC 就是 BSSID（单路由器）。
2. ❌ **"det_node 等 3 个包缺失、栈会在 4/8 挂掉"** —— **不成立**。
   实测 `ros2 pkg prefix det_node` → `/home/duzhong/dzrobot/install/det_node`；
   车侧 `4_det.log` 里 det_node **成功加载模型并跑起来**：
   `视觉检测: 模型=…/light_det.rknn | 线程=8 | 推理周期=33ms | 窗口=开`。
   上次那个"Package not found"是**我的检查脚本的 bug**：`. setup.bash 2>&1 | head` 把 source 放进了管道子 shell，
   工作区环境没进父 shell，于是所有工作区包都"找不到"。
   （`fake_amcl_node` / `openslam_gmapping` 确实不存在，但**当前 launch 根本不用它们** ——
   `localization.launch.py` 只起 `serial_reader_node`，而 `/odom/localization` 正是它发布的
   （`serial_reader.py:108`）。）
3. ⚠️ 另一条方法论教训：**`ros2 topic echo` 在话题还没有发布者时会"解析不到类型"并立刻退出**，
   用它做"等消息"的测试会产生**假阴性**。要等消息请用类型写死的 rclpy 订阅者
   （本仓库 `tools/link/probe/car_wait.py`）。

### 11.5 车侧 19:13 那次实跑（日志留痕，说明车的栈本身是好的）

```
[7_path] 已重锚里程计到 N10,y
[7_path] 路线构成：共 34 点 | 普通 30 | 倒车(R:) 3 | 射击(*) 1
[7_path] 自动出发：34 个坐标点 -> N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* … L5 N5
[7_path] >>> 任 务 开 始 <<<  航点数 34 | 扫描 continuous | 巡航 0.15 m/s | 桥洞保护 K1..K11
[7_path] ★进入桥洞强制直行保护 航段#5（锁航向 180.0deg，限速 0.1 m/s，关纯追踪转向）
```

其余 7 路（底盘/相机 usb_cam/定位/视觉/云台/射击/网页台）日志均正常；只有 `8_browser.log`
因宿主机没装 chromium snap 而起不来（**无关紧要**，网页台本身 `http://10.226.211.188:8080` 正常）。
日志尾部那条 `No publisher on /odom/localization yet` 出现在停栈瞬间（`pids/` 已清空 = 正常 stop），
**下次起栈后应在 3 秒健康检查里确认它不再出现**（`path_executor.py:840-881` 自带该检查）。

### 11.6 现在怎么用（已验证可用的最短路径）

```bash
# ── 小车侧 ──
cd ~/dzrobot && ./start_all.sh start      # ⚠️ auto_start=true → 10 秒后自动出发，先决定悬空/改配置
# 确认订阅者存在（判据：Subscription count: 1）
source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=118 ros2 topic info /nav/waypoint -v

# ── 无人机侧（域号必须跟车一致！）──
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=118
python3 ~/dzlink_tools/dz_send_waypoint.py route "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 N5"
# 发送器会打印「订阅者匹配数」：≥1 才算真的配上；=0 别怀疑车，先查域号/车的栈
```

**时序**（★ 2026-10-12 t16 更正口径）：`auto_start=true` 时车在起栈后自动出发 ——
当前真值是 `mission.auto_start_delay_s = **45.0 s**`（`robot.yaml:525`，当时是 10 s），
内置路线默认穿 **K10(下)/K5(上)**（当时记的 K9/K4 已过期）。
无人机要么在出发前把路线发下去，要么运行中重发（旧核收到新 `/nav/waypoint` 会重新出发）。
若要"等无人机情报再发车"，把 `config/robot.yaml` 的 `auto_start` 改 `false`（改前备份）。

### 11.7 本轮落盘的只读探针（可复用）

| 文件 | 作用 |
| --- | --- |
| `tools/link/probe/car_wait.py` | 车侧订阅者探针（类型写死，收到一条即打印并退出，可指定超时） |
| `tools/link/probe/drone_send.py` | 无人机侧发布探针（打印**订阅者匹配数**后发 N 条） |
| `tools/link/probe/car_probe.sh` / `drone_probe.sh` | 两端包装（域号取 `DZ_DOMAIN`，默认 118） |
| `tools/link/probe/car_recon*.sh` / `car_logs.sh` | 车侧只读侦察（环境/端口/包/启动逻辑/日志） |

> 传输方式：两端 **LF + md5 逐字节核对**（本轮实测 car_wait.py `c9779c8d…`、drone_send.py `50eef119…`）。
> ⚠️ 这些探针都**只读**：不起车上的栈、不发运动指令、不改车上任何文件。

---

## 十二、方案 A 已落地：域号收敛到唯一真相源（2026-10-07 19:52–19:57 实测）

用户决定：**选 A —— 保持 118**，把它钉进脚本，不再依赖 `~/.bashrc`。

### 12.1 车侧：1 个新文件 + 5 个入口

| 文件 | 改动 | 装完 md5 |
| --- | --- | --- |
| `~/dzrobot/dzros.env` | **新建**（唯一真相源） | `43e35e36ab6b5cd58e9d3cb574e4f33a` |
| `~/dzrobot/start_all.sh` | 改为 source `dzros.env`（保留内置兜底 118）+ 启动横幅打印域号 | `86c51556b5820c1a13bb004bc76d9749` |
| `~/dzrobot/monitor_aim.sh` | +2 行：source `dzros.env` | `290731e46b0c123c504e667dba5eeafd` |
| `~/dzrobot/tools/healthcheck.sh` | +2 行（缩进 2 空格，在 `set +u` 之后） | `ccdcf3b6c3c8bb88df651db34fe95afa` |
| `~/dzrobot/tools/restart_node.sh` | +2 行 | `3bf7f6b658185bb4f30c232aa1353d71` |
| `~/dzrobot/tools/test_voice.sh` | +2 行 | `14d7aff6aa66e9c88d2747a11689650c` |

车侧只读侦察确认过：**没有 systemd 自启单元、没有 crontab**，所以 ROS 入口就这 5 个，全部覆盖。
备份都在 `~/dzrobot/_backups/*.bak_20261007_1952xx / 1956xx`，回退命令见 `tools/link/patches/README.md`。

### 12.2 无人机侧：1 个新文件 + 发送器 3 处

| 文件 | 改动 | 装完 md5 |
| --- | --- | --- |
| `~/dzlink_tools/dzlink.env` | **新建**（与车侧逐字节对应的域号文件） | `a2cae2c0c01707a80fd562d52e47f76b` |
| `~/dzlink_tools/dz_send_waypoint.py` | 352 → 406 行：import 阶段自动读 env（晚于 `rclpy.init` 就无效）+ 每个子命令打印域号 + 无订阅者分支直接报域号 | `49173daa23f71d20e80c9a122d033942` |

### 12.3 验证证据（全部实测，不是推断）

| 用例 | 场景 | 期望 | 实测 |
| --- | --- | --- | --- |
| **A** | **空环境**（= systemd / cron / ssh 一行命令起栈） | 118 | **118** ✅ |
| B | 外部显式 `ROS_DOMAIN_ID=7` | 7 + 横幅报警 | 7 + 报警行 ✅ |
| C | 交互式终端 | 118 | **118** ✅ |
| D | `dzros.env` 缺失 | 回退内置 118 | **118** ✅ |
| E | `dzros.env` 写 200 | 全车跟随（证明"只改一个文件"） | **200** ✅ |
| F | 4 个辅助脚本插入的那 2 行逐字跑 | 118 | **118** ✅ |

外加：5 个入口 `bash -n` 全过、且在**表格里逐个核对了"接 dzros.env 的行号 < source ROS 的行号"**。

无人机侧：`contract` / `route --dry-run` 首行都打印 `ROS 域: 118（来自 dzlink.env）`；
真实 rclpy 发送一次 → `订阅者匹配数=0`（车侧栈没跑，符合预期）+ 新诊断行 + `exit 3`；
反证 `ROS_DOMAIN_ID=0` → **报警**而非静默走错域。

### 12.4 顺带挖出并修掉的同源隐患

车侧原本还有 4 个会起 ROS / 跑 `ros2` 命令但**不设域号**的入口，其中
**`tools/restart_node.sh`（单独重启节点）如果从非交互式环境调用，会把该节点起在域 0
—— 直接形成"一个节点与全车失联"的静默故障**；`tools/healthcheck.sh` 则会给假警报。
已一并接到 `dzros.env` 上。

### 12.5 唯一还没做的一件事

**真实起栈验证**（判据：车侧 `ros2 topic info /nav/waypoint -v` 的 Subscription = 1）。
它必须起车上的栈，而 `config/robot.yaml` 是 **`auto_start: true`**、
`auto_start_delay_s: **45.0**`（★ t16 更正：当时记的 10.0 已改，行号也从 237 移到 `mission:` 段 525）
—— **起栈 45 秒后车就会用内置路线出发**。
所以这一步等用户确认车处于悬空/安全状态（或临时把 `auto_start` 改 `false`）后再做。

正确时序（`auto_start=true` 时）：
1. 车侧 `./start_all.sh start`，输出里必须有 `ROS 链路域号: ROS_DOMAIN_ID=118 …`；
2. 等第 7 步 `7_path` 起完；
3. 无人机侧 `python3 ~/dzlink_tools/dz_send_waypoint.py route "…"` → 看**订阅者匹配数 ≥ 1**。

### 12.6 本轮踩到的 3 个工具坑（都不是车的问题）

1. **`source /opt/ros/humble/setup.bash` 不能在有 `set -u` 的脚本里裸调** ——
   它第 8 行展开未定义变量 `AMENT_TRACE_SETUP_FILES`，脚本会**静默终止**。
   我自己的验证脚本就复现了一次（车侧 `start_all.sh:135-137` 早有注释警告过同一个坑）。
2. **PowerShell 5.1 给 `ssh.exe` 传参会吃掉双引号**：`ssh host 'echo "a b"'` 到远端变成 `echo a b`。
   症状是"命令行为莫名不对"或 grep 静默无输出（本轮两次中招）。可靠做法：**远端要跑的东西写成脚本 scp 过去**。
3. **`pgrep -f 'ros2 launch'` 会匹配到 ssh 会话自己的命令行**，永远返回 1（虚惊）。
   改按可执行名 `comm` 过滤即可 —— 已固化成 `tools/link/patches/car_process_check.sh`。
4. 附带：`dzsmart.local`（mDNS）解析本轮**间歇性失败**，同一时刻按 IP 直连完全正常。
   脚本里**别写 mDNS 名字**，或至少准备 IP 兜底。

---

## 十三、2026-10-07 20:08 真机起栈验证：**链路端到端通过**（方案 A 收口）

用户批准后执行：备份并临时 `auto_start=false` → 起栈 → 跨机确认 → 停栈 → 逐字节还原配置。

### 13.1 起栈输出（关键行）

```
ROS 链路域号: ROS_DOMAIN_ID=118  ROS_LOCALHOST_ONLY=0  (车标准域 118)
1/8 底盘 … 8/8 网页弹窗     9 路进程全部拉起（新增 5b 自瞄转发）
```

这一行就是方案 A 要的效果：**从 start_all.sh 起栈，无论先前环境如何，域号都是 118。**

### 13.2 判据达成：无人机侧看到的车

```
=== /nav/waypoint 端点 ===
Type: std_msgs/msg/String
Publisher count: 0
Subscription count: 2
  ├─ Node name: path_executor_node     Reliability: RELIABLE  ← 与发送器 QoS 精确匹配 ★
  └─ Node name: serial_reader_node     Reliability: RELIABLE   ← 车侧第二个订阅者
```

无人机（域 118）看到的**车侧节点全图**：
`/path_executor_node` `/serial_reader_node` `/det_node` `/dzactuator` `/dzjudgment`
`/dzrknn_dataprocess` `/cam_viewer` `/usb_cam_node` `/aim_status_shim_in/out`
`/static_transform_publisher_*` ×2 —— **12 个节点，跨机可见**。

> 注：判据原写"Subscription count: 1"，实测是 **2**（`serial_reader_node` 也订阅 `/nav/waypoint`）。
> **≥1 即可**；发送器的 `订阅者匹配数` 会按实际节点数报。

### 13.3 车全程原地待命（已证）

`start_all.sh start` → 等待 → `stop`，然后读**进程退出后 flush 出来的** `7_path.log`：

```
grep -c '任务开始|自动出发'  →  0
```

`auto_start: false` 生效，车没有自动出发。日志里只剩停栈时的 rclpy 退出 traceback（正常噪声）。

### 13.4 收尾状态（已逐项核对）

| 项目 | 状态 |
| --- | --- |
| `config/robot.yaml` | **逐字节还原**（md5 `e6ec73233cdd1aba671358a68293e902` == 改动前备份） |
| `auto_start` | 已恢复 `true` |
| 栈 | 已停（受管 PID 文件 0、8080 无监听） |
| 备份 | `~/dzrobot/_backups/` 共 8 份（含 `robot.yaml.prerestore_*`） |

### 13.5 本轮新增的可复用工具

| 工具 | 作用 |
| --- | --- |
| `tools/link/patches/patch_car_autostart.py` | 开关车侧 `auto_start`（`--on/--status/--restore`，改前备份 + YAML 复核 + 失败回滚） |
| `tools/link/patches/drone_link_check.sh` | 无人机侧一条命令看清域号 + `/nav/waypoint` 端点 + 车侧节点图 |
| `tools/link/patches/car_process_check.sh` | 车侧进程体检（按 `comm` 过滤，避开 `pgrep -f` 自匹配） |

### 13.6 ★ 两个会误导判断的坑（下次别再踩）

1. **`7_path.log` 是 0 字节 ≠ 节点没起来。** Python 的 stdout 重定向到文件是**块缓冲**，
   不满 4KB 不落盘；节点退出（停栈）时才 flush。本轮就是这个现象 ——
   当时节点**早已作为订阅者出现在 DDS 图里**（`Subscription count: 2`）。
   **判活要用 `ros2 topic info` / 节点列表，不要看日志大小。**
2. **`ros2 topic echo` 默认 RELIABLE，收不到 BEST_EFFORT 话题。**
   本轮 `/nav/motion_state` 明明 `Publisher count: 1`，`echo --once` 15 秒一条都没有。
   要 echo 这类话题必须显式指定 best_effort QoS，否则会得出"车没在发"的错误结论。

另外两条既有（非本次改动引入）的噪声，看到不必惊慌：
- 停栈时 `start_all.sh: 行 80: /proc/<pid>/cmdline: 没有那个文件或目录` ——
  `cleanup_strays` 遍历 `/proc` 时进程恰好退出，纯竞态，不影响回收（本轮正常停了 9 个）。
- `path_executor_main` 退出时的 `ExternalShutdownException` + `failed to shutdown: rcl_shutdown already called` ——
  rclpy 收到 SIGTERM 的标准噪声。
