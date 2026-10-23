# RUNBOOK —— 让小车「等无人机指令再动」

> 目标：`./start_all.sh start` 之后小车**原地待命**，只有收到无人机的 `/nav/waypoint` 指令才动。
> 本文包含：① 要改哪些代码/配置（已落地）② 每次运行的完整流程 ③ 现场判据 ④ 已知坑。
> 全部结论都来自 2026-10-07 的真机实测，不是推断。

---

## 0. 结论先行

| 问题 | 答案 |
|---|---|
| 为什么起栈就自己跑？ | `robot.yaml: mission.auto_start: true` + `path_executor` 启动后自动下发内置路线（★ t16 更正：`auto_start_delay_s` 真值已从 10 s 改为 **45.0 s**，`robot.yaml:525`） |
| 怎么改成"等指令"？ | ① `auto_start: false` ② 给代码加 `DZ_AUTO_START` 环境变量覆盖（保住比赛一键出发） |
| 改完车还会自己动吗？ | **不会**。实测起栈后 35 秒（远超 10 秒延迟）`任 务 开 始` 出现 **0** 次 |
| 比赛一键出发还在吗？ | 在。`DZ_AUTO_START=1 ./start_all.sh start` |
| 怎么发指令？ | 无人机上 `python3 ~/dzlink_tools/dz_send_waypoint.py route "..."` |

---

## 1. 要改的代码/配置（三项，已全部落地）

| # | 在哪 | 文件 | 改什么 | 作用 |
|---|---|---|---|---|
| 1 | 小车 | `~/dzrobot/config/robot.yaml:237` | `mission.auto_start: true` → **`false`** | 起栈不再自动出发 |
| 2 | 小车 | `~/dzrobot/src/car_localization/car_localization/path_executor.py` | 自动出发定时器前加 **`DZ_AUTO_START` 环境变量覆盖** + `★自动出发【已关闭/已开启】` 启动横幅 | 不改 YAML 就能在"待命"和"一键出发"之间切换，且启动即可确认状态 |
| 3 | 小车 | `~/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py` | 与 #2 **逐字节同步** | `ros2 run` 实际执行的是这份 |

### ⚠️ 最大的坑：`src/` 改了不生效

`ros2 run car_localization path_executor_node` 读的是 **install 空间**的副本，不是 `src/`。
实测两份文件在改动前 md5 完全相同（`69e14b0c…`）但**是两份独立文件**（不是符号链接）。

- 只改 `src/` → **运行行为一点不变**（我第一次就中招了：横幅没出现）
- 正确做法：`cp src/... → install/...`，或跑 `colcon build --packages-select car_localization`
- 纯 Python 包，`colcon build` 就是逐字节拷贝，所以直接 `cp` 与重新编译**等价**

### 改动前后（path_executor.py）

```python
# 改前
        if AUTO_START:
            self._auto_start_timer = self.create_timer(
                AUTO_START_DELAY, self._auto_start_mission)

# 改后：三级取值，后者覆盖前者
        _as_env = os.environ.get('DZ_AUTO_START', '').strip().lower()
        _as_eff = AUTO_START                    # 1) YAML mission.auto_start
        _as_src = 'config mission.auto_start'
        if _as_env in ('1', 'true', 'yes', 'on'):        # 2) 环境变量强制开
            _as_eff, _as_src = True, 'env DZ_AUTO_START=' + _as_env
        elif _as_env in ('0', 'false', 'no', 'off'):     # 3) 环境变量强制关
            _as_eff, _as_src = False, 'env DZ_AUTO_START=' + _as_env
        if _as_eff:
            self._auto_start_timer = self.create_timer(
                AUTO_START_DELAY, self._auto_start_mission)
            self.get_logger().warn('★自动出发【已开启】来源=' + _as_src + '：…')
        else:
            self.get_logger().warn('★自动出发【已关闭】来源=' + _as_src + '：车原地待命…')
```

改动由 [patch_car_wait_for_drone.py](files/patch_car_wait_for_drone.py) 完成：精确整串替换
（命中数必须恰好 1，否则一个字节都不写）、自动备份、`py_compile` + `yaml.safe_load` 自检、
自检失败自动回滚、可 `--revert`。

---

## 2. 每次运行的完整流程（4 步）

### 第 1 步 · 摆车（物理动作，软件代替不了）

把车放在 **N10 起点、车头朝 Y+**。

> 为什么：小车定位是 **航迹推算**（`serial_reader` 从里程计积分），开机时它无条件认为
> 自己站在 N10 朝向 Y+。车实际摆在别处，它也会按"我在 N10"去跑。

### 第 2 步 · 起栈（小车）

```bash
cd ~/dzrobot && ./start_all.sh start
```

**两个必看确认点**：

```bash
# ① 起栈输出里应有（域号被钉死在脚本里，不再依赖 .bashrc）
ROS 链路域号: ROS_DOMAIN_ID=118  ROS_LOCALHOST_ONLY=0  (车标准域 118)

# ② 关键：7_path.log 第一行
head -3 ~/robot_logs/7_path.log
# ★自动出发【已关闭】来源=config mission.auto_start：车原地待命，只等外部 /nav/waypoint 指令，不会自己动
```

**看到这行 = 车在等指令。没看到这行就别往下走**（说明改动的第 3 项没同步）。

### 第 3 步 · 体检（无人机）

```bash
# 登无人机
ssh duzhong@dzsmart.local
source /opt/ros/humble/setup.bash
source ~/dzlink_tools/dzlink.env          # ★ 域号 118 的唯一真相源
python3 ~/dzlink_tools/dz_link_check.py
```

期望输出：

```
[1] /nav/waypoint 订阅者数量 = 2   （期望 2：path_executor_node + serial_reader_node）
    ✓ 两个订阅者都在
[2] 发探针 'load:/tmp/dzlink_preflight_no_such_file.txt'（只触发 error 日志，车不会动）
  ← 小车日志 [path_executor_node] Path file not found: /tmp/dzlink_preflight_no_such_file.txt
[3] 结果：
    ✓ PASS —— 小车回执已抓到：「无人机 → /nav/waypoint → path_executor 回调」整条链路是通的
```

> 这条探针**不产生任何位移**：`load:<不存在的文件>` 只会让 `path_executor` 打一条 error 日志。
> 它的价值是——**用不动机器的方式证明"指令真的到了"**。

### 第 4 步 · 发指令（无人机）

```bash
# ① 复位信念位姿（"我现在在 N10，朝 Y+"）—— 每次发路线前都要发
python3 ~/dzlink_tools/dz_send_waypoint.py route "reset:N10,y"

# ② 发路线。第一次联调建议用最短的 3 格，动一下就停：
python3 ~/dzlink_tools/dz_send_waypoint.py route "N10 M10 L10"
```

**判据**（三处任一看）：

| 在哪看 | 看到什么 |
|---|---|
| 车 `~/robot_logs/7_path.log` | `>>>  任 务 开 始  <<<`（★ 这就是"车收到了并出发了"） |
| 无人机（另开一个终端） | `python3 ~/dzlink_tools/dz_link_check.py --watch 30` 实时刷出上面那行 |
| 车 `~/robot_logs/3_localization.log` | `pos: (x, y)` 开始变化 = 车真的在动 |

---

## 3. 现场判据速查

| 我想知道 | 去哪看 | 什么算"过" |
|---|---|---|
| 车在不在待命 | 车 `~/robot_logs/7_path.log` | `★自动出发【已关闭】…车原地待命` |
| 车在不在听 | 任一侧 | `ros2 topic info /nav/waypoint` → `Subscription count: 2` |
| 指令到没到 | 车 `7_path.log` | `>>>  任 务 开 始  <<<` |
| 链路通不通 | 无人机 | `dz_link_check.py` → PASS |
| 车动没动 | 车 `3_localization.log` | `pos: (1.50, 0.50) | block: N10` 这个数一成不变 = 没动 |
| 底盘收到速度没有 | 车 `~/robot_logs/1_chassis.log` | 有非零速度帧 = 在跑 |

### 免费彩蛋：无人机可以直接看小车日志

`/rosout` **跨机可见**（本次实测）。所以不用 SSH 到车上 tail：

```bash
python3 ~/dzlink_tools/dz_link_check.py --watch 60     # 只看 warn/error（默认）
python3 ~/dzlink_tools/dz_link_check.py --watch 60 --all   # 连 INFO 一起看（会被 5Hz 位置行刷屏）
```

---

## 4. 常用变体

```bash
# ── 比赛：一键出发（起栈后自动跑内置路线；★ t16：延时真值 45.0 s，旧口径 10 s）──
cd ~/dzrobot && DZ_AUTO_START=1 ./start_all.sh start

# ── 强制待命（即使 YAML 写的是 true）──
cd ~/dzrobot && DZ_AUTO_START=0 ./start_all.sh start

# ── 内置的完整比赛路线（34 点）──
python3 ~/dzlink_tools/dz_send_waypoint.py route \
  "N10 M10 L10 L9 K9 J9 J10 I10 H10 G10* F10 F11 E11 R:F11 F10 F8 B8 B6 A6 R:B6 R:D6 C6 C4 C2 G2 H2 H1 I1 J1 J4 K4 L4 L5 N5"

# ── 只看链路（不发任何东西）──
python3 ~/dzlink_tools/dz_link_check.py --count

# ── 只发不动的探针（排查"指令到底到没到"）──
python3 ~/dzlink_tools/dz_send_waypoint.py route "load:/tmp/any_no_such_file.txt"
```

路线命令语法（读小车源码核实）：

| 写法 | 含义 |
|---|---|
| `N10 M10 L10` | 裸 token 列表 = 多航点（最常用）；分隔符空格/分号/竖线都行 |
| `G10*` | `*` = 该点停车打靶 |
| `R:F11` | `R:` 前缀 = 该段倒车（**必须在 token 最前面**） |
| `M10,y` | 单点 + 最终朝向（`y`=朝 Y+，`x`=朝 X+） |
| `reset:N10,y` | 复位信念位姿、清空路线（不发路线，只复位） |
| `load:/路径/文件.txt` | 读路径文件并出发 |

---

## 5. 急停与收工

```bash
# 车上：停全部节点（底盘随之失去速度指令）
cd ~/dzrobot && ./start_all.sh stop
```

> 实测：`stop` 后 9 个节点全停、8080 端口释放。
> **注意**：如果车正在跑，急停请优先用**物理方式**（断电/抱死），
> `stop` 的最后一帧速度指令是否被 STM32 保持**未经实测**，不要把它当唯一急停手段。

---

## 6. 已知坑清单（全部本次实测踩过）

| # | 坑 | 现象 | 对策 |
|---|---|---|---|
| 1 | **`src/` 改了不生效** | 代码改了、横幅没出现 | 同步 install 副本，或 `colcon build` |
| 2 | **域号 118 vs 0** | ping/SSH/网页全通，ROS 死活看不见对方，**一个错都不报** | 车 `dzros.env` + 无人机 `dzlink.env` 各自钉死；起栈横幅会打印 |
| 3 | **节点自己把日志压到 WARN** | `Path Executor Node Initialized`、`Odometry reset to …` 全看不见 | `path_executor.py:586` `set_level(WARN)`；只看 warn/error 收据：`★自动出发`、`任 务 开 始`、`Path file not found` |
| 4 | **`grep 任 务 开 始` 匹配不到** | 明明跑过却 grep 出 0 | 真实日志字间**有空格**：`任 务 开 始` |
| 5 | `ros2 topic echo` 默认 RELIABLE | BEST_EFFORT 话题一条都收不到，误判"车没在发" | 显式指定 best_effort |
| 6 | 发送器把订阅者数报成 1 | 实际稳定态是 2，看着像只匹配上一个 | 已修（多等 1.5 s 让发现收敛）；权威读数是 `ros2 topic info -v` |
| 7 | **B 通道（JSON 情报）车侧没人订阅** | `/dzlink/drone_to_car` 发出去石沉大海 | 车上跑的是**旧核 `path_executor`**，`src/` 里根本没有 `dzbrain`。想让无人机影响穿洞 → 只能用 A 通道把洞写进路线 token |

---

## 7. 回滚

```bash
ssh duzhong@duzhong.local

# 状态查看（只读，一个字节都不写）
python3 ~/patch_car_wait_for_drone.py --check

# 从最近一次备份回滚（同时还原 robot.yaml 与 path_executor.py）
python3 ~/patch_car_wait_for_drone.py --revert
```

备份目录：`~/dzrobot/_backups/autostart_wait_<时间戳>/`（含 `path_executor.py` + `robot.yaml`）。

---

## 8. 本次实测验证记录（2026-10-07）

| 验证项 | 命令 | 结果 |
|---|---|---|
| 分支 A：默认待命 | 单独跑 `path_executor`（不起底盘）13 s | `★自动出发【已关闭】来源=config mission.auto_start`；`任 务 开 始` = **0** ✓ |
| 分支 B：环境变量强制开 | `DZ_AUTO_START=1` 单独跑 16 s | `★自动出发【已开启】来源=env DZ_AUTO_START=1：10.0s 后自动下发内置路线 […]`；10 s 时出现 `自动出发：34 个坐标点` + `>>>  任 务 开 始  <<<` ✓ |
| 整车起栈 35 s | `./start_all.sh start` | 横幅【已关闭】、`任 务 开 始` = **0**、车没动 ✓ |
| 订阅关系 | 无人机 `ros2 topic info /nav/waypoint -v` | `Subscription count: 2`（`path_executor_node` RELIABLE + `serial_reader_node` RELIABLE）✓ |
| 跨机可见性 | 无人机 `ros2 node list` | 12 个小车节点 ✓ |
| **指令送达** | 无人机发 `load:/tmp/dzlink_ping_nonexistent.txt` ×2 | 车 `7_path.log` 出现 2 条 `[ERROR] Path file not found: …` —— 发 2 收 2 ✓ |
| 链路体检工具 | `dz_link_check.py` | 订阅者 = 2，探针回执抓到，**PASS** ✓ |
| 车全程静止 | 车 `3_localization.log` | `pos: (1.50, 0.50) | block: N10` 12 秒内一成不变 ✓ |
| 全程未出发 | 车 `7_path.log` | `任 务 开 始` 总计 **0** 次 ✓ |

**注**：`reset:N10,y` 的收据是 INFO 级、被 `path_executor.py:586` 的 `set_level(WARN)` 过滤，
所以在日志里看不到（不是没生效）。Humble 里没有 `SetLoggerLevel` 服务，无法在线抬级别；
要确认"任何指令都能到"，用坑 #3 的 error 级探针。

**收尾状态**：小车节点已全部停止（9/9），8080 无监听，`auto_start: false` 已就位，
备份齐全。车停在原地、没有任何待执行的路线。
