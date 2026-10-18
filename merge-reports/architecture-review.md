# air_ground 架构与代码质量评审报告

**评审对象**：`C:/Users/29212/Documents/Trae-CN/default/` 下的 air_ground 体系
**评审日期**：2026-10-17
**评审方法**：`risk-quality-reviewer` + `explore`（architecture-visualization 插件）。全文逐文件读取 + 命令实测验证。
**规模**：335 个 `.py`（62,428 行）+ C++/TensorRT 机载栈 + 部署工具链 + 5 份进度纪要。

> **证据分级**：`已验证` = 我亲自读代码/跑命令确认；`Agent 报告` = 子代理逐行阅读后报告，我抽样复核了其中的 P0 项；`需验证` = 有怀疑但未能在仓内闭环证实。

---

## 一、一句话结论

**算法与工程直觉明显高于一般学生项目（角度 wrap 无一处遗漏、引擎形状契约校验、RAII、部署带 md5 校验与自动回滚），但整个体系缺三样东西：版本控制、随包测试、失效安全。而最致命的是——真正要飞的那份代码，比你手上测试过的那份少了 10 个函数，其中包含路线正确性校验器。**

按现状外场演示，最可能的失败不是"算法不对"，而是**"大脑猝死 / 车带着满舵不停 / 复盘时拼不出时间轴"**。

---

## 二、系统架构画像（现状 current-state，证据：code）

### L2 容器视图

| 单元 | 位置 | 职责 | 语言/运行时 |
|---|---|---|---|
| `dzuav` | 无人机 Orin | RealSense → TensorRT D13 检测 → 格号分类 → 单应/PnP 定位 → UDP 广播 | C++17 / CUDA / TensorRT |
| `ag_integrate` | 无人机 Orin（ROS2） | 收 UDP 态势 → 世界模型 → 门控 → TSP+Dijkstra 规划 → 发布路线 | Python / rclpy |
| `dznav` | 地面车（ROS2） | 收路线 → 参考路径 → pure pursuit 跟踪 → 舵机/油门 | Python / rclpy |
| `定位栈 v2.1` | 无人机 Orin | Livox → FAST-LIO → jump guard → EKF → `/localization` | 独立包（本次未深审） |
| 部署工具链 | 开发机（Win） | paramiko/SSH 推送 + 编译 + 体检 + 回滚 | Python / PowerShell / bash |

### 数据流（已验证）

```
RealSense ──▶ dzuav(HbbDetector→CellClassifier→MapLocalizer)
                  │ UDP:50000  0x4001 态势帧 / 0x4002 车位姿帧
                  │ ⚠ 仅在 localized==true 时才发送（main.cpp:186-187）
                  ▼
        ag_integrate/integrate_node（后台收包线程 → queue(maxsize=1)）
                  │ 10Hz timer 回调内：evaluate_gate → build_mission
                  │   └─ pipeline.py:73,96 → mission_router.plan_route_zoned
                  │      └─ Held-Karp TSP（mission_router.py:547-549，2^n 无上限）
                  ▼ ROS2 String
        /nav/waypoint  ("reset:N10,y" + 路线 token 串)
                  ▼
        dznav/executor_node（解析 → refpath → pursuit 50Hz）
                  ▼ Twist
        /cmd_vel ──▶ dzactuator ──▶ 舵机/电机
```

### 死代码（已验证，Agent 报告 + 我复核 import 图）

`evidential_map.py`、`ground_projection.py`、`anchored_registration.py` 三个模块**全包无任何 import**。实际落格由机上 C++ `MapLocalizer` 完成，Python 侧只做格号级态势。这三个模块的 docstring 都声称"测试锁定"——而被引用的测试不存在于任何部署包内。

---

## 三、P0 级发现（外场前必须处理）

### P0-1 部署的代码不是测试的代码，且缺少路线正确性校验器 `已验证`

这是本次评审最重要的发现，链条完整：

1. **`mission_router.py` 存在 4 个版本**（实测行数）：

   | 副本 | 行数 |
   |---|---|
   | `路径规划交接_20261017/` | **1585** |
   | `_tmp_agplan/` | 1503 |
   | `无人机上机包_20261017/…/air_ground/` | **1026** ← 要飞的 |
   | `车端上机包_20261017/…/ag_integrate/` | 1026（与上机包逐字节相同，`diff -rq` 无输出）|

2. **部署版比交接版少 10 个顶层函数**（`comm` 实测）：
   `_classify_blocked`、`_derive_cross_gates`、`_pick_transit_dijkstra`、`_pos`、`_required_keep`、`_route_shape`、`_transit_blocked`、`_unreachable_detail`、**`_verify_zone_covered`**、**`ensure_mine_center_pass`**

3. **这些函数不是装饰**。交接版 `plan_route_zoned` 内部实测调用：
   ```
   :1473  blocked = _transit_blocked(snapshot)
   :1475  cross_gates = ... else _derive_cross_gates(snapshot, mid_boundary)
   :1500  sub_goal = ... else _pick_transit_dijkstra(...)
   :1529  _verify_zone_covered(r, z, mine_z, pas_z)     ← 分区覆盖校验
   :1547  return compress_route(full, _required_keep(snapshot))
   ```
   而 `pipeline.py:73,96` 确实调用 `plan_route_zoned`。**即部署版跑的是一个没有 `_verify_zone_covered`（分区必经格覆盖校验）、没有 `ensure_mine_center_pass`（雷心通过保证）、没有 `_transit_blocked`（中场阻断分类）的旧实现。**

4. **测试套件针对的是 1585 行那份**：`路径规划交接_20261017/tests/test_planning_hardening.py:13`
   ```python
   from ag_integrate.mission_router import ensure_mine_center_pass, plan_route
   ```
   `ensure_mine_center_pass` 在部署版中**不存在** → 拿测试套件跑部署代码，**连 import 都过不去**。

5. **3,168 行测试、15 个测试文件，一个都没进部署包**（`find` 实测：`无人机上机包…/air_ground` 与 `车端上机包…/air_ground` 下 `test*` 命中为 0）。测试只存在于 `_tmp_agplan/`（临时目录）和 `路径规划交接_20261017/`（交接目录）。

**后果**：你以为验证过的规划器，和天上飞的那个，不是同一个程序。少的恰好是"路线是否真的覆盖了必经格"这一类**正确性守卫**——失败模式是车安静地漏打一个区，而不是报错。

**建议**：外场前把 `路径规划交接_20261017/src/ag_integrate/` 定为唯一权威源，重打两个上机包，并把 `tests/` 一并随包。验收标准：在部署包目录内 `pytest tests/` 全绿，且 `md5sum mission_router.py` 与交接包一致。

---

### P0-2 全库无版本控制，且"要飞的二进制"源不可考 `已验证`

- `find . -name .git -type d` → **0 命中**。无 `.gitignore`（仅 `.pytest_cache` 自带的）。
- 项目自己的记录已承认：`车端上机包_20261017/README.md:82`"dznav、car_deploy 相关代码尚未提交 git，本包快照即为版本记录"；`进度/20261017-上机部署整理与格号修复-会话纪要.md` 待办#3"主仓库未提交代码…需 git 留档"。
- **更严重的是二进制溯源断裂**。`进度/20261017-live定位恒0门槛修复-会话纪要.md` 末段原文：
  > "截图 HUD（dets=/cells=）非 main.cpp 输出格式，live 程序可能是 main.cpp 的带 HUD 变体——`--probe` 会用 strings 特征串确认其家族与编译源"

  即：**板上正在跑的检测程序，不是由包里这份 `main.cpp` 编译出来的，且当前无法确定它由哪份源码编译。** 配合无 VCS，这意味着任何一次"改了但没生效"或"生效了但不知道改了什么"都无法排查。

**后果**：唯一定版手段是打 zip + 手写 md5 清单。335 个 py 文件中 61 组重名（171 个文件，占 51%），`remote_run.py` 有 6 份完全相同的副本，而 `mission_router.py` 有 4 份不同的。**"哪份是真的"这个问题目前无解。**

**建议**：立刻 `git init` 做一次全量快照提交（哪怕之后不再用），并把"板上二进制的编译源"作为一条 P0 上机待办——用 `strings` 特征串比对确认，确认前不要相信任何"我改过了"。

---

### P0-3 NaN 舵角 → **+30° 满舵**（不是停车） `已验证`

代码路径：
```python
# pursuit.py:168-169
steer = max(-self.servo.steer_limit_deg,
            min(self.servo.steer_limit_deg, steer))
```
```python
# executor_node.py:464-468  _publish()
m.linear.x  = float(v)
m.angular.z = float(steer_deg)   # 无 isfinite 检查
self.pub_cmd.publish(m)
```

Python 的 `min`/`max` 遇 NaN 返回**第一个参数**，实测：
```
min(30.0, nan)              = 30.0
max(-30.0, min(30.0, nan))  = 30.0     ← 满舵
```

**后果**：任何上游 NaN（里程计毛刺、投影退化、`k_peak` 开方域、除零）都会被钳位逻辑**转成向右 30° 满舵**，且车同时保持当前速度前冲。这是失效方向最坏的一种——不是停下来，是猛打方向。户外场地上这等于冲出边界或翻车。

**建议**：`_publish` 入口加双检，非有限即发 `(0,0)` 并进入 ESTOP 闩锁：
```python
if not (math.isfinite(v) and math.isfinite(steer_deg)):
    self._estop("non-finite command")
    v, steer_deg = 0.0, 0.0
```
验收标准：注入 `float('nan')` 的单测，断言 `/cmd_vel` 输出为 0 且状态进入 ESTOP。

---

### P0-4 无链路看门狗、无急停话题；部署脚本的停止路径**不归零油门** `已验证`

| 安全项 | 状态 | 证据 |
|---|---|---|
| 断链停车看门狗 | **缺失** | 仅路线级 `nav_timeout=180s`（executor_node.py:66,321-324）。`/nav/waypoint` 无心跳语义 → 链路断了车最长自主跑 **3 分钟** |
| 人工急停话题 | **缺失** | ESTOP 态只由越界触发（executor_node.py:305-310），无任何 `/estop` 订阅 |
| 数据陈旧拒止 | **缺失** | `header.stamp` 全程未读（executor_node.py:160-169）；内部时钟 `_t += 0.02` 开环累加（:287） |
| IMU 丢失失效保护 | **缺失** | `imu_seen=False` 不阻止进入 ACTIVE；`_has_odom` 写后从未被读（:142,163） |
| 退出归零油门 | **部分失效** | 见下 |

`executor_node.py` 的 `main()` 有 `finally` 发布零 Twist——这本身是对的。但：
```python
# executor_node.py main()
try:    rclpy.spin(node)
except KeyboardInterrupt: pass
finally:
    m = Twist(); node.pub_cmd.publish(m)   # 归零
```
**SIGTERM 的默认处置是终止进程，不展开 Python 栈，`finally` 不执行。** 而部署脚本用的正是 `pkill`（SIGTERM）：
```
dznav_deploy.py:137   pkill -f '{pat}' 2>/dev/null; true
dznav_deploy.py:152   pkill -f 'dznav.executor_node' 2>/dev/null; true
```

**后果**：车在跑的时候执行部署/停止 → 控制节点被 SIGTERM 直接杀死 → **最后一条油门/舵角命令留在总线上无人撤销**。`dznav_deploy.py:245-247` 还是"先 pkill legacy 再启 dznav"，若 `_start_dznav`（:145-147）抛异常，该路径**没有任何回滚**（自动回滚只覆盖 :250-256 的体检失败），结果车端两个控制栈全灭。

**建议**：
1. `signal.signal(SIGTERM, handler)` 显式归零后退出；
2. 独立 `/estop` 订阅（latched，`TRANSIENT_LOCAL`）；
3. 无人机心跳话题，超时 ≤1s 即停车；
4. 部署脚本改为"先启 dznav 验证健康，再停 legacy"，且失败路径补回滚。

---

### P0-5 决策节点无异常兜底，且 launch 不 respawn `Agent 报告（已复核 import 与调用链）`

```python
# integrate_node.py:581
except (ValueError, IndexError, KeyError) as e:   # 只捕三种
```
而 `_plan_core` 内有生产环境 `assert`（`mission_router.py:623` `assert best_route is not None`），`_route_needs_replan` / `_snap_signature`（:371-416）可抛 `AttributeError`/`TypeError`。**这些都会穿透 timer 回调 → `rclpy.spin` 抛出 → 进程退出**，而 `launch/integrate.launch.py:29-44` **没有 `respawn=True`**。

**后果**：赛中一次未预见的规划异常 = 整车"大脑"下线，车端 path_executor 失联，且除黑匣子外无任何告警。单点 + 高概率 + 不可恢复，三重叠加。

**建议**：timer 回调整体 `except BaseException` 兜底 + 黑匣子记录 + `respawn=True` + 移除生产路径上的 `assert`。

---

### P0-6 Held-Karp 2ⁿ 无上限，跑在 10Hz 单线程 executor 里 `Agent 报告`

```python
# mission_router.py:547-549
dp = [[INF]*n for _ in range(1 << n)]
```
`n` = 雷 + 箭头 + 射击位数量，**无规模上限**。误检是项目自述的顽疾（`mission_router.py:506-507`），而误检线性增大 `n`、指数级增大耗时。n=22 → 400 万状态 × Python 循环 → 数百 MB 内存、分钟级耗时。

它发生在 `integrate_node.py` 的 10Hz timer 回调内，而该节点用 `SingleThreadedExecutor`（:670 `rclpy.spin`）。**后果：executor 整体冻结，align 期间连 `/cmd_vel` 都停发。** 外场光照变化最容易触发。

**建议**：`n > 12` 时降级为最近邻/贪心 + 告警；规划移到独立线程或 `MultiThreadedExecutor` + 独立 callback group。

---

### P0-7 明文凭据随 40.9MB 分发 zip 扩散，且用户名==密码 `已验证`

```
无人机上机包_20261017/README.md:3    SSH 用户 `duzhong`
无人机上机包_20261017/README.md:26   set AG_ORIN_PASS=duzhong
_tmp_pw.txt                          全文即 "duzhong"（7 字节）
```
`进度/20261017-live定位恒0门槛修复-会话纪要.md` 第六节把 `set AG_ORIN_PASS=duzhong` 写成**飞行前操作第一步**。另有约 10 个部署脚本 paramiko 硬编码 `password="duzhong"`，`dk_train_pack_20261016/car_probe.py:13` 还有密码字典 `["duzhong","12345678"]`（泄露第二个口令）。

**这是飞控所在机载计算机的登录凭据，用户名与密码相同，已随至少 4 个 zip 分发到不可控范围。** 全部 SSH 工具（`remote_run.py:75`、`preflight.py:116`、`dznav_deploy.py:56`）用 `AutoAddPolicy()` 不校验主机密钥。

**建议**：改口令 + 改密钥认证 + 从 README/脚本中移除明文（改占位符）+ 删除 `_tmp_pw.txt` + 重打所有 zip。内网比赛可容忍 MITM 面，但"密码写进分发文档"不可容忍。

---

## 四、P1 级发现（强烈建议外场前处理）

### P1-1 禁火否决 TTL 建立在**已知会跳变**的无人机墙钟上 `已验证（跨模块组合，本次评审最有价值的发现之一）`

两个独立事实组合成一个安全洞：

- **事实 A**（`main.cpp:183-185`）：态势帧时间戳用 `std::chrono::system_clock`（墙钟）。
- **事实 B**（项目自己的进度纪要，`2026-10-17_无人机悬停排查与v2.1部署.md` 踩坑速记）：
  > "整机重启后系统时钟回跳（**无 NTP**），日志按 mtime 排序会错乱"
- **事实 C**（`world_model.py:98-109`）：红靶粘性禁火否决的 TTL 用 `frame.timestamp_ms` 计量，TTL=5s。

**后果**：机载时钟向前跳 >5s → `_red_seen` 里所有红靶记录**一次性全部过期** → 禁火否决清零 → **可能对友方红靶开火**。这是 fail-dangerous，且触发条件（无 NTP + 重启时钟跳变）项目已经实测到过。反向乱序帧又会让 TTL 永不过期。

**建议**：TTL 改用**本地 monotonic 收包时刻**，不要用对端墙钟。同时 `main.cpp` 改 `steady_clock` + 启动时一次性对齐 epoch。验收标准：注入时间戳前跳 10s 的帧序列，断言红靶否决仍生效。

### P1-2 未定位时完全静默，地面无法区分"没有目标"和"链路/定位挂了" `已验证`
```cpp
// main.cpp:186-187
if (localized || std::getenv("DZUAV_FORCE_UDP"))
    udp.send(timestamp, objects);
```
定位一丢（格号被遮挡、PnP 退化），车端就收不到任何包。**建议**：始终发 0x4001（可空目标列表）+ 增加 `localized` 状态位，车端按丢包超时进 failsafe。

### P1-3 相机 `capture.read()` 阻塞无看门狗 `已验证`
`main.cpp:328` 的空帧计数（:331-355）只在 read **返回 false** 时生效。Jetson argus/ISP 冻死时 read 会**永久阻塞**：进程活着、不再发一帧 UDP、地面完全无感。且 `:350-354` 断流只重开一次，二次失败即 `return 4` 进程退出（未见 systemd/respawn）；重开前也没有 `capture.release()`（Jetson GStreamer 常见坑）。SIGINT 也打不断阻塞的 read（`:387` 只置 flag）。

**建议**：独立看门狗线程，read 超过 N×帧间隔即重开管线并告警；多次退避重试；由外部 supervisor 拉起。

### P1-4 收包线程可无声死亡，且就绪检查 fail-open `Agent 报告`
- `integrate_node.py:298-299` `except OSError: break` → socket 出错收包线程**静默退出**，节点照常显示"就绪"，全节点无情报断流 watchdog。`_log_hold_once`（:644-647）同一 reason 只打一次日志，链路断了也看不出来。
- `integrate_node.py:641-642` `except Exception: return True` → **任何异常都被解释成"path_executor 已上线"**，路线照发进真空，且无日志。
- `integrate_node.py:316` `_car_true_yaw` 写入后**永不失效**（无时间戳、无 staleness TTL）：0x4002 断链后 align 闭环继续按最后一次观测转向。

### P1-5 部署脚本从不设 `DZ_NAV_CONFIG`，车上的 yaml 是死配置 `已验证`
```
executor_node.py:97   cfg_path = os.environ.get("DZ_NAV_CONFIG", "")
dznav_default.yaml:1  # 车上部署时拷到 ~/dznav/config/ 并设 DZ_NAV_CONFIG 指向它
```
`grep -rn "DZ_NAV_CONFIG" 车端上机包_20261017/` 只命中上述两处 + 一行 docstring。**部署脚本启动命令里没有它** → 车上实例永远跑内置 DEFAULTS。**现场调 yaml 参数完全不改变行为**，这是必踩的坑。

### P1-6 参数四源漂移，wheelbase 出现 4 次 `Agent 报告`
- `TrackerConfig.v_max=0.32 / a_lat_max=0.20`（pursuit.py:36,38）≠ DEFAULTS/yaml 的 `0.30 / 0.22`（executor_node.py:63-64、yaml:11-12）
- `wheelbase = 0.157` 出现在 `contract.py:32`（死常量，全包零引用）、`servo.py:26`、`executor_node.py:60`、`yaml:7` 共 **4 处**；而 `contract.py:32` 自注"发学员版为 0.17" → **wheelbase 是否与实车一致，仓内无法确认，需实测**
- `refpath.py:13` docstring 称 `arc_fallback` 默认 135°，代码实为 125°（:62）
- `_MID_BOUNDARY` 在 `mission_router.py:50`、`match_params.yaml:27`、`dzbrain_bridge.py:269` 三处，**一致性完全靠人肉**（挡板行错位 = 路线撞墙）
- `udp_port=50000` 默认值三处；时间参数三份（integrate_node.py:131-135、route_time.py:38-41、yaml:61-65）；`route_time.py:25` 的 `SOFT_CAP_S=165` 是模块常量，`over_soft_cap`（:99）用它而非可配置的 `time_soft_cap_s` → **改参数后该标志失真**

### P1-7 `monitor_link.py` 比赛日会直接崩 `已验证`
```python
# monitor_link.py:47-53（模块级 dict 字面量）
ObjectType.MINE:    "M",
ObjectType.ARROW:   "A",
```
而现行枚举（`intel_protocol.py`，2026-10-16 切 D13 13 类模型后）**只有 `PAPER`，没有 `MINE`、没有 `ARROW`**：
```
PAPER=0  CELL_LABEL=1  CAR=2  TARGET_RED=3  TARGET_BLUE=4  OBSTACLE=5
BUCKET_RED=6 BUCKET_GREEN=7 BUCKET_CYAN=8 CAR_START=9 CAR_END=10 UAV_START=11
```
模块级求值 → **AttributeError，地面态势看板打不开**。换模型后忘了同步工具。

### P1-8 `field_frame.py:53` 的"离格心过远丢弃"是**数学上永不成立的死守卫** `已验证`
```python
col = round(col_f)          # 最近整数
row = round(row_f)
...
if abs(col_f - col) > max_residual or abs(row_f - row) > max_residual:  # max_residual=0.5
    raise ValueError(...)
```
`round()` 之后 `abs(col_f - col) ≤ 0.5` **恒成立**，`> 0.5` 永假（恰好 0.5 时也为假）。所以 docstring 承诺的"离格心过远则抛 ValueError"从未生效——**任何场内点都被硬吸附到最近格**，半格（0.175m）外的检测照样落格进态势。而 `world_model.py:48-49`、`ground_projection.py:19` 都引用了这个承诺。
**建议**：阈值改 `<0.5`（如 0.35）才有效。

### P1-9 `MapLocalizer` 4 点即可过 PnP 质检，且重复 CellId 不去重 `已验证 + 需验证`
```cpp
// MapLocalizer.cpp:99   PnP 路径
const std::size_t need = std::max<std::size_t>(4, p.size() * 3 / 10);
// MapLocalizer.cpp:153  单应路径
const std::size_t need = std::max<std::size_t>(5, p.size() * 3 / 10);
```
**先给项目记一功**：单应路径取 5 而非 4 是**有意识的正确决策**，进度纪要写明了理由（"任意 4 点对应都精确确定一个单应，零验证力；≥5 才含至少 1 个过约束点"），并附了 6 场景仿真验证。md5 `f1f0f1d8c41d82ecd74a81572b90af6c` 与纪要一致 → **修复确实已进部署包**。这是全项目最好的一次工程实践记录。

**残留问题**：PnP 路径仍是 4 点起。恰好 4 个地标时 RANSAC 必然全内点，**一个误读格号也能拟合出中位误差≈0 的错误位姿**，全场目标世界坐标系统性错位并广播给车端。同一 `CellId` 出现两次（重复检测/误读）也无冲突剔除。`MapLocalizer.cpp:164-166` 的 `project()` 无 `isfinite` 防护，`q.at<double>(2)` 接近 0 时输出 inf/NaN，直接进 UDP 帧（`main.cpp:172-175` 无检查）。
**建议**：PnP 也要求 ≥5 点或加 leave-one-out 一致性检查；同 ID 多观测取置信度最高或报警；`project()` 后加 `std::isfinite`。

### P1-10 preflight 是纯咨询打印，且无人调用、路径口径已过期 `Agent 报告`
- `grep` 全部包，只有 `orin_deploy.py:4` 的注释提到它；`orin_bringup.sh` 起飞链路与它**零关联**，退出码无人消费 → **不是硬门禁**。
- 自身路径已过期：`preflight.py:200` 查 `~/ag_test/dzuav/build/dzuav`，而 `orin_deploy.py:44` 部署到 `~/dzuav_deploy/air_ground`。`orin_deploy.py:3-5` 亲口承认"路径出现过三处口径不一"，但 preflight 没跟着改。

**缺口清单**（对照真实飞行前所需，均未查）：链路质量与延迟、磁盘空间、**时间同步/钟差**（全库无 NTP 处理）、电池电量、GPS/EKF 定位状态、geofence/返航点、**ROS_DOMAIN_ID 冲突**（车端 source 了 dzros.env 却不校验域值，机载端完全不查）、**引擎↔当前 ONNX↔TensorRT 版本匹配**（只数引擎个数 ≥2，`preflight.py:190-194`，旧引擎/错版本引擎照样绿灯）、标定新鲜度（打印日期但不拦截）、话题级存活（只查端口监听，不查 `/fmu/out/*` 是否有数据）、uXRCE 链路状态（已知唯一断点，却由独立的 `uxrce_link_guard.py` 负责，preflight 不集成）。

### P1-11 零依赖钉扎，TensorRT 版本全程无校验 `Agent 报告`
全库无 `requirements.txt` / `environment.yaml` / `pyproject.toml` / `Dockerfile`。`grep getInferLibVersion|tensorrt.__version__|trtexec --version` → **0 命中**。`orin_convert_engines.sh:29-35` 只找 trtexec 可执行文件就直接转；头注释（:4-6）明知".engine 与 TRT 版本 + GPU 绑定"，却**不记录转换时的 TRT 版本、加载时也不比对** → 换机/刷机后静默失败或精度漂移。`run_all.ps1:36` 硬编码 `C:\Users\a\venvs\air_ground`。

### P1-12 空地时钟无同步，坠机后拼不出统一时间轴 `已验证（项目自述）`
`景深相机接入准备方案.md:104` 明文"与车端日志对账时以发车事件对齐，**不追求绝对时钟同步**"。而 `orin_flight_blackbox.py:315` 与车端黑匣子各记自己机器的 `time.time()`；`monitor_link.py:107` 的"延迟" = 地面钟 − 机载帧时间戳，**两钟不同步时该数字是噪声**（仅靠 `0<delay<5000` 过滤掩盖）。叠加 P1-1，这不只是复盘困难，是安全逻辑依赖了不可信时间。
**建议**：起飞前 `timedatectl` 手动对时一次，并把钟差写进黑匣子首条事件。成本极低，收益极高。

### P1-13 机上黑匣子当前是坏的 `已验证（项目自述）`
`进度/2026-10-17_无人机悬停排查与v2.1部署.md` 待办#3：**"flight_bb 黑匣子只记 2~3 行（待在线修，源码在机）"**。即当前唯一的事后取证手段实际失效，而修复排在下一次上机。另外 `orin_flight_blackbox.py` 无文件大小上限/轮转/磁盘余量检查（10Hz JSONL 长时间飞可写满 eMMC），且不录原始图像帧 → **视觉误检事后无法复现输入**。`orin_log_capture.py:127-128` 只 flush 从不 fsync（断电丢缓冲），`:210` 回放"假设 30fps"造假时间戳。

### P1-14 QoS 全部默认，从未与对端核对 `Agent 报告 / 需验证`
`integrate_node.py:231,236`、`dzbrain_bridge.py:118-120`、`executor_node.py:121-127` 所有 pub/sub 均默认 `RELIABLE / VOLATILE / depth 10`。若 dzactuator 的 `/odom`、`/imu_data` 以 `BEST_EFFORT` 发布（传感器流惯例），**QoS 不兼容会一帧都收不到，且只表现为"静默无数据"**。`/cmd_vel` 用 depth 10 会积压历史转向命令，实时控制应 depth 1。
**验证方式**：`ros2 topic info -v /odom` 逐对比对。这是 10 分钟能查完、但漏掉就全程静默失效的项。

### P1-15 转向无斜率限制与死区 `Agent 报告`
`pursuit.py:190-191` 速度有斜坡，**转向没有**；ARC 入口一拍瞬时满舵（:147-149）。50Hz 下舵机冲击与抖振。建议每拍 Δsteer 上限（如 5°/拍）+ 小死区。

### P1-16 IMU 首帧晚于 reset 时，开局朝向带任意偏置 `Agent 报告 / 需验证`
`fusion.py:131-135` reset 早于首帧 IMU 时 `yaw_anchor = th_reset`，首帧到达后 `on_imu:65` `th = yaw_rel + yaw_anchor` 引入任意 `yaw_rel(0)` 偏置；**注释称"等首帧后校正"但代码里没有校正逻辑**。且 ACTIVE 不检查 `imu_seen`。后果：开局朝向错、直线冲出。
**建议**：首帧 IMU 到达时令 `yaw_anchor = th_reset - yaw_rel`；ACTIVE 前强制 `imu_seen` 门控。需验证 IMU 是否恒先于 reset 到达。

---

## 五、P2 / P3 级发现（择要）

**UDP / 网络**
- `UdpBroadcaster.cpp:121` 用 `INADDR_BROADCAST`(255.255.255.255) 受限广播，不跨网段；多网卡 Jetson 上按默认路由选出口，**可能从错误网卡广播出去**（sendto 成功、对端收不到）。现场换 IP 段即全链路静默。建议 `SO_BINDTODEVICE` 或配置化子网定向广播，启动时打印实际出口。
- 车端 IP 三个默认值互相矛盾：`preflight.py:221` = `192.168.1.188`；`dznav_deploy.py:14` 示例 `192.168.0.198`；`deploy_pose_rknn.py:521` = `192.168.0.199`。车端 README:22 只能写"以现场为准（历史：199/198）"。**不带 `--host` 跑 preflight 会连错网段并报"车不通"。**

**实时性 / 性能**
- 实测 **fps=6.0~6.6**（Orin NX + D13 引擎 90-150ms/帧，进度纪要实锤）。叠加 `CellClassifier.cpp:68-97` 每个格号 **4 次 TTA 推理全串行**（各自 enqueue + streamSync）、`main.cpp:99` 全幅 undistort remap、1280×1280 检测，整条流水**单线程无任何并行**。建议 TTA 合成 batch=4 一次 enqueue，或隔帧做格号分类 + 结果缓存。
- `main.cpp:204-216` 每帧 `cout` 手拼 JSON 日志在热循环里（30fps 下 ostream 格式化 + 终端 IO 可占毫秒级）；`:186` 每帧 `getenv`（应启动时读一次）。
- `executor_node.py:203` `_cmd_load` 在回调内做阻塞文件 IO；`:287` `_t` 开环累加非墙钟 → **超时判据全部被拉长**。应用 `self.get_clock().now()`。
- `pursuit.py:180-188` 射击减速扫描遍历**所有**后续途经点（仅命中 shoot_stop 才 break），回折路线上直线距离 <0.5m 的远端停站会提前误减速；与触发逻辑 `:118-130`"只看紧邻点"不一致。

**逻辑正确性**
- `mission_router.py:967-991` `plan_route_zoned` 对每个分区都透传全局 `start_heading`/`final_heading`（:980 `**kwargs`）：中途区实际入场朝向是上一区末段朝向而非全局 start_heading，中间区 sub_goal 也被施加终点朝向约束 → 转向代价系统性算错，约束不可行时 ValueError 直接整局规划失败。**需验证**：构造两区用例对比真实代价。
- `integrate_node.py:109,528-531` align 爬行漂移自述 ~0.21m，**大于半格 0.175m**：漂移后 `reset:N11` 仍断言车在原格心，里程计锚点最多偏 0.6 格，全链航位推算基准带偏。
- `integrate_node.py:423-425` vs `550-556` 两条对齐启动路径不一致；若 0x4002 始终不来，`_do_heading_alignment` 每 tick 发零速干等 15s 超时（:500-503）才降级发车 —— 180s 硬顶里白丢 15s。
- `world_model.py:91-93` 蓝靶**不去重**（mines/passable/obstacles 都按格名去重）：同帧双检出同一蓝靶 → 两个 TargetRef → 生成两组射击位候选，组合数×2、两个停靠格都成必经点。
- `dzbrain_bridge.py:271-274,291-295` 桥洞/靶区**变空时不发撤销报文**（空列表直接 skip），车端吃旧值直到自身 30s `info_stale` 兜底。
- `evidential_map.py:107-121` 实为 log-odds 朴素累加而非 DS 融合：正证据无饱和上限、同一静止目标逐帧重复计数（独立性假设不成立）、`_last_t` 按格共享导致任一观测重置该格所有类别的衰减（:118,133,157）。**目前是死代码，但若将来接线需先修这三点。**
- `fusion.py:80-88` 里程计步长无离群上限，raw odom 跳变直接积入信念位置 → 假越界 ESTOP 或误 rebuild。
- `pursuit.py:254-255` "对齐弧"是 `arc_v=0.13` 前进满舵弧（R≈0.2-0.44m，servo.py:29-31），**非原地转**；图缘触发时可能直接越界闩锁 ESTOP。

**契约谎言 / 文档漂移**
- `main.cpp:170-171` 注释自认 length/width 字段与协议契约不符（填像素非米），"车端忽略所以无实害"——**契约谎言留给下一个人踩**。
- `integrate_node.py:80-81` 注释称"test_field_config_consistency 锁死 yaml/节点/常量三处口径一致"——**该测试不在部署包内**（见 P0-1）。
- `integrate_node.py:391` `if self.last_route is None` **永假**（:219 初始化为 `""`），注释与实现矛盾。
- 无人机包 `README.md:61` 步骤 4 要求运行 `C:\Users\29212\Documents\kimi\tasks\2026-10-14\...\orin_deploy.py` —— **kimi 目录不在包内**；`L3B_RUNBOOK.md:45,52,90` 同款 ×3。拿着 zip 在第二台电脑上按 README 走，步骤 4/5/6 全部报"找不到文件"。
- `orin_bringup.sh:28` 相机默认 `1920x1200@30`，而 `orin_run_dzuav.sh:4,39` 默认 `1920x1080@30` —— **两个默认分辨率不一致且都声称对应标定基准**。而 C++ 内参基准写死 1920×1080（`MapLocalizer.cpp:18-19`）。
- `orin_bringup.sh:24` `set -uo pipefail` **缺 `-e`**；`:61-71` 用固定 `sleep 3/2` 硬等而非就绪探测；`:66` 硬编码 `/dev/ttyACM0`（进度纪要实锤"飞控上电晚于 Orin 时 ttyACM0 重新枚举，Agent 旧 fd 失效"）；`:48` dzuav 二进制缺失仅 warn 照样起窗口。
- `package.xml:15` `exec_depend ag_core` 与代码软依赖设计（`integrate_node.py:40-44`）矛盾。

**代码质量杂项**
- `car_adapter.py:26-41` `_target_key` **原样定义两次**（第二个覆盖第一个），纯复制粘贴残留。
- 异常吞噬全录：`integrate_node.py:364-365, 641-642, 660-662`；`dzbrain_bridge.py:189, 223, 246, 329, 356, 377`；`executor_node.py:478-479`（`except Exception: pass`）；`dznav_deploy.py:68`；`pipeline.py:101-102`（`except ValueError: return commands` 无日志静默降级）。
- `mission_router.py:515,602,944` 用 `warnings.warn` 报运行期风险 —— **不进 ROS 日志，且默认同位置只出现一次**。
- 死变量：`executor_node.py:142-143` `_has_odom`、`_last_cmd` 写后永不读；`contract.py:32-36,85-90` `WHEELBASE/STEER_LIMIT_DEG/CMD_KEEPALIVE_S/deg/rad` **全包零引用**（grep 证实）—— 契约模块里的常量没人用，各模块各自硬编码，这正是 P1-6 漂移的根因。
- `executor_node.py:86-90` `_deep_update` 只覆盖已存在键，**yaml 拼错的键静默丢弃无告警**。
- god 函数实测行数：`IntegrateNode.__init__` 219 行（:62-280）、`plan_route` 201 行（:687-887）、`_plan_core` 147 行；C++ `processFrame` 124 行（main.cpp:96-219）、`MapLocalizer::solve` 110 行、`runCamera` 107 行、`HbbDetector::infer` 99 行；`refpath.build_reference` 111 行、`pursuit._track` 93 行。
- `HbbDetector.cpp:13` 与 `CellClassifier.cpp:12` 各有一份复制粘贴的 `dimsToString`；`main.cpp:1` 与 `:9` 重复 `#include <algorithm>`。
- `HbbDetector.cpp:112` 置信度直接与阈值比较，**假设引擎输出已过 sigmoid**（ultralytics 导出通常含，但若导出配置不同则是裸 logits，阈值全失效）。需 dump 一帧 output0 看数值域是否 [0,1]。
- `HbbDetector.cpp:85-86` `blobFromImage` 可能重分配 `blob_`，pinned 内存静默失效（注释 67-69 自己承认此风险）→ H2D 退化为 pageable 拷贝（伪异步、真阻塞）。而 `CellClassifier.cpp:85-89` 是手写 CHW 直填 `h_input_` —— **两处做法不一致**。
- `TrackerConfig` 是 dataclass 却在 `__post_init__`（pursuit.py:75-87）塞 11 个未声明字段。
- 交付区垃圾约 70MB：`recording_*.mp4` 18+21+14MB、`field_scan_2.mp4` 4.4MB、`field_pack.tar` 11MB，14 个散落脚本（`_tmp_diag.py`/`_iter1-3.py`/`_check.py`/`evidence.py`/`_render_*.py`），14 个 `__pycache__`、2 个 `.pytest_cache`。
- 权限位不一致：`orin_deploy.py`/`remote_run.py`/`preflight.py` 644，`orin_flight_blackbox.py`/`monitor_link.py` 755。zip 内文件名 GBK 编码，Python zipfile 读出乱码（跨平台解包隐患）。
- **全库无 TODO/FIXME/HACK 标记**——考虑到实际待办数量（进度纪要里一堆），这说明待办根本没进代码，只存在于会话纪要里。

---

## 六、质量属性评估

| 质量属性 | 评分 | 依据 |
|---|---|---|
| **安全性 (safety)** | 🔴 **2/10** | 无看门狗、无急停、NaN→满舵、SIGTERM 不归零、禁火 TTL 依赖会跳变的墙钟、未定位即静默 |
| **可靠性 (reliability)** | 🔴 **3/10** | 决策节点无异常兜底 + launch 不 respawn、2ⁿ 规划冻结 executor、相机阻塞无超时、收包线程可无声死亡 |
| **可测试性** | 🔴 **2/10** | 3168 行测试存在但**零随包**，且测的是未部署的版本；部署代码连测试的 import 都过不去 |
| **可维护性** | 🟠 **4/10** | 无 VCS、4 版本漂移、参数四源、219 行 `__init__`、大量死代码与契约谎言；但注释质量与事故可追溯性很好 |
| **可观测性** | 🟠 **4/10** | 黑匣子设计思路正确（fsync 分级、KILL 识别）但机上那份实际只记 2-3 行；无 NTP 导致空地日志无法对齐；monitor_link 崩 |
| **可部署性** | 🟠 **5/10** | `dznav_deploy.py` 是模板级（md5 + compileall + 体检 + 自动回滚）；但 preflight 是摆设、yaml 是死配置、README 指向包外路径 |
| **性能** | 🟠 **5/10** | 实测 6fps；pinned 内存/映射缓存/缓冲复用意识到位，但 TTA 串行 + 单线程流水 + 热循环日志 |
| **算法正确性** | 🟢 **7/10** | 角度 wrap **无一处遗漏**（罕见）；协议 CRC + 全量校验链完整；MapLocalizer 门槛修复有仿真验证与明确理由 |
| **安全性 (security)** | 🔴 **2/10** | 明文口令随 zip 分发、用户名==密码、AutoAddPolicy 不校验主机密钥、密码字典 |

---

## 七、亮点（如实，不吹捧）

这些是真正做得好的地方，值得保持：

1. **角度运算无一处漏 wrap**。逐点核对 `pursuit.py:163,234,294`、`executor_node.py:261,394,425`、`fusion.py:65,73,125-126`、`refpath.py:117` 均经 `wrap_pi`。漏 wrap 会导致 violent full-lock steering snap，这是学生项目最常见的致命 bug——**你们一个都没犯**。
2. **`MapLocalizer` 单应门槛取 5 而非 4，并写明了理由 + 6 场景仿真验证 + md5 留档**。这是教科书级的工程决策记录方式。`project()` 宁可抛异常也不返回 `(0,0)`（:159-163），注释明确解释了 (0,0) 恰是场地角点的危害——安全思维正确。
3. **`intel_protocol.py:130-173` 校验链完整严格**：magic / version / declared_len / 精确帧长 / 枚举范围 / NaN / CRC16 / suffix 逐层把关，坏帧只抛 ValueError 不落半成品，且提供对称 `build()` 便于回放注入。
4. **引擎输出形状契约校验**（`HbbDetector.cpp:56-60`、`CellClassifier.cpp:39-40`）：换错引擎立即抛错并打印实际 dims。很多生产代码都没这个习惯。
5. **`dznav_deploy.py` 是模板级部署器**：预检留档 → 推送 → md5 全量比对 → 远端 compileall → 待命启动（`DZ_AUTO_START=0`）→ 体检 → 失败自动回滚。
6. **`remote_run.py` 的重试语义设计优秀**：只重试连接阶段，明确写清为何不重试执行阶段（:66-69）；三处踩坑注释（getpass 静默死等 :44-47、CRLF/LF :312-316、GBK 控制台 :320-324）都带日期和事故复盘。
7. **进度纪要质量很高**：四根因定案、双实例实锤、"pgrep 15 字符截断"、"Livox 不响应 ICMP"、"飞控独立供电所以整机重启没断它"——这些是真金。`FILE_MANIFEST` 带 md5 + 字节数。
8. **C++ 侧 RAII 与性能意识扎实**：TRT 8.x/10.x 析构分流（`TensorRtEngine.hpp:59-73`）、全链删除拷贝构造、页锁定内存/发送缓冲复用/undistort 映射缓存、小端 `static_assert`（`UdpBroadcaster.cpp:19`）、构造失败显式抛错杜绝 fd=-1 静默失败（:57-68）。
9. **安全默认值意识**：`dry_run=True`（integrate_node.py:71）、UDP bind fail-fast（:245-254）、启动参数自检（:210-215）、`fire_guard` 作为独立闸门"邻红即否决、宁不打不误伤"（fire_guard.py:96-101）、默认待命铁律贯穿代码与部署。
10. **全部 4 个 `.sh` 实测 LF，无 CRLF 杀手**；`orin_build_verify.sh:108-111` 连 `grep -q` + pipefail 的 SIGPIPE 假阴性坑都修了。

---

## 八、未验证 / 证据缺口

诚实列出我没能在仓内闭环证实的项：

| 项 | 为什么没证实 | 怎么验证 |
|---|---|---|
| 板上 dzuav 二进制的编译源 | 无 VCS + 进度纪要自认"可能是 main.cpp 的带 HUD 变体" | 上机 `strings` 特征串比对（项目已写好 `fix_localized_gate.py --probe`） |
| QoS 是否真的不兼容 | 需运行中的 ROS2 图 | `ros2 topic info -v /odom /imu_data /cmd_vel /nav/waypoint` |
| wheelbase 0.157 是否与实车一致 | `contract.py:32` 自注"发学员版为 0.17" | 实测轴距 |
| SIGTERM 是否真的不归零 | 从代码语义推断（默认处置不展开栈），未实车验证 | 车跑动中 `kill -TERM` 观察 `/cmd_vel` |
| IMU 是否恒先于 reset 到达 | 需运行时序 | 录一段 startup 日志看消息顺序 |
| `plan_route_zoned` 分区朝向透传是否真算错 | 需构造两区用例 | 单测对比真实代价 |
| TensorRT 引擎与当前 TRT 版本是否匹配 | 引擎是二进制，仓内无版本记录 | 上机 `trtexec --version` + 加载测试 |
| 6fps 是否够用于任务 | 不知任务时序要求 | 需用户确认 |

---

*产出：`architecture-review.md`（本文件）、`risk-map.dot`（失效链关系图）、`remediation-plan.md`（分级整改计划）。*
