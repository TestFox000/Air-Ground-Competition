# 06 · 让小车「等无人机指令再动」

**类型**：小车侧配置 + 代码改动（**本仓库第一个车侧补丁**）
**状态**：✅ **已应用到真车并通过实测**（2026-10-07）
**风险**：低（关掉一个自动行为 + 加一层环境变量覆盖；两处都有备份 + 自检 + 一键回滚）
**要不要编译**：不要。但**必须同步 install 副本**（见下）

---

## 1. 起因

用户报告：`cd ~/dzrobot && ./start_all.sh start` 之后小车**自己在 10 秒后出发**了，
现场来不及接管。要求改成「等无人机的指令再动」。
（★ t16 更正：`auto_start_delay_s` 现真值为 **45.0 s**（`robot.yaml:525`）；上面 10 秒是当时的值。）

## 2. 根因

| 层 | 事实 |
|---|---|
| 配置 | `~/dzrobot/config/robot.yaml` → `path_executor.ros__parameters.auto_start: true` |
| 代码 | `path_executor.py:849` `if AUTO_START: self.create_timer(AUTO_START_DELAY, self._auto_start_mission)` |
| 行为 | 起栈 10 s 后自动把内置 34 点路线当外部指令下发 —— 与无人机发 `/nav/waypoint` 走**完全相同的路径**（`reset:` → `follow_route_command`） |

所以"自动出发"就是"自己给自己发了一条航点"。关掉它，车就只认外部 `/nav/waypoint`。

## 3. 改了什么

| # | 位置 | 改动 |
|---|---|---|
| 1 | 小车 `config/robot.yaml:237` | `auto_start: true` → **`false`**（含注释说明环境变量覆盖） |
| 2 | 小车 `src/car_localization/car_localization/path_executor.py:171` | 常量区注释更新（`AUTO_START= True` 那个默认值现在会被 YAML 覆盖） |
| 3 | 小车 `src/.../path_executor.py:848` | 定时器前插入 `DZ_AUTO_START` 环境变量解析（`1/true/yes/on` 强制开，`0/false/no/off` 强制关）+ `★自动出发【已关闭/已开启】来源=…` 启动横幅 |
| 4 | 小车 `install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py` | 与 #3 **逐字节同步**（md5 `25db4022…`） |

### ★ 关键发现：`ros2 run` 跑的是 install 副本，不是 `src/`

```
实测：ros2 run car_localization path_executor_node
     → /home/duzhong/dzrobot/install/car_localization/lib/python3.10/site-packages/car_localization/path_executor.py
```

- 两份文件改动前 **md5 完全相同**（`69e14b0c3f8d11e7af4d1b94505bc525`），但**是两份独立文件**（`readlink -f` 证明不是符号链接）
- 只改 `src/` → **运行行为一点不变**（本次第一轮就踩到了：横幅没出现、`DZ_AUTO_START` 无效）
- 纯 Python 包 `colcon build` 就是逐字节拷贝，所以 `cp src → install` 与重新编译**等价**
- 补丁脚本已内置这一步（`sync_install()`），会遍历 `install/**/path_executor.py` 并校验 md5

## 4. 怎么应用的（不是"用户手动替换文件"）

本次由助手**直接 SSH 应用**，因为它是车侧配置/代码改动且用户明确要求修好。过程全程可复核：

```bash
# 车上执行（脚本由 scp 上传到 ~/）
python3 ~/patch_car_wait_for_drone.py --check     # 只读：打印两个锚点的命中次数
python3 ~/patch_car_wait_for_drone.py             # 应用（含 install 同步）
python3 ~/patch_car_wait_for_drone.py --revert    # 回滚
```

脚本的安全设计（都用上了，不是摆设）：

1. **精确整串替换**，每处断言命中恰好 1 次；命中数不符 → **一个字节都不写**
   （第一次失败就是因为我把自检里的 YAML 键路径写成了 `mission.auto_start`，
   实际是 `path_executor.ros__parameters.auto_start` → 脚本**自动回滚**，车上文件零损伤）
2. 改前自动备份到 `~/dzrobot/_backups/autostart_wait_<时间戳>/`
3. 改后自检：`py_compile` + `yaml.safe_load` 并断言新值为 `False`
4. 自检失败**自动回滚**并打印失败原因

## 5. 实测验证（全部真机）

### 5.1 代码分支（安全的单节点测试：不起底盘 `dzactuator`，即使"出发"也只是把 Twist 发进空气）

| 分支 | 命令 | 结果 |
|---|---|---|
| 默认（无环境变量） | `timeout 13 ros2 run car_localization path_executor_node` | `★自动出发【已关闭】来源=config mission.auto_start：车原地待命，只等外部 /nav/waypoint 指令，不会自己动`；`任 务 开 始` = **0 次** ✓ |
| 强制开启 | `DZ_AUTO_START=1 timeout 16 ros2 run …` | `★自动出发【已开启】来源=env DZ_AUTO_START=1：10.0s 后自动下发内置路线 ['N10','M10',…]`；10 s 后 `自动出发：34 个坐标点 -> …` + `>>>  任 务 开 始  <<<` ✓ |

### 5.2 整车集成（真起栈）

| 验证项 | 结果 |
|---|---|
| 起栈 35 秒（远超 10 秒延迟） | 横幅【已关闭】，`任 务 开 始` = **0** → **车原地没动** |
| 车侧订阅关系（无人机侧读） | `/nav/waypoint` → `Subscription count: 2`：`path_executor_node`(RELIABLE) + `serial_reader_node`(RELIABLE) |
| 跨机可见性 | 无人机 `ros2 node list` 看到 **12 个**小车节点 |
| **指令送达**（发 2 收 2） | 无人机发 `load:/tmp/dzlink_ping_nonexistent.txt` ×2 → 车 `7_path.log` 出现 **2 条** `[ERROR] Path file not found: …` |
| 车全程静止 | `3_localization.log`：`pos: (1.50, 0.50) | block: N10` 12 秒内一成不变 |
| 全程未出发 | `7_path.log` 里 `任 务 开 始` 总计 **0 次** |
| 收尾 | `./start_all.sh stop` → 9/9 节点停止、8080 无监听 |

### 5.3 顺手查清的两件事

1. **`path_executor.py:586` 把日志级别压到 WARN**
   `self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)`
   → 所有 `info()` 收据（`Path Executor Node Initialized`、`Odometry reset to N10 facing y`、
   `Loaded N path points`）在 `7_path.log` 里**完全看不到**。
   现场可见的只有 warn/error 三级：`★自动出发…`、`>>>  任 务 开 始  <<<`、`Path file not found: …`。
   Humble 里**没有** `SetLoggerLevel(s)` 服务，无法在线抬级别 → 想验证"指令到了没"，
   用 `load:<不存在的文件>` 这种 **error 级探针**（不产生位移）。

2. **`/rosout` 跨机可见** → 无人机不用 SSH 就能实时看小车日志。
   已做成工具：`dz_link_check.py --watch N`。

## 6. 回滚

```bash
ssh duzhong@duzhong.local
python3 ~/patch_car_wait_for_drone.py --revert
```

从最近一次 `_backups/autostart_wait_*` 还原 `path_executor.py`(src) + `robot.yaml`，
**并自动把还原后的 src 同步回 install 副本**（否则 `ros2 run` 跑的还是打完补丁那份）。

### 回滚 + 重打补丁的闭环**已实测**（2026-10-07）

```
回滚前 : src 25db4022 / install 25db4022 / auto_start: false
--revert → 已还原 src + robot.yaml；[install] 同步 25db4022 -> 69e14b0c OK
回滚后 : src 69e14b0c / install 69e14b0c / auto_start: true      ← 与改动前逐字节相同
重新应用 → 自检通过；[install] 同步 69e14b0c -> 25db4022 OK
最终   : src 25db4022 / install 25db4022 / auto_start: false / 两份都标记「已打补丁」
```

`--check` 会把 src 与 install 两份的 md5 并排打印，并直接标出
「与 src 一致: 是/★否」——现场一眼就能看出有没有漏同步。

## 7. 产物

| 文件 | 说明 |
|---|---|
| [RUNBOOK.md](RUNBOOK.md) | **完整操作流程**：改动清单 / 每次运行 4 步 / 判据速查 / 变体 / 急停 / 已知坑（7 条）/ 回滚 |
| [files/patch_car_wait_for_drone.py](files/patch_car_wait_for_drone.py) | 车侧补丁脚本（规范源头：`tools/link/patches/patch_car_wait_for_drone.py`，两份一致） |
| [files/dz_link_check.py](files/dz_link_check.py) | 无人机侧链路体检 + 小车 `/rosout` 回执抓取（已部署 `~/dzlink_tools/`） |
| [../05-drone-to-car-waypoint/files/dz_send_waypoint.py](../05-drone-to-car-waypoint/files/dz_send_waypoint.py) | 发送器（本次修掉"订阅者数量报成 1"的采样瑕疵，已重新部署） |

## 8. 如实交代

- **本次没有真发路线让车跑**。用户的选项写的是"全程车不动"，我按这句话执行：
  只发过 `reset:N10,y` 和 `load:<不存在文件>` 两种**零位移**命令。
  "路线能驱动车"这件事由内置路线的自动出发分支（5.1 分支 B）从代码路径上证明，
  但**整车实跑未做** —— 需要车悬空或场地清空时再验。
- `R:F11` 这类倒车段的实际表现、`reset:` 后航迹推算的重锚精度，**本轮未涉及**。
- 车侧改动是**直接 SSH 写盘**的（有备份 + 自检 + 回滚），不是"用户手动替换文件"那条约定路径。
