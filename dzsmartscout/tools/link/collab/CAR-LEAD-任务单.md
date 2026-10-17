# 无人机 ↔ 小车 ROS2 直连 · 交接单（给「小车负责人」）

- **发起方**：无人机负责人（DSH 会话 `session-ac93b6d9`，工作区 `D:\User\Desktop\dzsmartscout`）
- **时间**：2026-10-06 19:31
- **目标**：让 `dzsmart.local`（无人机）与 `duzhong.local`（小车）在 **ROS2 层直接互通**（不经任何中转/代理/本机转发）

---

## 一、我已经完成的（**只读侦察，未改小车任何配置**）

| 项 | 无人机 dzsmart | 小车 duzhong |
|---|---|---|
| 平台 | Jetson Orin Nano / Ubuntu 22.04.5 | Orange Pi 5（RK3588）/ Jammy |
| 联网网卡 | `wlP1p1s0` **10.240.174.104/24** | `wlan0` **10.240.174.188/24** |
| 默认网关 | 10.240.174.68 | 10.240.174.68（同一个 AP） |
| ROS | **Humble**，`ROS_DOMAIN_ID` 未设（=0），RMW 默认 FastDDS | **Humble**，`ROS_DOMAIN_ID` 未设（=0），RMW 默认 FastDDS |
| 工作区 | `~/dzpx4/install/setup.bash` | `~/dzrobot/install/setup.bash` |
| 当前节点 | 无（只有 `/rosout`、`/parameter_events`） | 无（只有 `/rosout`、`/parameter_events`） |

**已实测的互通性**：
- 无人机 → 小车：`ping` 3/3 通，0% 丢包，RTT 约 10ms，ARP `REACHABLE`
- 小车 → 无人机：`ping` 3/3 通，0% 丢包，RTT 约 11ms，ARP `REACHABLE`

> 结论：L2/L3 已经通，**两端 ROS2 环境完全一致**（同发行版、同 domain、同 RMW），只差一次跨机 DDS 实收实发的实测。

---

## 二、请你执行的（**1 条命令**）

无人机侧我已经启动了一个 **45 分钟的信标**（19:31 起，2Hz 发布 `/dzlink_hello`），所以小车侧**随时跑都能碰上**：

```bash
bash /tmp/beacon.sh 60 duzhong
```

说明：
- 脚本我已经拷到小车 `/tmp/beacon.sh`（2035 字节，纯 LF，无 CR）。**不需要你写或编译任何东西。**
- 它只做两件事：以 2Hz 发布 `/dzlink_hello`，同时订阅同一话题做统计。
- **不改任何配置**，只在 `/tmp/dzlink_duzhong.log` 留一份接收日志。
- 如果 `/tmp/beacon.sh` 不在（比如 /tmp 被清理），重新取一份即可：
  ```bash
  scp duzhong@10.240.174.104:/tmp/beacon.sh /tmp/beacon.sh   # 无人机上那份是同一文件，SHA256 可核对
  ```

---

## 三、怎么算成功 + 怎么回报

判定标准（脚本最后会自己打印）：

```
== 收到本机信标: N 条     ← N≈2×秒数，说明本机 ROS2 正常
== 收到对端信标: M 条     ← ★ M > 0 就代表跨机 DDS 直连成立
==> RESULT: LINK_OK / LINK_FAIL
```

**回报方式**：把命令输出（或最后 4 行）写进
`D:\User\Desktop\dzsmartscout\tools\link\collab\car-side-result.md`

> 你写完不用通知我——我会在用户切回无人机负责人会话时读这个文件。

---

## 四、如果 `M = 0`（失败时的排查路径，按顺序）

**第 1 步：确认组播是否被 AP 拦截**（Humble 自带工具，两端各跑一条）

```bash
# 小车（先跑，它会等 15 秒）
source /opt/ros/humble/setup.bash
ros2 multicast receive
```
```bash
# 无人机（紧接着跑，我来执行；你告诉我什么时候开始就行）
ros2 multicast send
```
- 小车收到 `Received from ...` → 组播正常，问题在 DDS 配置，走第 2 步；
- 小车超时无输出 → **AP 开了客户端隔离/禁了组播**，走第 2 步（单播兜底）或考虑改走有线/热点。

**第 2 步：FastDDS 单播兜底**（绕过组播发现）

现成配置：`D:\User\Desktop\dzsmartscout\tools\link\fastdds-unicast-wifi.xml`（占位符 `__SELF_WIFI_IP__` / `__PEER_IP__` 已注释标明怎么填）。
小车侧要做的（**由你决定并执行，我不动小车的配置**）：

```bash
# 1) 把配置里的 __SELF_WIFI_IP__ 换成 10.240.174.188，__PEER_IP__ 换成 10.240.174.104
# 2) 放到车上任意路径，例如 /tmp/dzlink-fastdds.xml
# 3) 带着环境变量重跑信标
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/dzlink-fastdds.xml
bash /tmp/beacon.sh 60 duzhong
```
> 这一步只设**环境变量**（当前 shell 生效，不落盘），不碰 `~/.bashrc`、不碰 `~/dzrobot`。
> 无人机侧我会同步用同一套配置，两边配置对齐后再测。

---

## 五、边界（避免互相踩）

- 我**没有**改小车的任何配置、环境变量、项目文件；本次只往小车 `/tmp` 放了 `beacon.sh` 一个探测脚本。
- 小车侧的结论以**你的实测输出**为准；我不会替你执行小车侧命令。
- 你在这份任务单之外如果有自己的部署/重启动作，请留意 `/tmp/beacon.sh` 是独立进程（结束即退，无常驻）。
- 反向验证（小车发、无人机收）在无人机侧的信标日志里能独立看到：
  `/tmp/dzlink_dzsmart.log` 会出现 `HELLO from duzhong`，这是**不依赖你回报**的旁证。
