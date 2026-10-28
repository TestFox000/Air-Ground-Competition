# dzsmartscout · 无人机工作区

> 继承根 `AGENTS.md`（v3）全部铁律。本文件只放无人机专属判断，不重复全局规则。

## 关键路径（实测）

| 你要干什么 | 去哪 |
|---|---|
| 飞控上层（路径/录制/跟随） | `src/px4_path/` |
| 飞控底层（键盘/Goto/Offboard） | `src/px4_control/` |
| 机→车指令链路 | `src/ag_integrate/`（fire_guard.py 禁火闸） |
| 补丁 | `fc-audit/fixes/`（补丁 01–06） |
| 审计笔记 | `fc-audit/notes/` |
| 机上运行手册 | 机上 `~/dzpx4/1.txt` |

## 无人机专属判据

1. **飞控话题契约必须严格配对**：
   - 订阅 `/fmu/out/vehicle_local_position`（KeepLast5 best_effort）
   - 订阅 `/px4_control/goto_enabled`（reliable + **transient_local 锁存**）
   - 发布 `/px4_control/goto`（KeepLast10）
2. **录制与跟随是两个独立状态**：录制只在 Ctrl-C 析构时落盘（.tmp + rename 原子替换）；跟随路径**只在构造时读一次，无重载入口**。
3. **禁火闸独立于云台**：`fire_guard.py`（Python，靠停靠格邻格有蓝靶才放行）不依赖云台驱动，可单独移植。
4. **无人机侧无 git**：改完先本地验证，部署由用户确认。
5. **凭据风险**：根仓跟踪了 dzsmartscout 下的 P0 凭据脚本（见项目总览）。改部署脚本前先查是否含明文口令/私钥。
6. **机上启动顺序固定**（7 步，见机上运行手册）：livox → fast_lio → MicroXRCEAgent → px4_odom_convert → px4_imu_bridge → px4_control → px4_path。