# 循迹调试协议（E0~E6）——工具 tools/track_debug.py

原理只有一句：**把"规划线 / 车端信念 / 真实轨迹"画到同一张场地图上**。
真值来源 = 手机 + 三脚架俯拍 + 车顶 ArUco 码（`track` 子命令，精度 ~1-2cm）；
深度相机上车用于滑移/倒车的复核取证；无人机 0x4002 是 belief 之一，
全链测试时用 `sniff` 记下来与真值对齐（顺带验证 dzuav 的 PnP 精度）。

## 装备与准备

- **手机**：三脚架架 2~2.5m 高，俯拍测试区（不用拍全场，画面里至少有
  4 个网格线交点即可）；1080p60 优先（计时要 60fps）。
- **ArUco 码**：`python tools/track_debug.py tag --id 0 --size-mm 80`
  生成 PNG，100% 缩放打印，贴车顶正中（码向 = 车头向）。
- **喂路线（不需要无人机/sidecar）**：手发官方话题，注意**车端方言**
  （本机 x/y 与车端差 90°，本机 x→车端 -y、本机 y→车端 x）：

```bash
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "data: 'reset:N10,x'"
ros2 topic pub --once /nav/waypoint std_msgs/msg/String "data: 'N10 M9 M5,x'"
```

  ⚠️ 手发路线绕过了 integrate_node 的对齐与闸门（这正是隔离测试要的），
  但前提是把车**人工摆正**到 reset 断言的朝向，否则测的就是画圆复现。
- **对时**：开录 + 开车即 t=0；对齐两路数据用"起步瞬间"（视频里车动
  = 轨迹第一点；belief 里首条位姿变化），不依赖墙钟。

## 实验阶梯（每步只回答一个问题，数据决定一个配置）

| # | 路线（车端方言示例） | 回答的问题 | 决定 |
|---|----------------------|-----------|------|
| E0 | 任一直线 | 标定精度：相邻两格心实测距离应 0.35±0.02m | 工具可用性 |
| E1 | `N10 N5,x` 正交直线往返 | 巡航速度、加减速、直线横向漂移 | time_cruise_mps / time_accel_overhead_s |
| E2 | `A1 B2 C3 D4,y` 纯对角 | 对角段横向误差 vs E1 差多少 | allow_diagonal 去留 |
| E3 | 45°/90°/135° 各一转角（空旷处 + 桶旁各一次） | 转角切角量、到点半径内是否换向 | turn90_guard 实证 |
| E4 | `A1 A5 A1,x` 回头 | **真倒车还是画圈掉头**（overlay 红叉 + 倒车段报告） | reverse_gear 去留 |
| E5 | 拐角处放"雷"（任意标记物） | 轨迹距雷格心 ≤12cm？ | 雷格过心补点验证 |
| E6 | 全链：无人机起飞 + sidecar dry-run + 真车 | 三线对齐；0x4002 观测 vs 手机真值 | PnP 车位姿精度 |

## 数据处理

```bash
# 1) 视频 -> 轨迹 CSV（交互：点 4 个网格线交点 TL,TR,BR,BL + 输入线编号）
python tools/track_debug.py track D:/video/run01.mp4 --out traj01.csv

# 2) 叠加图 + 指标报告（横向误差/航点到达/倒车段/分段速度/雷格过心）
python tools/track_debug.py analyze --route 'N10 M9 M5,x' --traj traj01.csv
python tools/track_debug.py analyze --route-plan /tmp/ag_plan_live/route_plan.json \
    --traj traj06.csv          # E6 全链：直接吃机载产物

# 3) 笔记本侧记录无人机观测（只能笔记本跑！机上/车端同端口 bind 会 fail-fast）
python tools/track_debug.py sniff --duration 180
```

`analyze` 产物：`track_out/overlay.png`（规划线蓝/轨迹按时间着色/倒车红叉/
雷格 12cm 圆）+ `report.txt`。**回填口径**：分段速度中位 → `time_cruise_mps`；
倒车段近零而路线含回头 → 关 `reverse_gear`；E2 对角横向误差明显大于 E1 →
关 `allow_diagonal`；逐航点 dmin 大量 >15.7cm → 执行器到点切换/切角问题。

## 深度相机的角色

上车录像（RGBD）不做第一真值——它跟车一起动，全局位置还得靠外部参照。
它回答两个手机拍不到的问题：
1. **滑移**：原地转向/对齐爬行时，轮上里程计位移 vs RGBD 帧间位移的差值；
2. **倒车取证**：E4 时面向前方的相机画面倒退 = 物理倒车（overlay 红叉互证）。
相机型号定了（Realsense/Astra/…）再补 VO 对比脚本。

## 现场带回清单（每实验）

- traj*.csv + overlay.png + report.txt（`track_out/` 打包）
- E6 另带 sniff.csv（0x4002/0x4003 时间线）
- 当局用掉的 yaml/参数快照（改了哪个开关）
