---
来源: 用户2026-10-25全仓场地坐标统一指令
更新: 2026-10-25
status: in_progress
ns: traj-1025
agent: /root
owner: /root
reviewer: /root/baseline
树节点: TR-10
写域: 隔离副本core/config.py、config/**、scout/**、route/validate.py、tools/global_planner.py、shoot_planner.py、sim_map_render.py；副本dzsmartscout/tools/sim_airground与planner同步工具；本ns治理登记
---

# TRAJ-FRAME-TOOLS-20261025 · 外围消费者与全仓坐标审计

## 1 台账与目标
已读D-004/D-012/D-023/D-041；用户明确+X朝M10西、N9为北，以D-042取代旧场地命名。保留tour、左转正号、原物理门限。

## 2 复用与新增理由
复用core.geometry唯一真源及既有sync_planner_from_source.py；vendor是生成物不手改。没有新轮子。相机/机体/IMU本地轴不盲换，只迁移场地适配。

## 3 写域与状态
仅隔离副本，运行链写权未交还，不能宣称已切换共享工作区。核心/物理/运行链由三张子卡独占；root持本卡front matter所列外围文件。

## 4 验收
原生全量与逐套旧红对照；现行场地坐标引用审计；原plan_route/route_check；两图刚体旋转一致性；vendor同步verify-only与既有对拍。不得部署。

## 5 回滚点
隔离副本source-manifest.json、control-sync-manifest.json和共享源码原件；只输出本任务差异，不覆盖他人改动。

## 6 执行与结论
进行中；_tmp_session/traj-1025/frame-migration隔离副本已建立，所有修改只在该副本。

2026-10-25续：复制的core/config.py与robot.yaml起步复位改N10,x，原末N5,y按同一物理朝向转N5,x；shoot/global工具从core导入坐标真源；sim_airground的geo去掉重复公式，绘图屏幕适配新轴；sim_map_render保持A左N右、1上11下。未修改云台/IMU机体坐标。scout/**与test_scout、test_dual_shoot已按单写者转交TR-7；root不再写这些文件。TR-9追加car_localization生产者及现有测试夹具写域，仍限隔离副本。

用户追加验收：按地图规则随机64张新图。复用active stressbench.generateMap，固定seed20261025+i，四类布局都变化、64/64唯一，不按规划是否成功筛图。完整输入与生成器哈希在 `_tmp_session/traj-1025/frame-migration/random64/manifest.json`。两图仅保留对照，不替代64压测。外测仍未通过、运行链原文件写权仍未交还。

## 9 经验总结
现象：串口生产者、旧executor、离线工具、无人机生成副本均有旧公式。处置：迁移发布/消费整链，历史证据不改写，现行从真源取值，生成物复用同步入口。

## 10 文档变更
新增本卡；修改树、D-042及现行坐标说明；归档无。
