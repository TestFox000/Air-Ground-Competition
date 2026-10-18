# G9 折返根因与 E8 靶标核实

- 归档日期：2026-10-18
- 归档目录：`docs/成员交接/yu-binghuan/20261018-g9-deadend-e8-target-audit/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

两类问题的核实与修复（终版）：

1. 无人机视频侦察路线中，2 段（205245/205906）曾在雷格 G9 处出现折返（…H9 G9 H9 …）。根因查明为 E8 的幻影静靶把 G9 排进死角；剔除该幻影后折返随之消失。另据现场确认，该处 H9→G9→H9 本就是直线倒车进退（航向变化 0°，无转向弧线，车可倒车），即使保留也可执行。

2. E8 的「蓝@E8」核实为一只蓝色圆桶（φ≈0.25m）被蓝面分类头误判为靶（检测框 283×270px 异常巨大）；且 match_params.yaml 明确「全场唯一靶 = 移动靶 D7-E7」、全场侦察检出靶为空——与官方场地模型冲突。

修复（video_field_recon.py 靶先验裁决）：视频检出的静靶若落在移动靶轨道一格邻域内，判为误检剔除，回退移动靶两段式（未辨@D7 → 确认点，禁火 fail-safe）；剔除项写入 evidence.targets_reclassified_referee_conflict 可追溯。

结果：4 段全部 38 格、零重复格、排雷 8/8、避障与转弯可行性均通过、无朝桶射击位；pytest 路径规划交接_20261017/tests = 148 passed。证据含 E8 实物照与修复前后路线图对照。

遗留（非阻塞）：若现场确认 E8 格确有实体桶且车过不去，应把 E8 由「靶」改判「障碍」（当前只剔靶不新增障碍），届时重跑再归档。

## 文件清单

| 文件 | 大小 |
|---|---|
| `G9折返与E8靶标核实.md` | 5,282 B |
| `evidence/e8_box_205906_f240.jpg` | 24,690 B |
| `evidence/e8_object_205245_f2115.jpg` | 120,877 B |
| `evidence/e8_object_205906_f225.jpg` | 101,058 B |
| `evidence/e8_object_205906_f240.jpg` | 123,483 B |
| `evidence/route_render_205245_修复前_折返.png` | 42,799 B |
| `evidence/route_render_205245_修复后.png` | 42,318 B |
| `evidence/route_render_205906_修复前_折返.png` | 42,235 B |
| `evidence/route_render_205906_修复后.png` | 41,881 B |
| `evidence/waypoint_map_205245_修复后.png` | 50,713 B |
| `evidence/实测数据.json` | 3,572 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。