# G9 折返根因与 E8 靶标核实

- 归档日期：2026-10-18
- 归档目录：`docs/成员交接/yu-binghuan/20261018-g9-deadend-e8-target-audit/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

G9 折返根因与 E8 靶标核实（终版，2026-10-19 已同步为当前可复现结果）：

1. 折返触发源是 E8 的幻影静靶：两段视频检出「蓝@E8」（实为一只蓝色圆桶 φ≈0.25m 被蓝面分类头误判，检测框 283×270px 异常巨大），把 E8 当实靶后射击停靠位进入必经点集合，G9 被 TSP 排进死角，逼出 …H9 G9 H9 H8…。另据现场确认，该处本就是直线倒车进退（航向变化 0°、无转向弧线），即使保留也可执行。

2. E8 处无静靶与裁判先验冲突：match_params.yaml 明确「全场唯一靶 = 移动靶 D7-E7」，全场侦察检出靶为空；205245 的 E8 仅 2 票，091746 还把官方障碍 B7/E10 误判为蓝靶。

3. 修复（video_field_recon.py 靶先验裁决）：视频检出的静靶若落在移动靶轨道一格邻域内 → 判误检剔除、回退移动靶两段式（未辨@D7 → 确认点，禁火 fail-safe）；剔除项写入 evidence.targets_reclassified_referee_conflict。当前规划器下若保留 E8 当靶，还会触发「终点朝向不可达 → 降级进终点（丢 5 分）」，修复后一并消除。

4. 结果：四段全部 35 格、零重复格、排雷 8/8、避障与转弯可行性全通过、无射击位；pytest 路径规划交接_20261017/tests = 148 passed；结果可复现（41 个 PYTHONHASHSEED × 3 种转弯注入 × 带/不带 PYTHONPATH 全部同一路线）。

5. 版本注意：首版归档记录的「修复前 39 格/H9、修复后 38 格」是当天那次规划器版本下的结果；mission_router.py 最后修改于 2026-10-18 17:12，晚于那批出图（16:01–16:02），故本目录「修复后」渲染/航点/route_plan 已按 2026-10-19 重跑结果同步（输出根 视频侦察_20261019/）。

6. 遗留（非阻塞）：若现场确认 E8 格确有实体桶且车过不去，应把 E8 由「靶」改判「障碍」（当前只剔靶不新增障碍），届时重跑再归档。

## 文件清单

| 文件 | 大小 |
|---|---|
| `G9折返与E8靶标核实.md` | 6,329 B |
| `evidence/e8_box_205906_f240.jpg` | 24,690 B |
| `evidence/e8_object_205245_f2115.jpg` | 120,877 B |
| `evidence/e8_object_205906_f225.jpg` | 101,058 B |
| `evidence/e8_object_205906_f240.jpg` | 123,483 B |
| `evidence/route_plan_205245_修复后.json` | 2,556 B |
| `evidence/route_plan_205906_修复后.json` | 2,531 B |
| `evidence/route_render_205245_修复前_折返.png` | 42,799 B |
| `evidence/route_render_205245_修复后.png` | 42,449 B |
| `evidence/route_render_205906_修复前_折返.png` | 42,235 B |
| `evidence/route_render_205906_修复后.png` | 41,888 B |
| `evidence/waypoint_map_205245_修复后.png` | 51,109 B |
| `evidence/实测数据.json` | 4,398 B |

## 复现方式

```bash
# 视频侦察管线（Windows 可直接跑，无需 ROS）
python video_field_recon.py <video.mp4> --stride 15 --out <输出目录>
python render_waypoint_map.py <输出目录>
```

> 本 README 由 push_deliverable.py 自动生成。