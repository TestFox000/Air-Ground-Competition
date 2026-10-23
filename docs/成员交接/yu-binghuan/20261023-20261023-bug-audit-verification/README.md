# 审查复核：7 条未修发现逐条独立核验 + 部署差距新发现

- 归档日期：2026-10-23
- 归档目录：`docs/成员交接/yu-binghuan/20261023-20261023-bug-audit-verification/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

对 10-20 审查清单 7 条未修发现独立核验（底本：无人机 git HEAD 310558e + 收编基线 8cbfbb3 + Gitee main 0bc2196）。结论：7 条全部成立（行号按版本校正：#2 实际 L1299/基线L1275/main L1201；#5 实证在 main dznav executor_node.py L108+L466）。新发现：① 审查底本 205ffba 在 Gitee 全分支与无人机均不存在，行号证据指向无人机收编基线，需确认（或在车端仓库）；② 部署反向差距——F1 steadyNowS 与 nan_drop/z_src 前兆信号只在仓库 main 的 dzpx4_patch，无人机 ~/dzpx4 仍跑旧版（无 steadyNowS、arming==2 无==4），定位链咽喉件修复从未上机；按 main 整体部署会把 ==4 判据带上，须先拍板 #7；③ 机上收编后第一起新漂移：--sahi 批次 video_recon.py + export_recon_dataset.py 修改未提交。修复批次：先部署对账，再 C2/D1/D4，F4/D3 拍板后处理。

## 文件清单

| 文件 | 大小 |
|---|---|
| `审查复核-20261023.md` | 5,040 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。