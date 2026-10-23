# 审查修复补丁起草：D1/D4/C2 + 部署对账 B

- 归档日期：2026-10-24
- 归档目录：`docs/成员交接/yu-binghuan/20261024-20261023-audit-fix-patches/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

审查复核后续：起草三份修复补丁（只出 patch 不合入不部署）。D1=移动靶 blue_face 显式校验（裸 KeyError→可行动报错，点名靶格与非法值）；D4=射击格只标首次出现（回环重访不重复开火，弹量契约）；C2=MapLocalizer 内参纵横比安全化（16:9 行为逐位不变，非 16:9 fx/fy 同比缩放+主点重定位；同款函数三处：mjpg 13类/~/dzuav 7类/仓库main，C++ 需过机上编译验证）。本地验证：新增 tests/test_audit_fixes.py 11/11 过，全量 109 过/15 既有失败零新增（底本=无人机 310558e 部署态）。B 部署对账：仓库 main dzpx4_patch 与无人机 ~/dzpx4 差异四项——F1 steadyNowS 修复未上机（机上仍混时钟域）、armed 判据两版不一致（main ==2||==4 vs 机上 ==2，部署前必须先拍板#7）、nan_drop/z_src 前兆信号链整条未上机、localization_watchdog 整个未部署。待办：无人机链路恢复后 C2 机上编译验证+nan_drop 字段核验；205ffba 底本待确认（疑在车端）；机上 --sahi 批次漂移未提交。

## 文件清单

| 文件 | 大小 |
|---|---|
| `patches/B-部署对账.md` | 2,438 B |
| `patches/C2-intrinsics-aspect-safe.patch` | 2,224 B |
| `patches/D1-blueface-validation.patch` | 1,429 B |
| `patches/D4-shoot-star-once.patch` | 1,326 B |
| `test_audit_fixes.py` | 3,098 B |
| `make_patches.py` | 7,024 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。