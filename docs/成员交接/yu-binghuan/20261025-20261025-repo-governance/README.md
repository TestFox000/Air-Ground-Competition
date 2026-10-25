# 仓库治理执行：密码清除/分支留档/文件抢救/陷阱标记/整体备份/规范副本

- 归档日期：2026-10-25
- 归档目录：`docs/成员交接/yu-binghuan/20261025-20261025-repo-governance/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

对本地目录扫描报告的 5 项发现独立核验后执行修复：①删 default/_tmp_pw.txt 明文密码（7字符，引用它的临时脚本指向废弃主机随删失效）；②a-local-before-merge 7 个本地独有提交推送留档为 preserve/a-local-before-merge（09fd517a）；③抢救备份仓只此一份的未跟踪文件 tools/orin_chain.py（13.6KB Orin 一键链设计，其口径缺口与审查复核 B-部署对账同源，建议评审采纳）与 无人机一键部署.bat；④陷阱仓打只读标记（2037 删除是备份残缺、严禁 git add -A）；⑤default/ 2.3GB 整体 tar 备份；⑥自 Gitee 新建规范工作副本 air_ground@main ecb9bdce（clone 后擦除令牌）。附带核验：main 改写者是 Lead/TestFox000（团队有组织治理，今日活跃），双地址同仓，历史推送全部完好。遗留：tar 需拷外部介质、default/ 最小 git 化、orin_chain.py 归属确认、agv-pathplan 补 .gitignore。

## 文件清单

| 文件 | 大小 |
|---|---|
| `20261025-仓库治理/rescued_from_backup/tools/orin_chain.py` | 13,616 B |
| `20261025-仓库治理/rescued_from_backup/无人机一键部署.bat` | 315 B |
| `20261025-仓库治理/仓库治理记录-20261025.md` | 3,744 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。