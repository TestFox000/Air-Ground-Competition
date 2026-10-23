# 追加：机上仓库收编版本控制（git init 两段历史 + Gitee 快照分支）

- 归档日期：2026-10-23
- 归档目录：`docs/成员交接/yu-binghuan/20261023-20261023-orin-git-init/`
- 归档工具：`push_deliverable.py`（一键归档，见同目录说明）

## 说明

执行收编：机上 ~/dzuav_deploy/air_ground 此前 NO_GIT（漂移根因）。git init -b main，278 文件 2.6M；两段真实提交：8cbfbb3=收编时原始状态（含 10-22 C++ 规划接入与 --sahi 演进）、310558e=当日靶向对齐改动；engines/models 大二进制不入库由 BINARY_MANIFEST.md5 十条 md5 锁定；AGENTS.md 自 Gitee main 取随机入库；完整快照已推 Gitee snapshot/orin-20261023 分支（bundle 本机中转，令牌未上机未落盘）。安全边界：未触碰在跑的 --sahi 侦察进程、机上不设 remote、重复 init 防呆。

## 文件清单

| 文件 | 大小 |
|---|---|
| `git_init_orin.py` | 9,759 B |
| `README.md` | 6,589 B |

## 复现方式

```bash
# 视频侦察 → 路线规划（机上通用入口，任意视频；默认 stride 5 = 实测最优 8/8）
bash tools/orin_video_recon.sh <video.mp4>
bash tools/orin_video_recon.sh --replay-last   # 秒级：复用识别结果，按当前门控重算
```

> `video_field_recon.py` 与 `--stride 15` 是 2026-10-18 的旧归档口径，已退役：
> stride 15 会漏掉只短暂可见的雷（实测同段视频 8 颗只剩 2~3 颗，见 §6.10.2）。
> 本 README 由 push_deliverable.py 自动生成。