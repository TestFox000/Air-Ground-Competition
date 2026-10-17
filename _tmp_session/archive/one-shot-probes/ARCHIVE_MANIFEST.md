# t73 · 一次性探针归档清单（移动，非删除）

| 项 | 值 |
|---|---|
| 卡 | t73（依据 t65 审计 §① D-4 保守子集） |
| 执行 | `drone-link`，2026-10-15 21:3x（本机时钟） |
| 归档根 | `_tmp_session/archive/one-shot-probes/`（**保留相对工作区的路径**） |
| 移动条数 / 体积 | **26 个 / 19640 B（19.2 KB）** |
| 动作性质 | **只 move**，零删除；每个文件移动前后 md5 逐条比对一致 |
| 回滚 | 见文末「回滚」一节（一条命令，逐条可逆） |

## 1. 移动清单（原路径 → 新路径）

| # | 原路径（工作区相对） | 新路径（工作区相对） | 体积 | md5(前8) |
|---|---|---|---|---|
| 1 | `dzsmartscout/_pdf_arena.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_arena.py` | 766 B | `18258092` |
| 2 | `dzsmartscout/_pdf_arena_img.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_arena_img.py` | 1077 B | `1493014a` |
| 3 | `dzsmartscout/_pdf_arena_sheet.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_arena_sheet.py` | 1139 B | `f17fd8f0` |
| 4 | `dzsmartscout/_pdf_arena_sheet2.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_arena_sheet2.py` | 1252 B | `d32d3cdc` |
| 5 | `dzsmartscout/_pdf_arena_sheet3.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_arena_sheet3.py` | 1246 B | `741474f4` |
| 6 | `dzsmartscout/_pdf_extract.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_extract.py` | 877 B | `a94b8e5b` |
| 7 | `dzsmartscout/_pdf_vnc.py` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_pdf_vnc.py` | 757 B | `6cb8e10d` |
| 8 | `dzsmartscout/_probe_thermal.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/_probe_thermal.sh` | 233 B | `bfe36cdd` |
| 9 | `dzsmartscout/tools/_tmp_car_chassis_src.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_chassis_src.sh` | 637 B | `c5d091c4` |
| 10 | `dzsmartscout/tools/_tmp_car_code.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_code.sh` | 947 B | `3c3e3de3` |
| 11 | `dzsmartscout/tools/_tmp_car_logs.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_logs.sh` | 923 B | `1494ad97` |
| 12 | `dzsmartscout/tools/_tmp_car_logs2.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_logs2.sh` | 388 B | `f710bf97` |
| 13 | `dzsmartscout/tools/_tmp_car_preflight.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_preflight.sh` | 923 B | `44ef4be3` |
| 14 | `dzsmartscout/tools/_tmp_car_reset.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_reset.sh` | 578 B | `7e11bee6` |
| 15 | `dzsmartscout/tools/_tmp_car_src.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_src.sh` | 585 B | `72262054` |
| 16 | `dzsmartscout/tools/_tmp_car_src2.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_src2.sh` | 503 B | `e564983b` |
| 17 | `dzsmartscout/tools/_tmp_car_traj.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_traj.sh` | 1213 B | `408d7aa8` |
| 18 | `dzsmartscout/tools/_tmp_car_where.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_car_where.sh` | 919 B | `efeaf6c9` |
| 19 | `dzsmartscout/tools/_tmp_drone_gates.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_drone_gates.sh` | 1029 B | `3818db5f` |
| 20 | `dzsmartscout/tools/_tmp_drone_hist.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_drone_hist.sh` | 395 B | `1703418c` |
| 21 | `dzsmartscout/tools/_tmp_drone_topics.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_drone_topics.sh` | 410 B | `56884068` |
| 22 | `dzsmartscout/tools/_tmp_drone_values.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_drone_values.sh` | 1017 B | `7c7691cf` |
| 23 | `dzsmartscout/tools/_tmp_probe_auth.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp_probe_auth.sh` | 573 B | `09d06ec1` |
| 24 | `dzsmartscout/tools/_tmp/car_cfg.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp/car_cfg.sh` | 670 B | `b2d74ba0` |
| 25 | `dzsmartscout/tools/_tmp/car_probe_run.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp/car_probe_run.sh` | 259 B | `09dd78b7` |
| 26 | `dzsmartscout/tools/_tmp/dzsend.sh` | `_tmp_session/archive/one-shot-probes/dzsmartscout/tools/_tmp/dzsend.sh` | 324 B | `39c3cff6` |

## 2. 入选判据（为什么是这 26 个）

同时满足：

1. **来自 t65 审计 §① 的 91 条「无任何提及」清单**（零代码调用点 **且** 零文档提及）；
2. **文件名自我标识为一次性/临时**：根目录 `_pdf_*.py`（7）与 `_probe_*.sh`（1）；`tools/_tmp*_*.sh`（15）与 `tools/_tmp/*.sh`（3）。

> 名字是通用工具、或可能仍被人手动使用的条目**一律没动** —— 见 §3 待确认清单。

## 3. 待确认清单（**未动**，含理由）

### 3.1 同为「零调用点」但需你裁决的 8 条

| 条目 | 零调用点？ | 不动的理由 |
|---|---|---|
| `dzsmartscout/adapt_drone_plugin.mjs` | 是 | 名字是“适配插件”通用工具，非明示一次性；可能仍用于 DSH 插件打包流程 |
| `dzsmartscout/asar_list.js` | 是 | asar 内容列举小工具，属“开发工具”而非探针；将来重新打包插件时可能用 |
| `dzsmartscout/ascii_icon_check.py` | 是 | 图标自检工具（含 1 个死函数 `sample_ys()`）；用途偏工具，待你定义“一次性”边界 |
| `dzsmartscout/fix_mdns_nsswitch.py` | 是（代码） | **被车侧文档引用**：`dzrobot_v2/docs/agent_archive/v2-全量整理-2026-10-12.md:501`（作为“SSH 口令明文 17 处”证据行）；移动会让该引用失效 |
| `dzsmartscout/run_fix_drone_avahi.py` | 是（代码） | 同上（`:501`） |
| `dzsmartscout/vnc_handshake.ps1` | 是（代码） | **被车侧文档引用**：`...:503`（“VNC 口令明文”证据行） |
| `dzsmartscout/tools/_tmp_car_read2.sh` | 是（代码） | **被车侧文档引用**：`...:505`（同上，明文口令证据行） |
| `dzsmartscout/tools/_tmp_car_stop.sh` | 是（代码） | 同上（`:505`） |

### 3.2 本卡范围/禁则直接排除的（未动，仅列计数与理由）

| 组 | 条数 | 理由 |
|---|---|---|
| `dzsmartscout/tools/drone/**` | 33 | 卡内**明文禁则**：不动 tools/drone 里被引用或有文档提及的脚本；且这批是**手工 ssh 调用**的机上探针，零调用点 ≠ 未使用 |
| `dzsmartscout/tools/link/**` | 12 | 同上（含 t12/t18 取证脚本，历史卡的证据来源） |
| `dzsmartscout/dsh-plugin-dzdrone-ssh/**` | 7 | 卡内**明文禁则**：不动 dsh-plugin |
| `dzsmartscout/fc-audit/fixes/04-camera-bringup/files/make_checkerboard.py` | 1 | 是 fix 包的**载荷文件**（`files/` 目录=要拷到机器上的东西），不是临时脚本 |
| `dzsmartscout/fc-audit/src-mirror/px4_ros_com/launch/sensor_combined_listener.launch.py` | 1 | 第三方上游镜像（与 `dzsmartscout/src/**` 同源），删改须连带镜像策略 |
| `dzsmartscout/.pdfdeps/extract2.py` | 1 | 位于**内嵌依赖目录** `.pdfdeps/`（同目录 `pypdf` 是第三方库）；移动恐影响该目录自洽 |
| `dzsmartscout/drone-scripts/start-lidar-on-link.sh` | 1 | **机上**启动脚本（同 `tools/drone` 类：靠人手动在无人机上执行），风险同 3.1 的“手动调用” |
| `dzsmartscout/tools/planner/tests/test_planner_sync.py` | 1 | 是 **unittest 发现式调用**的测试，不是死文件（t65 已单列说明） |

## 4. 复验证据（零调用点匹配结果）

**方法**：AST/正则双证据之外，按 t65 同口径做**大小写敏感 + 词边界**的全语料字面匹配：
正则 `(?<![A-Za-z0-9_])<stem>(?![A-Za-z0-9_])` 或完整文件名子串；语料 = `dzsmartscout` + `dzrobot_v2` + `.agent-teams`（排除 `node_modules/novnc/__pycache__/install/build/log/.git/artifacts/_backups`），本次 **5203 个文本文件**。

| 阶段 | 结果 |
|---|---|
| 移动前（26 条逐个） | 唯一命中来自**本卡自身的审计产物**（`_tmp_session/cleanup-audit/t65_dzsmartscout审计.md`）与团队状态 `.agent-teams/…/team.json`；**无任何代码调用点**（5 条另有车侧文档引用 → 已剔除进 §3.1） |
| 移动后（26 条逐个重扫） | **真实引用 = 0 条**（排除归档自身与审计产物） |
| 完整性 | 源路径全部消失 = True；归档全部就位 = True；**归档 md5 与移动前逐条一致 = True** |

**原始记录**：`_move_raw.json`（本目录）——每条的 old/new/size/md5/moved_ok/source_gone。

## 5. 回滚（一条命令，逐条可逆）

```powershell
cd D:\User\Desktop\比赛存档\v1
# 依据本目录 _move_raw.json 逐条搬回原位（仅 move，不覆盖已存在文件）
python -c "import json,shutil,pathlib,os; ws=pathlib.Path('.'); d=json.load(open(r'_tmp_session/archive/one-shot-probes/_move_raw.json',encoding='utf-8')); [ (pathlib.Path(r['new']).exists() and not pathlib.Path(r['old']).exists() and (pathlib.Path(r['old']).parent.mkdir(parents=True,exist_ok=True), shutil.move(r['new'], r['old'])) ) for r in d ]; print('restore done')"
```

> 回滚后请再用同口径扫一次零调用点（方法见 §4），确认没有把活文件搬错。

## 6. 边界声明

* **零删除**：只有 `move`，共 26 次；每个文件移动前后 md5 一致（见 §1/§4）。
* 未动 `tools/drone/**`、`tools/link/**`、`dsh-plugin/**`（卡内明文禁则）；未动 `vendor/**`；未碰 `dzrobot_v2/**`（仅作为反向 grep 的被引用方读取）。
* **无 git 操作**（未 add/commit/rm；未用 `git mv`——纯文件系统 move）。
* 本卡写入：本清单 + `_move_raw.json` + 26 个归档文件（均为 move 产生）。

