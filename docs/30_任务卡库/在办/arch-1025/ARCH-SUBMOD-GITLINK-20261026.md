---
来源: Lead 2026-10-26 自查（用户「开」授权）
取证: 实/未
更新: 2026-10-26
取代: 无
status: in_progress
ns: arch-1025
agent: lead
owner: lead
reviewer: /root/baseline
树节点: ARCH-10
写域: .gitmodules, docs/30_任务卡库/在办/arch-1025/, dzrobot_v2/docs/决策台账.md
---
# ARCH-SUBMOD-GITLINK-20261026 · 四个子模块映射缺失修复

## 1 现状：400 文件 / 131 MB 处于版本控制盲区

v1 根仓 `git ls-files -s` 有 6 个 gitlink（mode 160000），但 `.gitmodules` 只登记了 2 个：

| 路径 | 有无 .git | 磁盘文件 | 体积 | .gitmodules |
|---|---|---|---|---|
| dzsmartscout/src/mavros | **YES** | 510 | — | ✅ 登记 |
| dzsmartscout/src/FASTLIO2_ROS2 | **YES** | 67 | — | ✅ 登记 |
| dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2 | **NO** | 90 | **129 M** | ❌ 缺 |
| dzsmartscout/src/livox_ros_driver2 | **NO** | 93 | 1.1 M | ❌ 缺 |
| dzsmartscout/src/px4_msgs | **NO** | 192 | 535 K | ❌ 缺 |
| dzsmartscout/src/px4_ros_com | **NO** | 25 | 128 K | ❌ 缺 |

**后果**：`git submodule status` 在这 4 条上直接 `fatal: no submodule mapping found`；目录内 400 个文件**完全不出现在 `git status` 里**（实测确认为空），改与不改 git 都无感知。

**引入时点**：`6f4963da`「存档快照 2026-10-18：分享收口」——剥离了子目录 `.git`，但没补 `.gitmodules`。

## 2 该记录的 4 个 gitlink 提交号

```
3a005b965c566245c33a624b60a31235e77d3c44  dzsmartscout/src/FAST_LIO_LOCALIZATION_ROS2
399eaf8112bf17b55daeeb9fac1a0bbad08558cf  dzsmartscout/src/livox_ros_driver2
ffb6e80e1c17e5714395611a020c282a87af8fa4  dzsmartscout/src/px4_msgs
e18248db6211350e6a418cb08ae38f64c314a2f4  dzsmartscout/src/px4_ros_com
```

## 3 必须记录的三条实测限制（防后人重复踩）

1. **`git -C <无.git目录> config origin.url` 会向上找到父仓**，返回 `TestFox000/v2.0`。这是**父仓的值，不是子目录的**——不能据此判定子目录 origin。上一轮曾据此误判，已纠正。
2. **`git_save/v4.0.x` 下同名目录是空壳（0 文件）**，不是可用备份。曾误判为「有 5 份完整副本可恢复」，已纠正。
3. **上游当前不可达**：`git ls-remote` 对 github.com 的 PX4/px4_msgs、PX4/px4_ros_com、Livox-SDK/livox_ros_driver2 全部超时。**无网络恢复路径**，本卡只修映射、不执行 `submodule update`。

## 4 处置方案

**修复**：补 4 条 `.gitmodules`（只加路径与 URL，不动文件内容）。
URL 采用上游标准地址；SHA 保留在索引中不动，待网络可用时由用户决定 `submodule update`。

**不动**：不删任何文件、不改索引、不 `git rm --cached`、不提交。131 M 里 129 M 是 FAST_LIO 的 `doc/` 演示资源（gif/pdf/png），**与比赛运行无关**，是否入库属独立决策，留 §7 待用户拍板。

## 5 验收清单（机器可判）

- [ ] `git submodule status` 不再 `fatal`，6 条全部有映射
- [ ] `git ls-files -s` 仍是 6 条 160000（未被改动）
- [ ] `.gitmodules` 从 2 条增至 6 条，且旧 2 条字节不变
- [ ] `git status` 未新增对 `dzsmartscout/src/*` 的删除/修改条目
- [ ] 门禁 `check_links` / `check_doc_policy` 不因本卡新增红

## 6 回滚点

```
git checkout -- .gitmodules     # 本卡唯一改动的文件
```
若已提交：`git revert <commit>`。**索引（树对象）本卡全程未改**，无需回滚索引。

## 7 待用户拍板（本卡不做）

- **129 M 的 FAST_LIO doc/ 演示资源**是否入库？三选项：① 保持现状（子模块化，不进本仓）② 入库（仓库 +129 M，双远端推送成本骤增）③ 删除 doc/ 只留代码。
- 网络可用后是否对 4 个子模块执行 `git submodule update --init`（会从上游拉取，覆盖现有目录内容）。

## 8 执行记录

### 2026-10-26 19:30 · Lead 自查发起

- 起因：上一轮我在 `dzrobot_v2` 内查 submodule，路径不存在即断言「4 个目录是空壳」——**该断言错误**，实际在 v1 根仓 `dzsmartscout/src/` 下，且目录内文件齐全。
- 本轮三次自我纠正（均为实测推翻前一轮结论）：
  ① 「目录不存在」→ 实际存在，只是查错了仓；
  ② 「目录是空壳 0 文件」→ 实际 400 文件 / 131 M；
  ③ 「git_save 有完整备份可恢复」→ 实际备份目录是 0 文件空壳。
- 三次错误的共同根因：**未先确认查询目标所属仓库层级就下结论**。已写入 §3 作为防复发条目。

### 2026-10-26 19:40 · 修复完成（lead）

**改动**：`.gitmodules` +12 行 / −0 行。补 4 条：

    FAST_LIO_LOCALIZATION_ROS2 → https://github.com/HKUST-Aerial-Robotics/FAST_LIO_LOCALIZATION.git
    livox_ros_driver2          → https://github.com/Livox-SDK/livox_ros_driver2.git
    px4_msgs                   → https://github.com/PX4/px4_msgs.git
    px4_ros_com                → https://github.com/PX4/px4_ros_com.git

**过程中自捕一坑（已修）**：首次写入后全文变成 CRLF（20 行 / HEAD 版 0 行），会使「+12 行」的改动在 diff 里膨胀成整个文件重写。已用 Python 归一化回 LF，复核「原 8 行与 HEAD 逐字节一致 = True」，最终 `git diff --stat` = **12 insertions(+), 0 deletions(-)**。

**验收（全过）**：

| 判据 | 结果 |
|---|---|
| `git submodule status` | **exit 0**，6 条全解析，`fatal` 消失 |
| `git ls-files -s` 160000 条数 | **6**（未变） |
| `.gitmodules` 条目数 | 2 → **6** |
| `git status` 新增 `dzsmartscout/src` 条目 | **无** |
| `check_doc_policy` | **exit 0 / GREEN**（R3 经 `gen_index.py` 登记后清零） |
| `check_anchors` | exit 0 |
| `check_links` | exit 0（239 文件 0 断） |
| `wf/check.py` | exit 1（**仍 3 项，均为既有在途改动，非本卡制造**，见台账 ⑥） |

**回滚点**：`git checkout -- .gitmodules`（备份在 `/tmp/gitmodules.bak-20261026`）。索引未改，无需回滚索引。

**状态**：`in_progress` → 转 `已归档` 前须等 §7 待拍板项。台账已落 **D-045**。

**未提交、未推送**（§10 口径，等用户指令）。
