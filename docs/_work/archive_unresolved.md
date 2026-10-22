# 归档未解决 / 需 Lead 裁决事项

> 卡号：task-1 ｜ owner：archivist ｜ 日期：2026-10-22 ｜ 配套：[archive_manifest.csv](archive_manifest.csv)、[`docs/_archive/README.md`](../_archive/README.md)

## 0. 结论

- 复制：**118 / 118 成功**，0 个复制失败、0 个源文件缺失。
- 哈希：**118 / 118 一致**（逐文件 `Get-FileHash -Algorithm SHA256` + 字节数双向比对），**0 条 FALSE**。
- 篇数与任务书要求**完全一致（118）**，但构成不同，见 U-01。
- 「复制失败 / 哈希不符 / 该归档但找不到」三类**均无条目**；以下 4 条是需要 Lead 知情或拍板的事项。

## U-01 ★ 篇数 118 与任务书一致，但**构成不同（巧合抵消）**

| 项 | 任务书口径 | 实测 | 差 |
|---|---|---|---|
| `dzrobot_v2/docs/**` 总篇数 | 79 | **81** | +2 |
| 必须留在原位的例外 | 6 | **8** | +2 |
| 该归档篇数 | 73 | **73** | **0** |
| 合计 | 118 | **118** | **0** |

**专项目录实测**：

| `dzrobot_v2/docs` 子目录 | 任务书记载 | 实测 | 一致？ |
|---|---|---|---|
| `archive/20261018` | 41 | 41 | ✅ |
| `library` | 5 | 5 | ✅ |
| `agent_archive` | 3 | 3 | ✅ |
| `archive/`（本层 README） | — | 1 | — |
| **根目录** | 30（= 79−41−5−3） | **32** | ❌ 差 2 |

⇒ 两个错误恰好抵消：**总篇数少记 2** 与 **例外少记 2**，使最终篇数仍落在 118。**处置建议**：按 118 认账，无需动作；但对外口径应以「81 篇 − 8 例外 = 73」为准，不要写「79 篇」。

## U-02 ★★ `工作流v2.md` 与 `索引标注.yaml` 是门禁活文件，**已按新规从归档中撤回**

- **任务书列 6 个例外**（`决策台账`/`复用清单`/`目录索引.md`/`目录索引.json`/`能力索引`/`TEAM_STATUS`）。
- **2026-10-22 更新后的 `AGENTS.md §8.1` 把例外扩大到 8 个**，新增 `工作流v2.md` 与 `索引标注.yaml`。证据（本卡实读）：
  - `dzrobot_v2/tools/wf/check.py:31-35` → `REQUIRED_DOCS = ["docs/决策台账.md", "docs/复用清单.md", "docs/工作流v2.md"]`
  - `dzrobot_v2/tools/wf/repo_index.py:13` → `ANN = DZ / "docs" / "索引标注.yaml"`；同文件 `:14-15` 输出 `docs/目录索引.md` / `.json`
- **本卡动作（可追溯）**：第一轮按任务书的 6 例外执行，曾把这两篇复制进 `_archive`（当时共 120 篇）；读到更新后的 `AGENTS.md` 后**已删除这两个归档副本**，并以 8 例外重跑全量复制 → 最终 118 篇。原件自始至终未被动过。
- **风险消除**：若按旧 6 例外删除 `dzrobot_v2/docs/**`，`check.py` 会因缺 `REQUIRED_DOCS` 直接判红。现已避免。

## U-03 二进制件说明（3 个 .zip）

`docs/agent_handoff/packs/{L0_速览包,L1_接管包,L2_代码包}.zip` 不是「文档篇」，但属任务书第 2 类来源（`agent_handoff/**` 的 18 篇口径已含这 3 个包 + 3 个 MANIFEST）。已按**字节**复制并用 sha256 核验一致。

- **未核实**：3 个 zip 的内部内容是否与各自 MANIFEST 逐项一致（**未解包核验**，本卡只做字节级哈希）。

## U-04 未核实 / 边界声明

1. **未扫描全仓**：只处理任务书指定的 5 处来源。是否存在 5 处以外的散落旧长期文档 = **未核实**（`_tmp_session/**`、`dzrobot_v2/_tmp_session/**` 已由任务书划归 Lead 另行处置）。
2. **未跑收口门禁**：`python tools/wf/check.py` **不在本卡验收范围**，本卡未执行；删除原件后必须由 Lead 跑一次（重点看 `check_links.py` 断链 = 0）。
3. **同名重复件**：`docs/瞄准代码对比报告_gitee_vs_本地_20261018.md` 与 `dzrobot_v2/docs/` 下同名件内容重复，两份都已按各自原路径分别归档，**未合并、未去重**；删原件时两份都要删。
4. **编码**：`archive_manifest.csv` = UTF-8 **无 BOM**、CRLF。Excel 双击中文会乱码，请用「数据 → 从文本/CSV → 文件原始格式 65001」导入。
5. **未改任何原件**：全部操作为 `Copy-Item`（仅覆盖 `_archive` 内副本）；原件与归档件逐条字节数 + sha256 相等即为证。
6. **README 里的核对口径**：`docs/_archive` 现有 **119** 个文件 = 118 篇归档 + 1 个 `README.md`（本目录说明），不是 120。

## 复跑命令

```powershell
cd "D:\User\Desktop\比赛存档\v1"

# 幂等重跑：重算 118 篇复制 + 哈希 + 重写 CSV（脚本带 BOM，PS 5.1 才能正确解析中文）
. .\docs\_work\_do_archive.ps1
# 期望输出：SOURCE COUNT: 118 / ROWS WRITTEN: 118 / MISMATCH COUNT: 0 / TOTAL BYTES: 5184331

# 一致性闸
Select-String -Path docs\_work\archive_manifest.csv -Pattern ",FALSE$"   # 期望：无输出
(Get-ChildItem docs\_archive -File -Recurse).Count                        # 期望：119
```
