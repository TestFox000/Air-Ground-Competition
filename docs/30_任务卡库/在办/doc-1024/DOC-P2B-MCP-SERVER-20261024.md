---
ns: doc-1024
agent: doc-mcpsmith
树节点: DOC-6
写域: docs/_工具/mcp_dzdoc_server.py, 插件 bundle 的 cordis.patch.yml
更新: 2026-10-24
status: in_progress
---

# DOC-P2B-MCP-SERVER-20261024 · stdio server + 白名单 + 注册

## 1 五必填

| 项 | 内容 |
|---|---|
| 读过的台账条目 | 无新增裁决。AGENTS §2（复用闸：新工具必须登记复用清单 + 能力索引）、§11.2（勿擅自增删远端）、§11.4（异常边界） |
| 复用检索结果 | 复用 docs/_工具/doc_search.py、gen_index.py、check_doc_policy.py、check_anchors.py、check_numbers.py、check_links.py、snapshot_docs.py 与 dzrobot_v2/tools/wf/check.py、check_links.py、check_reuse.py —— **server 只转发，不重写逻辑**（§2：复用清单是闸，不许造第二份） |
| 新增轮子及理由 | docs/_工具/mcp_dzdoc_server.py —— 现有工具是 CLI，子智能体无法一致调用；MCP 是 stdio JSON-RPC，主 agent 与子智能体同构可用 |
| 验收命令 | python docs/_工具/mcp_dzdoc_server.py 起服务后 list_tools 可见 8 个；doc_read 传 ../.env 被拒（返回 error 而非内容） |
| 回滚点 | server 是新文件；注册改动**必须回滚 cordis.patch.yml 那一段**（记下改前哈希） |

## 2 任务

1. 写 mcp_dzdoc_server.py（Python 3.12，stdio JSON-RPC），只暴露 8 个只读工具：
   doc_search / doc_read / doc_gate / doc_index_refresh / doc_policy_check / doc_anchors_check / doc_numbers_check / doc_links_check
2. doc_read 实现**路径白名单**：必须解析到 docs/ 内；.env、~/.ssh/id_rsa、任意绝对路径一律拒
3. doc_gate 串起 §8.6 顺序：gen_index → check_doc_policy → check_anchors → check_numbers → wf/check → check_links，返回 exit code + 摘要
4. **不注册** cleanup_* / salvage_* / repo_index --write 等 27 个危险脚本
5. 注册：照抄 C:\Users\TestFox\.dsh\cordis.patch.yml 里 rea-mcp 的格式，往**项目插件 bundle 的 cordis.patch.yml** 插 dzdoc-mcp（方案 B，不改全局 profile）
6. env 显式写 PYTHONIOENCODING: utf-8
7. 在 dzrobot_v2/docs/复用清单.md 与 能力索引.md 登记本 server（§2）

## 3 执行记录（追加）

执行时间：2026-10-24 18:2x–19:3x（Asia/Shanghai）。工作区根 D:/User/Desktop/比赛存档/v1。Python 3.12.10（C:/Users/TestFox/AppData/Local/Programs/Python/Python312/python.exe）。

### 3.1 交付物

| 文件 | 类型 | 说明 |
|---|---|---|
| `docs/_工具/mcp_dzdoc_server.py` | 新增 | stdio JSON-RPC server，Python 3.12，零第三方依赖，8 个只读工具 |
| `D:/User/Desktop/ZhiYuan/dsh_plugin/dsh-dzairground-ssh/cordis.patch.yml` | 修改 | 追加 `dzdoc-mcp` insert 条目（方案 B，项目插件 bundle） |
| `dzrobot_v2/docs/复用清单.md` | 修改 | §3 回归与门禁 加一行「文档只读 MCP server」 |
| `dzrobot_v2/docs/能力索引.md` | 修改 | 加一行「文档只读 MCP 封装」owner=doc-mcpsmith |

### 3.2 注册点与回滚哈希

注册格式照抄 `C:/Users/TestFox/.dsh/cordis.patch.yml` 的 rea-mcp：

    - insert:
        - id: dzdoc-mcp
          name: "@deepseek-ai/dsh-mcp-client"
          config:
            serverName: dzdoc
            transport: stdio
            command: C:\Users\TestFox\AppData\Local\Programs\Python\Python312\python.exe
            args:
              - D:\User\Desktop\比赛存档\v1\docs\_工具\mcp_dzdoc_server.py
            env:
              PYTHONIOENCODING: utf-8

- cordis.patch.yml 改前 sha256：`109a7fc7f41bbef00e29c520d176a09ecd46dc90bec83f3aa4348bbb7fdacef2`（317 B）
- 改后 sha256：`5a3bb92fb5eea13931abda87f6b2d1b5a3d1c841da5811ccd06c161203201e50`（1082 B）
- 回滚：删除第 10–23 行 dzdoc-mcp 那一整个 insert 条目，文件恢复 317 B 原状（原 prefix 逐字节保留，已验证）。
- 不改全局 profile；改的是项目插件 bundle（dsh-dzairground-ssh），卸载即消失。

### 3.3 验收：list_tools 8 个

JSON-RPC `initialize` → `notifications/initialized` → `tools/list`，返回 8 个工具：
`doc_search / doc_read / doc_gate / doc_index_refresh / doc_policy_check / doc_anchors_check / doc_numbers_check / doc_links_check`。

### 3.4 验收：doc_read 白名单（全部拒）

| 传入 path | 结果 |
|---|---|
| `../.env` | 拒：拒绝包含 .. 的路径 |
| `~/.ssh/id_rsa` | 拒：拒绝家目录路径（~） |
| `C:\\Users\\TestFox\\.env` | 拒：拒绝绝对路径 |
| `/etc/passwd`、`/` | 拒：拒绝绝对路径 |
| `20_资料索引库/../../.env`、`docs/../.env`、`a/../../b.md`、`..` | 拒：拒绝包含 .. 的路径〔失效：本行是负例测试输入清单，非真实文件引用〕 |
| `` | 拒：path 必须是非空字符串 |
| 目录（如 `20_资料索引库`） | 拒：不是普通文件 |

正常读取：`docs/README.md`、`docs/20_资料索引库/README.md`（201 行）、`docs/20_资料索引库/车端/01_架构与线程.md`（83 行）均 ok=True；`docs/` 前缀被容忍。

### 3.5 转发工具实测（经 MCP 调用）

| 工具 | 转发到 | 退出码 | 摘要 |
|---|---|---:|---|
| doc_search | `doc_search.py "TargetAimer" -k 3 --json` | 0 | total_documents=4, returned=3 |
| doc_index_refresh | `gen_index.py --ns doc-1024` | 0 | 重扫 26 条，保留外部 134 条 |
| doc_numbers_check | `check_numbers.py` | 0 | 52 条，默认只报不判红 |
| doc_links_check | `dzrobot_v2/tools/wf/check_links.py` | 0 | checked files: 176 broken: 0 |
| doc_policy_check | `check_doc_policy.py` | 1 | 8 条红线（全部是 P1c 编年 6 篇 + lead-1023 SIM-STUDIO-OPT 卡，他人工作流在跑，非本卡造成） |
| doc_anchors_check | `check_anchors.py` | 1 | 新增红 74（全部在他人未提交文档；本卡新增的 server 是 .py，不在 .md 锚点扫描范围） |

### 3.6 doc_gate 串起 §8.6 顺序

一次调用顺序跑 gen_index → check_doc_policy → check_anchors → check_numbers → dzrobot_v2 wf/check → check_links，返回 exit code + 每步摘要：

    gen_index=OK；check_doc_policy=RED；check_anchors=RED；check_numbers=OK；wf_check=OK；check_links=OK

综合 exit_code=1，但两处 RED 均为他人工作流既有漂移（见 3.5），非本卡新增。

### 3.7 危险脚本不注册

只暴露 8 个只读工具；调用 `salvage_repack` / 任何未注册名 → 返回 `未知工具；只暴露 8 个只读工具`（exit -32601），不执行。

### 3.8 插件侧健康检查

`node D:/User/Desktop/ZhiYuan/dsh_plugin/admin.mjs verify` → 3 通过 0 失败（dzairground-ssh 文件 116/116，link: 协议安全）。cordis.patch.yml 经 yaml.safe_load 解析通过，结构符合 `@deepseek-ai/dsh-mcp-client` 的 Config schema（serverName 匹配 `^[A-Za-z0-9_-]{1,32}$`，transport=stdio，args=数组，env=dict）。

### 3.9 复用闸

新文件 `docs/_工具/mcp_dzdoc_server.py` 已在 复用清单 §3 与 能力索引 登记（AGENTS §2）。server 只转发、不重写既有工具逻辑。

## 4 结论判定

**通过。** 验收两条均满足：
1. 起服务后 `tools/list` 可见 8 个工具（§3.3）；
2. `doc_read` 传 `../.env` 被拒且返回 error 而非内容（§3.4）。

另完成注册（方案 B）与两个登记（§3.2 / §3.9）。`doc_gate` 链已能按 §8.6 顺序跑通并返回 exit code + 摘要（§3.6）。门禁链中 check_doc_policy / check_anchors 的 RED 为他人工作流既有漂移，已在 §3.5 逐条指认，本卡未新增红。

## 10 文档变更

新增：docs/_工具/mcp_dzdoc_server.py
修改：D:/User/Desktop/ZhiYuan/dsh_plugin/dzairground-ssh/cordis.patch.yml（插 dzdoc-mcp）、dzrobot_v2/docs/复用清单.md、dzrobot_v2/docs/能力索引.md
归档：无
