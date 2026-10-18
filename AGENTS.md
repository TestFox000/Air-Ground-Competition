# AGENTS.md — 本仓智能体作业契约（任何 AI 智能体开工自动生效）

> 适用于所有智能体（trae-glm / kimi / qwen / 各队员自有智能体；Trae、Claude Code、
> CodeBuddy、Cursor 等工具的会话都会自动读取本文件）。人类队员同样适用。
> 依据：docs/团队协作与成果归属.md（规约 v2）· docs/多副本协作公约.md · docs/开工指引.md

## 开工（写任何代码/文档之前，必做）

```powershell
powershell -ExecutionPolicy Bypass -File tools/agent_sync.ps1
# 需要开任务分支时：
powershell -ExecutionPolicy Bypass -File tools/agent_sync.ps1 -Agent trae-glm -Task <任务名>
```

自动完成：fetch 全分支 → 主干/枢纽/canonical/members/agents 态势表 → **基底过期判定**。
- 退出码 0 = 基底新鲜；**2 = 基底过期（必须先同步再动手，公约铁律 2）**；
  3 = 你正坐在 main/master/枢纽上（禁止直接开发）；1 = fetch 失败（先修网络/凭据）。

## 分支与署名

- 任务分支：`agents/<智能体>/<任务>`，基底 = `origin/members/yu-binghuan`（枢纽），
  枢纽不可用时按 canonical → main 链式回退
- 谁的成果谁署名（规约 §2）；移植他人成果时提交用 `--author` 署原作者并在提交信息
  注明由谁移植、来自哪里

## 收工（产出成果/文档之后，必做）

```powershell
powershell -ExecutionPolicy Bypass -File tools/agent_publish.ps1 `
  -Agent <智能体名> -Message "<提交标题>" `
  -Files "docs/成果.md,tools/脚本.py"   # 多文件逗号串（一个参数）；或 -ChangedDocs 自动收集 docs/ 下改动与新增的 .md
# 移植他人成果加：-Author "原作者 <邮箱>"
```

自动完成：校验任务分支 → 只暂存指定文件 → 署名提交 → 推送 → **ls-remote 验证推送
真实生效**（修复旧 post-commit 钩子 `|| true` 掩盖推送失败的问题，公约铁律 3）→
FF 合入枢纽 `members/yu-binghuan` → **打印可直接发群的分享链接**（分支/文件/提交/PR 入口）。
`.md` 交付文档推完即把脚本打印的 blob 链接发群里，队友点开就是最新版。

## PR 门禁（合入 main 前）

CI 固定序+随机序双轮绿；测试基线只升不降；协议类目不回退（13 类为真源）；
C++ 改动必须机载编译验证（orin_build_verify.sh）后再上机。

## 禁止

- 在 main / master / members/yu-binghuan 上直接开发
- 跳过 agent_sync 直接改代码（旧基底开工 = 制造未来的冲突）
- 推完不验证就报完成
- 向 main 强推；改动 preserve/* 档案分支的历史（只增不改）
