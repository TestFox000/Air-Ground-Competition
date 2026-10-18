# ============================================================
# agent_sync.ps1 — 智能体开工脚本（AGENTS.md 契约的开工半边）
# 仓库：https://gitee.com/circle-of-joy/air_ground
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tools/agent_sync.ps1
#   powershell -ExecutionPolicy Bypass -File tools/agent_sync.ps1 -Agent trae-glm -Task <任务名>
#
# 做什么：fetch 全分支 → 主干/枢纽/canonical/members/agents 态势表 →
#         基底过期判定 → （可选）创建/切换任务分支 agents/<智能体>/<任务>
# 退出码：0 = 基底新鲜，可以开工
#         2 = 基底过期，必须先同步再动手（多副本协作公约铁律 2）
#         3 = 当前在 main/master/枢纽上，禁止直接开发，先切任务分支
#         1 = 错误（fetch 失败/不在仓库内等，先修环境）
# ============================================================

param(
    [string]$Agent = "",   # 智能体名（trae-glm/kimi/qwen/队员拼音），配合 -Task 开分支
    [string]$Task = ""     # 任务名：创建/切换 agents/<Agent>/<Task>
)

# 仓库根 = 本脚本所在 tools/ 的上一级（从任何目录调用都有效）
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not (Test-Path (Join-Path $repo ".git"))) {
    Write-Host "[失败] $repo 不是 git 仓库（本脚本需随仓库一起使用）" -ForegroundColor Red
    exit 1
}
Set-Location $repo

function Fail($msg) { Write-Host "[失败] $msg" -ForegroundColor Red; exit 1 }

Write-Host "==== 开工同步（Gitee: circle-of-joy/air_ground）===="

# 1) fetch 全分支
git fetch origin
if ($LASTEXITCODE -ne 0) {
    Write-Host "[提示] fetch 失败——网络/凭据问题。仍输出本地已知态势，但远端信息可能陈旧" -ForegroundColor Yellow
}

# 2) 当前分支与上下游
$cur = git rev-parse --abbrev-ref HEAD
Write-Host ""
Write-Host "[1/4] 当前分支：$cur"

# 3) 远端关键分支态势表
Write-Host ""
Write-Host "[2/4] 远端分支态势（主干/枢纽/canonical/成员/智能体）："
$refs = git for-each-ref --format="%(refname:short)|%(objectname:short)|%(contents:subject)" refs/remotes/origin/ |
    Where-Object { $_ -match '^origin/(main$|merge/|members/|agents/)' }
foreach ($r in $refs) {
    $p = $r -split '\|', 3
    $subject = if ($p[2].Length -gt 46) { $p[2].Substring(0, 46) + "…" } else { $p[2] }
    Write-Host ("    {0,-42} {1}  {2}" -f $p[0].Replace("origin/", ""), $p[1], $subject)
}

# 4) 枢纽近期动态（谁在推进什么）
Write-Host ""
Write-Host "[3/4] 枢纽 members/yu-binghuan 最近 5 条（他人进展速览）："
git log --oneline -5 origin/members/yu-binghuan 2>$null | ForEach-Object { Write-Host "    $_" }

# 5) 基底判定
Write-Host ""
Write-Host "[4/4] 基底判定："
$exit = 0
if ($cur -eq "main" -or $cur -eq "master" -or $cur -eq "members/yu-binghuan") {
    Write-Host "    [禁止] 你正坐在 $cur 上——主干/枢纽不允许直接开发" -ForegroundColor Red
    Write-Host "    应先建任务分支：agent_sync.ps1 -Agent <名> -Task <任务>"
    $exit = 3
} else {
    $behind = git rev-list --count "HEAD..origin/members/yu-binghuan" 2>$null
    if ($LASTEXITCODE -ne 0) { $behind = "?" }
    $aheadOfHub = git rev-list --count "origin/members/yu-binghuan..HEAD" 2>$null
    if ($behind -is [int] -and $behind -gt 0) {
        Write-Host "    [过期] 枢纽比你的基底新 $behind 个提交（你本地另有 $aheadOfHub 个未推）——先 git merge origin/members/yu-binghuan（或 rebase）再动手" -ForegroundColor Yellow
        $exit = 2
    } else {
        Write-Host "    [新鲜] 你的基底已含枢纽全部提交（本地未推 $aheadOfHub 个）——可以开工" -ForegroundColor Green
    }
    $vsMain = git rev-list --count "HEAD..origin/main" 2>$null
    Write-Host "    参考：你与 origin/main 的差集 = $vsMain 个提交（main 待 PR 合入时属正常）"
}

# 6) 可选：创建/切换任务分支
if ($Agent -and $Task) {
    if ($Agent -notmatch '^[a-z0-9-]+$' -or $Task -notmatch '^[A-Za-z0-9_-]+$') {
        Fail "Agent/Task 命名不合法（Agent 小写字母数字连字符；Task 字母数字下划线连字符）"
    }
    $tb = "agents/$Agent/$Task"
    git rev-parse --verify --quiet "refs/heads/$tb" 2>$null | Out-Null
    if ($LASTEXITCODE -eq 0) {
        git checkout $tb
        if ($LASTEXITCODE -ne 0) { Fail "切换 $tb 失败（先处理未提交改动：git stash）" }
        Write-Host ""
        Write-Host "[分支] 已切换到既有任务分支：$tb"
    } else {
        $base = $null
        foreach ($cand in @("origin/members/yu-binghuan", "origin/merge/canonical-v2.1", "origin/main")) {
            git rev-parse --verify --quiet "refs/remotes/$cand" 2>$null | Out-Null
            if ($LASTEXITCODE -eq 0) { $base = $cand; break }
        }
        if (-not $base) { Fail "找不到可用基底分支" }
        git checkout -b $tb $base
        if ($LASTEXITCODE -ne 0) { Fail "从 $base 创建 $tb 失败" }
        Write-Host ""
        Write-Host "[分支] 已基于 $base 创建任务分支：$tb"
    }
}

Write-Host ""
Write-Host "下一步：干活 → 收工执行 tools/agent_publish.ps1（署名提交+推送验证+合枢纽+分享链接）"
exit $exit
