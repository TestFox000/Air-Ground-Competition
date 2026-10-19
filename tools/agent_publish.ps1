# ============================================================
# agent_publish.ps1 — 智能体收工脚本（AGENTS.md 契约的收工半边）
# 仓库：https://gitee.com/circle-of-joy/air_ground
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tools/agent_publish.ps1 `
#       -Agent trae-glm -Message "feat(xxx): 标题" `
#       -Files docs/成果.md tools/xxx.py
#   自动收集 docs/ 下改动+新增的 .md：用 -ChangedDocs 代替 -Files
#   移植他人成果：-Author "原作者 <邮箱>"（提交信息注明由谁移植）
#
# 做什么：校验任务分支 → 只暂存指定文件 → 署名提交 → 推送 →
#         ls-remote 验证推送真实生效（公约铁律 3，修复旧钩子 || true 盲区）→
#         FF 合入枢纽 members/yu-binghuan → 打印可发群的分享链接
# 退出码：0 = 全部成功；1 = 失败（看红色输出，修完重跑）
# ============================================================

param(
    [Parameter(Mandatory = $true)][string]$Agent,     # 智能体名/队员拼音（trae-glm/kimi/qwen/...）
    [Parameter(Mandatory = $true)][string]$Message,   # 提交标题
    [string]$Body = "",          # 提交正文（可选）
    [string[]]$Files = @(),      # 要提交的文件/目录（与 -ChangedDocs 二选一或并用）
    [switch]$ChangedDocs,        # 自动收集 docs/ 下改动与新增的 .md
    [string]$Author = "",        # 移植他人成果时署原作者："名 <邮箱>"
    [string]$Email = "",         # 覆盖默认署名邮箱（默认 <Agent>@air-ground.team）
    [switch]$NoHub               # 跳过自动合入枢纽
)

# 仓库根 = 本脚本所在 tools/ 的上一级（从任何目录调用都有效）
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not (Test-Path (Join-Path $repo ".git"))) {
    Write-Host "[失败] $repo 不是 git 仓库（本脚本需随仓库一起使用）" -ForegroundColor Red
    exit 1
}
Set-Location $repo

# 编码基线：git 输出恒为 UTF-8；GBK 控制台不设此行会把中文回显解码成乱码
#（提交本身不受影响，但控制台不可读，2026-10-19 与 agent_sync 同款修复）。
try {
    [Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
    $OutputEncoding = New-Object System.Text.UTF8Encoding($false)
} catch { }

function Fail($msg) {
    Write-Host ""
    Write-Host "[失败] $msg" -ForegroundColor Red
    Write-Host "本次未推送任何内容到远端，修复后重跑本脚本即可" -ForegroundColor Yellow
    exit 1
}

if ($Agent -notmatch '^[a-z0-9-]+$') { Fail "Agent 名不合法：$Agent（小写字母/数字/连字符）" }
if (-not $Email) { $Email = "$Agent@air-ground.team" }

Write-Host "==== 收工发布（$Agent → Gitee）===="

# 1) 分支校验：必须在自己的 agents/<Agent>/ 任务分支上
$cur = git rev-parse --abbrev-ref HEAD
if ($cur -notlike "agents/$Agent/*") {
    Fail "当前分支是 '$cur'，不是你的任务分支 agents/$Agent/*。先跑 tools/agent_sync.ps1 -Agent $Agent -Task <任务名> 切到/创建任务分支（改动不会被丢，会随切换保留）"
}
Write-Host "[1/7] 任务分支：$cur"

# 2) 收集并校验文件清单
$targets = @()
if ($ChangedDocs) {
    # core.quotepath=false 让中文路径原样输出（默认会被转义成 "docs/\346..."）
    $docChanges = git -c core.quotepath=false status --porcelain |
        Where-Object { $_ -match '(docs/\S*\.md)\s*$' } |
        ForEach-Object { ($_.Trim() -replace '^[AMD?R!]+\s+', '') -replace '^"', '' -replace '"$', '' }
    $targets += $docChanges
}
foreach ($f in $Files) { $targets += ($f -split ',') }   # 兼容 "a,b,c" 单串与重复 -Files 两种传法
$targets = $targets | ForEach-Object { ($_.Trim() -replace '\\', '/') } | Where-Object { $_ } | Select-Object -Unique
if (-not $targets) { Fail "没有要提交的文件：用 -Files 指定路径，或加 -ChangedDocs 收集 docs/ 下的 .md 改动" }
foreach ($t in $targets) {
    if (-not (Test-Path $t)) { Fail "文件不存在：$t" }
}
Write-Host "[2/7] 待提交 $($targets.Count) 个文件："
$targets | ForEach-Object { Write-Host "    $_" }

# 3) 暂存（只动指定文件，其他改动不碰）
git add -- $targets
if ($LASTEXITCODE -ne 0) { Fail "git add 失败" }
$staged = git diff --cached --name-only
$missing = $targets | Where-Object { $staged -notcontains $_ -and -not (git ls-files --error-unmatch $_ 2>$null) }
if ($missing) { Fail "这些文件没能进入暂存区：$($missing -join ', ')" }

# 4) 署名提交（谁的成果谁署名；移植他人成果用 -Author）
$commitArgs = @("-c", "user.name=$Agent", "-c", "user.email=$Email", "commit", "-m", $Message)
if ($Body) { $commitArgs += @("-m", $Body) }
if ($Author) { $commitArgs += @("--author", $Author) }
git @commitArgs
if ($LASTEXITCODE -ne 0) { Fail "git commit 失败（暂存区未变，修复后重跑）" }
$sha = git rev-parse --short HEAD
Write-Host "[3/7] 已提交：$sha  $(if ($Author) { "原作者署名：$Author" } else { "署名：$Agent <$Email>" })"

# 5) 推送
git push -u origin $cur
if ($LASTEXITCODE -ne 0) { Fail "git push 失败（网络/权限——若是权限问题找队长加协作者）" }

# 6) 推送验证（公约铁律 3：推完必验证，防止 || true 钩子掩盖失败）
Start-Sleep -Milliseconds 500
$remoteSha = git ls-remote origin "refs/heads/$cur"
if (-not $remoteSha -or ($remoteSha -split '\s+')[0] -notlike "$(git rev-parse HEAD)*") {
    Fail "推送验证失败：远端分支未指向本次提交 $sha——远端可能被他人推进过，先 fetch 合并再重推"
}
Write-Host "[4/7] 推送验证通过（远端 $(($remoteSha -split '\s+')[0].Substring(0,7)) == 本地 $sha）" -ForegroundColor Green
Write-Host "[5/7] 任务分支已上云：$cur"

# 7) FF 合入枢纽（agents 成果 → members/yu-binghuan，等余秉桓审核后 PR 进 main）
if (-not $NoHub) {
    git fetch origin members/yu-binghuan
    if ($LASTEXITCODE -eq 0) {
        $hubSha = git rev-parse origin/members/yu-binghuan
        $headSha = git rev-parse HEAD
        if ($hubSha -eq $headSha) {
            Write-Host "[6/7] 枢纽已与任务分支同步" -ForegroundColor Green
        } else {
            git merge-base --is-ancestor origin/members/yu-binghuan HEAD
            if ($LASTEXITCODE -eq 0) {
                git push origin "HEAD:members/yu-binghuan"
                if ($LASTEXITCODE -ne 0) {
                    Write-Host "[6/7] 枢纽合入失败（推送出错）——任务分支已成功，稍后手动合入" -ForegroundColor Yellow
                } else {
                    Write-Host "[6/7] 已 FF 合入枢纽 members/yu-binghuan（等余秉桓审核后 PR 进 main）" -ForegroundColor Green
                }
            } else {
                Write-Host "[6/7] 枢纽已被他人推进（非 FF）——请先 merge origin/members/yu-binghuan 解决后再推枢纽；任务分支不受影响" -ForegroundColor Yellow
            }
        }
    } else {
        Write-Host "[6/7] 跳过枢纽合入（fetch 失败）；任务分支已成功" -ForegroundColor Yellow
    }
} else {
    Write-Host "[6/7] 已按 -NoHub 跳过枢纽合入"
}

# 8) 分享链接（发群即用）
Write-Host ""
Write-Host "[7/7] 分享链接（复制发群）："
Write-Host "    分支：https://gitee.com/circle-of-joy/air_ground/tree/$cur"
Write-Host "    提交：https://gitee.com/circle-of-joy/air_ground/commit/$sha"
foreach ($t in $targets) { Write-Host "    文件：https://gitee.com/circle-of-joy/air_ground/blob/$cur/$t" }
Write-Host "    PR 入口（里程碑用，审核门禁见规约 §4）："
Write-Host "    https://gitee.com/circle-of-joy/air_ground/pull/new/circle-of-joy:members/yu-binghuan...circle-of-joy:main"
exit 0
