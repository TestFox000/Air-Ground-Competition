# ============================================================
# onboard_member.ps1 — 空地协同仓库一键接入脚本
# 仓库：https://gitee.com/circle-of-joy/air_ground
#
# 用法（任选其一）：
#   powershell -ExecutionPolicy Bypass -File onboard_member.ps1 -Name 陈驰
#   powershell -ExecutionPolicy Bypass -File onboard_member.ps1 -Name 陈驰 -TargetDir D:\air_ground
#   不带参数直接运行则进入交互模式（右键"使用 PowerShell 运行"也可以）
#
# 免克隆直取本文件（单文件下载后直接运行）：
#   https://gitee.com/circle-of-joy/air_ground/raw/main/tools/onboard_member.ps1
#   （main 合入前临时用 .../raw/members/yu-binghuan/tools/onboard_member.ps1）
#
# 依据：docs/团队协作与成果归属.md §5（规约 v2）。
# 本脚本只做四件事：克隆/复用仓库、配置本仓署名、切到你的 members/ 分支、
# 自检推送权限。不改动你机器上任何其他文件。
# ============================================================

param(
    [string]$Name = "",       # 中文姓名（名册八人之一，自动解析分支）
    [string]$Pinyin = "",     # 分支标识（名册外的新队员手动指定，如 zhang-san）
    [string]$Email = "",      # 署名邮箱，默认 <拼音>@air-ground.team
    [string]$TargetDir = "",  # 仓库落盘位置，默认当前目录下 air_ground
    [string]$RepoUrl = "https://gitee.com/circle-of-joy/air_ground.git",
    [switch]$NoPause          # 自动化调用时不暂停（人工使用不需要）
)

# ---- 名册（规约 v2 · 八位队员：姓名 -> members/ 分支）----
$Roster = [ordered]@{
    "余秉桓" = "yu-binghuan"
    "黄宗启" = "huang-zongqi"
    "万涛涛" = "wan-taotao"
    "贾凯旋" = "jia-kaixuan"
    "王江涛" = "wang-jiangtao"
    "程驰"   = "cheng-chi"
    "刘亚辉" = "liu-yahui"
    "胡锦宇" = "hu-jinyu"
}

function Fail($msg) {
    Write-Host ""
    Write-Host "[失败] $msg" -ForegroundColor Red
    if (-not $NoPause) { Read-Host "按回车退出" | Out-Null }
    exit 1
}

Write-Host "==== 空地协同仓库一键接入（Gitee: circle-of-joy/air_ground）===="

# 1) 补齐参数（必要时交互）
if (-not $Name) {
    Write-Host "队员名册："
    foreach ($k in $Roster.Keys) { Write-Host ("    {0}  ->  members/{1}" -f $k, $Roster[$k]) }
    $Name = Read-Host "输入你的中文姓名（上表之一；名册外队员直接回车改填拼音）"
}
if ($Roster.Contains($Name)) {
    $Pinyin = $Roster[$Name]
} elseif (-not $Pinyin) {
    $Pinyin = Read-Host "输入你的分支标识拼音（如 zhang-san）"
}
if ($Pinyin -notmatch '^[a-z][a-z0-9-]*$') {
    Fail "拼音分支标识格式不对：'$Pinyin'（只能小写字母/数字/连字符，且字母开头）"
}
if (-not $Email) { $Email = "$Pinyin@air-ground.team" }
if (-not $TargetDir) { $TargetDir = Join-Path (Get-Location) "air_ground" }
$Branch = "members/$Pinyin"

Write-Host "[1/6] 队员：$Name    分支：$Branch    署名：$Name <$Email>"

# 2) git 可用性
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Fail "未检测到 git。请先安装：https://git-scm.com/download/win （国内镜像：https://registry.npmmirror.com/binary.html?path=git-for-windows/ 装完重开终端再跑本脚本）"
}

# 3) 克隆或复用
if (Test-Path (Join-Path $TargetDir ".git")) {
    Write-Host "[2/6] 复用已有仓库：$TargetDir"
} else {
    Write-Host "[2/6] 首次克隆仓库到：$TargetDir （历史含部署包，几十 MB，请耐心）"
    git clone $RepoUrl $TargetDir
    if ($LASTEXITCODE -ne 0) { Fail "克隆失败——检查网络；或先用浏览器打开 $RepoUrl 确认能访问" }
}

# 4) 本仓署名（不动全局配置）
git -C $TargetDir config user.name $Name
git -C $TargetDir config user.email $Email
Write-Host "[3/6] 署名已配置（仅此仓库生效）：$Name <$Email>"

# 5) 同步远端并切到自己的分支
git -C $TargetDir fetch origin
if ($LASTEXITCODE -ne 0) { Fail "git fetch 失败——检查网络后重试" }
git -C $TargetDir rev-parse --verify --quiet "refs/remotes/origin/$Branch" 2>$null | Out-Null
if ($LASTEXITCODE -eq 0) {
    git -C $TargetDir checkout $Branch
    if ($LASTEXITCODE -ne 0) { Fail "切换到 $Branch 失败——本地若有未提交改动请先处理（git stash）再重跑" }
    Write-Host "[4/6] 已切换到你的长期分支：$Branch"
} else {
    # 兜底建分支：优先 canonical（含 13 类协议全量修复），其次 main（canonical 并入后）
    $Base = $null
    foreach ($cand in @("origin/merge/canonical-v2.1", "origin/main")) {
        git -C $TargetDir rev-parse --verify --quiet "refs/remotes/$cand" 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { $Base = $cand; break }
    }
    if (-not $Base) { Fail "远端无 $Branch，也找不到可用的基底分支——联系队长余秉桓" }
    git -C $TargetDir checkout -b $Branch $Base
    if ($LASTEXITCODE -ne 0) { Fail "从 $Base 创建 $Branch 失败——联系队长余秉桓" }
    Write-Host "[4/6] 远端暂无 $Branch，已基于 $Base 为你创建，首次 push 时自动上传"
}

# 6) 推送权限自检（--dry-run 只验证权限，不会上传任何内容）
git -C $TargetDir push --dry-run origin $Branch 2>$null
if ($LASTEXITCODE -eq 0) {
    Write-Host "[5/6] 推送权限自检通过" -ForegroundColor Green
} else {
    Write-Host "[5/6] 暂无推送权限：请先在 gitee.com 注册账号，把你的 Gitee 用户名发给队长余秉桓，待他把你加为仓库成员（开发者权限）后重跑本脚本" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "[6/6] 接入完成。日常四步（详见 docs/团队协作与成果归属.md §5 与 docs/多副本协作公约.md）："
Write-Host "    开工    git pull                       （公约铁律：开工前必拉最新）"
Write-Host "    干活    git add <文件>; git commit -m '说明'   （谁的成果谁署名，规约 §2）"
Write-Host "    收工    git push; git ls-remote origin | findstr $Pinyin   （铁律：推完必验证）"
Write-Host "    里程碑  Gitee 网页发起 PR：$Branch -> main   （审核门禁见规约 §4）"
Write-Host ""
Write-Host "    已有成果的队员请先看：docs/成员交接/吸收决策与待验证清单-20261018.md（V1~V17 待办认领）"
if (-not $NoPause) { Read-Host "按回车退出" | Out-Null }
