# 一键恢复训练脚本 — 下台电脑双击即可
#
# 用法：
#   1. 把原始视频放到本脚本同目录，命名为 video.mp4
#   2. 右键 -> 使用 PowerShell 运行（或双击 setup_and_train.bat）
#
# 自动完成：
#   - 检测/创建 Python 3.12 虚拟环境
#   - 安装依赖 + torch cu128
#   - 修复 5070 Ti 的 NVRTC DLL
#   - 从 video.mp4 抽帧 + pHash 去重
#   - 切分 train/val/test
#   - 启动训练守护进程（崩溃自动 resume）

$ErrorActionPreference = "Stop"
$workDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $workDir

$python = "python"
$venvPython = Join-Path $workDir ".venv\Scripts\python.exe"
$video = Join-Path $workDir "video.mp4"

function Log($msg) {
    $ts = Get-Date -Format "HH:mm:ss"
    Write-Host "[$ts] $msg" -ForegroundColor Cyan
}

function Step($name, $script) {
    Log ">>> $name"
    try {
        & $script
        if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne $null) {
            throw "exit code $LASTEXITCODE"
        }
        Log "    OK"
    } catch {
        Log "    FAIL: $_"
        Read-Host "按回车退出"
        exit 1
    }
}

Log "=== 一键恢复训练 ==="
Log "工作目录: $workDir"

# ---- 1. 检查视频 ----
Step "检查视频文件" {
    if (-not (Test-Path $video)) {
        throw "未找到 video.mp4，请把原始视频放到 $workDir\video.mp4"
    }
    Log "    视频大小: $([math]::Round((Get-Item $video).Length/1GB, 2)) GB"
}

# ---- 2. 检测 Python ----
Step "检测 Python 3.12" {
    $ver = & $python --version 2>&1
    Log "    $ver"
    if ($ver -notmatch "3\.12") {
        throw "需要 Python 3.12，当前 $ver。请从 python.org 安装"
    }
}

# ---- 3. 创建虚拟环境 ----
Step "创建/检测虚拟环境 .venv" {
    if (-not (Test-Path $venvPython)) {
        & $python -m venv .venv
        Log "    已创建 .venv"
    } else {
        Log "    .venv 已存在"
    }
}

# ---- 4. 安装依赖 ----
Step "安装 ultralytics + opencv" {
    & $venvPython -m pip install --upgrade pip -q 2>&1 | Out-Null
    & $venvPython -m pip install ultralytics opencv-python numpy pyyaml matplotlib `
        -i https://pypi.tuna.tsinghua.edu.cn/simple -q 2>&1 | Select-Object -Last 2
}

# ---- 5. 安装 torch cu128 ----
Step "安装 torch 2.7.0+cu128" {
    $installed = & $venvPython -c "import torch; print(torch.__version__)" 2>&1
    if ($installed -match "cu128") {
        Log "    已安装 $installed，跳过"
    } else {
        Log "    从官方源下载（较慢，请耐心等待）..."
        & $venvPython -m pip install torch torchvision `
            --index-url https://download.pytorch.org/whl/cu128 2>&1 | Select-Object -Last 3
    }
}

# ---- 6. 修复 NVRTC DLL（5070 Ti 必做）----
Step "修复 NVRTC DLL" {
    $torchLib = Join-Path $workDir ".venv\Lib\site-packages\torch\lib"
    $nvrtc = Join-Path $torchLib "nvrtc64_120_0.dll"
    $alt = Join-Path $torchLib "nvrtc64_120_0.alt.dll"
    if ((Test-Path $alt) -and (Test-Path $nvrtc)) {
        $bak = "$nvrtc.bak"
        if (-not (Test-Path $bak)) {
            Copy-Item $nvrtc $bak -Force
        }
        Copy-Item $alt $nvrtc -Force
        Log "    已用 .alt.dll 替换 nvrtc64_120_0.dll"
    } else {
        Log "    跳过（未找到 alt.dll）"
    }
    # 验证 torch 能加载
    & $venvPython -c "import torch; assert torch.cuda.is_available(), 'CUDA 不可用'; print('CUDA OK')"
}

# ---- 7. 抽帧 ----
Step "抽帧 + pHash 去重" {
    if ((Get-ChildItem "capture\aerial_field\images" -ErrorAction SilentlyContinue).Count -gt 0) {
        Log "    图片已存在，跳过抽帧"
    } else {
        & $venvPython tools\ml\extract_frames.py --video $video `
            --out capture\aerial_field\images --fps 2
    }
}

# ---- 8. 切分数据集 ----
Step "切分 train/val/test" {
    if (Test-Path "capture\datasets\aerial_field\data.yaml") {
        Log "    data.yaml 已存在，跳过切分"
    } else {
        & $venvPython tools\ml\split_capture.py `
            --images capture\aerial_field\images `
            --labels capture\aerial_field\labels `
            --out capture\datasets\aerial_field
    }
}

# ---- 9. 启动训练守护 ----
Log ""
Log "=== 环境就绪，启动训练 ==="
Log "训练将在后台运行，崩溃自动恢复"
Log "日志: train_guard.log"
Log "按 Ctrl+C 停止守护脚本"
Log ""

& $venvPython tools\ml\guard_train.py
