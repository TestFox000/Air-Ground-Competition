# ============================================================
# fast_launch.ps1 —— 比赛日一键快发（预检 ≤10 秒）
# ============================================================
# 用途：比赛当天，车摆好位后一条命令发车。
# 预检内容（全部秒级）：
#   ① 栈冷态检查（dzbrain 未在跑，防双栈抢底盘）
#   ② 冻结清单对账：车端 6 个关键文件 sha256 与
#      .launch_manifest.sha256 逐条比对 —— 有人动过 config/src
#      却没重跑全量预检，这里立刻红灯，绝不带病发车
# 通过后：bash start_all.sh restart（约 40s 起栈，10s 后自动发车）
#
# ★ 铁律：.launch_manifest.sha256 只能在【全量预检】全部通过后
#   重新生成（部署对账 ✅ + colcon 编译 ✅ + 32 套门禁 ✅）。
#   改了任何 src/ 或 config/ 而未重跑全量预检 = 清单对账必红。
# ============================================================
param([string]$Ip = '192.168.137.241')

Write-Output '[1/2] 栈冷态 + 冻结清单对账 ...'
$check = ssh -o ConnectTimeout=8 duzhong@$Ip 'cd /home/duzhong/dzrobot && if pgrep -f dzbrain >/dev/null; then echo STACK_RUNNING; else echo STACK_COLD; fi; sha256sum -c .launch_manifest.sha256 2>&1 | tail -8'
Write-Output $check

if ($check -match 'STACK_RUNNING') {
    Write-Output '✋ 栈还在跑：先 start_all.sh stop 或确认安全后重试。'; exit 2
}
if ($check -match 'FAILED|没有那个文件|No such file') {
    Write-Output '✋ 冻结清单对账失败：config/src 与全量预检时的状态不一致，先重跑全量预检再生成清单。'; exit 3
}

Write-Output '[2/2] 对账通过，重启栈（10s 后自动发车）...'
ssh -o ConnectTimeout=8 duzhong@$Ip 'cd /home/duzhong/dzrobot && bash start_all.sh restart 2>&1 | tail -12'
