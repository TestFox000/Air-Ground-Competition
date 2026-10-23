#!/usr/bin/env bash
# ==============================================================================
#  deploy_supervisor.sh —— 把 dzlink_supervisor 及其配套文件部署到无人机
#
#  为什么要有这个脚本（而不是手敲 scp）：
#    本仓库有**两次**踩坑记录：PowerShell 5.1 给 ssh.exe 传参会把双引号吃掉，
#    于是"远端命令行为莫名不对"。可靠做法就是把要在远端跑的东西写成脚本、scp 过去。
#    本脚本把「上传 + 两端 md5 逐字节核对 + 远端语法自检 + 真机 ROS 冒烟」串成一条命令。
#
#  用法（在本地 Windows PowerShell 里跑；需要 ssh/scp 可用）
#    bash tools/link/deploy_supervisor.sh                 # 全流程（含 --with-smoke）
#    bash tools/link/deploy_supervisor.sh --no-smoke       # 只上传+核对，不跑 ROS 冒烟
#    bash tools/link/deploy_supervisor.sh --dry-run        # 只打印将要执行的命令
#    bash tools/link/deploy_supervisor.sh --host 10.29.122.104 --user duzhong
#    bash tools/link/deploy_supervisor.sh --domain 118 --no-domain-override
#
#  默认目标：duzhong@10.29.122.104:~/dzlink_tools/（无人机，SSH 免密）
#  铁律：**只上传到 ~/dzlink_tools/**（新建目录），不碰无人机上任何既有文件/配置；
#        不装包；只在 /tmp 和 ~/dzlink_tools 里写东西。
# ==============================================================================
set -u

# ---------------------------------------------------------------- 默认参数 ----
HOST="10.29.122.104"
USER_NAME="duzhong"
REMOTE_DIR="\$HOME/dzlink_tools"          # 用字面 $HOME，让远端 shell 展开
REMOTE_DIR_PLAIN="~/dzlink_tools"
DOMAIN="118"
WITH_SMOKE=1
DRY_RUN=0
SSH_OPTS="-4 -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=12"

while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="$2"; shift 2 ;;
    --user) USER_NAME="$2"; shift 2 ;;
    --remote-dir) REMOTE_DIR="$2"; REMOTE_DIR_PLAIN="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --no-smoke) WITH_SMOKE=0; shift ;;
    --with-smoke) WITH_SMOKE=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --no-domain-override) DOMAIN=""; shift ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
done

LOCAL_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET="${USER_NAME}@${HOST}"

# 要传的文件：本地相对路径 → 远端文件名
FILES="
dzlink_supervisor.py:dzlink_supervisor.py
smoke_ros.sh:smoke_ros.sh
README.md:README_supervisor.md
tests/test_dzlink_supervisor.py:test_dzlink_supervisor.py
"

step() { printf '\n=== %s ===\n' "$*"; }
say()  { printf '  %s\n' "$*"; }
run() {
  if [ "$DRY_RUN" = "1" ]; then printf '  [dry-run] %s\n' "$*"; return 0; fi
  "$@"
}

md5_of() {
  if command -v md5sum >/dev/null 2>&1; then md5sum "$1" | awk '{print $1}'
  elif command -v md5 >/dev/null 2>&1;    then md5 -q "$1"
  else python -c "import hashlib,sys;print(hashlib.md5(open(sys.argv[1],'rb').read()).hexdigest())" "$1"
  fi
}

step "0. 目标与前置检查"
say "本地源目录 : $LOCAL_DIR"
say "远端        : ${TARGET}:${REMOTE_DIR_PLAIN}/"
say "ROS 域号    : ${DOMAIN:-(不动，用远端 dzlink.env)}"
say "冒烟测试    : $([ "$WITH_SMOKE" = "1" ] && echo 开 || echo 关)"
for f in $FILES; do
  local_f="${f%%:*}"
  if [ ! -f "$LOCAL_DIR/$local_f" ]; then
    echo "ERROR: 缺文件 $LOCAL_DIR/$local_f" >&2; exit 1
  fi
done
say "待传文件：$(echo "$FILES" | tr '\n' ' ')"

step "1. SSH 连通性"
run ssh $SSH_OPTS "$TARGET" 'echo REMOTE_OK; hostname; whoami' || {
  echo "ERROR: SSH 不通（$TARGET）。检查无人机是否在线 / 免密是否还在。" >&2; exit 1; }

step "2. 建目录（只新建，不覆盖既有文件）"
run ssh $SSH_OPTS "$TARGET" "mkdir -p $REMOTE_DIR_PLAIN/perception/out $REMOTE_DIR_PLAIN/logs $REMOTE_DIR_PLAIN/routes && ls -ld $REMOTE_DIR_PLAIN $REMOTE_DIR_PLAIN/perception/out $REMOTE_DIR_PLAIN/logs $REMOTE_DIR_PLAIN/routes"

step "3. scp 上传"
for f in $FILES; do
  local_f="${f%%:*}"; remote_f="${f##*:}"
  say "→ $local_f  ⇒  $REMOTE_DIR_PLAIN/$remote_f"
  run scp -4 -q "$LOCAL_DIR/$local_f" "$TARGET:$REMOTE_DIR_PLAIN/$remote_f" || {
     echo "ERROR: scp 失败：$local_f" >&2; exit 1; }
done
# deploy 脚本自己也留一份在远端（方便下次在无人机侧直接跑/查）
run scp -4 -q "$LOCAL_DIR/deploy_supervisor.sh" "$TARGET:$REMOTE_DIR_PLAIN/deploy_supervisor.sh"

step "4. 逐字节核对（本地 md5 vs 远端 md5）"
FAIL=0
for f in $FILES; do
  local_f="${f%%:*}"; remote_f="${f##*:}"
  lm="$(md5_of "$LOCAL_DIR/$local_f")"
  rm_="$(ssh $SSH_OPTS "$TARGET" "md5sum $REMOTE_DIR_PLAIN/$remote_f 2>/dev/null | awk '{print \$1}'")"
  if [ "$lm" = "$rm_" ]; then
    say "OK   $remote_f  $lm"
  else
    say "DIFF $remote_f  本地=$lm  远端=$rm_"
    FAIL=1
  fi
done
[ "$FAIL" = "1" ] && { echo "ERROR: md5 不一致，停止（传输被改写过？）" >&2; exit 1; }

step "5. 远端语法与自检（不需要 ROS）"
run ssh $SSH_OPTS "$TARGET" "sed -i 's/\r\$//' $REMOTE_DIR_PLAIN/dzlink_supervisor.py $REMOTE_DIR_PLAIN/smoke_ros.sh 2>/dev/null; chmod +x $REMOTE_DIR_PLAIN/smoke_ros.sh $REMOTE_DIR_PLAIN/dzlink_supervisor.py; python3 -m py_compile $REMOTE_DIR_PLAIN/dzlink_supervisor.py && echo PY_COMPILE_OK"
run ssh $SSH_OPTS "$TARGET" "python3 $REMOTE_DIR_PLAIN/dzlink_supervisor.py selftest 2>/dev/null | tail -3"
run ssh $SSH_OPTS "$TARGET" "python3 $REMOTE_DIR_PLAIN/dzlink_supervisor.py contract 2>/dev/null | head -6"

step "6. 本机单测也在远端跑一遍（纯逻辑，不需要 ROS）"
run ssh $SSH_OPTS "$TARGET" "cd $REMOTE_DIR_PLAIN && python3 -m unittest test_dzlink_supervisor 2>&1 | tail -4"

if [ "$WITH_SMOKE" = "1" ]; then
  step "7. 真机 ROS 冒烟（域 $DOMAIN；用临时订阅者代替小车，不动小车）"
  DOMAIN_ARG=""
  [ -n "$DOMAIN" ] && DOMAIN_ARG="--domain $DOMAIN"
  run ssh $SSH_OPTS "$TARGET" "chmod +x $REMOTE_DIR_PLAIN/smoke_ros.sh; DZ_REMOTE_DIR=$REMOTE_DIR_PLAIN DOMAIN=$DOMAIN bash $REMOTE_DIR_PLAIN/smoke_ros.sh"
  SMOKE_RC=$?
  say "smoke 退出码=$SMOKE_RC（0=全过 / 3=发得出去但没人订阅 / 2=没有 ROS 环境）"
else
  step "7. 跳过冒烟（--no-smoke）"
fi

step "8. 远端现状"
run ssh $SSH_OPTS "$TARGET" "echo '--- ${REMOTE_DIR_PLAIN} ---'; ls -l $REMOTE_DIR_PLAIN; echo '--- 日志尾部 ---'; ls -l $REMOTE_DIR_PLAIN/logs 2>/dev/null | tail -3"

step "完成"
cat <<EOF
下一次怎么跑（无人机上，先 source 两句）：
  source /opt/ros/humble/setup.bash
  source \$HOME/dzlink_tools/dzlink.env          # 域号唯一真相源 = 118

  # ① 只发一次情报（验证报文，不常驻）
  python3 ~/dzlink_tools/dzlink_supervisor.py once --bridge lower=K9 --bridge upper=K4

  # ② 常驻（心跳 + 感知文件转发 + 车回传闭环 + 命令文件）
  python3 ~/dzlink_tools/dzlink_supervisor.py run \\
      --perception-glob '~/dzlink_tools/perception/out/*.jsonl' \\
      --cmd-file ~/dzlink_tools/cmd.jsonl \\
      --log-file ~/dzlink_tools/logs/supervisor.jsonl

  # ③ 下发路线 / 复位
  python3 ~/dzlink_tools/dzlink_supervisor.py route "N10 M10 L10 L9 K9 J9"
  python3 ~/dzlink_tools/dzlink_supervisor.py reset N10,y

  # ④ 追加命令（常驻进程会即时读走）
  echo '{"type":"bridge","half":"lower","block":"K9"}' >> ~/dzlink_tools/cmd.jsonl
  echo '{"type":"route","data":"N10 M10 L10"}'        >> ~/dzlink_tools/cmd.jsonl
EOF
