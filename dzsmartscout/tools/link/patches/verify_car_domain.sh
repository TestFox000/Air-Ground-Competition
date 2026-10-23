#!/bin/bash
# ==============================================================================
#  车侧域号方案 A 完整版验证（只读，不动任何节点）
#
#  覆盖：
#    · 唯一真相源 dzros.env
#    · 5 个 ROS 入口的语法 + 是否接上 dzros.env + 接入位置是否早于 source ROS
#    · 三种起法的域号一致性（空环境 / 显式覆盖 / 当前环境）
#    · 文件缺失时的回退、以及"改一个文件全车跟随"
#  安全：所有用例都截断到首次 `stop_all` 调用之前，且自带 spawn/stop_all 计数检查。
#  用法：bash verify_car_domain.sh
# ==============================================================================
set -u
cd "$HOME/dzrobot" || { echo "❌ 进不去 ~/dzrobot"; exit 1; }

echo "=== 0) 唯一真相源 dzros.env ==="
if [ -f dzros.env ]; then
  ls -l dzros.env
  md5sum dzros.env
  grep -n 'DZ_ROS_DOMAIN\|export' dzros.env
else
  echo "❌ dzros.env 不存在"
fi

echo
echo "=== 1) 5 个 ROS 入口 ==="
printf '%-26s %-8s %-10s %-12s %s\n' 文件 '语法' 'dzros行' 'sourceROS行' 判定
for f in start_all.sh monitor_aim.sh tools/healthcheck.sh tools/restart_node.sh tools/test_voice.sh; do
  if [ ! -f "$f" ]; then printf '%-26s %s\n' "$f" "缺失"; continue; fi
  if bash -n "$f" 2>/dev/null; then syn=OK; else syn=FAIL; fi
  dz=$(grep -n 'dzros\.env' "$f" | head -1 | cut -d: -f1)
  ro=$(grep -n '/opt/ros/humble/setup.bash' "$f" | head -1 | cut -d: -f1)
  if [ -z "$dz" ]; then verdict="未接入 ❌"; elif [ -z "$ro" ]; then verdict="无ROS调用"; elif [ "$dz" -lt "$ro" ]; then verdict="早于 ✅"; else verdict="晚于 ❌"; fi
  printf '%-26s %-8s %-10s %-12s %s\n' "$f" "$syn" "${dz:-—}" "${ro:-—}" "$verdict"
done

# ---- 截断点：首次 `stop_all` 调用（行首 stop_all + 空白；不含 `stop_all() {`）----
CUT=$(grep -nE '^stop_all[[:space:]]' start_all.sh | head -1 | cut -d: -f1)
[ -n "$CUT" ] || { echo "❌ 找不到 stop_all 调用行"; exit 1; }
BAN=$(grep -n '链路自检' start_all.sh | head -1 | cut -d: -f1)
[ -n "$BAN" ] || { echo "❌ 找不到链路自检横幅"; exit 1; }
echo
echo "截断点：第 $CUT 行（保留 1..$((CUT - 1))）   横幅：第 $BAN 行"

build_prefix() {   # $1=输出文件  $2=yes 则把 WS 重定向到 /tmp/fakedz
  sed -n "1,$((CUT - 1))p" start_all.sh > "$1"
  sed -n "${BAN},$((BAN + 4))p" start_all.sh >> "$1"
  if [ "$2" = yes ]; then
    sed -i -e 's|^WS=/home/duzhong/dzrobot|WS=/tmp/fakedz|' \
           -e 's|^source "\$WS/install/setup.bash"|:|' "$1"
  fi
  cat >> "$1" <<'EOF'

echo "  >>> 实际生效: ROS_DOMAIN_ID=[$ROS_DOMAIN_ID]  ROS_LOCALHOST_ONLY=[$ROS_LOCALHOST_ONLY]"
EOF
}

build_prefix /tmp/dzprefix.sh no
build_prefix /tmp/dzprefix_fake.sh yes

BAD=$(grep -cE '^spawn |^stop_all[[:space:]]' /tmp/dzprefix.sh)
echo "安全检查：前缀里的 spawn/stop_all 调用数 = $BAD （必须 0）"
[ "$BAD" = "0" ] || { echo "❌ 前缀不安全，终止"; exit 1; }

echo
echo "--- A) 空环境（模拟 systemd / cron / ssh 一行命令起栈）★ 关键用例 ---"
env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin bash /tmp/dzprefix.sh 2>&1 | tail -3
echo
echo "--- B) 外部显式覆盖 ROS_DOMAIN_ID=7（应保持 7 且横幅报警）---"
env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin ROS_DOMAIN_ID=7 bash /tmp/dzprefix.sh 2>&1 | tail -4
echo
echo "--- C) 当前环境（模拟交互式终端起栈）---"
bash /tmp/dzprefix.sh 2>&1 | tail -3

echo
echo "--- D) dzros.env【缺失】时应回退到内置 118 ---"
rm -rf /tmp/fakedz && mkdir -p /tmp/fakedz
env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin bash /tmp/dzprefix_fake.sh 2>&1 | tail -3
echo
echo "--- E) dzros.env 写 200 时应全车跟随（证明"只改一个文件"）---"
printf 'DZ_ROS_DOMAIN=200\nexport ROS_DOMAIN_ID=200\n' > /tmp/fakedz/dzros.env
env -i HOME="$HOME" PATH=/usr/bin:/bin:/usr/local/bin bash /tmp/dzprefix_fake.sh 2>&1 | tail -3

echo
echo "--- F) 4 个辅助脚本插入的那 2 行（逐字照抄）直接跑一遍 ---"
env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c 'if [ -f "$HOME/dzrobot/dzros.env" ]; then . "$HOME/dzrobot/dzros.env"; fi; echo "  >>> ROS_DOMAIN_ID=[$ROS_DOMAIN_ID]"'

echo
echo "判定：A/C/D 必须 118；B=7、E=200 属正常（显式覆盖 / 单一真相源生效）。"
rm -rf /tmp/fakedz
