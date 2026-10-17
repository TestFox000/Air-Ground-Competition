#!/bin/bash
# Read-only recon of the car: domain origin, autostart, start_all.sh shape.
echo "=== A. ROS_DOMAIN_ID 的所有引用（排除 install/build）==="
grep -rn "ROS_DOMAIN_ID" "$HOME/dzrobot" "$HOME/dzrobot_v2" "$HOME/.bashrc" "$HOME/.profile" 2>/dev/null \
  | grep -v -E "/(install|build|log)/" | head -20
echo "(以上为全部命中)"
echo
echo "=== B. start_all.sh 前 45 行 ==="
sed -n '1,45p' "$HOME/dzrobot/start_all.sh"
echo
echo "=== C. start_all.sh 的步骤/跳过逻辑 ==="
grep -n -E "det_node|SKIP|skip|STEP|step|[0-9]_[a-z]" "$HOME/dzrobot/start_all.sh" | head -30
echo
echo "=== D. 有没有开机自启（赛场翻车点）==="
systemctl list-unit-files 2>/dev/null | grep -i -E "dz|robot" | head
crontab -l 2>/dev/null | head
ls "$HOME/.config/autostart/" 2>/dev/null | head
echo "(全部为空 = 没有自启)"
echo
echo "=== E. auto_start / mission_reset 现值 ==="
grep -n -E "auto_start|mission_reset|auto_start_delay" "$HOME/dzrobot/config/robot.yaml" | head
echo
echo "=== F. 最近一次启动的日志时间 ==="
ls -lt "$HOME/dzrobot/log/" 2>/dev/null | head -8
