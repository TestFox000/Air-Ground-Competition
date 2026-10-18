#!/usr/bin/env bash
set -euo pipefail
BK=/home/duzhong/dzuav/backup_20261017_2150
mkdir -p "$BK"
D=/home/duzhong/dzuav_deploy/air_ground/tools
S=/home/duzhong/dzpx4/src/uav_camera_code/tools
cp -a "$D/orin_bringup.sh" "$BK/orin_bringup.deploy.sh"
cp -a "$D/autostart_agup.sh" "$BK/autostart_agup.sh"
cp -a "$S/orin_bringup.sh" "$BK/orin_bringup.source.sh"
for f in "$D/orin_bringup.sh" "$S/orin_bringup.sh" "$D/autostart_agup.sh"; do
  sed -i 's#/dev/ttyACM0#/dev/ttyACM1#g' "$f"
done
for f in "$D/orin_bringup.sh" "$S/orin_bringup.sh"; do
  sed -i 's#orin_agent_watchdog.py"#orin_agent_watchdog.py --device /dev/ttyACM1"#' "$f"
  sed -i 's#px4_odom_convert.launch.py"#px4_odom_convert.launch.py mode:=odom"#' "$f"
done
sed -i 's#orin_agent_watchdog.py \\#orin_agent_watchdog.py --device /dev/ttyACM1 \\#' "$D/autostart_agup.sh"
echo ===AFTER====
grep -n ttyACM "$D/orin_bringup.sh" "$S/orin_bringup.sh" "$D/autostart_agup.sh"
echo ===WDOG====
grep -n orin_agent_watchdog "$D/orin_bringup.sh" "$S/orin_bringup.sh" "$D/autostart_agup.sh"
echo ===ODOM====
grep -n px4_odom_convert "$D/orin_bringup.sh" "$S/orin_bringup.sh"
echo ===SYNTAX====
bash -n "$D/orin_bringup.sh" && echo bringup_deploy_syntax_ok
bash -n "$S/orin_bringup.sh" && echo bringup_source_syntax_ok
bash -n "$D/autostart_agup.sh" && echo autostart_syntax_ok
