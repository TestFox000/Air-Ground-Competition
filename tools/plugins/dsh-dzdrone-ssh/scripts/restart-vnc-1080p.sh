#!/bin/bash
# Restart the drone's VNC at 1920x1080 (requested), opening 5901 as asked.
#
# Ordering matters: the geometry is written into BOTH the systemd unit and
# ~/.vnc/config BEFORE the restart, otherwise whichever one is not updated would
# quietly put 720p back. The restart goes through systemd so the unit's own
# ExecStartPre/ExecStart sequence runs — equivalent to `vncserver -kill` + start,
# but the service stays supervised.
#
# Known cost (accepted): every X client on :1 dies with the server, including
# det_node's vision window.
PW=duzhong

echo "=== 1. geometry -> 1920x1080 in unit + config ==="
echo "$PW" | sudo -S -p '' sed -i 's/-geometry 1280x720/-geometry 1920x1080/' /etc/systemd/system/vncserver@:1.service
sed -i 's/^geometry=.*/geometry=1920x1080/' "$HOME/.vnc/config"
echo "unit:    $(grep ExecStart= /etc/systemd/system/vncserver@:1.service)"
echo "config:  $(grep ^geometry "$HOME/.vnc/config")"
echo "localhost: $(grep ^localhost "$HOME/.vnc/config") (kept — already no)"

echo
echo "=== 2. ufw allow 5901/tcp (rule added; ufw itself stays inactive) ==="
echo "$PW" | sudo -S -p '' ufw allow 5901/tcp

echo
echo "=== 3. restart vncserver@:1 ==="
echo "$PW" | sudo -S -p '' systemctl daemon-reload
echo "$PW" | sudo -S -p '' systemctl restart vncserver@:1
sleep 10

echo
echo "=== 4. after ==="
echo "service: $(systemctl is-active vncserver@:1)"
XAUTHORITY=$HOME/.Xauthority DISPLAY=:1 xdpyinfo 2>/dev/null | grep dimensions || echo "(xdpyinfo: display not up)"
ss -lnt | grep 5901 || echo "(5901 NOT listening)"
ps -eo args | grep '[X]tigervnc' | head -1
echo "--- det_node after restart ---"
if pgrep -af 'det_node --ros-args' >/dev/null; then
  pgrep -af 'det_node --ros-args' | head -2
else
  echo "det_node NOT running (killed with the old X server)"
fi
echo "--- desktop ---"
pgrep -af 'pcmanfm --desktop' >/dev/null && echo "LXDE desktop up" || echo "(desktop not up yet)"
