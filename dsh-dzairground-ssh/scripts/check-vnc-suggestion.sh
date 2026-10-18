#!/bin/bash
# Evaluate the suggested vncserver/ufw commands against the car's actual state.
# Usage: scp this, then ssh ... "bash /tmp/x.sh"

echo "=== 1. is the VNC server localhost-only? (suggested -localhost no) ==="
ps -eo args | grep '[X]tigervnc' | head -1
grep -n localhost ~/.vnc/config 2>/dev/null

echo
echo "=== 2. ufw state (suggested ufw allow 5901/tcp) ==="
systemctl is-active ufw 2>/dev/null || echo "ufw unit not found"
echo duzhong | sudo -S -p '' ufw status 2>/dev/null | head -8

echo
echo "=== 3. current geometry (was set to 720p per your earlier request) ==="
XAUTHORITY=$HOME/.Xauthority DISPLAY=:1 xdpyinfo 2>/dev/null | grep dimensions || echo "(display :1 not queryable)"
grep ExecStart /etc/systemd/system/vncserver@:1.service

echo
echo "=== 4. would a vncserver restart kill anything important? ==="
echo "-- processes with DISPLAY=:1 --"
for p in /proc/[0-9]*; do
  if tr '\0' '\n' < "$p/environ" 2>/dev/null | grep -q '^DISPLAY=:1$'; then
    pid=${p#/proc/}
    cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null | cut -c1-90)
    [ -n "$cmd" ] && echo "  pid=$pid $cmd"
  fi
done
echo "-- det_node specifically --"
pgrep -af 'det_node' | head -3 || echo "  det_node NOT running"

echo
echo "=== 5. is 5901 already reachable from outside? ==="
ss -lnt | grep 5901
