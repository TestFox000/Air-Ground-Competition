#!/bin/bash
# Probe the car for remote-desktop and file-access facilities.
# Run with: Get-Content probe-vnc.sh -Raw | ssh duzhong@duzhong.local "bash -s"

echo "=== session / display ==="
echo "XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-?} DISPLAY=${DISPLAY:-?(unset)}"
loginctl list-sessions --no-legend 2>/dev/null | head -10

echo
echo "=== VNC-ish processes ==="
ps -eo pid,etime,cmd 2>/dev/null | grep -Ei 'vnc|xrdp|websockify|novnc' | grep -v grep
echo "(end of process list)"

echo
echo "=== listening TCP ports ==="
ss -lntp 2>/dev/null | head -40

echo
echo "=== VNC binaries ==="
for b in x11vnc wayvnc tigervncserver vncserver Xvnc x0vncserver websockify novnc_proxy vncpasswd; do
  p=$(command -v "$b" 2>/dev/null)
  if [ -n "$p" ]; then echo "FOUND $b -> $p"; else echo "no    $b"; fi
done

echo
echo "=== installed packages (vnc/desktop/web) ==="
dpkg -l 2>/dev/null | grep -Ei 'vnc|xrdp|novnc|websockify|tiger' | awk '{print $2, $3}'

echo
echo "=== VNC ports actually reachable on loopback (5900-5910) ==="
for p in 5900 5901 5902 6080; do
  if timeout 2 bash -c "echo > /dev/tcp/127.0.0.1/$p" 2>/dev/null; then echo "open   $p"; else echo "closed $p"; fi
done

echo
echo "=== who is on :1 (vnc display) ==="
ls -l /tmp/.X11-unix/ 2>/dev/null

echo
echo "=== Xvnc / tigervnc service units ==="
systemctl list-units --type=service --all 2>/dev/null | grep -Ei 'vnc|x11' || echo "(no vnc unit)"

echo
echo "=== sftp subsystem available? ==="
grep -i '^subsystem' /etc/ssh/sshd_config 2>/dev/null || echo "(no subsystem line readable)"
command -v sftp-server 2>/dev/null || ls -l /usr/lib/openssh/sftp-server 2>/dev/null || echo "(sftp-server path not found by name)"
