#!/bin/bash
# Count VNC blacklist entries per address family in the car's own log.
L='/home/duzhong/.vnc/duzhong:5901.log'
echo "blacklisted-v4=$(grep -ac 'blacklisted.*\[10\.' "$L" || true)"
echo "blacklisted-v6=$(grep -ac 'blacklisted.*\[240' "$L" || true)"
echo "--- distinct sources ever in the log ---"
grep -ao '\[[0-9a-f:.]*\]' "$L" | sort -u | head -12
echo "--- last 8 lines ---"
tail -c 2000 "$L" | tr -d '\000' | tail -8
