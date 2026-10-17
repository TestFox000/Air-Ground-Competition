#!/bin/bash
# What happened in the drone's VNC log around 16:17 (the browser attempts), and
# is this machine blacklisted right now?
L='/home/duzhong/.vnc/duzhong:5901.log'
echo "=== entries after 16:11 (post-restart) ==="
# The log has binary bytes; force text mode and keep lines mentioning time/conn.
tr -d '\000' < "$L" | awk '/^Mon Oct  5 16:1[1-9]|^Mon Oct  5 16:2[0-9]/{p=1} p' | head -40
echo
echo "=== blacklist count since restart ==="
tr -d '\000' < "$L" | awk '/^Mon Oct  5 16:11/{p=1} p' | grep -c 'blacklisted' || true
