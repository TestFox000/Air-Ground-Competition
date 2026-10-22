#!/bin/bash
# Did the browser's 16:17 VNC attempts reach this machine at all?
L='/home/duzhong/.vnc/duzhong:5901.log'
echo "=== all Connections lines after 16:11 ==="
tr -d '\000' < "$L" | awk '/^Mon Oct  5 16:11/{p=1} p && /Connections:/' | head -30
echo
echo "=== timestamped blocks after 16:11 (time lines only) ==="
tr -d '\000' < "$L" | awk '/^Mon Oct  5 16:1[1-9]|^Mon Oct  5 16:2[0-9]/{print}' | head -20
