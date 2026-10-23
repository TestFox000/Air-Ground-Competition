#!/bin/bash
L=/home/duzhong/r3_field_20261021/logs/7_dzbrain.log
echo ===RESET===
grep -n '复位 belief' "$L" | tail -3
echo ===CTX===
N=$(grep -n '复位 belief' "$L" | tail -1 | cut -d: -f1)
if [ -n "$N" ]; then sed -n "$((N-2)),$((N+30))p" "$L"; fi
echo ===GUARD===
grep -n '安全STOP\|SAFE_STOP\|航向\|不开火' "$L" | tail -10
echo ===TAIL===
tail -25 "$L"
