#!/usr/bin/env bash
# Drone-side bidirectional corroboration check for the dzsmart <-> duzhong ROS2 link.
# READ-ONLY: inspects an existing log, changes nothing.
# Usage:  scp verify_bidir.sh duzhong@dzsmart.local:/tmp/ ; ssh duzhong@dzsmart.local 'bash /tmp/verify_bidir.sh'

LOG=/tmp/dzlink_dzsmart.log
SELF_TAG='HELLO from dzsmart'
PEER_TAG='HELLO from duzhong'

echo "=== 1. identity / time ==="
hostname
date '+%F %T %Z'
uptime -p

echo
echo "=== 2. log file ==="
stat -c '%n  %s bytes  mtime=%y' "$LOG" 2>/dev/null || echo "LOG MISSING"
printf 'total lines: %s\n' "$(wc -l < "$LOG" 2>/dev/null)"

echo
echo "=== 3. counts by source tag ==="
grep -o 'HELLO from [A-Za-z0-9_.-]*' "$LOG" 2>/dev/null | sort | uniq -c

TOTAL=$(grep -c 'HELLO from ' "$LOG" 2>/dev/null)
OWN=$(grep -c "$SELF_TAG" "$LOG" 2>/dev/null)
PEER=$(grep -c "$PEER_TAG" "$LOG" 2>/dev/null)
TOTAL=${TOTAL:-0}
OWN=${OWN:-0}
PEER=${PEER:-0}

echo
echo "TOTAL=$TOTAL OWN_dzsmart=$OWN PEER_duzhong=$PEER"

echo
echo "=== 4. peer samples: first 3 / last 3 with line numbers ==="
grep -n "$PEER_TAG" "$LOG" 2>/dev/null | head -3
echo "   ... [skipped] ..."
grep -n "$PEER_TAG" "$LOG" 2>/dev/null | tail -3

echo
echo "=== 5. interleaving evidence: 3 lines before/after the first peer line ==="
FIRST=$(grep -n "$PEER_TAG" "$LOG" 2>/dev/null | head -1 | cut -d: -f1)
if [ -n "$FIRST" ]; then
  START=$((FIRST - 3))
  [ "$START" -lt 1 ] && START=1
  END=$((FIRST + 3))
  sed -n "${START},${END}p" "$LOG"
else
  echo "no peer lines found"
fi

echo
echo "=== 6. beacon process ==="
pgrep -af 'beacon.sh' 2>/dev/null || echo "no beacon process running"

echo
echo "=== 7. verdict ==="
if [ "$PEER" -gt 0 ]; then
  echo "==> REVERSE_OK   car -> drone direction received $PEER peer beacons"
else
  echo "==> REVERSE_FAIL no peer beacons from duzhong"
fi
