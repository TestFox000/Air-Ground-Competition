#!/bin/bash
# t28: add static peer mapping to /etc/hosts (idempotent, backups kept)
SELF="$(hostname)"
if [ "$SELF" = "dzsmart" ]; then
  LINE="192.168.137.79 duzhong.local duzhong  # t28 static"
else
  LINE="192.168.137.52 dzsmart.local dzsmart  # t28 static"
fi
cp /etc/hosts /etc/hosts.bak_t28 2>/dev/null
if ! grep -qF "$LINE" /etc/hosts; then
  echo "$LINE" >> /etc/hosts
  echo "ADDED: $LINE"
else
  echo "ALREADY: $LINE"
fi
echo "--- hosts (no comments) ---"
grep -v '^#' /etc/hosts | grep -v '^$'
echo "--- getent ---"
getent hosts dzsmart.local duzhong.local || true

