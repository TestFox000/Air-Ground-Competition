#!/bin/bash
set -e
# remove any misplaced deny-interfaces line
sed -i '/^deny-interfaces=docker0$/d' /etc/avahi/avahi-daemon.conf
# insert it under [server] if not already present
grep -q '^deny-interfaces=docker0$' /etc/avahi/avahi-daemon.conf || \
  sed -i '/^\[server\]$/a deny-interfaces=docker0' /etc/avahi/avahi-daemon.conf
# fix /etc/hosts
sed -i 's/^127\.0\.1\.1.*$/127.0.1.1\tdzsmart/' /etc/hosts
echo "=== avahi-daemon.conf [server] head ==="
grep -n -A2 '^\[server\]' /etc/avahi/avahi-daemon.conf | head -6
echo "=== /etc/hosts 127.0.1.1 ==="
grep '127.0.1.1' /etc/hosts
echo "=== restart avahi ==="
systemctl restart avahi-daemon
sleep 2
systemctl is-active avahi-daemon
