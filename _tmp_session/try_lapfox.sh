#!/bin/bash
# try connecting to the laptop hotspot, auto-revert if no ICS IP
LOG=/tmp/lapfox_try.log
: > $LOG
nmcli device wifi connect "LapFox" password "0d000721" >> $LOG 2>&1
RC=$?
echo "CONNECT_EXIT=$RC" >> $LOG
sleep 6
if ip -4 addr show wlan0 | grep -q "192.168.137."; then
  echo "GOT_ICS_IP=yes" >> $LOG
else
  echo "GOT_ICS_IP=no -> reverting to testfox" >> $LOG
  nmcli con up testfox >> $LOG 2>&1
fi
echo "ACTIVE: $(nmcli -t -f NAME,DEVICE,STATE con show --active | tr '\n' ' ')" >> $LOG
ip -4 addr show wlan0 | grep inet >> $LOG 2>&1
