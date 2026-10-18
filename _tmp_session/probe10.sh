set +e
echo "== kernel log: usb/cdc events today (needs journal perms) =="
journalctl -k --no-pager -n 400 2>&1 | grep -i -E 'cdc_acm|usb 1-2\.1|ttyACM' | tail -20
echo "== journal access check =="
journalctl -k -n 1 --no-pager 2>&1 | tail -2
