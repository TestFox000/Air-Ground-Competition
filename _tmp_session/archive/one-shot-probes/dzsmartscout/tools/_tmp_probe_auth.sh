echo "=== 1) sshpass presence ==="
command -v sshpass || echo NO_SSHPASS
ls -la /usr/bin/sshpass /usr/local/bin/sshpass 2>/dev/null
echo "=== 2) key auth drone->car ==="
ssh -o BatchMode=yes -o StrictHostKeyChecking=no -o ConnectTimeout=6 duzhong@10.226.211.188 'hostname; whoami; echo KEY_OK' 2>&1 | head -6
echo "=== 3) ssh keys on drone ==="
ls -la ~/.ssh/ 2>/dev/null | head -12
echo "=== 4) drone identity ==="
hostname; hostname -I
echo "=== 5) can drone see car ip ==="
ping -c1 -W2 10.226.211.188 >/dev/null 2>&1 && echo CAR_PING_OK || echo CAR_PING_FAIL
echo DONE
