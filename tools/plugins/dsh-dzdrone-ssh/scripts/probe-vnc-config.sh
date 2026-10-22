#!/bin/bash
# Inspect the drone's VNC setup and verify a candidate VNC password.
# Usage: Get-Content probe-vnc-config.sh -Raw | ssh duzhong@dzsmart.local "bash -s"

echo "=== ~/.vnc/config ==="
cat ~/.vnc/config 2>/dev/null || echo "(no ~/.vnc/config)"

echo
echo "=== ~/.vnc/xstartup (head) ==="
head -20 ~/.vnc/xstartup 2>/dev/null || echo "(none)"

echo
echo "=== the running unit ==="
systemctl cat vncserver@:1.service 2>/dev/null | grep -vE '^\s*#' | grep -v '^$'

echo
echo "=== unit files on disk ==="
ls -l /etc/systemd/system/vncserver* /lib/systemd/system/vncserver* /usr/lib/systemd/system/vncserver* 2>/dev/null

echo
echo "=== current X display geometry on :1 ==="
XAUTHORITY=$HOME/.Xauthority DISPLAY=:1 xdpyinfo 2>/dev/null | grep -E 'dimensions|resolution' || echo "(xdpyinfo unavailable)"

echo
echo "=== processes with DISPLAY=:1 (what would die on a VNC restart) ==="
for p in /proc/[0-9]*; do
  if tr '\0' '\n' < "$p/environ" 2>/dev/null | grep -q '^DISPLAY=:1$'; then
    pid=${p#/proc/}
    echo "pid=$pid cmd=$(tr '\0' ' ' < $p/cmdline 2>/dev/null)"
  fi
done

echo
echo "=== verify a candidate VNC password against ~/.vnc/passwd ==="
python3 - <<'PY'
import subprocess, os
candidates = [b'duzhong', b'dzsmartcar']
path = os.path.expanduser('~/.vnc/passwd')
try:
    raw = open(path, 'rb').read()
except OSError as error:
    print('cannot read', path, error)
    raise SystemExit(0)
print('passwd file:', len(raw), 'bytes, hex =', raw[:8].hex())

def obfuscate_key(pw):
    # VNC derives the DES key from the first 8 bytes of the password with every
    # byte's bit order reversed.
    padded = pw[:8].ljust(8, b'\0')
    return bytes(int(format(b, '08b')[::-1], 2) for b in padded)

for cand in candidates:
    key = obfuscate_key(cand)
    result = subprocess.run(
        ['openssl', 'enc', '-des-ecb', '-d', '-nopad', '-K', key.hex()],
        input=raw[:8], capture_output=True)
    plain = result.stdout
    ok = plain[:len(cand)] == cand[:len(cand)] and plain[len(cand):len(cand)+1] in (b'', b'\0')
    print(f'candidate {cand!r}: decrypted={plain!r} -> {"MATCH" if ok else "no"}')
PY
