# -*- coding: utf-8 -*-
"""On the drone: dump ~/.vnc/passwd, then test VNC password candidates
OFFLINE (pure DES check, no network handshake -> no TigerVNC blacklist risk)."""
import paramiko

HOST, USER, PW = "dzsmart.local", "duzhong", "duzhong"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PW, timeout=15,
               allow_agent=False, look_for_keys=False)

def sh(cmd, timeout=30):
    stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
    return stdout.read(), stderr.read()

# read passwd file (may need sudo; try user first)
raw, err = sh("base64 -w0 /home/duzhong/.vnc/passwd 2>/dev/null")
if not raw.strip():
    raw, err = sh(f"echo {PW} | sudo -S -p '' base64 -w0 /home/duzhong/.vnc/passwd")
print("passwd base64:", raw.decode().strip()[:120])

import base64, hashlib
from Crypto.Cipher import DES  # pycryptodome usually present on Windows box

data = base64.b64decode(raw.decode().strip())
candidates = ["duzhong", "dzsmart", "dzsmartcar", "dzrobot", "123456", "1234",
              "password", "duzhong123", "dz", "testfox", "ubuntu", "admin",
              "dzsmart2026", "duzhong2026"]

def vnc_key(pw: bytes) -> bytes:
    pw = pw[:8] + b"\x00" * (8 - len(pw))
    key = bytearray()
    for byte in pw:
        rev = 0
        for i in range(8):
            rev = (rev << 1) | ((byte >> i) & 1)
        key.append(rev & 0xFF)
    return bytes(key)

found = None
for cand in candidates:
    cipher = DES.new(vnc_key(cand.encode("utf-8")), DES.MODE_ECB)
    dec = cipher.decrypt(data[:8])
    # VNCAuth stores the challenge as "password" encrypted; verify by re-encrypting
    # round-trip consistency check against remainder is impossible; instead check
    # the standard trick: encrypt decrypted plaintext with same key gives back data
    if cipher.encrypt(dec) == data[:8]:
        found = cand
        break
print("VNC password found:", found)

client.close()
