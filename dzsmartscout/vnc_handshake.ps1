# VNC (RFB 3.8) auth handshake against the drone, password 'duzhong'.
$ErrorActionPreference = 'Stop'
$hostName = 'dzsmart.local'
$port = 5901
$password = 'duzhong'

$client = New-Object System.Net.Sockets.TcpClient
$client.Connect($hostName, $port)
$stream = $client.GetStream()
$stream.ReadTimeout = 5000

function Read-Bytes([int]$count) {
  $buf = New-Object byte[] $count
  $total = 0
  while ($total -lt $count) {
    $n = $stream.Read($buf, $total, $count - $total)
    if ($n -le 0) { throw "EOF after $total/$count bytes" }
    $total += $n
  }
  return $buf
}

# 1. banner
$banner = Read-Bytes 12
$bannerText = [Text.Encoding]::ASCII.GetString($banner)
Write-Output "banner: $bannerText"
if ($bannerText -notmatch '^RFB \d{3}\.\d{3}\n$') { throw 'bad banner' }

# 2. send version 3.8
$ver = [Text.Encoding]::ASCII.GetBytes("RFB 003.008`n")
$stream.Write($ver, 0, $ver.Length)

# 3. security types
$count = (Read-Bytes 1)[0]
if ($count -eq 0) {
  $reason = [Text.Encoding]::ASCII.GetString((Read-Bytes (Read-Bytes 4 | Select-Object -Skip 0) | ForEach-Object { $_ }))
  Write-Output "server refused: $reason"; exit 1
}
$types = Read-Bytes $count
Write-Output ("security types: " + (($types | ForEach-Object { $_ }) -join ','))
if ($types -notcontains 2 -and $types -notcontains 1) { throw 'VncAuth not offered' }
$chosen = if ($types -contains 2) { 2 } else { 1 }
Write-Output "choosing VncAuth ($chosen)"
$stream.Write([byte[]]@($chosen), 0, 1)

# 4. challenge
$challenge = Read-Bytes 16

# 5. DES encrypt challenge with VNC key schedule
function ReverseBits([byte]$b) { $r = 0; for ($i = 0; $i -lt 8; $i++) { $r = ($r -shl 1) -bor (($b -shr $i) -band 1) }; return [byte]$r }
$pw = [Text.Encoding]::ASCII.GetBytes($password.PadRight(8, [char]0).Substring(0, 8))
$key = New-Object byte[] 8
for ($i = 0; $i -lt 8; $i++) { $key[$i] = ReverseBits $pw[$i] }
$des = [System.Security.Cryptography.DES]::Create()
$des.Mode = [System.Security.Cryptography.CipherMode]::ECB
$des.Padding = [System.Security.Cryptography.PaddingMode]::None
$des.Key = $key
$resp = $des.CreateEncryptor().TransformFinalBlock($challenge, 0, 16)
$stream.Write($resp, 0, 16)

# 6. auth result
$status = Read-Bytes 4
$code = [BitConverter]::ToUInt32(@($status[3], $status[2], $status[1], $status[0]), 0)
if ($code -eq 0) {
  Write-Output "AUTH OK - VNC password 'duzhong' accepted"
  # clean disconnect: shared-flag byte 1 then close
  try { $stream.Write([byte[]]@(1), 0, 1) } catch {}
} else {
  Write-Output "AUTH FAILED code=$code"
}
$stream.Close(); $client.Close()
