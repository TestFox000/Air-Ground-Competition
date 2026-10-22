$ErrorActionPreference = "Stop"
$root = "D:\User\Desktop\比赛存档\v1"
Set-Location $root
$arca = Join-Path $root "docs\_archive"
$csv  = Join-Path $root "docs\_work\archive_manifest.csv"

New-Item -ItemType Directory -Force -Path $arca | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $root "docs\_work") | Out-Null

$srcs = New-Object System.Collections.Generic.List[string]

$exclTop = @("README.md", "目录索引.md")
Get-ChildItem -LiteralPath (Join-Path $root "docs") -File | Where-Object { $_.Extension -in @(".md", ".txt") } | ForEach-Object {
  if ($exclTop -notcontains $_.Name) { $srcs.Add("docs/" + $_.Name) }
}
foreach ($sub in @("docs\agent_handoff", "docs\merge")) {
  Get-ChildItem -LiteralPath (Join-Path $root $sub) -File -Recurse | ForEach-Object {
    $srcs.Add($_.FullName.Substring($root.Length + 1).Replace("\", "/"))
  }
}
# 8 个门禁活文件必须留在原位：check.py:31-35 REQUIRED_DOCS + repo_index.py:13 注解源 + check_reuse.py:22
$excl4 = @("决策台账.md", "复用清单.md", "工作流v2.md", "目录索引.md", "目录索引.json", "索引标注.yaml", "能力索引.md", "TEAM_STATUS.md")
Get-ChildItem -LiteralPath (Join-Path $root "dzrobot_v2\docs") -File -Recurse | ForEach-Object {
  $rel = $_.FullName.Substring($root.Length + 1).Replace("\", "/")
  $dir = $rel.Substring(0, $rel.LastIndexOf("/"))
  if (($dir -eq "dzrobot_v2/docs") -and ($excl4 -contains $_.Name)) { return }
  $srcs.Add($rel)
}
Get-ChildItem -LiteralPath (Join-Path $root "dzsmartscout\docs") -File -Recurse | ForEach-Object {
  $srcs.Add($_.FullName.Substring($root.Length + 1).Replace("\", "/"))
}

[string[]]$sorted = $srcs.ToArray()
[Array]::Sort($sorted, [System.StringComparer]::Ordinal)
Write-Output ("SOURCE COUNT: " + $sorted.Count)

$nl = [string]([char]13) + [string]([char]10)
$lines = New-Object System.Collections.Generic.List[string]
$lines.Add("原路径,归档路径,字节数,原sha256,归档sha256,一致")
$bad = New-Object System.Collections.Generic.List[string]
$totalBytes = [int64]0
$counts = @{}
foreach ($rel in $sorted) {
  $s = Join-Path $root ($rel.Replace("/", "\"))
  $drel = "docs/_archive/" + $rel
  $d = Join-Path $root ($drel.Replace("/", "\"))
  $ddir = Split-Path -Parent $d
  if (-not (Test-Path -LiteralPath $ddir)) { New-Item -ItemType Directory -Force -Path $ddir | Out-Null }
  Copy-Item -LiteralPath $s -Destination $d -Force
  $hs = (Get-FileHash -LiteralPath $s -Algorithm SHA256).Hash.ToLower()
  $hd = (Get-FileHash -LiteralPath $d -Algorithm SHA256).Hash.ToLower()
  $len = [int64](Get-Item -LiteralPath $s).Length
  $lenD = [int64](Get-Item -LiteralPath $d).Length
  $same = if (($hs -eq $hd) -and ($len -eq $lenD)) { "TRUE" } else { "FALSE" }
  if ($same -ne "TRUE") { $bad.Add($rel) }
  $totalBytes += $len
  $lines.Add(($rel + "," + $drel + "," + $len + "," + $hs + "," + $hd + "," + $same))
  if ($rel.StartsWith("docs/agent_handoff/")) { $k = "S2_agent_handoff" }
  elseif ($rel.StartsWith("docs/merge/")) { $k = "S3_merge" }
  elseif ($rel.StartsWith("docs/")) { $k = "S1_docs_top" }
  elseif ($rel.StartsWith("dzrobot_v2/docs/")) { $k = "S4_dzrobot_v2" }
  elseif ($rel.StartsWith("dzsmartscout/docs/")) { $k = "S5_dzsmartscout" }
  else { $k = "OTHER" }
  if ($counts.ContainsKey($k)) { $counts[$k] = $counts[$k] + 1 } else { $counts[$k] = 1 }
}

$text = ($lines -join $nl) + $nl
$enc = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($csv, $text, $enc)

Write-Output ("ROWS WRITTEN (excl header): " + ($lines.Count - 1))
Write-Output ("MISMATCH COUNT: " + $bad.Count)
if ($bad.Count -gt 0) { $bad | ForEach-Object { Write-Output ("  BAD " + $_) } }
Write-Output ("TOTAL BYTES: " + $totalBytes)
Write-Output "--- per-source counts ---"
foreach ($k in ($counts.Keys | Sort-Object)) { Write-Output ("  " + $k + " = " + $counts[$k]) }
Write-Output "--- 8 例外应全部未进 _archive ---"
foreach ($f in $excl4) { Write-Output ("  " + $f + " archived=" + (Test-Path -LiteralPath ("docs\_archive\dzrobot_v2\docs\" + $f))) }
Write-Output "--- _archive 实际文件数 ---"
(Get-ChildItem -LiteralPath "docs\_archive" -File -Recurse).Count
