
$ErrorActionPreference = 'Stop'
$root = (Get-Location).Path
$snap = Join-Path $root 'docs\_archive\snapshots'
New-Item -ItemType Directory -Force -Path $snap | Out-Null
$report = Join-Path $root 'docs\_work\pack_report.txt'
Set-Content $report ("pack report " + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
function Pack($name, $srcRel) {
    $src = Join-Path $root $srcRel
    if (-not (Test-Path $src)) { Add-Content $report "SKIP $name (no source)"; return }
    $out = Join-Path $snap ($name + '.tar.gz')
    $before = (Get-ChildItem $src -Recurse -File -Force | Measure-Object).Count
    if (Test-Path $out) { Remove-Item $out -Force }
    & tar -czf $out -C (Split-Path $src -Parent) (Split-Path $src -Leaf) 2>$null
    if ($LASTEXITCODE -ne 0) { Add-Content $report ("FAIL " + $name + " tar exit " + $LASTEXITCODE); return }
    $inTar = (& tar -tzf $out | Measure-Object).Count
    Add-Content $report ("{0}: files={1} members={2} size={3:N1}MB" -f $name, $before, $inTar, ((Get-Item $out).Length/1MB))
    if ($inTar -ge $before) { Remove-Item $src -Recurse -Force; Add-Content $report "  -> source removed" }
    else { Add-Content $report "  -> KEPT source (member count lower)" }
}
Pack 'ext_gitee_air_ground_20261022' '_tmp_session\gitee_air_ground'
Pack 'ext_yb_tree_20261022' '_tmp_session\yb_tree'
Pack 'ext_gitee_main_tree_20261022' '_tmp_session\gitee_main_tree'
Pack 'ext_carsnap_tree_20261022' '_tmp_session\carsnap_tree'
Pack 'process_TRACK-FIELD-R3_20261022' '_tmp_session\TRACK-FIELD-R3-20261020-01'
$tmp = Join-Path $root '_tmp_session'
if (Test-Path $tmp) {
    $left = Get-ChildItem $tmp -Force -ErrorAction SilentlyContinue
    if ($left) {
        $out = Join-Path $snap 'process_drafts_20261022.tar.gz'
        $before = (Get-ChildItem $tmp -Recurse -File -Force | Measure-Object).Count
        if (Test-Path $out) { Remove-Item $out -Force }
        & tar -czf $out -C $root '_tmp_session' 2>$null
        $inTar = (& tar -tzf $out | Measure-Object).Count
        Add-Content $report ("process_drafts: files={0} members={1} size={2:N1}MB" -f $before, $inTar, ((Get-Item $out).Length/1MB))
        if ($inTar -ge $before) { Remove-Item $tmp -Recurse -Force; Add-Content $report "  -> _tmp_session removed" }
        else { Add-Content $report "  -> KEPT _tmp_session" }
    }
}
$dv = Join-Path $root 'dzrobot_v2\_tmp_session'
if (Test-Path $dv) {
    $out = Join-Path $snap 'dzrobot_v2_tmp_session_20261022.tar.gz'
    $before = (Get-ChildItem $dv -Recurse -File -Force | Measure-Object).Count
    if (Test-Path $out) { Remove-Item $out -Force }
    & tar -czf $out -C (Join-Path $root 'dzrobot_v2') '_tmp_session' 2>$null
    $inTar = (& tar -tzf $out | Measure-Object).Count
    Add-Content $report ("dzrobot_v2_tmp_session: files={0} members={1} size={2:N1}MB" -f $before, $inTar, ((Get-Item $out).Length/1MB))
    if ($inTar -ge $before) { Remove-Item $dv -Recurse -Force; Add-Content $report "  -> source removed" }
}
Add-Content $report ("DONE " + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
