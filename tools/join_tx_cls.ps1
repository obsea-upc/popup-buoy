# Joins what the buoys transmitted (from their LogFiles) with what CLS received, so
# the reception rate has a real denominator instead of a modelled one.
#
#   .\tools\join_tx_cls.ps1 -TxDir .\logs -Cls .\cls_sensors.json -Out .\joined.csv
#
# A transmission is matched to a reception first by payload - the log records the
# exact 46 hex characters sent - and, for copies that arrived with flipped bits, by
# time inside -WindowS seconds. Each row carries the elevation the buoy believed the
# satellite was at, how far into the pass it was, and whether CLS needed to correct
# anything (2_BCH).
param(
  [Parameter(Mandatory=$true)][string]$TxDir,
  [Parameter(Mandatory=$true)][string]$Cls,
  [Parameter(Mandatory=$true)][string]$Out,
  [string]$Buoys = "B1=217459,B2=294848,B3=217089,B4=217475,B5=216573",
  [int]$WindowS = 120
)

$refOf = @{}
foreach ($kv in $Buoys.Split(',')) { $p = $kv.Split('='); $refOf[$p[0].Trim()] = $p[1].Trim() }

$j = Get-Content $Cls -Raw | ConvertFrom-Json
$rx = $j.contents | Group-Object deviceMsgUid | ForEach-Object { $_.Group | Sort-Object { [int][bool]$_.sensors } -Descending | Select-Object -First 1 } |
  ForEach-Object {
    $raw = $_.rawData
    if (-not $raw) { return }
    # Data frames carry 46 hex of payload after an 8-hex header (KIM) or none
    # (Arribada). Position messages are shorter - 26 hex of payload - and must be in
    # the pool too, or the GPS transmissions in the log find nothing to match and
    # steal the receptions of the data messages around them.
    $off = if ($_.msgType -eq 'ARGOS #1' -and $raw.Length -le 32) { 0 }
           elseif ($_.msgType -eq 'ARGOS #1') { 0 }
           elseif ($raw.Length -ge 34) { 8 } else { return }
    $take = [math]::Min(46, $raw.Length - $off)
    $bch = if ($_.sensors -match '2_BCH\\?"?:\s*\\?"(-?\d+)') { [int]$Matches[1] } else { $null }
    [pscustomobject]@{
      ref = $_.deviceRef; t = [datetime]::Parse($_.msgDatetime, [Globalization.CultureInfo]::InvariantCulture)
      hex = $raw.Substring($off, $take).ToLower(); sat = $_.kineisMetadata.sat
      level = $_.kineisMetadata.level; snr = $_.kineisMetadata.snr; bch = $bch; used = $false
    }
  }
Write-Output ("recepciones cargadas: {0}" -f @($rx).Count)

# index by payload and by buoy for the two matching passes
$byHex = @{}
foreach ($r in $rx) {
  $k = $r.ref + '|' + $r.hex
  if (-not $byHex.ContainsKey($k)) { $byHex[$k] = New-Object Collections.Generic.List[object] }
  $byHex[$k].Add($r)
}
$byRef = @{}
foreach ($r in $rx) {
  if (-not $byRef.ContainsKey($r.ref)) { $byRef[$r.ref] = New-Object Collections.Generic.List[object] }
  $byRef[$r.ref].Add($r)
}
foreach ($k in @($byRef.Keys)) { $byRef[$k] = [Collections.Generic.List[object]]@($byRef[$k] | Sort-Object t) }

# Two passes on purpose. Matching by time first would let a transmission whose copy
# arrived corrupted steal the clean reception of its neighbour 30 s away, and the
# error cascades down the whole session. So every exact payload match is resolved
# first, and only what is left over is matched by time.
$all = New-Object Collections.Generic.List[object]
foreach ($f in (Get-ChildItem (Join-Path $TxDir "*_tx.csv"))) {
  $tx = @(Import-Csv $f.FullName)
  if (-not $tx) { continue }
  $ref = $refOf[$tx[0].buoy]
  foreach ($x in $tx) {
    $all.Add([pscustomobject]@{
      x = $x; ref = $ref; t = [datetime]::Parse($x.utc, [Globalization.CultureInfo]::InvariantCulture)
      hit = $null; clean = $false })
  }
}
foreach ($e in $all) {                      # pass 1: exact payload
  if (-not $e.x.payload) { continue }
  $k = $e.ref + '|' + $e.x.payload.ToLower()
  if (-not $byHex.ContainsKey($k)) { continue }
  $h = $byHex[$k] | Where-Object { -not $_.used -and [math]::Abs(($_.t - $e.t).TotalSeconds) -le $WindowS } |
       Sort-Object { [math]::Abs(($_.t - $e.t).TotalSeconds) } | Select-Object -First 1
  if ($h) { $h.used = $true; $e.hit = $h; $e.clean = $true }
}
foreach ($e in $all) {                      # pass 2: whatever is left, by time
  if ($e.hit -or -not $byRef.ContainsKey($e.ref)) { continue }
  $h = $byRef[$e.ref] | Where-Object { -not $_.used -and [math]::Abs(($_.t - $e.t).TotalSeconds) -le $WindowS } |
       Sort-Object { [math]::Abs(($_.t - $e.t).TotalSeconds) } | Select-Object -First 1
  if ($h) { $h.used = $true; $e.hit = $h; $e.clean = $false }
}

$rows = New-Object Collections.Generic.List[object]
foreach ($e in $all) {
    $x = $e.x; $hit = $e.hit; $clean = $e.clean
    $rows.Add([pscustomobject]@{
      buoy = $x.buoy; utc = $x.utc; kind = $x.kind; status = $x.status
      sat = $x.sat; elevNow = $x.elevNow; elevMax = $x.elevMax; sinceStart = $x.sinceStart; passDur = $x.passDur
      wake = $x.wake
      recibido = [int][bool]$hit
      limpio = if ($hit) { [int]$clean } else { 0 }
      satRx = if ($hit) { $hit.sat } else { '' }
      level = if ($hit) { $hit.level } else { '' }
      snr = if ($hit) { $hit.snr } else { '' }
      bch = if ($hit -and $null -ne $hit.bch) { $hit.bch } else { '' }
    })
  }
$rows | Export-Csv -NoTypeInformation -Encoding utf8 $Out
$unused = @($rx | Where-Object { -not $_.used }).Count
Write-Output ("transmisiones: {0}, emparejadas: {1}, recepciones sin emparejar: {2}" -f `
  $rows.Count, @($rows | Where-Object recibido -eq 1).Count, $unused)
