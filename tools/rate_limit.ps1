# Checks a campaign against the CLS rate limit: at most 511 messages in any sliding
# window of 511 minutes per beacon (Philippe, CLS, 25 Sep 2026).
#
#   .\tools\rate_limit.ps1 -Buoys "B5=216573=path\LogFile.txt,B1=294850=path\LogFile.txt" `
#       -Cls "path\c24p_*.json" -Out result.json
#
# For every buoy it counts, from its LogFile, the transmissions in the trailing 511
# minutes at each TX, and from CLS the receptions. Each reception is paired with the
# buoy's nearest transmission (msgDatetime is the satellite's own timestamp and lands a
# few seconds after the TX line, far less than the 30 s between messages), so every TX
# is known to have arrived or not. That gives the real test: if CLS throttled a beacon
# over the limit, the transmissions sent while over it would arrive less often than
# the ones sent under it, at the same elevation.
param(
  [Parameter(Mandatory=$true)][string]$Buoys,
  [Parameter(Mandatory=$true)][string]$Cls,
  [string]$Out,
  [int]$Limit = 511,
  [int]$WindowMin = 511,
  [int]$MatchS = 20
)
$inv = [Globalization.CultureInfo]::InvariantCulture
$E0 = [datetime]'2026-01-01'
$W = $WindowMin * 60

$all = @(); foreach ($f in (Get-ChildItem $Cls)) { $all += (Get-Content -Raw $f.FullName | ConvertFrom-Json).contents }
$rxByRef = @{}
foreach ($g in ($all | Group-Object deviceMsgUid | % { $_.Group[0] } | Group-Object deviceRef)) {
  $rxByRef[$g.Name] = @($g.Group | % { ([datetime]::Parse($_.msgDatetime, $inv) - $E0).TotalSeconds } | Sort-Object)
}

# Trailing count at each element of a sorted time list (the element itself included).
function Trailing([double[]]$s) {
  $c = New-Object int[] $s.Count; $j = 0
  for ($i = 0; $i -lt $s.Count; $i++) { while ($s[$i] - $s[$j] -ge $W) { $j++ }; $c[$i] = $i - $j + 1 }
  , $c
}
# Trailing count sampled on a grid.
function Series([double[]]$s, [double]$from, [double]$to) {
  $out = @(); $k0 = 0; $k1 = 0
  for ($x = $from; $x -le $to; $x += 1800) {
    while ($k1 -lt $s.Count -and $s[$k1] -le $x) { $k1++ }
    while ($k0 -lt $k1 -and $s[$k0] -le $x - $W) { $k0++ }
    $out += , [int]($k1 - $k0)
  }
  , $out
}
function Band($e) { if ($null -eq $e) { 'sin' } elseif ($e -lt 15) { '<15' } elseif ($e -lt 30) { '15-30' } elseif ($e -lt 50) { '30-50' } else { '50+' } }

$res = foreach ($spec in $Buoys.Split(',')) {
  $p = $spec.Split('=', 3); $id = $p[0].Trim(); $ref = $p[1].Trim(); $log = $p[2].Trim()
  $tx = New-Object Collections.Generic.List[object]
  foreach ($line in [IO.File]::ReadLines((Resolve-Path $log).ProviderPath)) {
    if ($line -match '^(\S+?)----State \S+ - TX;([^;]*);([^;]*);([^;]*);([^;]*);') {
      $tx.Add([pscustomobject]@{ t = ([datetime]::Parse($Matches[1], $inv) - $E0).TotalSeconds; kind = $Matches[2]; ok = $Matches[3] -eq 'OK'
        elev = $(if ($Matches[5]) { [int]$Matches[5] } else { $null }); got = $false; over = $false })
    }
  }
  $tx = @($tx | Sort-Object t)
  $ts = [double[]]@($tx | % { $_.t })
  $trTx = Trailing $ts
  for ($i = 0; $i -lt $tx.Count; $i++) { $tx[$i].over = $trTx[$i] -gt $Limit }

  # Pair every reception with the nearest transmission.
  $rx = [double[]]@($rxByRef[$ref] | ? { $_ -ge $ts[0] - 60 -and $_ -le $ts[-1] + 60 })
  $lags = New-Object Collections.Generic.List[double]; $k = 0
  foreach ($r in $rx) {
    while ($k + 1 -lt $ts.Count -and [math]::Abs($ts[$k + 1] - $r) -le [math]::Abs($ts[$k] - $r)) { $k++ }
    $d = $r - $ts[$k]
    if ([math]::Abs($d) -le $MatchS) { $tx[$k].got = $true; $lags.Add($d) }
  }
  $trRx = if ($rx.Count) { Trailing $rx } else { @(0) }

  $mxTx = ($trTx | Measure -Maximum).Maximum; $iTx = [array]::IndexOf($trTx, [int]$mxTx)
  $mxRx = ($trRx | Measure -Maximum).Maximum; $iRx = [array]::IndexOf($trRx, [int]$mxRx)
  # Minutes spent over the limit, on a 1-minute grid.
  $overMin = 0; $j0 = 0; $j1 = 0
  for ($x = $ts[0]; $x -le $ts[-1]; $x += 60) {
    while ($j1 -lt $ts.Count -and $ts[$j1] -le $x) { $j1++ }
    while ($j0 -lt $j1 -and $ts[$j0] -le $x - $W) { $j0++ }
    if ($j1 - $j0 -gt $Limit) { $overMin++ }
  }
  # Reception rate under and over the limit, per elevation band (pass maximum).
  $bands = foreach ($bd in '15-30', '30-50', '50+', '<15') {
    $u = @($tx | ? { -not $_.over -and (Band $_.elev) -eq $bd }); $o = @($tx | ? { $_.over -and (Band $_.elev) -eq $bd })
    [ordered]@{ band = $bd; nU = $u.Count; rU = $(if ($u.Count) { [math]::Round(@($u | ? got).Count / $u.Count, 3) } else { $null })
      nO = $o.Count; rO = $(if ($o.Count) { [math]::Round(@($o | ? got).Count / $o.Count, 3) } else { $null }) }
  }
  $uAll = @($tx | ? { -not $_.over }); $oAll = @($tx | ? over)
  $sorted = @($lags | Sort-Object)
  [ordered]@{
    id = $id; ref = $ref; tx = $tx.Count; txOk = @($tx | ? ok).Count; rx = $rx.Count; matched = $lags.Count
    lagMed = $(if ($sorted.Count) { [math]::Round($sorted[[int]($sorted.Count / 2)], 1) } else { $null })
    from = $E0.AddSeconds($ts[0]).ToString('yyyy-MM-ddTHH:mm'); to = $E0.AddSeconds($ts[-1]).ToString('yyyy-MM-ddTHH:mm')
    maxTx = [int]$mxTx; maxTxEnd = $E0.AddSeconds($ts[$iTx]).ToString('yyyy-MM-ddTHH:mm')
    maxRx = [int]$mxRx; maxRxEnd = $(if ($rx.Count) { $E0.AddSeconds($rx[$iRx]).ToString('yyyy-MM-ddTHH:mm') } else { '' })
    overMin = $overMin; txOver = $oAll.Count
    rUnder = $(if ($uAll.Count) { [math]::Round(@($uAll | ? got).Count / $uAll.Count, 3) } else { $null })
    rOver = $(if ($oAll.Count) { [math]::Round(@($oAll | ? got).Count / $oAll.Count, 3) } else { $null })
    bands = @($bands)
    t0 = $E0.AddSeconds($ts[0]).ToString('yyyy-MM-ddTHH:mm')
    sTx = (Series $ts $ts[0] $ts[-1]); sRx = (Series $rx $ts[0] $ts[-1])
  }
}
$res | % { "{0} tx={1} rx={2} (paired {3}, lag med {4} s) | max tx/511min={5} at {6} | over limit {7} min, {8} tx | max rx/511min={9} at {10} | rx rate under={11} over={12}" -f `
  $_.id, $_.tx, $_.rx, $_.matched, $_.lagMed, $_.maxTx, $_.maxTxEnd, $_.overMin, $_.txOver, $_.maxRx, $_.maxRxEnd, $_.rUnder, $_.rOver
  foreach ($b in $_.bands) { if ($b.nO) { "    {0,-6} under {1,5} tx {2:P1} | over {3,5} tx {4:P1}" -f $b.band, $b.nU, $b.rU, $b.nO, $b.rO } } }
if ($Out) { @($res) | ConvertTo-Json -Depth 6 -Compress | Set-Content -Encoding UTF8 $Out }
