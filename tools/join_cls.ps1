# Joins what the buoy transmitted (tx.csv, from tx_log.ps1) against what CLS
# received, message by message, on the timestamp.
#
#   .\tools\join_cls.ps1 -TxCsv "...\tx.csv" -ClsJson "...\cls_receptions.json"
#
# Both clocks are UTC - the buoy stamps its log from the DS3231 and CLS stamps
# msgDatetime - so the match is direct. A few seconds of slack covers the delay
# between the firmware writing its log line and the burst actually radiating.
#
# This is the only way to get a real reception rate: the denominator is what was
# sent, which lives only in the buoy's own log, not what a plan said it would
# send.

param(
  [Parameter(Mandatory=$true)][string]$TxCsv,
  [Parameter(Mandatory=$true)][string]$ClsJson,
  [int]$ToleranceSec = 8
)

$ci = [cultureinfo]::InvariantCulture
$st = [System.Globalization.DateTimeStyles]::AdjustToUniversal -bor [System.Globalization.DateTimeStyles]::AssumeUniversal

$tx = Import-Csv $TxCsv | ForEach-Object {
  [PSCustomObject]@{
    T       = [datetime]::Parse($_.Time, $ci, $st)
    Kind    = $_.Kind
    Sat     = $_.Sat
    ElevNow = if ($_.ElevNow) { [int]$_.ElevNow } else { $null }
    ElevMax = if ($_.ElevMax) { [int]$_.ElevMax } else { $null }
    IntoS   = if ($_.IntoS)   { [int]$_.IntoS }   else { $null }
    PassS   = if ($_.PassS)   { [int]$_.PassS }   else { $null }
    Rx      = $false
    Lvl     = $null
    RxSats  = ''
  }
}

$j = Get-Content $ClsJson -Raw | ConvertFrom-Json
$rx = $j.contents | Group-Object deviceMsgUid | ForEach-Object { $_.Group[0] } | ForEach-Object {
  [PSCustomObject]@{ T = [datetime]::Parse($_.msgDatetime + "Z", $ci, $st)
                     Sat = ($_.kineisMetadata.sat -replace '^KIN','')
                     Lvl = $_.kineisMetadata.level
                     Used = $false }
}

Write-Output ("transmitidos {0}   recibidos (unicos en CLS) {1}" -f $tx.Count, $rx.Count)
Write-Output ""

# Match each reception to the nearest transmission. Several receptions can share
# one transmission - that is two satellites hearing the same burst, which is the
# whole point of a stacked session, so it must not be counted as two sends.
foreach ($r in $rx) {
  $best = $null; $bestGap = [double]::MaxValue
  foreach ($t in $tx) {
    $gap = [math]::Abs(($r.T - $t.T).TotalSeconds)
    if ($gap -lt $bestGap) { $bestGap = $gap; $best = $t }
  }
  if ($best -and $bestGap -le $ToleranceSec) {
    $best.Rx = $true
    $best.RxSats = ($best.RxSats + ' ' + $r.Sat).Trim()
    if ($null -eq $best.Lvl -or $r.Lvl -gt $best.Lvl) { $best.Lvl = $r.Lvl }
    $r.Used = $true
  }
}

$unmatched = @($rx | Where-Object { -not $_.Used })
$hit = @($tx | Where-Object { $_.Rx })
Write-Output ("emparejados: {0} transmisiones oidas de {1} enviadas = {2:N1} %" -f $hit.Count, $tx.Count, (100.0*$hit.Count/$tx.Count))
if ($unmatched.Count) {
  Write-Output ("recepciones sin transmision cercana: {0}" -f $unmatched.Count)
  $unmatched | ForEach-Object { Write-Output ("    {0:HH:mm:ss}  {1}" -f $_.T, $_.Sat) }
}

Write-Output ""
Write-Output "=== recepcion por elevacion instantanea ==="
Write-Output "  elev      enviados  oidos   tasa"
$tx | Where-Object { $null -ne $_.ElevNow } |
  Group-Object { [math]::Floor($_.ElevNow / 10) * 10 } |
  Sort-Object { [int]$_.Name } | ForEach-Object {
    $n = $_.Group.Count
    $k = @($_.Group | Where-Object { $_.Rx }).Count
    $bar = '#' * [int](20.0 * $k / [math]::Max($n,1))
    Write-Output ("  {0,2}-{1,2}      {2,5}  {3,5}   {4,5:N0} %  {5}" -f $_.Name, ([int]$_.Name+9), $n, $k, (100.0*$k/$n), $bar)
  }

Write-Output ""
Write-Output "=== recepcion por satelite ==="
$tx | Where-Object { $_.Sat -and $_.Sat -ne 'none' } | Group-Object Sat | Sort-Object Name | ForEach-Object {
  $n = $_.Group.Count
  $k = @($_.Group | Where-Object { $_.Rx }).Count
  $e = ($_.Group | Measure-Object ElevMax -Maximum).Maximum
  Write-Output ("  {0,-4} pico {1,2} deg   {2,3} enviados  {3,3} oidos   {4,5:N0} %" -f $_.Name, $e, $n, $k, (100.0*$k/$n))
}

Write-Output ""
Write-Output "=== GPS frente a datos ==="
$tx | Group-Object Kind | ForEach-Object {
  $n = $_.Group.Count
  $k = @($_.Group | Where-Object { $_.Rx }).Count
  Write-Output ("  {0,-5} {1,4} enviados  {2,3} oidos   {3,5:N0} %" -f $_.Name, $n, $k, (100.0*$k/$n))
}

Write-Output ""
Write-Output "=== recepcion por posicion en el pase ==="
Write-Output "  (fraccion del pase visto desde 5 grados, que es como se atribuye)"
$tx | Where-Object { $_.PassS -and $_.PassS -gt 0 } |
  Group-Object { [math]::Min(9, [math]::Floor(10.0 * $_.IntoS / $_.PassS)) } |
  Sort-Object { [int]$_.Name } | ForEach-Object {
    $n = $_.Group.Count
    $k = @($_.Group | Where-Object { $_.Rx }).Count
    Write-Output ("  {0,3}-{1,3} %   {2,4} enviados  {3,3} oidos   {4,5:N0} %" -f ([int]$_.Name*10), ([int]$_.Name*10+9), $n, $k, (100.0*$k/$n))
  }

# Two satellites hearing one burst is the stacked-session payoff, so count it.
$dbl = @($tx | Where-Object { $_.RxSats -match ' ' })
Write-Output ""
Write-Output ("transmisiones oidas por mas de un satelite a la vez: {0}" -f $dbl.Count)
$dbl | ForEach-Object { Write-Output ("    {0:HH:mm:ss}  {1}  ({2})" -f $_.T, $_.RxSats, $_.Kind) }
