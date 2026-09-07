# Pulls the transmission record out of a buoy LogFile and writes it as a CSV
# ready to join against a CLS reception export on the timestamp.
#
# Every transmission logs a line like
#   2026-09-04T06:12:34----State DM - TX;DATA;OK;2D;71;75;120;257;1000
# which is kind, result, satellite, elevation now, pass maximum, seconds into
# the pass, pass length, power. When nothing is overhead the satellite field is
# "none" and the geometry fields are empty - normal for recovery messages.
#
#   .\tools\tx_log.ps1 -LogFile "path\to\LogFile.txt" -OutCsv "tx.csv"

param(
  [Parameter(Mandatory=$true)][string]$LogFile,
  [string]$OutCsv
)

if (-not (Test-Path $LogFile)) { throw "no such log: $LogFile" }

$ci = [cultureinfo]::InvariantCulture
$st = [System.Globalization.DateTimeStyles]::AdjustToUniversal -bor `
      [System.Globalization.DateTimeStyles]::AssumeUniversal

$tx       = New-Object Collections.Generic.List[object]
$sessions = New-Object Collections.Generic.List[object]

foreach ($line in (Get-Content $LogFile)) {
  # The RTC stamp is UTC, the same base the CLS export uses, so no conversion.
  if ($line -notmatch '^(\S+?)----State (\S+) - (.*)$') { continue }
  $when  = [datetime]::Parse($Matches[1], $ci, $st)
  $state = $Matches[2]
  $body  = $Matches[3]

  if ($body -match '^TX;([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);([^;]*);(.*)$') {
    $tx.Add([PSCustomObject]@{
      Time    = $when
      State   = $state
      Kind    = $Matches[1]
      Result  = $Matches[2]
      Sat     = $Matches[3]
      ElevNow = if ($Matches[4]) { [int]$Matches[4] } else { $null }
      ElevMax = if ($Matches[5]) { [int]$Matches[5] } else { $null }
      IntoS   = if ($Matches[6]) { [int]$Matches[6] } else { $null }
      PassS   = if ($Matches[7]) { [int]$Matches[7] } else { $null }
      PowerMw = [int]$Matches[8]
    })
  }
  elseif ($body -match '^SPP: (\d+) passes above ([\d.]+) deg, (\d+) sessions, (\d+) too small\. Taking (\d+) pass\(es\): (.*)$') {
    $sessions.Add([PSCustomObject]@{
      Time = $when; Passes = [int]$Matches[1]; MinElev = [double]$Matches[2]
      Sessions = [int]$Matches[3]; TooSmall = [int]$Matches[4]
      Took = [int]$Matches[5]; Sats = $Matches[6].Trim()
    })
  }
}

Write-Output ("transmissions: {0}    predictions: {1}" -f $tx.Count, $sessions.Count)
if ($tx.Count -eq 0) { Write-Output "nothing to report"; return }

Write-Output ("window: {0:u} .. {1:u}" -f $tx[0].Time, $tx[$tx.Count-1].Time)
Write-Output ""

# --- what went out ---------------------------------------------------------
Write-Output "=== by kind and result ==="
$tx | Group-Object Kind, Result | Sort-Object Name |
  ForEach-Object { Write-Output ("  {0,-12} {1,5}" -f $_.Name, $_.Count) }

$errs = @($tx | Where-Object { $_.Result -ne 'OK' })
if ($errs.Count) {
  Write-Output ("`n  {0} module errors ({1:N1} % of transmissions)" -f `
                $errs.Count, (100.0 * $errs.Count / $tx.Count))
}

# --- geometry the messages went out under ----------------------------------
$withSat = @($tx | Where-Object { $_.Sat -and $_.Sat -ne 'none' })
$blind   = @($tx | Where-Object { -not $_.Sat -or $_.Sat -eq 'none' })

Write-Output ("`n=== attribution ===")
Write-Output ("  with a satellite overhead : {0}" -f $withSat.Count)
Write-Output ("  none overhead (recovery)  : {0}" -f $blind.Count)

if ($withSat.Count) {
  Write-Output "`n=== messages per satellite ==="
  $withSat | Group-Object Sat | Sort-Object Count -Descending | ForEach-Object {
    $e = ($_.Group | Measure-Object ElevNow -Average).Average
    Write-Output ("  {0,-4} {1,5}   mean elevation {2,5:N1} deg" -f $_.Name, $_.Count, $e)
  }

  # Buckets are what a reception curve gets fitted on, so print the counts now:
  # a bucket with a handful of messages will not say anything tomorrow either.
  Write-Output "`n=== messages per elevation bucket ==="
  $withSat | Group-Object { [math]::Floor($_.ElevNow / 10) * 10 } |
    Sort-Object { [int]$_.Name } | ForEach-Object {
      Write-Output ("  {0,3}-{1,3} deg  {2,5}" -f $_.Name, ([int]$_.Name + 9), $_.Count)
    }

  if (@($withSat | Group-Object PowerMw).Count -gt 1) {
    Write-Output "`n=== messages per power level ==="
    $withSat | Group-Object PowerMw | Sort-Object { [int]$_.Name } |
      ForEach-Object { Write-Output ("  {0,5} mW  {1,5}" -f $_.Name, $_.Count) }
  }
}

# --- what the planner chose ------------------------------------------------
if ($sessions.Count) {
  Write-Output "`n=== predictions ==="
  $last = $sessions[$sessions.Count - 1]
  Write-Output ("  floor {0} deg, typically {1} passes -> {2} sessions, {3} too small" -f `
                $last.MinElev, $last.Passes, $last.Sessions, $last.TooSmall)
  $multi = @($sessions | Where-Object { $_.Took -gt 1 })
  Write-Output ("  wakes that served more than one pass: {0} of {1}" -f $multi.Count, $sessions.Count)
  foreach ($s in $multi) { Write-Output ("    {0:u}  {1}" -f $s.Time, $s.Sats) }
}

if ($OutCsv) {
  # Round-trip the time as ISO 8601 UTC so the join key is unambiguous.
  $tx | Select-Object @{n='Time';e={$_.Time.ToString('yyyy-MM-ddTHH:mm:ssZ')}},
                      State, Kind, Result, Sat, ElevNow, ElevMax, IntoS, PassS, PowerMw |
    Export-Csv -Path $OutCsv -NoTypeInformation -Encoding UTF8
  Write-Output ("`nwrote {0} rows -> {1}" -f $tx.Count, $OutCsv)
}
