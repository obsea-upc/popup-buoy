# Turns a buoy's LogFile.txt into two CSVs: one row per transmission and one per
# wake. With them the reception rate has a real denominator - what the buoy actually
# sent - instead of a model of what it should have sent.
#
#   .\tools\parse_logfile.ps1 -Log "...\B5\LogFile.txt" -Buoy B5 -OutDir .\out
#
# Produces <OutDir>\<Buoy>_tx.csv and <OutDir>_<Buoy>_wake.csv.
#
# Log lines look like:
#   2026-09-17T10:31:07----State DM - TX;DATA;OK;MC;6;35;38;814;1000
#   2026-09-17T10:30:00----State DM - Argos coverage OK, sending data for 344 seconds...
# The TX fields are kind, status, satellite, elevation now, elevation max of the pass,
# seconds since the pass started, pass duration, power in mW.

param(
  [Parameter(Mandatory=$true)][string]$Log,
  [Parameter(Mandatory=$true)][string]$Buoy,
  [Parameter(Mandatory=$true)][string]$OutDir
)

New-Item -ItemType Directory -Force $OutDir | Out-Null
$fs = [IO.File]::Open((Resolve-Path $Log).ProviderPath, 'Open', 'Read', 'ReadWrite')
$rd = New-Object IO.StreamReader($fs)

$tx = New-Object Collections.Generic.List[object]
$wakes = New-Object Collections.Generic.List[object]
$cur = $null
$lineNo = 0
$sending = $null

while (-not $rd.EndOfStream) {
  $line = $rd.ReadLine(); $lineNo++
  if ($line.Length -lt 25) { continue }
  $t = $line.Substring(0, 19)
  $body = $line.Substring(19)
  $i = $body.IndexOf(' - ')
  if ($i -lt 0) { continue }
  $state = ($body.Substring(0, $i) -replace '-+State ', '').Trim()
  $msg = $body.Substring($i + 3).Trim()

  # a new wake starts at the INITIALIZATION banner
  if ($msg -like '*INITIALIZATION*') {
    if ($cur) { $wakes.Add($cur) }
    $cur = [pscustomobject]@{
      buoy = $Buoy; start = $t; state = $state; coverage = ''; dataSecs = 0; gpsSecs = 0
      passes = 0; gpsFix = ''; vin = ''; temp = ''; txData = 0; txGps = 0; txErr = 0
      endOfFile = 0; overlap = 0; sppErr = 0; noBattery = 0
    }
    continue
  }
  if (-not $cur) { continue }

  if ($msg -like 'Argos coverage OK*') {
    $cur.coverage = 'OK'
    if ($msg -match 'data for (\d+) seconds') { $cur.dataSecs = [int]$Matches[1] }
    if ($msg -match 'GPS for (\d+) seconds') { $cur.gpsSecs = [int]$Matches[1] }
  }
  elseif ($msg -like 'No ARGOS coverage*') { $cur.coverage = 'NONE' }
  elseif ($msg -match 'SPP session: (\d+) pass') { $cur.passes = [int]$Matches[1] }
  elseif ($msg -match 'GPS fix mode \(NMEA GSA\): (\d)') { $cur.gpsFix = $Matches[1] }
  elseif ($msg -like '*Failed to fix*' -or $msg -like '*GPS not fixed*') { $cur.gpsFix = 'fail' }
  elseif ($msg -match 'Vin \(up\): ([\d\.]+)') { $cur.vin = $Matches[1] }
  elseif ($msg -match 'RTC temp: ([\d\.\-]+)') { $cur.temp = $Matches[1] }
  elseif ($msg -like '*No battery pack detected*') { $cur.noBattery = 1 }
  elseif ($msg -like '*end of the file*') { $cur.endOfFile = 1 }
  elseif ($msg -like '*overlapping*') { $cur.overlap = 1 }
  elseif ($msg -like '*SPP*error*' -or $msg -like '*no satellite*') { $cur.sppErr++ }
  elseif ($msg -like 'Sending :*') { $sending = ($msg -split ':')[-1].Trim() }
  elseif ($msg -like 'TX;*') {
    $f = $msg -split ';'
    if ($f.Count -ge 9) {
      $tx.Add([pscustomobject]@{
        buoy = $Buoy; utc = $t; kind = $f[1]; status = $f[2]
        sat = $f[3]; elevNow = $f[4]; elevMax = $f[5]; sinceStart = $f[6]; passDur = $f[7]; power = $f[8]
        wake = $cur.start; payload = $sending
      })
      if ($f[1] -eq 'DATA') { $cur.txData++ } else { $cur.txGps++ }
      if ($f[2] -ne 'OK') { $cur.txErr++ }
    }
  }
}
if ($cur) { $wakes.Add($cur) }
$rd.Close()

$tx | Export-Csv -NoTypeInformation -Encoding utf8 (Join-Path $OutDir "${Buoy}_tx.csv")
$wakes | Export-Csv -NoTypeInformation -Encoding utf8 (Join-Path $OutDir "${Buoy}_wake.csv")
Write-Output ("{0}: {1} lineas, {2} despertares, {3} transmisiones ({4} datos, {5} GPS, {6} con error)" -f `
  $Buoy, $lineNo, $wakes.Count, $tx.Count, @($tx | Where-Object kind -eq 'DATA').Count,
  @($tx | Where-Object kind -eq 'GPS').Count, @($tx | Where-Object status -ne 'OK').Count)
