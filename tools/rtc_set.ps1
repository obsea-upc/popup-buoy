# Sets the buoy's RTC to UTC from this PC's clock, talking to the rtc_set sketch.
#
#   .\tools\rtc_set.ps1 -Port COM10            # set, then read back
#   .\tools\rtc_set.ps1 -Port COM10 -ReadOnly  # only compare
#
# The DS3231 restarts its seconds when they are written, so the command is sent
# on the PC's second boundary. The PC clock is only as good as Windows time sync;
# check that first if the buoy has to be right to the second.

param(
  [Parameter(Mandatory=$true)][string]$Port,
  [switch]$ReadOnly,
  [int]$Baud = 115200
)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 3000
$sp.NewLine = "`n"
$sp.DtrEnable = $true; $sp.RtsEnable = $true
$sp.Open()
Start-Sleep -Milliseconds 300
$sp.DtrEnable = $false; $sp.RtsEnable = $false

# Drain the banner until the sketch says it is listening.
$t0 = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt 8) {
  try {
    $l = $sp.ReadLine().TrimEnd()
    if ($l -match '^(NOW|FAIL|AVISO|===)') { Write-Output "  < $l" }
    if ($l -like "esperando*" -or $l -like "FAIL*") { break }
  } catch [TimeoutException] { break }
}

function Get-Rtc {
  $sp.WriteLine("GET")
  while ($true) {
    $l = $sp.ReadLine().TrimEnd()
    if ($l -like "NOW *" -or $l -like "FAIL*") { return $l }
  }
}

function Show-Offset([string]$line) {
  $pc = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
  $p = $line.Split(' ')
  Write-Output ("RTC {0}   PC {1:yyyy-MM-ddTHH:mm:ss} UTC   diferencia RTC-PC: {2} s" -f `
    $p[2], [DateTime]::UtcNow, ([long]$p[1] - $pc))
}

$before = Get-Rtc
if ($before -like "FAIL*") { $sp.Close(); throw "la placa respondio: $before" }
Write-Output "antes:"
Show-Offset $before

if (-not $ReadOnly) {
  # Wait for the PC's next whole second and send that second.
  $now = [DateTimeOffset]::UtcNow
  Start-Sleep -Milliseconds (1000 - $now.Millisecond)
  $u = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
  $sp.WriteLine("SET $u")
  $ok = $null
  while ($true) { $l = $sp.ReadLine().TrimEnd(); if ($l -like "OK *" -or $l -like "FAIL*") { $ok = $l; break } }
  if ($ok -like "FAIL*") { $sp.Close(); throw "la placa respondio: $ok" }
  Start-Sleep -Seconds 2
  Write-Output "despues:"
  Show-Offset (Get-Rtc)
}
$sp.Close()
