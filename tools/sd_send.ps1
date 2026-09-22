# Sends a text file to the buoy's SD card, talking to the sd_put sketch.
#
#   .\tools\sd_send.ps1 -Port COM12 -LocalFile "hardware\tests\2026-09-14\dataFile.txt" `
#                       -RemotePath "/PopUpBuoy_2/dataFile.txt"
#
# Waits for the board's per-line acknowledgement rather than streaming, because
# the ESP32 stalls for tens of milliseconds whenever the SD card commits a block
# and anything sent during that window is lost.

param(
  [Parameter(Mandatory=$true)][string]$Port,
  [Parameter(Mandatory=$true)][string]$LocalFile,
  [Parameter(Mandatory=$true)][string]$RemotePath,
  [int]$Baud = 115200
)

if (-not (Test-Path $LocalFile)) { throw "no existe: $LocalFile" }
# .NET keeps its own working directory and does not follow Set-Location, so a
# relative path has to be resolved before it reaches ReadAllLines.
$LocalFile = (Resolve-Path $LocalFile).ProviderPath
# Opened with ReadWrite sharing so a file still open in Excel can be read. A read
# failure must stop the script: a .NET exception is not terminating in PowerShell,
# and with empty files allowed it used to carry on and blank the file on the card.
try {
  $fs = [IO.File]::Open($LocalFile, 'Open', 'Read', 'ReadWrite')
  $rd = New-Object IO.StreamReader($fs)
  $text = $rd.ReadToEnd()
  $rd.Close()
} catch { throw "no se puede leer $LocalFile : $($_.Exception.Message)" }
$lines = @($text -split "`r?`n" | Where-Object { $_.Length -gt 0 })
if ($lines.Count -eq 0) { Write-Output "fichero sin lineas: se dejara vacio en la tarjeta" }
Write-Output ("fichero local: {0} lineas, {1} bytes" -f $lines.Count, (Get-Item $LocalFile).Length)

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 20000
$sp.NewLine = "`n"
$sp.DtrEnable = $true; $sp.RtsEnable = $true
$sp.Open()
Start-Sleep -Milliseconds 300
$sp.DtrEnable = $false; $sp.RtsEnable = $false

# Drain the banner.
$t0 = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt 6) {
  try { $l = $sp.ReadLine().TrimEnd(); Write-Output "  < $l" } catch [TimeoutException] { break }
}

Write-Output "enviando ..."
$sp.WriteLine("PUT $RemotePath $($lines.Count)")

$ready = $false
$t0 = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt 10) {
  try {
    $l = $sp.ReadLine().TrimEnd()
    if ($l -eq "READY") { $ready = $true; break }
    if ($l -like "FAIL*") { $sp.Close(); throw "la placa respondio: $l" }
  } catch [TimeoutException] { }
}
if (-not $ready) { $sp.Close(); throw "la placa no respondio READY" }

$sw = [Diagnostics.Stopwatch]::StartNew()
$n = 0
foreach ($line in $lines) {
  $sp.WriteLine($line)
  # Block on the acknowledgement: this is the flow control.
  while ($true) {
    $ack = $sp.ReadLine().TrimEnd()
    if ($ack -eq "K") { break }
    if ($ack -like "FAIL*") { $sp.Close(); throw "la placa respondio: $ack" }
  }
  $n++
  if ($n % 250 -eq 0) {
    Write-Output ("  {0,5} / {1}   ({2:N0} lineas/s)" -f $n, $lines.Count, ($n / $sw.Elapsed.TotalSeconds))
  }
}

$result = $null
$t0 = Get-Date
while (((Get-Date) - $t0).TotalSeconds -lt 30) {
  try {
    $l = $sp.ReadLine().TrimEnd()
    if ($l -like "DONE*" -or $l -like "FAIL*") { $result = $l; break }
  } catch [TimeoutException] { }
}
$sp.Close()

Write-Output ""
if ($result -like "DONE*") {
  $p = $result.Split(' ')
  Write-Output ("OK: la placa escribio {0} lineas, {1} bytes en {2}" -f $p[1], $p[2], $RemotePath)
  if ([int]$p[1] -ne $lines.Count) { Write-Output ("AVISO: esperaba {0} lineas" -f $lines.Count) }
} else {
  Write-Output "FALLO: $result"
}
Write-Output ("tiempo: {0:N0} s" -f $sw.Elapsed.TotalSeconds)
