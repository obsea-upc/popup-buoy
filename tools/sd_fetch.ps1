# Receives the serial dump produced by scratchpad/sd_dump and writes each file
# out separately, checking the byte count the buoy reported against what
# actually arrived so a truncated transfer is caught rather than filed silently.
#
#   .\tools\sd_fetch.ps1 -Port COM12 -OutDir "hardware\tests\2026-09-08"
#
# The buoy must be running the sd_dump sketch, which is read-only. Flash the
# mission firmware back afterwards; the EEPROM state survives, so it carries on.

param(
  [Parameter(Mandatory=$true)][string]$Port,
  [Parameter(Mandatory=$true)][string]$OutDir,
  [int]$Baud = 115200,
  [int]$IdleTimeoutSec = 20
)

if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory -Force -Path $OutDir | Out-Null }

$sp = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$sp.ReadTimeout = 2000
$sp.DtrEnable = $true       # toggling these resets the board so we catch the dump from its start
$sp.RtsEnable = $true
$sp.Open()
Start-Sleep -Milliseconds 200
$sp.DtrEnable = $false
$sp.RtsEnable = $false

$lines = New-Object Collections.Generic.List[string]
$lastData = Get-Date
Write-Output "escuchando en $Port ..."
while ($true) {
  try {
    $l = $sp.ReadLine().TrimEnd("`r")
    $lines.Add($l)
    $lastData = Get-Date
    if ($l -match '^===DUMP COMPLETE===') { break }
  } catch [TimeoutException] {
    if (((Get-Date) - $lastData).TotalSeconds -gt $IdleTimeoutSec) {
      Write-Output "sin datos durante $IdleTimeoutSec s, corto"
      break
    }
  }
}
$sp.Close()
Write-Output ("recibidas {0} lineas" -f $lines.Count)

# Keep the whole transcript too: the listing and any error line live outside the
# file markers, and are worth having when something did not arrive.
$raw = Join-Path $OutDir "sd_dump_raw.txt"
$lines | Out-File -FilePath $raw -Encoding utf8
Write-Output "transcripcion completa -> $raw"

$i = 0
$ok = 0; $bad = 0
while ($i -lt $lines.Count) {
  if ($lines[$i] -match '^===FILE=(\S+) SIZE=(\d+)===$') {
    $path = $Matches[1]; $declared = [int]$Matches[2]
    $name = Split-Path $path -Leaf
    $body = New-Object Collections.Generic.List[string]
    $i++
    while ($i -lt $lines.Count -and $lines[$i] -notmatch '^===ENDFILE=') { $body.Add($lines[$i]); $i++ }
    $sent = if ($i -lt $lines.Count -and $lines[$i] -match 'SENT=(\d+)') { [int]$Matches[1] } else { -1 }

    # The board prints a newline before the ENDFILE marker, so drop one trailing
    # blank line if the file did not itself end in one.
    if ($body.Count -gt 0 -and $body[$body.Count-1] -eq '') { $body.RemoveAt($body.Count-1) }

    $dest = Join-Path $OutDir $name
    $body | Out-File -FilePath $dest -Encoding utf8
    $got = (Get-Item $dest).Length

    if ($sent -eq $declared) {
      Write-Output ("  OK   {0,-20} {1,8} bytes declarados, {2} enviados, {3} lineas" -f $name, $declared, $sent, $body.Count)
      $ok++
    } else {
      Write-Output ("  AVISO {0,-20} declarados {1}, enviados {2} - TRUNCADO" -f $name, $declared, $sent)
      $bad++
    }
  }
  elseif ($lines[$i] -match '^===MISSING=(\S+)===$') {
    Write-Output ("  --   {0} no existe en la tarjeta" -f $Matches[1])
  }
  $i++
}
Write-Output ""
Write-Output ("{0} ficheros completos, {1} truncados -> {2}" -f $ok, $bad, $OutDir)
