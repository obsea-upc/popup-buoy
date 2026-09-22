# Leaves one buoy ready to deploy: card written, clock set, state machine reset
# and the mission firmware flashed, in that order, with a check at the end that
# reads the buoy's own boot output to confirm it found what we just wrote.
#
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files hardware\tests\2026-09-24
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files ... -DryRun
#
# The order is not arbitrary. The card is written first, by a sketch that does
# nothing else, so a failed transfer never leaves the mission firmware running
# against a half-written file. The state byte is cleared after the card and
# before the firmware, because that is the only window where nothing else is
# touching the EEPROM. And the firmware goes last so the buoy is already
# complete the first time it boots as itself.
#
# What goes where on the card (from the firmware's own paths):
#   /conf.txt                       config.cpp
#   /AOP.txt                        satellite_spp.cpp
#   /sat_transm.csv                 sat_module.cpp
#   /progressFile.txt               satellite_tx.cpp   (reset to 1:0)
#   /PopUpBuoy_<id>/dataFile.txt    popup-buoy.ino, built from idBuoy
#   /LogFile.txt                    emptied, unless -KeepLog
#
# The manifest that comes with a packed data file is NOT copied: it is a ground
# file, the buoy has no use for it, and it must not be mistaken for one of these.

param(
  [Parameter(Mandatory=$true)][string]$Port,
  [Parameter(Mandatory=$true)][int]$Buoy,
  [Parameter(Mandatory=$true)][string]$Files,
  # 4 = DM, which is where a buoy about to be thrown in the water belongs.
  [int]$State = 4,
  [string]$Fqbn = 'esp32:esp32:esp32',
  [switch]$SkipRtc,
  [switch]$KeepLog,
  # Check everything and flash nothing. Use it the day before.
  [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$step = 0
function Say($msg)  { Write-Output ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $msg) }
function Head($msg) { $script:step++; Write-Output ""; Write-Output ("=== {0}. {1} ===" -f $script:step, $msg) }
function Die($msg)  { throw $msg }

# --- what has to be on the card, and where -----------------------------------
$plan = @(
  @{ local = 'conf.txt';        remote = '/conf.txt';        patch = $true  }
  @{ local = 'AOP.txt';         remote = '/AOP.txt';         patch = $false }
  @{ local = 'sat_transm.csv';  remote = '/sat_transm.csv';  patch = $false }
  @{ local = 'progressFile.txt'; remote = '/progressFile.txt'; patch = $false }
  @{ local = 'dataFile.txt';    remote = "/PopUpBuoy_$Buoy/dataFile.txt"; patch = $false }
)

Head "Comprobaciones previas"
if (-not (Test-Path $Files)) { Die "no existe la carpeta de ficheros: $Files" }
$Files = (Resolve-Path $Files).ProviderPath
Say "carpeta: $Files"
Say "boya $Buoy, puerto $Port, estado inicial $State"

foreach ($f in $plan) {
  $p = Join-Path $Files $f.local
  if (-not (Test-Path $p)) { Die "falta $($f.local) en $Files" }
  $f.path = $p
  $f.lines = @([IO.File]::ReadAllLines($p)).Count
  Say ("{0,-18} -> {1,-34} {2,6} lineas" -f $f.local, $f.remote, $f.lines)
}

# The data file is the one that is worth checking rather than trusting. Every
# frame is 46 hex characters after the "<row>:"; anything else means a truncated
# or half-edited file, and it would be found out four days into a deployment.
$data = @([IO.File]::ReadAllLines((Join-Path $Files 'dataFile.txt')))
$bad = 0; $row = 0; $rowsOk = $true
foreach ($l in $data) {
  if (-not $l) { continue }
  $row++
  $parts = $l -split ':', 2
  if ($parts.Count -ne 2 -or [int]$parts[0] -ne $row) { $rowsOk = $false }
  if ($parts[-1].Trim().Length -ne 46) { $bad++ }
}
if ($bad -gt 0) { Die "$bad tramas del dataFile no miden 46 caracteres" }
if (-not $rowsOk) { Die "la numeracion de filas del dataFile no es consecutiva desde 1" }
Say ("dataFile: {0} filas, todas de 46 caracteres y numeradas en orden" -f $row)

$man = Join-Path $Files 'dataFile.manifest.txt'
if (Test-Path $man) { Say "manifiesto presente (se queda en tierra, no va a la tarjeta)" }

# conf.txt: the two values that decide the campaign, said out loud so they are
# checked by a human and not only by this script.
$conf = @([IO.File]::ReadAllLines((Join-Path $Files 'conf.txt')))
$minElev = ($conf | Where-Object { $_ -match '^MinElev=' }) -replace '^MinElev=', ''
$reps    = ($conf | Where-Object { $_ -match '^NumberOfSendingEachLineFromData=' }) -replace '^.*=', ''
if (-not $minElev) { Die "conf.txt no trae MinElev" }
Say "conf.txt: MinElev=$minElev, repeticiones por linea=$reps"
if (-not ($conf | Where-Object { $_ -match '^idBuoy=' })) { Die "conf.txt no trae idBuoy" }
Say "conf.txt: idBuoy se sustituira por $Buoy"

foreach ($sk in @('tools\sd_put', 'tools\rtc_set', 'tools\eeprom_state', '.')) {
  if (-not (Test-Path (Join-Path $root $sk))) { Die "falta el sketch $sk" }
}
$ports = @((arduino-cli board list) -split "`r?`n" | Where-Object { $_ -match "^$Port\s" })
if ($ports.Count -eq 0) { Die "no veo $Port; conecta la boya o corrige el puerto" }
Say "puerto visible"

# --- helpers -----------------------------------------------------------------
function Flash($sketchRelative, $what) {
  Say "compilando y flasheando $what"
  if ($DryRun) {
    $out = & arduino-cli compile --fqbn $Fqbn (Join-Path $root $sketchRelative) 2>&1
    if ($LASTEXITCODE -ne 0) { Die "no compila ${what}: $out" }
    Say "  compila (DryRun: no se flashea)"
    return
  }
  $out = & arduino-cli compile --fqbn $Fqbn --upload -p $Port (Join-Path $root $sketchRelative) 2>&1
  if ($LASTEXITCODE -ne 0) { Die "fallo flasheando ${what}: $out" }
  Start-Sleep -Milliseconds 2500     # el puerto vuelve a aparecer tras el reset
  Say "  flasheado"
}

# Opens the port, sends the lines and returns everything the board said. DTR is
# left alone on purpose: on the CH9102 the EN line follows RTS only, and raising
# DTR resets the board in the middle of the conversation.
function Talk([string[]]$send, [int]$listenMs = 4000) {
  $sp = New-Object IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
  $sp.DtrEnable = $false; $sp.RtsEnable = $false
  $sp.ReadTimeout = 400
  $log = New-Object Collections.Generic.List[string]
  try {
    $sp.Open()
    Start-Sleep -Milliseconds 1800    # deja arrancar al sketch
    foreach ($s in $send) { $sp.WriteLine($s); Start-Sleep -Milliseconds 300 }
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.ElapsedMilliseconds -lt $listenMs) {
      try { $log.Add($sp.ReadLine().TrimEnd()) } catch { }
    }
  } finally { if ($sp.IsOpen) { $sp.Close() } }
  $log
}

# --- 2. the card -------------------------------------------------------------
Head "Tarjeta SD"
Flash 'tools\sd_put' 'sd_put (escritor de SD)'

$tmpConf = Join-Path $env:TEMP ("conf_boya{0}.txt" -f $Buoy)
($conf | ForEach-Object { $_ -replace '^idBuoy=.*', "idBuoy=$Buoy" }) |
  Set-Content -Path $tmpConf -Encoding ASCII
Say "conf.txt preparado para la boya $Buoy"

foreach ($f in $plan) {
  $src = if ($f.patch) { $tmpConf } else { $f.path }
  Say ("subiendo {0} ({1} lineas)" -f $f.remote, $f.lines)
  if ($DryRun) { Say "  (DryRun)"; continue }
  # sd_send throws on any failure, and ErrorActionPreference=Stop turns that into
  # the end of this script: a card left half written is not something to carry on
  # from. Checking $LASTEXITCODE here would be worse than useless - it is not set
  # by a PowerShell script, so it would still hold whatever arduino-cli last left.
  & (Join-Path $PSScriptRoot "sd_send.ps1") -Port $Port -LocalFile $src -RemotePath $f.remote
}

if (-not $KeepLog) {
  # Emptied rather than left to grow across campaigns: sd_put keeps the old one
  # as LogFile.txt.bak, so nothing is lost and the next analysis starts clean.
  $empty = Join-Path $env:TEMP 'empty_log.txt'
  Set-Content -Path $empty -Value $null -Encoding ASCII
  Say "vaciando /LogFile.txt (el anterior queda como .bak)"
  if (-not $DryRun) { & (Join-Path $PSScriptRoot 'sd_send.ps1') -Port $Port -LocalFile $empty -RemotePath '/LogFile.txt' }
}

# --- 3. the clock ------------------------------------------------------------
if ($SkipRtc) {
  Head "Reloj (omitido)"
} else {
  Head "Reloj"
  Flash 'tools\rtc_set' 'rtc_set'
  if (-not $DryRun) {
    & (Join-Path $PSScriptRoot "rtc_set.ps1") -Port $Port   # throws if the board does not answer
  } else { Say "(DryRun) se pondria el reloj a la hora UTC del PC" }
}

# --- 4. the state machine ----------------------------------------------------
Head "EEPROM"
Flash 'tools\eeprom_state' 'eeprom_state'
if ($DryRun) {
  Say "(DryRun) se enviaria STATE $State"
} else {
  $said = Talk @("STATE $State") 3000
  $ee = $said | Where-Object { $_ -match '^(OK )?EE ' } | Select-Object -Last 1
  if (-not $ee) { Die "la EEPROM no contesto" }
  Say "EEPROM ahora: $ee"
  $bytes = @(($ee -replace '^OK ', '' -replace '^EE ', '') -split '\s+')
  if ([int]$bytes[0] -ne $State) { Die "el estado quedo en $($bytes[0]) y pedi $State" }
  if ($bytes.Count -lt 8) { Die "esta EEPROM tiene $($bytes.Count) bytes; el firmware nuevo usa 8" }
  if ([int]$bytes[7] -ne 0) { Die "la marca de fichero agotado sigue puesta; la boya se iria a LOWPWR" }
  Say "estado $State, contadores a cero, marca de fichero agotado limpia"
}

# --- 5. the firmware ---------------------------------------------------------
Head "Firmware de mision"
Flash '.' 'popup-buoy'

# --- 6. read back what the buoy says -----------------------------------------
Head "Verificacion"
if ($DryRun) {
  Write-Output ""
  Say "DryRun terminado: todo lo que se puede comprobar sin escribir, comprobado."
  return
}

$boot = Talk @() 30000
$expected = $row       # lineas del dataFile que acabamos de subir
$seenRows = $boot | Where-Object { $_ -match 'MaxRowDataFile is : (\d+)' } | Select-Object -Last 1
$seenElev = $boot | Where-Object { $_ -match 'Minimum Elevation: ([\d\.]+)' } | Select-Object -Last 1
$seenState = $boot | Where-Object { $_ -match 'CurrentState of POP_UP_BUOY: (\w+)' } | Select-Object -First 1

$ok = $true
if ($seenRows -match 'MaxRowDataFile is : (\d+)') {
  if ([int]$Matches[1] -eq $expected) { Say "dataFile: la boya lee $($Matches[1]) filas, las que subimos" }
  else { Say "AVISO: la boya lee $($Matches[1]) filas y subimos $expected"; $ok = $false }
} else { Say "AVISO: no he visto el numero de filas en el arranque"; $ok = $false }

if ($seenElev -match 'Minimum Elevation: ([\d\.]+)') { Say "MinElev leido por la boya: $($Matches[1])" }
else { Say "AVISO: no he visto MinElev en el arranque"; $ok = $false }

if ($seenState) { Say ("estado al arrancar: " + ($seenState -replace '.*: ', '')) }

Write-Output ""
if ($ok) {
  Say "BOYA $Buoy LISTA."
} else {
  Say "BOYA ${Buoy}: revisa los avisos de arriba antes de desplegarla."
  exit 1
}
