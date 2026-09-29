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
  [switch]$DryRun,
  # Transmit slot (0-5) written as TX_SLOT in conf.txt. -1 takes it from the fleet
  # table below, so buoys deployed together never share one.
  [int]$TxSlot = -1,
  # Which data file goes on the card when the folder holds one per module
  # (dataFile_kim.txt / dataFile_arribada.txt, from tools/fountain_image.py). Empty
  # takes it from the fleet table below.
  [ValidateSet('', 'kim', 'arribada')][string]$Module = ''
)

# Satellite module per buoy, 1 Oct 2026: Arribadas (401 MHz, LDA2, 27 dBm) on 1, 2 and
# 4 - buoy 4 swaps its KIM1, which went deaf in the cold, for one; KIM1 on 5 and 7.
$moduleTable = @{ 1 = 'arribada'; 2 = 'arribada'; 4 = 'arribada' }
if (-not $Module) { $Module = if ($moduleTable.ContainsKey($Buoy)) { $moduleTable[$Buoy] } else { 'kim' } }

# Fleet slot table (from 1 Oct 2026, see "Transmit slots" in conf.h): six slots of
# 5 s in the 30 s cycle. The buoys that fly together get one each; 3 and 6 share
# the last one, so do not deploy them together without giving one of them another.
$slotTable = @{ 1 = 0; 2 = 1; 4 = 2; 5 = 3; 7 = 4; 3 = 5; 6 = 5 }
if ($TxSlot -lt 0) {
  if (-not $slotTable.ContainsKey($Buoy)) { throw "la boya $Buoy no tiene hueco en la tabla; pasa -TxSlot 0..5" }
  $TxSlot = $slotTable[$Buoy]
}
if ($TxSlot -gt 5) { throw "TxSlot tiene que estar entre 0 y 5" }

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
# A folder with one data file per module (fountain files, from 1 Oct 2026): take the
# one of this buoy's module. KIM1 frames are 46 hex, Arribada 48.
$frameHex = 46
if (Test-Path (Join-Path $Files "dataFile_$Module.txt")) {
  ($plan | Where-Object { $_.local -eq 'dataFile.txt' }).local = "dataFile_$Module.txt"
  if ($Module -eq 'arribada') { $frameHex = 48 }
}

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
$dataLocal = ($plan | Where-Object { $_.remote -like '*/dataFile.txt' }).local
$data = @([IO.File]::ReadAllLines((Join-Path $Files $dataLocal)))
$bad = 0; $row = 0; $rowsOk = $true
foreach ($l in $data) {
  if (-not $l) { continue }
  $row++
  $parts = $l -split ':', 2
  if ($parts.Count -ne 2 -or [int]$parts[0] -ne $row) { $rowsOk = $false }
  if ($parts[-1].Trim().Length -ne $frameHex) { $bad++ }
}
if ($bad -gt 0) { Die "$bad tramas de $dataLocal no miden $frameHex caracteres" }
if (-not $rowsOk) { Die "la numeracion de filas de $dataLocal no es consecutiva desde 1" }
Say ("{0} (modulo {1}): {2} filas, todas de {3} caracteres y numeradas en orden" -f $dataLocal, $Module, $row, $frameHex)

foreach ($man in @('dataFile.manifest.txt', "manifest_$Module.json")) {
  if (Test-Path (Join-Path $Files $man)) { Say "$man presente (se queda en tierra, no va a la tarjeta)" }
}

# conf.txt: the two values that decide the campaign, said out loud so they are
# checked by a human and not only by this script.
$conf = @([IO.File]::ReadAllLines((Join-Path $Files 'conf.txt')))
$minElev = ($conf | Where-Object { $_ -match '^MinElev=' }) -replace '^MinElev=', ''
$reps    = ($conf | Where-Object { $_ -match '^NumberOfSendingEachLineFromData=' }) -replace '^.*=', ''
if (-not $minElev) { Die "conf.txt no trae MinElev" }
Say "conf.txt: MinElev=$minElev, repeticiones por linea=$reps"
if (-not ($conf | Where-Object { $_ -match '^idBuoy=' })) { Die "conf.txt no trae idBuoy" }
Say "conf.txt: idBuoy se sustituira por $Buoy"
if (-not ($conf | Where-Object { $_ -match '^TX_SLOT=' })) { Die "conf.txt no trae TX_SLOT (pon TX_SLOT=X)" }
Say "conf.txt: TX_SLOT se sustituira por $TxSlot (emite en el segundo $($TxSlot * 5 + 2.5) de cada medio minuto)"

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
function Talk([string[]]$send, [int]$listenMs = 4000, [switch]$Reset) {
  $sp = New-Object IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
  $sp.DtrEnable = $false; $sp.RtsEnable = $false
  $sp.ReadTimeout = 400
  $log = New-Object Collections.Generic.List[string]
  try {
    $sp.Open()
    if ($Reset) {
      # Pulse EN (it follows RTS on the CH9102) so the whole boot is heard. The
      # port is otherwise opened seconds after the upload's own reset, by which
      # time the configuration lines have already gone by unread - which is why
      # the 22 Sep dry run reported "no he visto MinElev".
      $sp.RtsEnable = $true; Start-Sleep -Milliseconds 100; $sp.RtsEnable = $false
    } else {
      Start-Sleep -Milliseconds 1800    # deja arrancar al sketch
    }
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
($conf | ForEach-Object { $_ -replace '^idBuoy=.*', "idBuoy=$Buoy" -replace '^TX_SLOT=.*', "TX_SLOT=$TxSlot" }) |
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

Say "reiniciando la boya y escuchando su arranque (40 s)"
$boot = Talk @() 40000 -Reset
$expected = $row       # lineas del dataFile que acabamos de subir
$seenRows = $boot | Where-Object { $_ -match 'MaxRowDataFile is : (\d+)' } | Select-Object -Last 1
$seenElev = $boot | Where-Object { $_ -match 'Minimum Elevation: ([\d\.]+)' } | Select-Object -Last 1
$seenState = $boot | Where-Object { $_ -match 'CurrentState of POP_UP_BUOY: (\w+)' } | Select-Object -First 1
$seenBoard = $boot | Where-Object { $_ -match '^Board V\d' } | Select-Object -First 1
$seenSat   = $boot | Where-Object { $_ -match 'Satellite module detection ----> (\S.*)' } | Select-Object -Last 1
$seenList  = $boot | Where-Object { $_ -match 'SAT module .*(confirmed by SD list|not listed|MISMATCH)' } | Select-Object -Last 1
$seenCfg   = $boot | Where-Object { $_ -match 'Configuration_ERR|AFMT_ERR|SAT MODULE DEAD|NOT DETECTED' }
$seenBatt  = $boot | Where-Object { $_ -match 'Vin \((up|MAX17048)\)' } | Select-Object -First 1
$seenDumb  = $boot | Where-Object { $_ -match 'Brownout|Guru Meditation|Card Mount Failed' }

$ok = $true
if ($seenBoard) { Say ("placa: " + ($seenBoard -replace '^Board ', '')) }
else { Say "AVISO: no he visto la deteccion de placa (firmware viejo?)"; $ok = $false }

if ($seenSat -match '----> (\S.*)') {
  if ($Matches[1] -match 'NONE') { Say "AVISO: no se detecta ningun modulo de satelite"; $ok = $false }
  else { Say "modulo de satelite: $($Matches[1])" }
} else { Say "AVISO: no he visto la deteccion del modulo de satelite"; $ok = $false }
if ($seenList -match 'not listed|MISMATCH') { Say "AVISO: el modulo no cuadra con sat_transm.csv: $seenList"; $ok = $false }
elseif ($seenList) { Say "sat_transm.csv: modulo confirmado" }
foreach ($l in $seenCfg)  { Say "AVISO: $($l -replace '^Writing in LogFile.txt ---State \w+ - ', '')"; $ok = $false }
foreach ($l in $seenDumb) { Say "AVISO: $l"; $ok = $false }
if ($seenBatt) { Say ("bateria: " + ($seenBatt -replace '^.*Vin', 'Vin')) }
if ($seenRows -match 'MaxRowDataFile is : (\d+)') {
  if ([int]$Matches[1] -eq $expected) { Say "dataFile: la boya lee $($Matches[1]) filas, las que subimos" }
  else { Say "AVISO: la boya lee $($Matches[1]) filas y subimos $expected"; $ok = $false }
} else { Say "AVISO: no he visto el numero de filas en el arranque"; $ok = $false }

if ($seenElev -match 'Minimum Elevation: ([\d\.]+)') {
  if ([double]$Matches[1] -eq [double]$minElev) { Say "MinElev leido por la boya: $($Matches[1])" }
  else { Say "AVISO: la boya lee MinElev $($Matches[1]) y conf.txt dice $minElev"; $ok = $false }
} else { Say "AVISO: no he visto MinElev en el arranque"; $ok = $false }

$seenSlot = $boot | Where-Object { $_ -match 'TX slot: (\d+)' } | Select-Object -Last 1
if ($seenSlot -match 'TX slot: (\d+)') {
  if ([int]$Matches[1] -eq $TxSlot) { Say "hueco de transmision leido por la boya: $($Matches[1])" }
  else { Say "AVISO: la boya lee el hueco $($Matches[1]) y se pidio $TxSlot"; $ok = $false }
} else { Say "AVISO: no he visto TX slot en el arranque (firmware sin huecos?)"; $ok = $false }

if ($seenState -match ': (\w+)') {
  $st = $Matches[1]
  $want = @{ 0='CONFIG'; 1='DEPLOY'; 2='SEABED'; 4='DM'; 5='LOWPWR'; 6='FRM' }[$State]
  if ($st -eq $want) { Say "estado al arrancar: $st" }
  else { Say "AVISO: arranca en $st y se pidio $want"; $ok = $false }
} else { Say "AVISO: no he visto el estado al arrancar"; $ok = $false }

Write-Output ""
if ($ok) {
  Say "BOYA $Buoy LISTA."
} else {
  Say "BOYA ${Buoy}: revisa los avisos de arriba antes de desplegarla."
  exit 1
}
