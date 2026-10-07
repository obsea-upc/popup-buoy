# Leaves one buoy ready to deploy: card written, clock set, state machine reset
# and the mission firmware flashed, in that order, with a check at the end that
# reads the buoy's own boot output to confirm it found what we just wrote.
#
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files hardware\tests\2026-10-01
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files ... -Only rtc      # just the clock (+ firmware back)
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files ... -Only verify   # just listen to the boot
#   .\tools\prepare_buoy.ps1 -Port COM15 -Buoy 5 -Files ... -DryRun
#
# Built for campaign days, where it has to be quick (1 Oct 2026: ~5.5 min a buoy
# when nothing failed, hours when something did). Now ~1 min:
#   - one helper sketch (tools/buoy_prep) does card, clock and EEPROM in a single
#     serial session at 921600, lines acknowledged in blocks. The old way flashed
#     three sketches and reset the board for every file, waiting 20 s each time;
#   - sketches are compiled once and the binaries reused until a source changes
#     (cache in %LOCALAPPDATA%\popup-buoy-build), so each buoy is upload-only;
#   - the AOP is fetched from CLS only if the one in the folder is not from today;
#   - it fails fast and says why: port missing or busy (a serial monitor open),
#     no SD, no RTC, satellite module silent after three tries;
#   - the boot check stops as soon as it has seen everything.
#
# The order is not arbitrary. The card is written first, so a failed transfer never
# leaves the mission firmware running against a half-written file. The state byte
# is cleared after the card and before the firmware, the only window where nothing
# else touches the EEPROM. And the firmware goes last so the buoy is already
# complete the first time it boots as itself. That is also why any of sd/rtc/ee
# brings the firmware back on afterwards: the helper sketch replaces it.
#
# What goes where on the card (from the firmware's own paths):
#   /conf.txt                       config.cpp
#   /AOP.txt                        satellite_spp.cpp
#   /sat_transm.csv                 sat_module.cpp
#   /progressFile.txt               satellite_tx.cpp   (reset to 1:0)
#   /PopUpBuoy_<id>/dataFile.txt    popup-buoy.ino, built from idBuoy
#   /LogFile.txt                    emptied, unless -KeepLog
# The manifest that comes with a packed data file stays on the ground.

param(
  [Parameter(Mandatory=$true)][string]$Port,
  [Parameter(Mandatory=$true)][int]$Buoy,
  [Parameter(Mandatory=$true)][string]$Files,
  # 4 = DM, which is where a buoy about to be thrown in the water belongs.
  [int]$State = 4,
  [string]$Fqbn = 'esp32:esp32:esp32',
  [switch]$SkipRtc,
  [switch]$KeepLog,
  # Do not fetch the AOP even if the folder's one is old.
  [switch]$SkipAop,
  # Check everything and flash nothing.
  [switch]$DryRun,
  # Only some steps: sd, rtc, ee, fw, verify. Default: all of them.
  [ValidateSet('sd', 'rtc', 'ee', 'fw', 'verify')][string[]]$Only = @(),
  # Transmit slot (0-5) written as TX_SLOT in conf.txt. -1 takes it from the fleet table.
  [int]$TxSlot = -1,
  # Which data file goes on the card when the folder holds one per module. Empty
  # takes it from the fleet table below.
  [ValidateSet('', 'kim', 'arribada')][string]$Module = ''
)

# Satellite module per buoy. From the 8 Oct 2026 campaign the KIM1 is parked and
# every buoy flies an Arribada; -Module kim still works for a bench test.
$moduleTable = @{}
if (-not $Module) { $Module = if ($moduleTable.ContainsKey($Buoy)) { $moduleTable[$Buoy] } else { 'arribada' } }

# Fleet slot table (see "Transmit slots" in conf.h): six slots of 5 s in the 30 s
# cycle. 3 and 6 share the last one: do not deploy them together.
$slotTable = @{ 1 = 0; 2 = 1; 4 = 2; 5 = 3; 7 = 4; 3 = 5; 6 = 5 }
if ($TxSlot -lt 0) {
  if (-not $slotTable.ContainsKey($Buoy)) { throw "la boya $Buoy no tiene hueco en la tabla; pasa -TxSlot 0..5" }
  $TxSlot = $slotTable[$Buoy]
}
if ($TxSlot -gt 5) { throw "TxSlot tiene que estar entre 0 y 5" }

$steps = @(if ($Only.Count) { $Only } else { 'sd', 'rtc', 'ee', 'fw', 'verify' })   # @(): one step must stay a list
if ($SkipRtc) { $steps = @($steps | Where-Object { $_ -ne 'rtc' }) }
$needPrep = [bool]($steps | Where-Object { $_ -in 'sd', 'rtc', 'ee' })
if ($needPrep -and 'fw' -notin $steps) { $steps += 'fw' }        # the helper replaced the firmware
if ('fw' -in $steps -and 'verify' -notin $steps) { $steps += 'verify' }

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$BAUD_PREP = 921600
$BLOCK = 64                      # must match BLOCK in tools/buoy_prep
$step = 0
$total = [Diagnostics.Stopwatch]::StartNew()
$lap = [Diagnostics.Stopwatch]::StartNew()
# Write-Host, not Write-Output: Say is also called inside functions that return a
# value, and Write-Output would end up in that value instead of on the screen.
function Say($msg)  { Write-Host ("[{0}] {1}" -f (Get-Date -Format 'HH:mm:ss'), $msg) }
function Head($msg) {
  if ($script:step -gt 0) { Say ("  ({0:N0} s)" -f $script:lap.Elapsed.TotalSeconds) }
  $script:lap.Restart(); $script:step++; Write-Output ""; Write-Output ("=== {0}. {1} ===" -f $script:step, $msg)
}
function Die($msg)  { Write-Output ""; Say "PARADO: $msg"; Say ("(tras {0:N0} s)" -f $total.Elapsed.TotalSeconds); exit 1 }

# --- the port: say at once if it is missing or busy ---------------------------
function Wait-Port([int]$ms = 8000) {
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.ElapsedMilliseconds -lt $ms) {
    if ([IO.Ports.SerialPort]::GetPortNames() -contains $Port) { return $true }
    Start-Sleep -Milliseconds 200
  }
  return $false
}
function Assert-Port {
  if (-not (Wait-Port 3000)) {
    $others = ([IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join ', '
    Die "no veo $Port (puertos ahora: $others). Cable suelto, otra boya en otro COM, o la placa sin alimentacion"
  }
  $sp = New-Object IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
  $sp.DtrEnable = $false; $sp.RtsEnable = $false
  try { $sp.Open() }
  catch [UnauthorizedAccessException] { Die "$Port esta OCUPADO por otro programa: cierra el monitor serie (VS Code, Arduino IDE, PuTTY)" }
  catch { Die "no puedo abrir ${Port}: $($_.Exception.Message)" }
  finally { if ($sp.IsOpen) { $sp.Close() } }
}

# --- compile once, upload many -------------------------------------------------
$cacheRoot = Join-Path $env:LOCALAPPDATA 'popup-buoy-build'
function Get-Build($sketchRelative, $name) {
  $dir = Join-Path $root $sketchRelative
  $out = Join-Path $cacheRoot $name
  $stamp = Join-Path $out '.stamp'
  $srcs = @(Get-ChildItem -Path $dir -File | Where-Object { $_.Extension -in '.ino', '.cpp', '.h', '.c' })
  $newest = ($srcs | Measure-Object -Property LastWriteTimeUtc -Maximum).Maximum
  $key = "$Fqbn|$newest|$($srcs.Count)"
  if ((Test-Path $stamp) -and ((Get-Content $stamp -Raw).Trim() -eq $key)) {
    Say "  $name ya compilado (sin cambios en el codigo)"
    return @{ dir = $dir; out = $out }
  }
  Say "  compilando $name (una vez; las siguientes boyas lo reutilizan)"
  New-Item -ItemType Directory -Force -Path $out | Out-Null
  $ErrorActionPreference = 'Continue'      # arduino-cli talks on stderr; that is not a failure
  $log = & arduino-cli compile --fqbn $Fqbn --output-dir $out $dir 2>&1
  if ($LASTEXITCODE -ne 0) { Die "no compila ${name}: $($log | Select-Object -Last 15 | Out-String)" }
  Set-Content -Path $stamp -Value $key -Encoding ASCII
  return @{ dir = $dir; out = $out }
}
function Upload($build, $name) {
  if ($DryRun) { Say "  (DryRun) no se carga $name"; return }
  for ($try = 1; $try -le 2; $try++) {
    if (-not (Wait-Port 8000)) { Die "$Port ha desaparecido antes de cargar $name" }
    $ErrorActionPreference = 'Continue'
    $log = & arduino-cli upload --fqbn $Fqbn -p $Port --input-dir $build.out $build.dir 2>&1
    if ($LASTEXITCODE -eq 0) { Say "  $name cargado"; [void](Wait-Port 8000); Start-Sleep -Milliseconds 300; return }
    $why = ($log | Select-String 'error|Error|denied|exist' | Select-Object -First 2 | Out-String).Trim()
    if ($why -match 'denied|Acceso') { Die "$Port OCUPADO al cargar ${name}: cierra el monitor serie" }
    Say "  fallo cargando $name (intento $try): $why"
    Start-Sleep -Milliseconds 1500
  }
  Die "no he podido cargar $name en $Port"
}

# --- one serial session with buoy_prep ------------------------------------------
$script:sp = $null
function Open-Prep {
  $script:sp = New-Object IO.Ports.SerialPort($Port, $BAUD_PREP, 'None', 8, 'One')
  $sp.DtrEnable = $false; $sp.RtsEnable = $false
  $sp.NewLine = "`n"; $sp.ReadTimeout = 200; $sp.WriteBufferSize = 65536
  try { $sp.Open() } catch [UnauthorizedAccessException] { Die "$Port OCUPADO: cierra el monitor serie" }
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.ElapsedMilliseconds -lt 6000) {
    $sp.WriteLine("PING")
    $l = Read-Until '^(PREP|FAIL)' 500
    if ($l -match '^PREP') { return $l }
  }
  Die "la placa no contesta al programa de preparacion (buoy_prep)"
}
function Read-Until([string]$pattern, [int]$ms) {
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.ElapsedMilliseconds -lt $ms) {
    try { $l = $script:sp.ReadLine().TrimEnd() } catch [TimeoutException] { continue }
    if ($l -match $pattern) { return $l }
  }
  return $null
}
function Put([string]$local, [string]$remote) {
  $lines = @([IO.File]::ReadAllLines($local) | Where-Object { $_.Length -gt 0 })
  $t = [Diagnostics.Stopwatch]::StartNew()
  $sp.WriteLine("PUT $remote $($lines.Count)")
  $r = Read-Until '^(READY|FAIL)' 8000
  if ($r -ne 'READY') { Die "la tarjeta no acepta ${remote}: $r" }
  for ($i = 0; $i -lt $lines.Count; $i += $BLOCK) {
    $chunk = $lines[$i..([Math]::Min($i + $BLOCK, $lines.Count) - 1)]
    $sp.Write((($chunk -join "`n") + "`n"))
    $want = [Math]::Min($i + $BLOCK, $lines.Count)
    $k = Read-Until '^(K \d+|FAIL)' 20000
    if ($k -notmatch "^K $want$") { Die "escribiendo $remote se corto en la linea ~$($i + 1): $k" }
  }
  $d = Read-Until '^(DONE|FAIL)' 30000
  if ($d -notmatch '^DONE (\d+) (\d+)') { Die "la tarjeta no confirmo ${remote}: $d" }
  if ([int]$Matches[1] -ne $lines.Count) { Die "$remote quedo con $($Matches[1]) lineas y mande $($lines.Count)" }
  Say ("  {0,-30} {1,5} lineas  {2,7} bytes  {3,5:N1} s" -f $remote, $Matches[1], $Matches[2], $t.Elapsed.TotalSeconds)
}

# =============================================================================
Head "Comprobaciones previas"
if (-not (Test-Path $Files)) { Die "no existe la carpeta de ficheros: $Files" }
$Files = (Resolve-Path $Files).ProviderPath
Say "carpeta: $Files"
Say "boya $Buoy, puerto $Port, modulo $Module, hueco $TxSlot, estado $State, pasos: $($steps -join ' ')"

# The AOP of the day: fetched once, reused for every buoy prepared that day.
$aopPath = Join-Path $Files 'AOP.txt'
$aopToday = (Test-Path $aopPath) -and ((Get-Item $aopPath).LastWriteTimeUtc.Date -eq [DateTime]::UtcNow.Date)
if ('sd' -in $steps) {
  if ($aopToday) { Say "AOP.txt de hoy ($((Get-Item $aopPath).LastWriteTimeUtc.ToString('HH:mm')) UTC): no se descarga" }
  elseif ($SkipAop) { Say "AVISO: AOP.txt no es de hoy y -SkipAop: se sube el que hay" }
  else {
    Say "AOP.txt no es de hoy: descargandolo de CLS"
    $bash = @('C:\Program Files\Git\bin\bash.exe', 'C:\Program Files\Git\usr\bin\bash.exe') | Where-Object { Test-Path $_ } | Select-Object -First 1
    if (-not $bash) { Die "no encuentro el bash de Git para get_aop.sh; bajalo a mano o usa -SkipAop" }
    $posix = $Files -replace '\\', '/'
    if ($posix -match '^([A-Za-z]):(.*)$') { $posix = '/' + $Matches[1].ToLower() + $Matches[2] }
    Push-Location $root
    $ErrorActionPreference = 'Continue'
    try { $o = & $bash 'tools/get_aop.sh' $posix 2>&1; $code = $LASTEXITCODE } finally { Pop-Location; $ErrorActionPreference = 'Stop' }
    if ($code -ne 0) { Die "no he podido bajar el AOP: $($o | Out-String)" }
    Say "  $($o | Select-Object -Last 1)"
  }
}

$plan = @(
  @{ local = 'conf.txt';         remote = '/conf.txt';         patch = $true  }
  @{ local = 'AOP.txt';          remote = '/AOP.txt';          patch = $false }
  @{ local = 'sat_transm.csv';   remote = '/sat_transm.csv';   patch = $false }
  @{ local = 'progressFile.txt'; remote = '/progressFile.txt'; patch = $false }
  @{ local = 'dataFile.txt';     remote = "/PopUpBuoy_$Buoy/dataFile.txt"; patch = $false }
)
$frameHex = 46
if (Test-Path (Join-Path $Files "dataFile_$Module.txt")) {
  ($plan | Where-Object { $_.local -eq 'dataFile.txt' }).local = "dataFile_$Module.txt"
  if ($Module -eq 'arribada') { $frameHex = 48 }
}
foreach ($f in $plan) {
  $p = Join-Path $Files $f.local
  if (-not (Test-Path $p)) { Die "falta $($f.local) en $Files" }
  $f.path = $p
}

# The data file is worth checking rather than trusting: every frame is $frameHex
# hex characters after "<row>:", rows numbered from 1.
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
Say ("{0}: {1} filas de {2} caracteres, numeradas en orden" -f $dataLocal, $row, $frameHex)

$conf = @([IO.File]::ReadAllLines((Join-Path $Files 'conf.txt')))
$minElev = ($conf | Where-Object { $_ -match '^MinElev=' }) -replace '^MinElev=', ''
if (-not $minElev) { Die "conf.txt no trae MinElev" }
if (-not ($conf | Where-Object { $_ -match '^idBuoy=' })) { Die "conf.txt no trae idBuoy" }
if (-not ($conf | Where-Object { $_ -match '^TX_SLOT=' })) { Die "conf.txt no trae TX_SLOT (pon TX_SLOT=X)" }
Say "conf.txt: MinElev=$minElev, idBuoy -> $Buoy, TX_SLOT -> $TxSlot (emite en el segundo $($TxSlot * 5 + 2.5))"

if (-not $DryRun) { Assert-Port; Say "$Port libre" }

# =============================================================================
if ($needPrep) {
  Head "Tarjeta, reloj y EEPROM (buoy_prep)"
  $prep = Get-Build 'tools\buoy_prep' 'buoy_prep'
  Upload $prep 'buoy_prep'
  if (-not $DryRun) {
    $hello = Open-Prep
    try {
      if ($hello -match 'lost=1') { Say "AVISO: el RTC habia perdido la alimentacion (revisa la pila)" }
      if ('sd' -in $steps) {
        if ($hello -notmatch 'sd=1') { Die "la placa no ve la tarjeta SD (bien metida? rele de SD?)" }
        $tmpConf = Join-Path $env:TEMP ("conf_boya{0}.txt" -f $Buoy)
        ($conf | ForEach-Object { $_ -replace '^idBuoy=.*', "idBuoy=$Buoy" -replace '^TX_SLOT=.*', "TX_SLOT=$TxSlot" }) |
          Set-Content -Path $tmpConf -Encoding ASCII
        foreach ($f in $plan) { Put ($(if ($f.patch) { $tmpConf } else { $f.path })) $f.remote }
        if (-not $KeepLog) {
          $empty = Join-Path $env:TEMP 'empty_log.txt'; Set-Content -Path $empty -Value $null -Encoding ASCII
          Put $empty '/LogFile.txt'        # the old one stays as LogFile.txt.bak
        }
      }
      if ('rtc' -in $steps) {
        if ($hello -notmatch 'rtc=1') { Die "el RTC no contesta por I2C" }
        $sp.WriteLine("RTC GET"); $before = Read-Until '^(NOW|FAIL)' 2000
        Start-Sleep -Milliseconds (1000 - [DateTimeOffset]::UtcNow.Millisecond)   # on the PC's second edge
        $u = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
        $sp.WriteLine("RTC SET $u"); $ok = Read-Until '^(OK|FAIL)' 2000
        if ($ok -notmatch '^OK') { Die "no se pudo poner la hora: $ok" }
        Start-Sleep -Milliseconds 1100
        $sp.WriteLine("RTC GET"); $after = Read-Until '^(NOW|FAIL)' 2000
        $pc = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
        $offB = if ($before -match '^NOW (\d+)') { [long]$Matches[1] - $u } else { $null }
        $offA = if ($after -match '^NOW (\d+)') { [long]$Matches[1] - $pc } else { 99 }
        if ([Math]::Abs($offA) -gt 1) { Die "el RTC sigue desviado $offA s tras ponerlo en hora" }
        Say "  reloj: estaba a $offB s, ahora a $offA s de la hora UTC del PC"
      }
      if ('ee' -in $steps) {
        $sp.WriteLine("EE STATE $State"); $ee = Read-Until '^(OK EE|FAIL)' 2000
        if ($ee -notmatch '^OK EE') { Die "la EEPROM no contesto: $ee" }
        $bytes = @(($ee -replace '^OK EE ', '') -split '\s+')
        if ([int]$bytes[0] -ne $State -or [int]$bytes[7] -ne 0) { Die "la EEPROM quedo en '$ee'" }
        Say "  EEPROM: estado $State, contadores a cero, marca de fichero agotado limpia"
      }
    } finally { if ($sp -and $sp.IsOpen) { $sp.Close() } }
  }
}

if ('fw' -in $steps) {
  Head "Firmware de mision"
  $fw = Get-Build '.' 'popup-buoy'
  Upload $fw 'popup-buoy'
}

if ($DryRun) { Head "DryRun"; Say "Todo lo que se puede comprobar sin escribir, comprobado."; exit 0 }
if ('verify' -notin $steps) { Say ("Hecho en {0:N0} s." -f $total.Elapsed.TotalSeconds); exit 0 }

# =============================================================================
Head "Verificacion (arranque de la boya)"
$boot = New-Object Collections.Generic.List[string]
$v = New-Object IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
$v.DtrEnable = $false; $v.RtsEnable = $false; $v.ReadTimeout = 200; $v.NewLine = "`n"
try { $v.Open() } catch [UnauthorizedAccessException] { Die "$Port OCUPADO: cierra el monitor serie" }
$notDetected = 0
try {
  # Pulse EN (it follows RTS on the CH9102) so the whole boot is heard.
  $v.RtsEnable = $true; Start-Sleep -Milliseconds 100; $v.RtsEnable = $false
  $sw = [Diagnostics.Stopwatch]::StartNew()
  while ($sw.ElapsedMilliseconds -lt 45000) {
    try { $l = $v.ReadLine().TrimEnd() } catch [TimeoutException] { continue }
    $boot.Add($l)
    if ($l -match 'NOT DETECTED') { $notDetected++; if ($notDetected -ge 3) { break } }
    # The row count is the last thing printed before the first session: stop there.
    if ($l -match 'MaxRowDataFile is : \d+') { Start-Sleep -Milliseconds 300; break }
  }
} finally { if ($v.IsOpen) { $v.Close() } }

$expected = $row
$seenRows  = $boot | Where-Object { $_ -match 'MaxRowDataFile is : (\d+)' } | Select-Object -Last 1
$seenElev  = $boot | Where-Object { $_ -match 'Minimum Elevation: ([\d\.]+)' } | Select-Object -Last 1
$seenState = $boot | Where-Object { $_ -match 'CurrentState of POP_UP_BUOY: (\w+)' } | Select-Object -First 1
$seenBoard = $boot | Where-Object { $_ -match '^Board V\d' } | Select-Object -First 1
$seenSat   = $boot | Where-Object { $_ -match 'Satellite module detection ----> (\S.*)' } | Select-Object -Last 1
$seenList  = $boot | Where-Object { $_ -match 'SAT module .*(confirmed by SD list|not listed|MISMATCH)' } | Select-Object -Last 1
$seenRconf = $boot | Where-Object { $_ -match 'RCONF (OK|MISMATCH|ERR)' } | Select-Object -Last 1
$seenCfg   = $boot | Where-Object { $_ -match 'Configuration_ERR|AFMT_ERR|SAT MODULE DEAD' }
$seenBatt  = $boot | Where-Object { $_ -match 'Vin \((up|MAX17048)\)' } | Select-Object -First 1
$seenRtc   = $boot | Where-Object { $_ -match 'RTC reports lost power' } | Select-Object -First 1
$seenSlot  = $boot | Where-Object { $_ -match 'TX slot: (\d+)' } | Select-Object -Last 1
$seenDumb  = $boot | Where-Object { $_ -match 'Brownout|Guru Meditation|Card Mount Failed' }

$ok = $true
function Warn($m) { Say "AVISO: $m"; $script:ok = $false }
if ($seenBoard) { Say ("placa: " + ($seenBoard -replace '^Board ', '')) } else { Warn "no he visto la deteccion de placa" }
# The board each buoy had last time, so a V2 misread as a V1 (which would leave its
# GPS unpowered) does not go by unnoticed. A deliberate hardware swap shows up here too.
$boardsFile = Join-Path $cacheRoot 'boards.txt'
if ($seenBoard -match '^Board (V\d)') {
  $nowBoard = $Matches[1]
  $known = @{}
  if (Test-Path $boardsFile) { Get-Content $boardsFile | ForEach-Object { $k, $b = $_ -split '=', 2; if ($b) { $known[$k] = $b } } }
  if ($known.ContainsKey("$Buoy") -and $known["$Buoy"] -ne $nowBoard) {
    Warn "la boya $Buoy era $($known["$Buoy"]) la ultima vez y ahora se detecta $nowBoard. Si has cambiado la placa, ok; si no, la deteccion falla"
  }
  $known["$Buoy"] = $nowBoard
  New-Item -ItemType Directory -Force -Path $cacheRoot | Out-Null
  $known.GetEnumerator() | Sort-Object Name | ForEach-Object { "$($_.Name)=$($_.Value)" } | Set-Content $boardsFile -Encoding ASCII
}
if ($notDetected -ge 3) {
  Warn "el modulo de satelite NO contesta (3 intentos). Revisa bateria conectada, conector y cable interno; luego -Only verify"
} elseif ($seenSat -match '----> (\S.*)') { Say "modulo de satelite: $($Matches[1])" }
else { Warn "no he visto la deteccion del modulo de satelite" }
if ($seenList -match 'not listed|MISMATCH') { Warn "el modulo no cuadra con sat_transm.csv: $seenList" }
elseif ($seenList) { Say "sat_transm.csv: modulo confirmado" }
if ($seenRconf) { Say ($seenRconf -replace '^.* - ', '') }
foreach ($l in $seenCfg)  { Warn ($l -replace '^Writing in LogFile.txt ---State \w+ - ', '') }
foreach ($l in $seenDumb) { Warn $l }
if ($seenRtc) { Warn "la boya dice que el RTC perdio la alimentacion (OSF): pila del RTC" }
if ($seenBatt) { Say ("bateria: " + ($seenBatt -replace '^.*Vin', 'Vin')) }
if ($seenRows -match 'MaxRowDataFile is : (\d+)') {
  if ([int]$Matches[1] -eq $expected) { Say "dataFile: la boya lee $($Matches[1]) filas, las que subimos" }
  else { Warn "la boya lee $($Matches[1]) filas y subimos $expected" }
} elseif ($notDetected -lt 3) { Warn "no he visto el numero de filas en el arranque" }
if ($seenElev -match 'Minimum Elevation: ([\d\.]+)') {
  if ([double]$Matches[1] -eq [double]$minElev) { Say "MinElev leido por la boya: $($Matches[1])" }
  else { Warn "la boya lee MinElev $($Matches[1]) y conf.txt dice $minElev" }
} else { Warn "no he visto MinElev en el arranque" }
if ($seenSlot -match 'TX slot: (\d+)') {
  if ([int]$Matches[1] -eq $TxSlot) { Say "hueco de transmision leido por la boya: $($Matches[1])" }
  else { Warn "la boya lee el hueco $($Matches[1]) y se pidio $TxSlot" }
} else { Warn "no he visto TX slot en el arranque" }
if ($seenState -match ': (\w+)') {
  $want = @{ 0='CONFIG'; 1='DEPLOY'; 2='SEABED'; 4='DM'; 5='LOWPWR'; 6='FRM' }[$State]
  if ($Matches[1] -eq $want) { Say "estado al arrancar: $($Matches[1])" } else { Warn "arranca en $($Matches[1]) y se pidio $want" }
} else { Warn "no he visto el estado al arrancar" }

Say ("  ({0:N0} s)" -f $lap.Elapsed.TotalSeconds)
Write-Output ""
if ($ok) { Say ("BOYA $Buoy LISTA en {0:N0} s." -f $total.Elapsed.TotalSeconds) }
else { Say ("BOYA ${Buoy}: revisa los avisos de arriba ({0:N0} s)." -f $total.Elapsed.TotalSeconds); exit 1 }
