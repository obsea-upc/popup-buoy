# Summarises a CLS retrieve-bulk dump for a multi-buoy run: what each buoy got,
# pass by pass, so the buoys can be compared on the same satellite at the same time.
#
#   ./tools/cls.sh retrieve-bulk '{...,"deviceRefs":[...]}' > cls.json
#   .\tools\cls_summary.ps1 -Json cls.json -Buoys "B1=217459,B2=294848,B3=217089" `
#       -DataFile "hardware\tests\2026-09-17\PopUpBuoy_X\dataFile.txt"
#
# - Deduplicates by deviceMsgUid, keeping the record with metadata: CLS returns
#   KIM messages twice, one with GPS decoded and one without.
# - A pass is one satellite's receptions with no gap over -PassGapMin minutes.
# - With -DataFile, every data frame is looked up in the file; a frame that is
#   not there arrived with flipped bits, which CLS delivers without complaint.

param(
  [Parameter(Mandatory=$true)][string]$Json,
  [Parameter(Mandatory=$true)][string]$Buoys,        # "B1=217459,B5=216573"
  [string]$DataFile,
  [int]$PassGapMin = 10,
  # For the what-if by elevation floor (needs -DataFile too):
  [string]$Aop,
  [double]$Lat = 41.2236,
  [double]$Lon = 1.7363,
  [string]$Start,                                    # "09:18" or "09:18,B2=10:05" (UTC, date of the data)
  [int[]]$Floors = @(5, 15, 25, 30, 40),
  # Extra sections (need -Aop): where in a session receptions fall, and buoy
  # against buoy on the same pass.
  [switch]$Sessions,
  # Write one CSV row per received data frame: buoy, time, resolved data-file row,
  # bit errors. That row number is how far through the file the buoy had got.
  [string]$ExportRows
)

$names = [ordered]@{}
foreach ($kv in $Buoys.Split(',')) { $p = $kv.Split('='); $names[$p[1].Trim()] = $p[0].Trim() }

$j = Get-Content $Json -Raw | ConvertFrom-Json
if ($j.pageInfo.hasNextPage) { Write-Output "AVISO: hay mas paginas, el resumen esta incompleto" }

$msgs = $j.contents |
  Group-Object deviceMsgUid |
  ForEach-Object { $_.Group | Sort-Object { [int][bool]$_.kineisMetadata }, { [int][bool]$_.gpsLocLat } -Descending | Select-Object -First 1 } |
  Where-Object { $names.Contains($_.deviceRef) } |
  ForEach-Object {
    [pscustomobject]@{
      buoy  = $names[$_.deviceRef]
      t     = [datetime]::Parse($_.msgDatetime, [Globalization.CultureInfo]::InvariantCulture)
      type  = $_.msgType
      sat   = $_.kineisMetadata.sat
      level = [double]$_.kineisMetadata.level
      snr   = [double]$_.kineisMetadata.snr
      raw   = $_.rawData
      lat   = $_.gpsLocLat
      lon   = $_.gpsLocLon
    }
  } | Sort-Object t

# Data frames: a KIM frame carries an 8-hex header, then the 46-hex data line.
$lines = $null
if ($DataFile) {
  $fs = [IO.File]::Open((Resolve-Path $DataFile).ProviderPath, 'Open', 'Read', 'ReadWrite')
  $rd = New-Object IO.StreamReader($fs); $txt = $rd.ReadToEnd(); $rd.Close()
  $lines = New-Object 'System.Collections.Generic.HashSet[string]'
  foreach ($l in ($txt -split "`r?`n")) { $h = ($l -split ':')[-1].Trim().ToLower(); if ($h.Length -ge 24) { [void]$lines.Add($h.Substring(0,24)) } }
}
# Which messages are data frames, and where the data line starts in rawData:
#   KIM1:     8-hex header + 46-hex line (248 bits). A frame whose header took bit
#             errors can come back typed "operation-mo-event" instead of SENSOR.
#   Arribada: no header, the line padded to 48 hex; its GPS message is 32 hex.
function Get-DataOffset($m) {
  if (-not $m.raw) { return -1 }
  if ($m.type -eq 'KIM1-UPC-SENSOR') { return 8 }
  if ($m.type -eq 'operation-mo-event' -and $m.raw.Length -ge 54) { return 8 }
  if ($m.type -eq 'ARGOS #1' -and $m.raw.Length -eq 48) { return 0 }
  return -1
}
function Test-IsGps($m) {
  return ($m.type -like '*GPS*') -or ($m.type -eq 'ARGOS #1' -and $m.raw -and $m.raw.Length -eq 32)
}
function Test-Frame($m) {
  $o = Get-DataOffset $m
  if (-not $lines -or $o -lt 0) { return $null }
  return $lines.Contains($m.raw.Substring($o,24).ToLower())
}

Write-Output ""
Write-Output ("Mensajes unicos {0}, de {1:HH:mm} a {2:HH:mm} UTC" -f $msgs.Count, ($msgs | Select-Object -First 1).t, ($msgs | Select-Object -Last 1).t)

# ---- per buoy
Write-Output ""
Write-Output "POR BOYA"
$rows = foreach ($ref in $names.Keys) {
  $b = $names[$ref]; $m = @($msgs | Where-Object buoy -eq $b)
  $frames = @($m | ForEach-Object { Test-Frame $_ } | Where-Object { $_ -ne $null })
  [pscustomobject]@{
    boya = $b; ref = $ref; recibidos = $m.Count
    GPS = @($m | Where-Object { Test-IsGps $_ }).Count
    primero = if ($m) { $m[0].t.ToString('HH:mm') } else { '-' }
    ultimo  = if ($m) { $m[-1].t.ToString('HH:mm') } else { '-' }
    nivel = if ($m) { [math]::Round(($m | Measure-Object level -Average).Average,1) } else { $null }
    snr   = if ($m) { [math]::Round(($m | Measure-Object snr -Average).Average,1) } else { $null }
    tramas_ok  = if ($lines) { @($frames | Where-Object { $_ }).Count } else { '' }
    tramas_mal = if ($lines) { @($frames | Where-Object { -not $_ }).Count } else { '' }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200

# ---- per pass
Write-Output "POR PASE (recibidos por boya; nivel medio del pase)"
$passes = @()
foreach ($g in ($msgs | Where-Object sat | Group-Object sat)) {
  $cur = $null
  foreach ($m in ($g.Group | Sort-Object t)) {
    if ($cur -and ($m.t - $cur.end).TotalMinutes -le $PassGapMin) { $cur.end = $m.t; $cur.msgs += $m }
    else { $cur = [pscustomobject]@{ sat = $g.Name; start = $m.t; end = $m.t; msgs = @($m) }; $passes += $cur }
  }
}
$table = foreach ($p in ($passes | Sort-Object start)) {
  $o = [ordered]@{ sat = $p.sat; inicio = $p.start.ToString('HH:mm'); fin = $p.end.ToString('HH:mm') }
  foreach ($b in $names.Values) { $o[$b] = @($p.msgs | Where-Object buoy -eq $b).Count }
  $o['nivel'] = [math]::Round(($p.msgs | Measure-Object level -Average).Average,1)
  [pscustomobject]$o
}
$table | Format-Table -AutoSize | Out-String -Width 200

# ---- estimated send/receive performance, from the data-file row each frame carries
# The buoy sends the data file one row per transmission, in order, so the highest
# row heard is a lower bound on the rows it has sent by then. Each frame is matched
# to its nearest row by bit distance (so frames with flipped bits still count).
#
# The file repeats one 145-row image block byte for byte, so a frame names its row
# only modulo 145. Resolved against the fleet: every buoy starts at row 1 with the
# same plan, so none can be further on than the furthest buoy heard around that
# time. Pass 1 takes the first candidate at or after the buoy's previous row, which
# is right for any buoy heard often; pass 2 takes the highest candidate that is not
# past the fleet's furthest row within -FleetWindowMin, plus a margin. A buoy that
# is more than a whole block behind the fleet would be placed a block too far on.
#
# Rows sent after the last reception are outside both numerator and denominator.
# GPS messages are not in this figure.
if ($lines) {
  $pop = @(0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4)
  $rowsHex = @($txt -split "`r?`n" | Where-Object { $_.Trim() } | ForEach-Object { ($_ -split ':')[-1].Trim().ToLower() })
  $rowsNib = foreach ($r in $rowsHex) { ,([int[]]($r.ToCharArray() | ForEach-Object { [Convert]::ToInt32([string]$_,16) })) }
  $maxBits = 24          # of 184: random data sits near 92, so anything over this is not a row
  $fleetWindowMin = 15
  $margin = 15
  # All candidate rows (1-based) at the minimum bit distance.
  function Get-Candidates([string]$d) {
    $dn = [int[]]($d.ToCharArray() | ForEach-Object { [Convert]::ToInt32([string]$_,16) })
    $best = 999; $cands = @()
    for ($i = 0; $i -lt $rowsNib.Count; $i++) {
      $rn = $rowsNib[$i]; $n = 0
      for ($k = 0; $k -lt 46 -and $n -le $best; $k++) { $n += $pop[$dn[$k] -bxor $rn[$k]] }
      if ($n -lt $best) { $best = $n; $cands = @($i + 1) } elseif ($n -eq $best) { $cands += ($i + 1) }
    }
    return [pscustomobject]@{ bits = $best; rows = $cands }
  }

  $frames = @($msgs | Where-Object { (Get-DataOffset $_) -ge 0 } | ForEach-Object {
    $c = Get-Candidates $_.raw.Substring((Get-DataOffset $_), 46).ToLower()
    if ($c.bits -le $maxBits) { [pscustomobject]@{ buoy = $_.buoy; t = $_.t; bits = $c.bits; rows = $c.rows; row = 0 } }
  })

  # Pass 1: first candidate at or after the buoy's previous row.
  foreach ($g in ($frames | Group-Object buoy)) {
    $last = 1
    foreach ($f in ($g.Group | Sort-Object t)) {
      $pick = $f.rows | Where-Object { $_ -ge $last } | Select-Object -First 1
      if (-not $pick) { $pick = $f.rows[-1] }
      $f.row = $pick; $last = $pick
    }
  }
  # Pass 2 needs a row prediction, which only the pass model can give; it is set
  # up further down and applied in Resolve-Rows. Without -Aop, pass 1 stands: it is
  # right while the buoys are heard often enough to stay inside one 145-row block,
  # which stops being true after a few hours.
  function Resolve-Rows($predictRow) {
    foreach ($g in ($frames | Group-Object buoy)) {
      $last = 1
      foreach ($f in ($g.Group | Sort-Object t)) {
        if ($predictRow) {
          # The file repeats every 145 rows, so pick the candidate closest to where
          # this buoy should be by now. Robust as long as the prediction is out by
          # less than half a block - 72 rows, over half an hour of solid transmitting.
          $exp = & $predictRow $f.buoy $f.t
          $f.row = ($f.rows | Sort-Object { [math]::Abs($_ - $exp) } | Select-Object -First 1)
        } else {
          $near = @($frames | Where-Object { $_.buoy -ne $f.buoy -and [math]::Abs(($_.t - $f.t).TotalMinutes) -le $fleetWindowMin })
          $cap = if ($near) { ($near | Measure-Object row -Maximum).Maximum + $margin } else { [int]::MaxValue }
          $ok = @($f.rows | Where-Object { $_ -ge $last -and $_ -le $cap })
          $f.row = if ($ok) { $ok[-1] } else { ($f.rows | Where-Object { $_ -ge $last } | Select-Object -First 1) }
          if (-not $f.row) { $f.row = $f.rows[-1] }
        }
        $last = $f.row
      }
    }
  }

  # ---- pass model: elevation over time, and from it the row each buoy should be on
  $predictRow = $null
  if ($Aop) {
    . (Join-Path $PSScriptRoot 'sat_elev.ps1')
    $sats = @(Read-Aop $Aop)
    $kineis = @($sats | Where-Object { $_.Name -match '^\d[A-Z]$' })
    $day = ($msgs | Select-Object -First 1).t.Date
    $end = ($msgs | Select-Object -Last 1).t
    $startOf = @{}; $defStart = $null
    if ($Start) {
      foreach ($p in $Start.Split(',')) {
        if ($p -match '^(\w+)=(\d+:\d+)$') { $startOf[$Matches[1]] = $day + [timespan]$Matches[2] }
        else { $defStart = $day + [timespan]$p.Trim() }
      }
    }
    $step = 10
    $cache = @{}
    function Get-MaxElev($list, $key, [datetime]$t) {
      $k = "$key|$($t.Ticks)"
      if (-not $cache.ContainsKey($k)) {
        $u = [datetime]::SpecifyKind($t, 'Utc'); $m = -90.0
        foreach ($s in $list) { $e = [SatElev]::Elevation($s, $u, $Lat, $Lon); if ($e -gt $m) { $m = $e } }
        $cache[$k] = $m
      }
      return $cache[$k]
    }
    function Get-Start($b) {
      if ($startOf.ContainsKey($b)) { return $startOf[$b] }
      if ($defStart) { return $defStart }
      return ($frames | Where-Object buoy -eq $b | Sort-Object t | Select-Object -First 1).t
    }
    function Get-SatList($b) {
      # The Arribada transmits outside the legacy Argos band, so only Kineis hears it.
      if (@($msgs | Where-Object { $_.buoy -eq $b -and $_.type -eq 'ARGOS #1' }).Count -gt 0) { return @($kineis, 'K') }
      return @($sats, 'A')
    }

    # A buoy at MinElev 5 transmits one row every 30 s through each pass, so counting
    # those slots gives the row it should be on at any moment - which is what tells
    # one repetition of the 145-row block from the next.
    #
    # Lead is 0 on purpose although the firmware wakes 120 s early: the GPS fix eats
    # that time, so transmissions effectively start when the pass opens. Measured on
    # 17 Sep - by 18:34 the monotone chain of heard rows reached 634 (B1) and 631
    # (B5) and this model gives 623, while assuming a 150 s lead gave 779.
    $clocks = @{}
    function Get-RowClock($b) {
      if ($clocks.ContainsKey($b)) { return $clocks[$b] }
      # Always every satellite, including for the Arribada: the planner wakes for
      # every pass in the AOP and transmits, whether or not that satellite's band
      # can hear it. Only the what-if windows below care who is listening.
      $list = $sats; $key = 'A'
      $t0 = Get-Start $b
      $rows = New-Object 'System.Collections.Generic.List[int]'
      $sent = 0.0; $lead = 0
      for ($t = $t0; $t -le $end.AddMinutes(5); $t = $t.AddSeconds($step)) {
        # awake if a pass is open now or opens within the lead time
        $awake = ((Get-MaxElev $list $key $t) -ge 5) -or ((Get-MaxElev $list $key $t.AddSeconds($lead)) -ge 5)
        if ($awake) { $sent += $step / 30.0 }
        $rows.Add([int][math]::Floor($sent) + 1)
      }
      $clocks[$b] = [pscustomobject]@{ t0 = $t0; rows = $rows }
      return $clocks[$b]
    }
    $predictRow = {
      param($b, [datetime]$t)
      $c = Get-RowClock $b
      $i = [int][math]::Floor(($t - $c.t0).TotalSeconds / $step)
      if ($i -lt 0) { return 1 }
      if ($i -ge $c.rows.Count) { return $c.rows[$c.rows.Count - 1] }
      return $c.rows[$i]
    }
    Resolve-Rows $predictRow
  }

  if ($ExportRows) {
    $frames | Sort-Object buoy, t |
      Select-Object buoy, @{n='utc';e={$_.t.ToString('yyyy-MM-ddTHH:mm:ss')}}, row, bits |
      Export-Csv -NoTypeInformation -Encoding utf8 $ExportRows
    Write-Output ("filas exportadas a {0} ({1} tramas)" -f $ExportRows, $frames.Count)
  }

  Write-Output "RENDIMIENTO ESTIMADO (solo tramas de datos)"
  $perf = foreach ($ref in $names.Keys) {
    $b = $names[$ref]
    $mine = @($frames | Where-Object buoy -eq $b | Sort-Object t)
    if (-not $mine) { [pscustomobject]@{ boya = $b; filas_enviadas = 0; filas_recibidas = 0; limpias = 0; recepcion = '-'; copias = 0; hasta = '-' }; continue }
    $got = @{}
    foreach ($f in $mine) {
      if (-not $got.ContainsKey($f.row)) { $got[$f.row] = @() }
      $got[$f.row] += $f.bits
    }
    $frames_b = $mine
    # Rows sent by the time of the last reception: the pass model when there is one
    # (it counts the rows of sessions never heard at all), else the highest row heard.
    $sent = ($got.Keys | Measure-Object -Maximum).Maximum
    if ($predictRow) { $sent = [math]::Max($sent, (& $predictRow $b $mine[-1].t)) }
    $clean = @($got.Keys | Where-Object { ($got[$_] | Measure-Object -Minimum).Minimum -eq 0 }).Count
    [pscustomobject]@{
      boya = $b
      filas_enviadas = $sent
      filas_recibidas = $got.Count
      limpias = $clean
      recepcion = '{0:P0}' -f ($got.Count / $sent)
      recep_limpia = '{0:P0}' -f ($clean / $sent)
      copias = $frames_b.Count
      hasta = $frames_b[-1].t.ToString('HH:mm')
    }
  }
  $perf | Format-Table -AutoSize | Out-String -Width 200

  # ---- what-if by elevation floor
  # A run at MinElev 5 transmits through every pass, so the part of it sent while
  # the best satellite stood above X is what a run at MinElev X would have sent.
  # Sent = seconds with some satellite above X / 30 s; received = distinct rows
  # first heard at such a moment. Buoys at a lower floor also get a 120 s early
  # wake, so the 5 deg line runs a little under the row-based figure above - that
  # gap is the check on the method. The Arribada is only heard by Kineis
  # satellites, so its windows use those alone. GPS messages are not counted.
  if ($Aop) {
    Write-Output ("SI EL SUELO FUERA X GRADOS (estimado desde esta prueba a 5, hasta {0:HH:mm} UTC)" -f $end)
    $what = foreach ($ref in $names.Keys) {
      $b = $names[$ref]
      $mine = @($frames | Where-Object buoy -eq $b)
      if (-not $mine) { continue }
      $sl = Get-SatList $b; $list = $sl[0]; $key = $sl[1]
      $t0 = Get-Start $b
      # first reception time per row
      $firstHeard = @{}
      foreach ($f in ($mine | Sort-Object t)) { if (-not $firstHeard.ContainsKey($f.row)) { $firstHeard[$f.row] = $f.t } }
      foreach ($x in $Floors) {
        $secs = 0
        for ($t = $t0; $t -lt $end; $t = $t.AddSeconds($step)) { if ((Get-MaxElev $list $key $t) -ge $x) { $secs += $step } }
        $tx = [math]::Round($secs / 30)
        $rx = @($firstHeard.Values | Where-Object { $_ -ge $t0 -and (Get-MaxElev $list $key ([datetime]::new(($_.Ticks / ($step*1e7)) * ($step*1e7)))) -ge $x }).Count
        [pscustomobject]@{
          boya = $b; suelo = $x; minutos_tx = [math]::Round($secs / 60)
          enviadas = $tx; recibidas = $rx
          recepcion = if ($tx) { '{0:P0}' -f ($rx / $tx) } else { '-' }
          recibidas_por_hora = [math]::Round($rx / (($end - $t0).TotalHours), 1)
        }
      }
    }
    $what | Format-Table -AutoSize | Out-String -Width 200

    # ---- where in a session receptions fall, and buoy against buoy per pass
    # Two separate questions that get confused: does a session start badly (the
    # transmitter warming up), and is one buoy simply worse than another (its
    # antenna). The first needs the slot index inside the session, the second needs
    # the same pass for both buoys.
    if ($Sessions) {
      # Sessions = spans where a pass is open, merged when closer than the planner's
      # merge gap. Slots are 30 s from the start of the span.
      function Get-Sessions($list, $key, [datetime]$t0, [int]$mergeGapS = 150) {
        $spans = @(); $cur = $null
        for ($t = $t0; $t -lt $end; $t = $t.AddSeconds($step)) {
          if ((Get-MaxElev $list $key $t) -ge 5) {
            if ($cur -and ($t - $cur.fin).TotalSeconds -le $mergeGapS) { $cur.fin = $t }
            else { $cur = [pscustomobject]@{ ini = $t; fin = $t }; $spans += $cur }
          }
        }
        return $spans
      }

      Write-Output "RECEPCION SEGUN LA POSICION DEL MENSAJE EN LA SESION"
      $prof = foreach ($ref in $names.Keys) {
        $b = $names[$ref]
        $mine = @($frames | Where-Object buoy -eq $b)
        if ($mine.Count -lt 10) { continue }
        $sl = Get-SatList $b
        $spans = Get-Sessions $sl[0] $sl[1] (Get-Start $b)
        $heard = @{}; foreach ($f in $mine) { $heard[$f.row] = $true }
        $clock = Get-RowClock $b
        $bins = [ordered]@{}
        foreach ($sp in $spans) {
          $slots = [int][math]::Floor(($sp.fin - $sp.ini).TotalSeconds / 30) + 1
          $row0 = & $predictRow $b $sp.ini
          for ($i = 1; $i -le $slots; $i++) {
            $bin = if ($i -le 3) { '1-3' } elseif ($i -le 6) { '4-6' } elseif ($i -le 10) { '7-10' } else { '11+' }
            if (-not $bins.Contains($bin)) { $bins[$bin] = @(0, 0) }
            $v = $bins[$bin]; $v[0]++
            if ($heard.ContainsKey($row0 + $i - 1)) { $v[1]++ }
            $bins[$bin] = $v
          }
        }
        foreach ($k in $bins.Keys) {
          [pscustomobject]@{ boya = $b; mensajes_de_la_sesion = $k; ranuras = $bins[$k][0]; recibidas = $bins[$k][1]
            recepcion = '{0:P0}' -f ($bins[$k][1] / $bins[$k][0]) }
        }
      }
      $prof | Format-Table -AutoSize | Out-String -Width 200

      # The first messages of a session are also the lowest-elevation ones, so the
      # profile above cannot tell a cold transmitter from plain geometry. Crossing
      # the two separates them: at the same elevation, early slots against late.
      Write-Output "ELEVACION x POSICION EN LA SESION (recibidas/ranuras)"
      $cross = foreach ($ref in $names.Keys) {
        $b = $names[$ref]
        $mine = @($frames | Where-Object buoy -eq $b)
        if ($mine.Count -lt 30) { continue }
        $sl = Get-SatList $b
        $spans = Get-Sessions $sl[0] $sl[1] (Get-Start $b)
        $heard = @{}; foreach ($f in $mine) { $heard[$f.row] = $true }
        $cells = [ordered]@{}
        foreach ($sp in $spans) {
          $slots = [int][math]::Floor(($sp.fin - $sp.ini).TotalSeconds / 30) + 1
          $row0 = & $predictRow $b $sp.ini
          for ($i = 1; $i -le $slots; $i++) {
            $t = $sp.ini.AddSeconds(30 * ($i - 1))
            $e = Get-MaxElev $sl[0] $sl[1] ([datetime]::new([math]::Floor($t.Ticks / ($step * 1e7)) * [long]($step * 1e7)))
            $eb = if ($e -lt 15) { '05-15' } elseif ($e -lt 30) { '15-30' } else { '30+  ' }
            $ib = if ($i -le 6) { 'primeros 6' } else { 'resto' }
            $k = "$eb|$ib"
            if (-not $cells.Contains($k)) { $cells[$k] = @(0, 0) }
            $v = $cells[$k]; $v[0]++; if ($heard.ContainsKey($row0 + $i - 1)) { $v[1]++ }
            $cells[$k] = $v
          }
        }
        foreach ($k in ($cells.Keys | Sort-Object)) {
          $p = $k.Split('|')
          [pscustomobject]@{ boya = $b; elevacion = $p[0]; posicion = $p[1]; ranuras = $cells[$k][0]
            recibidas = $cells[$k][1]; recepcion = '{0:P0}' -f ($cells[$k][1] / $cells[$k][0]) }
        }
      }
      $cross | Format-Table -AutoSize | Out-String -Width 200

      Write-Output "BOYA CONTRA BOYA EN EL MISMO PASE (nivel medio dBm / recibidos)"
      $pairRows = foreach ($p in ($passes | Sort-Object start)) {
        $o = [ordered]@{ sat = $p.sat; inicio = $p.start.ToString('HH:mm') }
        foreach ($b in $names.Values) {
          $m = @($p.msgs | Where-Object buoy -eq $b)
          $o[$b] = if ($m) { '{0:N1} / {1}' -f ($m | Measure-Object level -Average).Average, $m.Count } else { '-' }
        }
        [pscustomobject]$o
      }
      $pairRows | Format-Table -AutoSize | Out-String -Width 200
    }
  }
}

# ---- positions
$gps = @($msgs | Where-Object { $_.lat })
if ($gps) {
  Write-Output "POSICIONES DECODIFICADAS POR CLS"
  $gps | ForEach-Object { "  {0} {1:HH:mm:ss}  {2}  {3}  {4}" -f $_.buoy, $_.t, $_.lat, $_.lon, $_.sat }
}
