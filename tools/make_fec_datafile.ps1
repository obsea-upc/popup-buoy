# Rewrites a seabed data file so every frame carries two bytes of error
# correction, and repeats it N times.
#
#   .\tools\make_fec_datafile.ps1 -Source <dataFile.txt> -Out <dataFile_fec.txt> -Reps 50
#
# The frame keeps the 23 bytes the buoy has always sent, so nothing in the
# firmware changes. What changes is how they are divided:
#
#     before   [seg 1B][line 1B][ data 21B                    ]
#     after    [seg 1B][line 1B][ data 19B            ][fec 2B]
#
# The two parity bytes are a shortened BCH(184,168) over the whole 21 bytes,
# header included - a frame that lands in the wrong segment is worse than a
# frame that is lost - and they repair any one or two flipped bits. See
# tools\bch.ps1 for the codec and its self-test.
#
# The image content is taken from the source file rather than re-encoded, so the
# JPEG bytes are identical to the ones already flying; only the chunking moves.
# Costs about 9.5 % more frames per image and, on the corruption measured in the
# 17-21 Sep 2026 campaign, takes the share of received frames that are usable
# from 59 % to about 82 %.

param(
  [Parameter(Mandatory=$true)][string]$Source,
  [Parameter(Mandatory=$true)][string]$Out,
  [int]$Reps = 50,
  [int]$Chunk = 38,          # hex characters of payload per frame (19 bytes)
  [int]$Segments = 22
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'bch.ps1')

# --- read the source, shared-mode so an open Excel or editor cannot stop us ---
try {
  $fs = [IO.File]::Open((Resolve-Path $Source), 'Open', 'Read', 'ReadWrite')
  $rd = New-Object IO.StreamReader($fs)
  $text = $rd.ReadToEnd()
  $rd.Close(); $fs.Close()
} catch { throw "no se puede leer $Source : $($_.Exception.Message)" }

$lines = @($text -split "`r?`n" | Where-Object { $_.Length -gt 0 })
Write-Output ("origen: {0} lineas" -f $lines.Count)

# --- split off the sentinels and take exactly one repetition ------------------
$prefix = New-Object Collections.Generic.List[string]
$frames = New-Object Collections.Generic.List[string]
foreach ($l in $lines) {
  $hex = if ($l -match ':') { $l.Substring($l.IndexOf(':') + 1) } else { $l }
  if ($hex.Length -ne 46) { throw "linea con payload de $($hex.Length) caracteres, esperaba 46: $l" }
  if ($hex -match '^fa(fa|fb|fc)') { $prefix.Add($hex); continue }
  $frames.Add($hex)
}
Write-Output ("centinelas: {0}, tramas de imagen: {1}" -f $prefix.Count, $frames.Count)

# --- rebuild each segment's stream from one repetition ------------------------
# Walk until every segment has been seen and then stops growing: the file holds
# the same image over and over, and one pass of it is all we need.
$stream = @{}
$seen = @{}
foreach ($hex in $frames) {
  $seg  = [Convert]::ToInt32($hex.Substring(0, 2), 16)
  $line = [Convert]::ToInt32($hex.Substring(2, 2), 16)
  $key = "$seg/$line"
  if ($seen.ContainsKey($key)) { continue }   # second repetition onwards
  $seen[$key] = $true
  if (-not $stream.ContainsKey($seg)) { $stream[$seg] = New-Object 'Collections.Generic.SortedList[int,string]' }
  $stream[$seg].Add($line, $hex.Substring(4))
}

$total = 0
$segHex = @{}
foreach ($s in ($stream.Keys | Sort-Object)) {
  $data = -join $stream[$s].Values
  # Cut at the JPEG end-of-image marker, dropping the 'f' padding of the last
  # frame. Same rule the reconstruction uses.
  $end = $data.IndexOf('ffd9')
  if ($end -lt 0) { throw "el segmento $s no termina en ffd9; el fichero de origen no tiene el formato esperado" }
  $data = $data.Substring(0, $end + 4)
  $segHex[$s] = $data
  $total += $data.Length
}
if ($segHex.Count -ne $Segments) { throw "he encontrado $($segHex.Count) segmentos, esperaba $Segments" }
Write-Output ("imagen: {0} segmentos, {1} caracteres hex ({2} bytes)" -f $segHex.Count, $total, ($total / 2))

# --- re-chunk with room for the parity, and encode ----------------------------
$outFrames = New-Object Collections.Generic.List[string]
foreach ($s in ($segHex.Keys | Sort-Object)) {
  $data = $segHex[$s]
  $n = 0
  for ($i = 0; $i -lt $data.Length; $i += $Chunk) {
    $n++
    $piece = $data.Substring($i, [Math]::Min($Chunk, $data.Length - $i))
    if ($piece.Length -lt $Chunk) { $piece = $piece.PadRight($Chunk, 'f') }
    if ($n -gt 255) { throw "el segmento $s necesita mas de 255 tramas" }
    $body = ('{0:x2}{1:x2}' -f $s, $n) + $piece
    $outFrames.Add($body + (Invoke-BchEncode $body))
  }
}
Write-Output ("tramas por repeticion: {0} (antes {1}, {2:P1} mas)" -f `
  $outFrames.Count, $seen.Count, (($outFrames.Count - $seen.Count) / $seen.Count))

# --- verify the round trip before anything is written -------------------------
# Decode every frame we just built and rebuild the segments from the result. If
# this does not reproduce the source byte for byte, the file does not get made.
$check = @{}
foreach ($f in $outFrames) {
  $r = Invoke-BchDecode $f
  if (-not $r.ok -or $r.errors -ne 0) { throw "una trama recien creada no decodifica limpia: $f" }
  $seg  = [Convert]::ToInt32($r.hex.Substring(0, 2), 16)
  $line = [Convert]::ToInt32($r.hex.Substring(2, 2), 16)
  if (-not $check.ContainsKey($seg)) { $check[$seg] = New-Object 'Collections.Generic.SortedList[int,string]' }
  $check[$seg].Add($line, $r.hex.Substring(4))
}
foreach ($s in ($segHex.Keys | Sort-Object)) {
  $rebuilt = -join $check[$s].Values
  $end = $rebuilt.IndexOf('ffd9')
  if ($end -ge 0) { $rebuilt = $rebuilt.Substring(0, $end + 4) }
  if ($rebuilt -ne $segHex[$s]) { throw "el segmento $s no se reconstruye igual que en el origen" }
}
Write-Output "verificacion: los 22 segmentos se reconstruyen identicos al origen"

# --- write it out -------------------------------------------------------------
$sb = New-Object Text.StringBuilder
$row = 0
foreach ($p in $prefix) { $row++; [void]$sb.Append("$row`:$p`n") }
for ($r = 1; $r -le $Reps; $r++) {
  foreach ($f in $outFrames) { $row++; [void]$sb.Append("$row`:$f`n") }
}
[IO.File]::WriteAllText($Out, $sb.ToString(), (New-Object Text.UTF8Encoding($false)))

Write-Output ""
Write-Output ("escrito {0}" -f $Out)
Write-Output ("{0} lineas = {1} centinelas + {2} repeticiones x {3} tramas" -f $row, $prefix.Count, $Reps, $outFrames.Count)
