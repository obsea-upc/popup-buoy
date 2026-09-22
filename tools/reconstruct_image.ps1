# Rebuilds the seabed image from the frames a buoy managed to get through, the same
# way the onshore script does (image recosntruction/ReconstructDEF.py) but without
# needing Python: the JPEG header template is reattached per segment, the segments
# are decoded and pasted side by side.
#
#   .\tools\reconstruct_image.ps1 -Frames B5.txt -Out B5.jpg
#
# -Frames is one frame per line, "<row>:<hex>" or bare hex: 4 hex of header (segment,
# line) plus 42 of payload. Repeated copies of a frame are allowed - the most common
# copy of each line wins, which is the best a ground segment can do while the payload
# carries no checksum.
#
# The header template (header.txt) is a full JPEG header whose ZZZZZZZZZ stands for
# the 9 hex characters of Baseline DCT data that travel at the start of each segment.

param(
  [Parameter(Mandatory=$true)][string]$Frames,
  [Parameter(Mandatory=$true)][string]$Out,
  [string]$Header = "image recosntruction\header.txt",
  [int]$Segments = 22,
  [int]$Width = 1156,
  [int]$Height = 868,
  [switch]$Quiet,
  # A file listing the frames known to be intact. A copy that appears in it wins over
  # every other copy of that line; a line with none still uses the best available,
  # because a JPEG decodes through a few flipped bits but not through a missing chunk.
  #
  # In practice this is the set CLS itself reports as uncorrected - the `sensors`
  # field of retrieve-bulk, `2_BCH` = 0, which in the Sep 2026 campaign was right
  # 1173 times out of 1173. It can also be the original file, as an oracle, to
  # measure the ceiling.
  [string]$Verify,
  # The frames carry two bytes of error correction (see tools\make_fec_datafile.ps1):
  # 4 hex of header, 38 of payload, 4 of parity. Every copy is passed through the
  # decoder first, so a frame with one or two flipped bits is repaired instead of
  # being thrown away or, worse, silently used as it arrived.
  [switch]$Fec,
  # The manifest written beside a packed data file (make_fec_datafile.ps1 -Packed).
  # In a packed file the segments are one continuous stream rather than each
  # starting on a fresh frame, so the frame header is a sequence number and the
  # manifest is the only thing that says where each segment begins. Packed files
  # always carry the parity, so this turns -Fec on by itself.
  [string]$Manifest
)

Add-Type -AssemblyName System.Drawing
$packed = [bool]$Manifest
if ($packed) { $Fec = $true }
if ($Fec) { . (Join-Path $PSScriptRoot 'bch.ps1') }

# Payload hex per frame, which is what a missing line has to be padded out to.
$chunkHex = if ($Fec) { 38 } else { 42 }

function Read-TextShared([string]$p) {
  $fs = [IO.File]::Open((Resolve-Path $p).ProviderPath, 'Open', 'Read', 'ReadWrite')
  $rd = New-Object IO.StreamReader($fs); $t = $rd.ReadToEnd(); $rd.Close(); $t
}

$tpl = (Read-TextShared $Header).Trim()
$lines = @((Read-TextShared $Frames) -split "`r?`n" | Where-Object { $_.Trim() })

$good = $null
if ($Verify) {
  $good = New-Object 'System.Collections.Generic.HashSet[string]'
  foreach ($l in ((Read-TextShared $Verify) -split "`r?`n")) {
    $h = ($l -split ':')[-1].Trim().ToLower()
    if ($h.Length -lt 46) { continue }
    # Stored in the same shape the copies are kept in below: with the parity
    # stripped when the frames carry it, so the two can be compared at all.
    if ($Fec) { [void]$good.Add((Invoke-BchDecode $h.Substring(0,46)).hex) }
    else      { [void]$good.Add($h.Substring(0,46)) }
  }
}

# segment -> line -> list of copies, or, packed, sequence number -> list of copies
$seg = @{}
$seq = @{}
$fecStats = @{ clean = 0; fixed1 = 0; fixed2 = 0; beyond = 0 }
foreach ($l in $lines) {
  $hex = ($l -split ':')[-1].Trim().ToLower()
  if ($hex.Length -lt 46) { continue }

  # Sentinels first, on the raw frame. They carry no parity, so letting them
  # reach the decoder would both pollute the correction counts and, worse, let a
  # sentinel be "corrected" into something with a valid sequence number and
  # written into the middle of the image stream.
  if ($hex -match '^fa(fa|fb|fc)') { continue }

  if ($Fec) {
    # Repair what the parity can repair, and keep the rest as it arrived: a JPEG
    # often decodes through a few wrong bits, so an uncorrectable frame is still
    # worth more than a hole. It is recorded, though - "beyond" is the count that
    # says how much of the corruption was too deep for two bytes of parity.
    $r = Invoke-BchDecode $hex.Substring(0,46)
    if     ($r.errors -eq 0) { $fecStats.clean++ }
    elseif ($r.errors -eq 1) { $fecStats.fixed1++ }
    elseif ($r.errors -eq 2) { $fecStats.fixed2++ }
    else                     { $fecStats.beyond++ }
    $hex = $r.hex
  }

  if ($packed) {
    # One key space: the sequence number. Which segment a frame belongs to is not
    # in the frame at all, it comes from the manifest further down.
    $q = [Convert]::ToInt32($hex.Substring(0,4),16)
    if ($q -lt 1 -or $q -gt 0xFAF9) { continue }         # sentinels (0xfa..) and noise
    if (-not $seq.ContainsKey($q)) { $seq[$q] = New-Object Collections.Generic.List[string] }
    $seq[$q].Add($hex.Substring(4))
    continue
  }

  $s = [Convert]::ToInt32($hex.Substring(0,2),16)
  $n = [Convert]::ToInt32($hex.Substring(2,2),16)
  if ($s -lt 1 -or $s -gt $Segments) { continue }        # sentinels (0xfa..) and noise
  if (-not $seg.ContainsKey($s)) { $seg[$s] = @{} }
  if (-not $seg[$s].ContainsKey($n)) { $seg[$s][$n] = New-Object Collections.Generic.List[string] }
  $seg[$s][$n].Add($hex.Substring(4))
}

# --- packed mode: rebuild the stream, then cut it up with the manifest --------
if ($packed) {
  $layout = New-Object Collections.Generic.List[object]
  $nFrames = 0
  foreach ($l in ((Read-TextShared $Manifest) -split "`r?`n")) {
    $t = $l.Trim()
    if ($t -match 'tramas=(\d+)') { $nFrames = [int]$Matches[1] }
    if ($t.StartsWith('#') -or -not $t) { continue }
    $p = $t -split ';'
    $layout.Add([pscustomobject]@{ seg = [int]$p[0]; off = [int]$p[1]; len = [int]$p[2] })
  }
  if ($layout.Count -eq 0) { throw "el manifiesto $Manifest no trae ningun segmento" }
  if ($nFrames -eq 0) { $nFrames = [int][math]::Ceiling((($layout[-1].off + $layout[-1].len) / $chunkHex)) }

  # Two streams, the same two choices the unpacked path makes per line: one that
  # only uses copies known intact, one that uses the best copy available. A frame
  # that never arrived becomes zeros either way - in a packed stream a hole cannot
  # be dropped, or everything after it would shift.
  function Build-Stream($onlyVerified) {
    $sb = New-Object Text.StringBuilder
    for ($q = 1; $q -le $nFrames; $q++) {
      $piece = $null
      if ($seq.ContainsKey($q)) {
        $copies = $seq[$q]
        if ($good) {
          $head = '{0:x4}' -f $q
          $piece = $copies | Where-Object { $good.Contains($head + $_) } | Select-Object -First 1
        }
        if (-not $piece -and -not $onlyVerified) {
          $piece = ($copies | Group-Object | Sort-Object Count -Descending | Select-Object -First 1).Name
        }
      }
      if (-not $piece) { $piece = '0' * $chunkHex }
      [void]$sb.Append($piece)
    }
    $sb.ToString()
  }
  $streamVerified = if ($good) { Build-Stream $true } else { $null }
  $streamBest = Build-Stream $false

  $segOf = @{}
  foreach ($e in $layout) { $segOf[$e.seg] = $e }
  $haveFrames = @{}
  foreach ($e in $layout) {
    # How many of this segment's frames arrived at all, for the per-segment line
    # the loop below prints. A frame that straddles a boundary counts for both.
    $first = [int][math]::Floor($e.off / $chunkHex) + 1
    $last  = [int][math]::Ceiling(($e.off + $e.len) / $chunkHex)
    $n = 0
    for ($q = $first; $q -le $last; $q++) { if ($seq.ContainsKey($q)) { $n++ } }
    $haveFrames[$e.seg] = $n
  }
}

$segW = [int][math]::Floor($Width / $Segments)
$canvas = New-Object Drawing.Bitmap($Width, $Height, [Drawing.Imaging.PixelFormat]::Format24bppRgb)
$g = [Drawing.Graphics]::FromImage($canvas)
$g.Clear([Drawing.Color]::Black)
$g.InterpolationMode = [Drawing.Drawing2D.InterpolationMode]::HighQualityBicubic

$okSeg = 0; $partSeg = 0; $lostSeg = 0
for ($i = 1; $i -le $Segments; $i++) {
  $left = ($i - 1) * $segW
  $right = if ($i -ne $Segments) { $i * $segW } else { $Width }
  $w = $right - $left

  # Assemble a segment from a chosen copy per line. "onlyVerified" drops the lines
  # with no clean copy, leaving a hole; without it the best copy is used, holes and
  # all. Neither is always better - a JPEG sometimes survives a wrong chunk and
  # sometimes not - so both are tried below and the one that decodes wins.
  function Build-Segment($segNum, $onlyVerified) {
    if ($packed) {
      # The stream is already assembled; a segment is a slice of it, and the first
      # 9 hex of that slice are the Baseline DCT bytes the template is missing.
      if (-not $segOf.ContainsKey($segNum)) { return $null }
      $stream = if ($onlyVerified) { $streamVerified } else { $streamBest }
      if (-not $stream) { return $null }
      $e = $segOf[$segNum]
      if ($e.off + $e.len -gt $stream.Length) { return $null }
      $slice = $stream.Substring($e.off, $e.len)
      if ($slice.Length -lt 9) { return $null }
      $d = $tpl.Replace("ZZZZZZZZZ", $slice.Substring(0,9)) + $slice.Substring(9)
      $cut = $d.IndexOf('ffd9')
      if ($cut -ge 0) { $d = $d.Substring(0, $cut + 4) }
      return $d
    }

    if (-not $seg.ContainsKey($segNum) -or $seg[$segNum].Count -eq 0) { return $null }
    $pick = @{}
    foreach ($n in ($seg[$segNum].Keys | Sort-Object)) {
      $copies = $seg[$segNum][$n]
      $verified = $null
      if ($good) {
        $head = ('{0:x2}{1:x2}' -f $segNum, $n)
        $verified = $copies | Where-Object { $good.Contains($head + $_) } | Select-Object -First 1
      }
      if ($verified) { $pick[$n] = $verified }
      elseif (-not $onlyVerified) { $pick[$n] = ($copies | Group-Object | Sort-Object Count -Descending | Select-Object -First 1).Name }
    }
    $nums = @($pick.Keys | Sort-Object)
    if ($nums.Count -eq 0 -or $pick[$nums[0]].Length -lt 9) { return $null }
    $data = $tpl.Replace("ZZZZZZZZZ", $pick[$nums[0]].Substring(0,9)) + $pick[$nums[0]].Substring(9)
    foreach ($n in ($nums | Select-Object -Skip 1)) { $data += $pick[$n] }
    for ($n = $nums[0]; $n -le $nums[-1]; $n++) { if (-not $pick.ContainsKey($n)) { $data += ('00' * ($chunkHex / 2)) } }
    $end = $data.IndexOf('ffd9')
    if ($end -ge 0) { $data = $data.Substring(0, $end + 4) }
    return $data
  }

  function Try-Decode($data) {
    if (-not $data) { return $null }
    try {
      $bytes = [byte[]]::new($data.Length / 2)
      for ($k = 0; $k -lt $bytes.Length; $k++) { $bytes[$k] = [Convert]::ToByte($data.Substring($k*2,2),16) }
      return [Drawing.Image]::FromStream((New-Object IO.MemoryStream(,$bytes)))
    } catch { return $null }
  }

  $img = $null
  if ($good) { $img = Try-Decode (Build-Segment $i $true) }
  if (-not $img) { $img = Try-Decode (Build-Segment $i $false) }
  if ($img) {
    $g.DrawImage($img, (New-Object Drawing.Rectangle($left, 0, $w, $Height)))
    $img.Dispose()
    $have = if ($packed) { $haveFrames[$i] } else { $seg[$i].Count }
    if (-not $Quiet) { Write-Output ("  seg {0,2}: {1,2} lineas, decodifica" -f $i, $have) }
    $okSeg++
  } else {
    # black placeholder, exactly what the onshore script does
    $br = New-Object Drawing.SolidBrush([Drawing.Color]::FromArgb(16,16,16))
    $g.FillRectangle($br, (New-Object Drawing.Rectangle($left, 0, $w, $Height)))
    $br.Dispose()
    if (-not $Quiet) { Write-Output ("  seg {0,2}: sin datos utiles, va en negro" -f $i) }
    $lostSeg++
  }
}
$g.Dispose()

$enc = [Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq 'image/jpeg' }
$pars = New-Object Drawing.Imaging.EncoderParameters(1)
$pars.Param[0] = New-Object Drawing.Imaging.EncoderParameter([Drawing.Imaging.Encoder]::Quality, 85L)
$canvas.Save((Join-Path (Get-Location) $Out), $enc, $pars)
$canvas.Dispose()
if ($Fec) {
  $tot = $fecStats.clean + $fecStats.fixed1 + $fecStats.fixed2 + $fecStats.beyond
  Write-Output ("correccion de errores: {0} intactas, {1} reparadas de 1 bit, {2} de 2 bits, {3} irreparables (de {4})" -f `
    $fecStats.clean, $fecStats.fixed1, $fecStats.fixed2, $fecStats.beyond, $tot)
}
Write-Output ("{0}: {1} segmentos decodificados, {2} en negro" -f $Out, $okSeg, $lostSeg)
