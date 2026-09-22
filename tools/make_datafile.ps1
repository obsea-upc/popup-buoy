# Builds a data file with N repetitions of the image block, keeping the head of an
# existing file byte for byte - so a buoy already part way through the old file can
# be given the new one without its progressFile pointing at different content.
#
#   .\tools\make_datafile.ps1 -Source hardware\tests\2026-09-17\PopUpBuoy_X\dataFile.txt `
#                             -Repetitions 50 -Out dataFile_50rep.txt
#
# Lines are "<row>:<hex>", the row numbers running 1..prefix+N*block. Both the block
# and the prefix that precedes it are detected: the current image file opens with
# three sentinel frames (fafa/fafb/fafc) that are sent ONCE, followed by the
# 145-frame image block repeated over and over.

param(
  [Parameter(Mandatory=$true)][string]$Source,
  [Parameter(Mandatory=$true)][int]$Repetitions,
  [Parameter(Mandatory=$true)][string]$Out,
  [int]$BlockLength = 0
)

$fs = [IO.File]::Open((Resolve-Path $Source).ProviderPath, 'Open', 'Read', 'ReadWrite')
$rd = New-Object IO.StreamReader($fs); $txt = $rd.ReadToEnd(); $rd.Close()
$src = @($txt -split "`r?`n" | Where-Object { $_.Trim() })
$hex = @($src | ForEach-Object { ($_ -split ':')[-1].Trim() })
Write-Output ("origen: {0} lineas" -f $hex.Count)

# The period: the last line of the file also appears one block earlier.
$last = $hex[$hex.Count - 1]
if ($BlockLength -le 0) {
  for ($k = $hex.Count - 2; $k -ge 0; $k--) {
    if ($hex[$k] -ne $last) { continue }
    $p = $hex.Count - 1 - $k
    $ok = $true
    for ($i = $hex.Count - 1; $i - $p -ge 0 -and $i -ge $hex.Count - 1 - 3 * $p; $i--) {
      if ($hex[$i] -ne $hex[$i - $p]) { $ok = $false; break }
    }
    if ($ok) { $BlockLength = $p; break }
  }
  if ($BlockLength -le 0) { throw "no se encuentra la repeticion: pasa -BlockLength" }
}

# The prefix: everything before the first line from which the period holds all the
# way to the end (the sentinel frames, sent once).
$offset = 0
for ($o = 0; $o -lt $hex.Count; $o++) {
  $ok = $true
  for ($i = $o + $BlockLength; $i -lt $hex.Count; $i++) {
    if ($hex[$i] -ne $hex[$i - $BlockLength]) { $ok = $false; break }
  }
  if ($ok) { $offset = $o; break }
}
$prefix = if ($offset -gt 0) { $hex[0..($offset - 1)] } else { @() }
$block = $hex[$offset..($offset + $BlockLength - 1)]
Write-Output ("cabecera que no se repite: {0} lineas   bloque: {1} lineas   repeticiones en el origen: {2:N2}" -f `
  $prefix.Count, $BlockLength, (($hex.Count - $offset) / $BlockLength))

$total = $offset + $Repetitions * $BlockLength
if ($total -lt $hex.Count) { throw "el origen ya tiene $($hex.Count) lineas, mas que las $total pedidas" }

$sb = New-Object Text.StringBuilder
for ($i = 1; $i -le $total; $i++) {
  $h = if ($i -le $offset) { $prefix[$i - 1] } else { $block[($i - 1 - $offset) % $BlockLength] }
  [void]$sb.Append($i).Append(':').Append($h).Append("`n")
}
[IO.File]::WriteAllText((Join-Path (Split-Path -Parent ((Resolve-Path $Source).ProviderPath)) $Out), $sb.ToString(), (New-Object Text.UTF8Encoding($false)))

$outPath = Join-Path (Split-Path -Parent ((Resolve-Path $Source).ProviderPath)) $Out
$new = @([IO.File]::ReadAllLines($outPath) | Where-Object { $_.Trim() })
$same = $true
for ($i = 0; $i -lt $src.Count; $i++) { if ($new[$i] -ne $src[$i]) { $same = $false; break } }
Write-Output ("escrito: {0}" -f $outPath)
Write-Output ("{0} lineas = {1} repeticiones, {2:N0} bytes" -f $new.Count, ($new.Count / $BlockLength), (Get-Item $outPath).Length)
Write-Output ("las primeras {0} lineas son identicas al origen: {1}" -f $src.Count, $same)
