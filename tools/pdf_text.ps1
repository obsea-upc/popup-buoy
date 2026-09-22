# Pulls the text out of a PDF with nothing installed: inflates the FlateDecode
# content streams and reads the text-showing operators. Rebuilt in tools/ because
# the August version lived in a scratch directory and was lost.
#
#   .\tools\pdf_text.ps1 -Path hardware\articles\I2MTC_DeLaVega.pdf
#   .\tools\pdf_text.ps1 -Path ... -Match 'reception'      # only lines that match
#
# Works on ordinary embedded fonts. A PDF whose fonts are CID-encoded with no
# ToUnicode map gives glyph indices rather than words - the journal article in
# hardware/articles is one of those, and nothing short of OCR fixes it.

param(
  [Parameter(Mandatory=$true)][string]$Path,
  [string]$Match,
  [int]$Context = 0
)

$bytes = [IO.File]::ReadAllBytes((Resolve-Path $Path).ProviderPath)
$latin = [Text.Encoding]::GetEncoding(28591)     # byte <-> char, no loss
$raw = $latin.GetString($bytes)

# Every stream, inflated when it is Flate (skip the 2-byte zlib header).
$texts = New-Object Collections.Generic.List[string]
$rx = [regex]'stream\r?\n'
foreach ($m in $rx.Matches($raw)) {
  $start = $m.Index + $m.Length
  $end = $raw.IndexOf('endstream', $start)
  if ($end -lt 0) { continue }
  $head = $raw.Substring([math]::Max(0, $m.Index - 400), [math]::Min(400, $m.Index))
  $data = $latin.GetBytes($raw.Substring($start, $end - $start))
  if ($head -notmatch 'FlateDecode') { continue }
  try {
    $ms = New-Object IO.MemoryStream(,$data[2..($data.Length-1)])
    $ds = New-Object IO.Compression.DeflateStream($ms, [IO.Compression.CompressionMode]::Decompress)
    $sr = New-Object IO.StreamReader($ds, $latin)
    $s = $sr.ReadToEnd()
    $sr.Close()
    # Fonts and images inflate too, and their bytes happen to match the text
    # operators often enough to fill the output with mojibake. A page's content
    # stream always has BT and a showing operator.
    # Note TJ takes an array, so the string is followed by "] TJ", not ") TJ".
    if ($s -match 'BT' -and $s -match '(?m)(Tj|TJ)\s') { $texts.Add($s) }
  } catch { }
}

# Text operators: (string) Tj, and [(a) -300 (b)] TJ - where the strings inside the
# array are NOT each followed by an operator, so the strings are taken as they come
# and the positioning operators (Td, TD, T*, ET) are what breaks the line.
$out = New-Object Collections.Generic.List[string]
foreach ($t in $texts) {
  $line = New-Object Text.StringBuilder
  foreach ($op in [regex]::Matches($t, '(?s)\((?<s>(?:\\.|[^\\()])*)\)|(?<td>T\*|TD|Td)|(?<et>ET)')) {
    if ($op.Groups['s'].Success) {
      $s = $op.Groups['s'].Value -replace '\\([()\\])', '$1' -replace '\\n', ' ' -replace '\\r', ' '
      [void]$line.Append($s)
    } elseif ($op.Groups['td'].Success -or $op.Groups['et'].Success) {
      if ($line.Length) { $out.Add($line.ToString()); [void]$line.Clear() }
    }
  }
  if ($line.Length) { $out.Add($line.ToString()) }
}

if ($Match) {
  for ($i = 0; $i -lt $out.Count; $i++) {
    if ($out[$i] -match $Match) {
      $a = [math]::Max(0, $i - $Context); $b = [math]::Min($out.Count - 1, $i + $Context)
      $out[$a..$b] | ForEach-Object { "  $_" }
      if ($Context) { '  ---' }
    }
  }
} else {
  $out
}
