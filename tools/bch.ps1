# BCH(184,168,t=2) over GF(2^8) - two bytes of parity that correct up to two
# flipped bits anywhere in an Argos frame.
#
# Why this exists: the BCH that Argos already carries protects the message
# header, not our 21 bytes of payload, so a frame CLS reports as "corrected"
# still reaches us with bits wrong. The 17-21 Sep 2026 campaign measured how
# deep that damage is: of the KIM1 frames received, 59 % arrive untouched, 15 %
# with exactly one bit wrong and 8 % with two. Correcting up to two bits takes
# the usable share from 59 % to about 82 %, which is worth more than the 9.5 %
# of payload the parity costs.
#
# Nothing in the buoy changes. The parity is written into the data file on land
# and checked on land; the firmware sends the 23 bytes it has always sent.
#
# Dot-source this file to use it:
#   . tools\bch.ps1
#   $parity = Invoke-BchEncode 'aabbcc...'       # 42 hex in, 4 hex out
#   $r = Invoke-BchDecode '<46 hex>'             # ok / hex / errors
#
# Run it directly to execute the self-test:
#   powershell -File tools\bch.ps1 -SelfTest

param([switch]$SelfTest)

# --- GF(2^8), primitive polynomial x^8+x^4+x^3+x^2+1 (0x11D) ----------------
$script:GfExp = New-Object 'int[]' 512
$script:GfLog = New-Object 'int[]' 256
$x = 1
for ($i = 0; $i -lt 255; $i++) {
  $script:GfExp[$i] = $x
  $script:GfLog[$x] = $i
  $x = $x -shl 1
  if ($x -band 0x100) { $x = $x -bxor 0x11D }
}
for ($i = 255; $i -lt 512; $i++) { $script:GfExp[$i] = $script:GfExp[$i - 255] }

function script:GfMul([int]$a, [int]$b) {
  if ($a -eq 0 -or $b -eq 0) { return 0 }
  $script:GfExp[$script:GfLog[$a] + $script:GfLog[$b]]
}
function script:GfDiv([int]$a, [int]$b) {
  if ($b -eq 0) { throw "division by zero in GF(2^8)" }
  if ($a -eq 0) { return 0 }
  $script:GfExp[($script:GfLog[$a] - $script:GfLog[$b] + 255) % 255]
}
function script:GfPow([int]$a, [int]$n) {
  if ($a -eq 0) { return 0 }
  $script:GfExp[($script:GfLog[$a] * $n) % 255]
}

# --- Generator polynomial ---------------------------------------------------
# g(x) = m1(x) * m3(x), the minimal polynomials of alpha and alpha^3. Computed
# here rather than written down, so there is no constant to get wrong: each
# minimal polynomial is the product of (x - alpha^k) over the conjugates of the
# root, which for a binary code comes out with coefficients in {0,1}.
function script:MinimalPoly([int]$rootExp) {
  $conj = New-Object 'System.Collections.Generic.List[int]'
  $e = $rootExp
  do { $conj.Add($e); $e = ($e * 2) % 255 } while ($e -ne $rootExp)

  # poly[i] is the GF(2^8) coefficient of x^i; start with the constant 1
  $poly = @(1)
  foreach ($c in $conj) {
    $root = $script:GfExp[$c]
    $next = New-Object 'int[]' ($poly.Count + 1)
    for ($i = 0; $i -lt $poly.Count; $i++) {
      $next[$i + 1] = $next[$i + 1] -bxor $poly[$i]            # x * poly
      $next[$i]     = $next[$i] -bxor (script:GfMul $poly[$i] $root)  # + root * poly
    }
    $poly = $next
  }
  foreach ($c in $poly) { if ($c -ne 0 -and $c -ne 1) { throw "minimal polynomial is not binary" } }
  $poly
}

$script:GenBits = $null
function script:Generator() {
  if ($null -ne $script:GenBits) { return $script:GenBits }
  $m1 = script:MinimalPoly 1
  $m3 = script:MinimalPoly 3
  # multiply m1 by m3 over GF(2)
  $prod = New-Object 'int[]' ($m1.Count + $m3.Count - 1)
  for ($i = 0; $i -lt $m1.Count; $i++) {
    if ($m1[$i] -eq 0) { continue }
    for ($j = 0; $j -lt $m3.Count; $j++) {
      if ($m3[$j] -ne 0) { $prod[$i + $j] = $prod[$i + $j] -bxor 1 }
    }
  }
  if ($prod.Count -ne 17) { throw "generator degree is $($prod.Count - 1), expected 16" }
  # Return most-significant first, which is the order the division below walks.
  $script:GenBits = @()
  for ($i = $prod.Count - 1; $i -ge 0; $i--) { $script:GenBits += $prod[$i] }
  $script:GenBits
}

# --- Bit helpers ------------------------------------------------------------
function script:HexToBits([string]$hex) {
  $bits = New-Object 'int[]' ($hex.Length * 4)
  for ($i = 0; $i -lt $hex.Length; $i++) {
    $v = [Convert]::ToInt32($hex.Substring($i, 1), 16)
    for ($b = 0; $b -lt 4; $b++) { $bits[$i * 4 + $b] = ($v -shr (3 - $b)) -band 1 }
  }
  $bits
}
function script:BitsToHex([int[]]$bits) {
  $sb = New-Object Text.StringBuilder
  for ($i = 0; $i -lt $bits.Count; $i += 4) {
    $v = ($bits[$i] -shl 3) -bor ($bits[$i+1] -shl 2) -bor ($bits[$i+2] -shl 1) -bor $bits[$i+3]
    [void]$sb.Append($v.ToString('x'))
  }
  $sb.ToString()
}

# --- Encode -----------------------------------------------------------------
# 42 hex chars (21 bytes) in, 4 hex chars (2 bytes) of parity out. The parity is
# the remainder of message(x) * x^16 divided by g(x), so the 46-hex frame that
# results is a multiple of g(x) and decodes with zero syndromes when intact.
function Invoke-BchEncode([string]$hex) {
  if ($hex.Length -ne 42) { throw "Invoke-BchEncode wants 42 hex chars, got $($hex.Length)" }
  $g = script:Generator
  $msg = script:HexToBits $hex
  $reg = New-Object 'int[]' ($msg.Count + 16)
  [Array]::Copy($msg, $reg, $msg.Count)

  for ($i = 0; $i -lt $msg.Count; $i++) {
    if ($reg[$i] -eq 0) { continue }
    for ($j = 0; $j -le 16; $j++) {
      if ($g[$j] -ne 0) { $reg[$i + $j] = $reg[$i + $j] -bxor 1 }
    }
  }
  script:BitsToHex $reg[($msg.Count)..($msg.Count + 15)]
}

# --- Decode -----------------------------------------------------------------
# 46 hex chars in (42 of payload + 4 of parity). Returns:
#   ok      - true when the frame is intact or was corrected
#   hex     - the 42 hex chars of payload, corrected where it could be
#   errors  - how many bits were flipped back (0, 1 or 2), or -1 when the
#             damage is beyond what two bytes of parity can repair
function Invoke-BchDecode([string]$hex) {
  if ($hex.Length -ne 46) { throw "Invoke-BchDecode wants 46 hex chars, got $($hex.Length)" }
  $bits = script:HexToBits $hex
  $n = $bits.Count            # 184

  # Syndromes: S1 = r(alpha), S3 = r(alpha^3). Bit i sits on x^(n-1-i).
  $s1 = 0; $s3 = 0
  for ($i = 0; $i -lt $n; $i++) {
    if ($bits[$i] -eq 0) { continue }
    $p = $n - 1 - $i
    $s1 = $s1 -bxor $script:GfExp[$p % 255]
    $s3 = $s3 -bxor $script:GfExp[(3 * $p) % 255]
  }

  if ($s1 -eq 0 -and $s3 -eq 0) {
    return [pscustomobject]@{ ok = $true; hex = $hex.Substring(0, 42); errors = 0 }
  }

  # One error: S3 = S1^3 and the position is the log of S1.
  if ($s1 -ne 0) {
    $s1cubed = script:GfPow $s1 3
    if ($s1cubed -eq $s3) {
      $p = $script:GfLog[$s1]
      if ($p -lt $n) {
        $bits[$n - 1 - $p] = $bits[$n - 1 - $p] -bxor 1
        return [pscustomobject]@{ ok = $true; hex = (script:BitsToHex $bits).Substring(0, 42); errors = 1 }
      }
      # Position outside the shortened code: the damage is not a single bit.
      return [pscustomobject]@{ ok = $false; hex = $hex.Substring(0, 42); errors = -1 }
    }

    # Two errors: sigma(x) = 1 + S1 x + ((S3 + S1^3)/S1) x^2, found by Chien search.
    $sigma1 = $s1
    $sigma2 = script:GfDiv ($s3 -bxor $s1cubed) $s1
    $roots = New-Object 'System.Collections.Generic.List[int]'
    for ($p = 0; $p -lt $n; $p++) {
      $inv = $script:GfExp[(255 - ($p % 255)) % 255]     # alpha^-p
      $v = 1 -bxor (script:GfMul $sigma1 $inv) -bxor (script:GfMul $sigma2 (script:GfMul $inv $inv))
      if ($v -eq 0) { $roots.Add($p) }
    }
    if ($roots.Count -eq 2) {
      foreach ($p in $roots) { $bits[$n - 1 - $p] = $bits[$n - 1 - $p] -bxor 1 }
      return [pscustomobject]@{ ok = $true; hex = (script:BitsToHex $bits).Substring(0, 42); errors = 2 }
    }
  }

  # S1 = 0 with S3 != 0, or no valid error locator: more than two bits are wrong.
  [pscustomobject]@{ ok = $false; hex = $hex.Substring(0, 42); errors = -1 }
}

# --- Self-test --------------------------------------------------------------
if ($SelfTest) {
  $rnd = New-Object Random 20260922
  $hexChars = '0123456789abcdef'
  function NewPayload { -join (1..42 | ForEach-Object { $hexChars[$rnd.Next(16)] }) }
  function FlipBits([string]$frame, [int[]]$positions) {
    $bits = script:HexToBits $frame
    foreach ($p in $positions) { $bits[$p] = $bits[$p] -bxor 1 }
    script:BitsToHex $bits
  }

  $g = script:Generator
  Write-Output ("generador g(x), 17 coeficientes: " + ($g -join ''))
  Write-Output ""

  $cases = 400
  $okClean = 0; $ok1 = 0; $ok2 = 0; $detected3 = 0; $miscorrected3 = 0
  for ($c = 0; $c -lt $cases; $c++) {
    $payload = NewPayload
    $frame = $payload + (Invoke-BchEncode $payload)

    $r = Invoke-BchDecode $frame
    if ($r.ok -and $r.errors -eq 0 -and $r.hex -eq $payload) { $okClean++ }

    $p1 = $rnd.Next(184)
    $r = Invoke-BchDecode (FlipBits $frame @($p1))
    if ($r.ok -and $r.errors -eq 1 -and $r.hex -eq $payload) { $ok1++ }

    $p2 = $rnd.Next(184); while ($p2 -eq $p1) { $p2 = $rnd.Next(184) }
    $r = Invoke-BchDecode (FlipBits $frame @($p1, $p2))
    if ($r.ok -and $r.errors -eq 2 -and $r.hex -eq $payload) { $ok2++ }

    $p3 = $rnd.Next(184); while ($p3 -eq $p1 -or $p3 -eq $p2) { $p3 = $rnd.Next(184) }
    $r = Invoke-BchDecode (FlipBits $frame @($p1, $p2, $p3))
    if (-not $r.ok) { $detected3++ } elseif ($r.hex -ne $payload) { $miscorrected3++ }
  }

  Write-Output ("sin errores      : {0,4}/{1} recuperadas" -f $okClean, $cases)
  Write-Output ("1 bit cambiado   : {0,4}/{1} recuperadas" -f $ok1, $cases)
  Write-Output ("2 bits cambiados : {0,4}/{1} recuperadas" -f $ok2, $cases)
  Write-Output ("3 bits cambiados : {0,4}/{1} detectadas como irreparables, {2} corregidas mal" -f $detected3, $cases, $miscorrected3)
  Write-Output ""
  if ($okClean -eq $cases -and $ok1 -eq $cases -and $ok2 -eq $cases) {
    Write-Output "AUTOTEST OK: corrige todo lo que promete corregir."
  } else {
    Write-Output "AUTOTEST FALLIDO"
    exit 1
  }
}
