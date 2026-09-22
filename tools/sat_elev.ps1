# Satellite elevation from an AOP file, on the PC, with the same orbit model the
# firmware uses (previpass.c / previpass_util.c: circular orbit, mean-motion drift
# from the semi-major-axis drift, node longitude drifting per revolution).
#
# Dot-source it, then:
#   $aop = Read-Aop "hardware\tests\2026-09-17\AOP.txt"
#   [SatElev]::Elevation($aop[0], [datetime]"2026-09-17T10:07:00Z", 41.2236, 1.7363)
#   Get-SatElevations $aop ([datetime]"2026-09-17T10:07:00Z") 41.2236 1.7363
#
# Elevation is in degrees and negative below the horizon (previpass only computes
# it inside a pass; here it is continuous so windows above any floor can be found).

if (-not ('SatElev' -as [type])) {
Add-Type -TypeDefinition @"
using System;
public class AopSat {
  public string Name;
  public DateTime Bulletin;      // UTC
  public double A, Inc, Node, NodeDrift, PeriodMin, ADriftMPerDay;
}
public static class SatElev {
  const double R = 6378.137;
  const double D2R = Math.PI / 180.0;
  public static double Elevation(AopSat s, DateTime utc, double latDeg, double lonDeg) {
    double xb = Math.Cos(latDeg*D2R)*Math.Cos(lonDeg*D2R);
    double yb = Math.Cos(latDeg*D2R)*Math.Sin(lonDeg*D2R);
    double zb = Math.Sin(latDeg*D2R);
    double t = (utc - s.Bulletin).TotalSeconds;
    double period = s.PeriodMin * 60.0;
    double n0 = 2*Math.PI / period;
    double revs = t / period;
    double n = n0 - 1.5 * (s.ADriftMPerDay/1000.0) / s.A / 86400.0 * 2*Math.PI * revs;
    double earthRevPerSec = s.NodeDrift * D2R / period;
    double m = n * t;
    double latS = Math.Asin(Math.Sin(m) * Math.Sin(s.Inc*D2R));
    double lonAn = Math.Atan(Math.Tan(m) * Math.Cos(s.Inc*D2R));
    if (Math.Cos(m) < 0) lonAn += Math.PI;
    double lonS = s.Node*D2R + lonAn + earthRevPerSec * t;
    double xs = Math.Cos(latS)*Math.Cos(lonS), ys = Math.Cos(latS)*Math.Sin(lonS), zs = Math.Sin(latS);
    double d2 = (xs-xb)*(xs-xb) + (ys-yb)*(ys-yb) + (zs-zb)*(zs-zb);
    double v = 2*Math.Asin(Math.Sqrt(d2)/2);            // central angle beacon-subsatellite
    return Math.Atan2(s.A*Math.Cos(v) - R, s.A*Math.Sin(v)) / D2R;
  }
}
"@
}

function Read-Aop([string]$Path) {
  $fs = [IO.File]::Open((Resolve-Path $Path).ProviderPath, 'Open', 'Read', 'ReadWrite')
  $rd = New-Object IO.StreamReader($fs); $txt = $rd.ReadToEnd(); $rd.Close()
  foreach ($l in ($txt -split "`r?`n")) {
    $f = $l.Trim() -split '\s+'
    if ($f.Count -lt 17) { continue }
    $s = New-Object AopSat
    $s.Name = $f[0]
    $s.Bulletin = [datetime]::SpecifyKind((New-Object DateTime ([int]$f[5]), ([int]$f[6]), ([int]$f[7]), ([int]$f[8]), ([int]$f[9]), ([int]$f[10])), 'Utc')
    $inv = [Globalization.CultureInfo]::InvariantCulture
    $s.A = [double]::Parse($f[11], $inv); $s.Inc = [double]::Parse($f[12], $inv); $s.Node = [double]::Parse($f[13], $inv)
    $s.NodeDrift = [double]::Parse($f[14], $inv); $s.PeriodMin = [double]::Parse($f[15], $inv); $s.ADriftMPerDay = [double]::Parse($f[16], $inv)
    $s
  }
}

# Elevation of every satellite at one instant, highest first.
function Get-SatElevations($Aop, [datetime]$Utc, [double]$Lat, [double]$Lon) {
  $Aop | ForEach-Object { [pscustomobject]@{ sat = $_.Name; elev = [SatElev]::Elevation($_, $Utc, $Lat, $Lon) } } | Sort-Object elev -Descending
}
