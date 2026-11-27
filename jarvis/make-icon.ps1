# Generates jarvis.ico (16-256 px, PNG-compressed entries) and public\jarvis.png using System.Drawing.
Add-Type -AssemblyName System.Drawing
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
function New-JarvisPng([int]$s) {
  $bmp = New-Object System.Drawing.Bitmap $s, $s, ([System.Drawing.Imaging.PixelFormat]::Format32bppArgb)
  $g = [System.Drawing.Graphics]::FromImage($bmp)
  $g.SmoothingMode = 'AntiAlias'; $g.InterpolationMode = 'HighQualityBicubic'; $g.Clear([System.Drawing.Color]::Transparent)
  # rounded-square background
  $r = [single]($s * 0.22); $rect = New-Object System.Drawing.RectangleF 0, 0, ($s - 1), ($s - 1)
  $path = New-Object System.Drawing.Drawing2D.GraphicsPath
  $path.AddArc($rect.X, $rect.Y, 2*$r, 2*$r, 180, 90); $path.AddArc($rect.Right - 2*$r, $rect.Y, 2*$r, 2*$r, 270, 90)
  $path.AddArc($rect.Right - 2*$r, $rect.Bottom - 2*$r, 2*$r, 2*$r, 0, 90); $path.AddArc($rect.X, $rect.Bottom - 2*$r, 2*$r, 2*$r, 90, 90); $path.CloseFigure()
  $bg = New-Object System.Drawing.Drawing2D.LinearGradientBrush $rect, ([System.Drawing.Color]::FromArgb(255,26,30,42)), ([System.Drawing.Color]::FromArgb(255,10,11,16)), 90
  $g.FillPath($bg, $path)
  # outer glow
  $c = $s / 2.0
  $gp = New-Object System.Drawing.Drawing2D.GraphicsPath; $gr = $s * 0.40; $gp.AddEllipse([single]($c-$gr), [single]($c-$gr), [single](2*$gr), [single](2*$gr))
  $pgb = New-Object System.Drawing.Drawing2D.PathGradientBrush $gp
  $pgb.CenterColor = [System.Drawing.Color]::FromArgb(150,124,156,255); $pgb.SurroundColors = @([System.Drawing.Color]::FromArgb(0,124,156,255))
  $g.FillPath($pgb, $gp)
  # ring
  if ($s -ge 24) { $pw = [single][Math]::Max(1.0, $s * 0.025); $pen = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(170,150,175,255)), $pw
    $rr = $s * 0.30; $g.DrawEllipse($pen, [single]($c-$rr), [single]($c-$rr), [single](2*$rr), [single](2*$rr)) }
  # core
  $cp = New-Object System.Drawing.Drawing2D.GraphicsPath; $cr = $s * 0.17; $cp.AddEllipse([single]($c-$cr), [single]($c-$cr), [single](2*$cr), [single](2*$cr))
  $cb = New-Object System.Drawing.Drawing2D.PathGradientBrush $cp
  $cb.CenterColor = [System.Drawing.Color]::FromArgb(255,255,255,255); $cb.SurroundColors = @([System.Drawing.Color]::FromArgb(255,124,156,255))
  $g.FillPath($cb, $cp)
  $g.Dispose()
  $ms = New-Object System.IO.MemoryStream; $bmp.Save($ms, [System.Drawing.Imaging.ImageFormat]::Png); $bmp.Dispose()
  return ,$ms.ToArray()
}
$sizes = 16, 24, 32, 48, 64, 128, 256
$pngs = foreach ($s in $sizes) { ,(New-JarvisPng $s) }
$ms = New-Object System.IO.MemoryStream; $bw = New-Object System.IO.BinaryWriter $ms
$bw.Write([uint16]0); $bw.Write([uint16]1); $bw.Write([uint16]$sizes.Count)
$offset = 6 + 16 * $sizes.Count
for ($i = 0; $i -lt $sizes.Count; $i++) {
  $s = $sizes[$i]; $d = $pngs[$i]; $dim = if ($s -ge 256) { 0 } else { $s }
  $bw.Write([byte]$dim); $bw.Write([byte]$dim); $bw.Write([byte]0); $bw.Write([byte]0)
  $bw.Write([uint16]1); $bw.Write([uint16]32); $bw.Write([uint32]$d.Length); $bw.Write([uint32]$offset); $offset += $d.Length
}
foreach ($d in $pngs) { $bw.Write([byte[]]$d) }
$bw.Flush(); [IO.File]::WriteAllBytes((Join-Path $root 'jarvis.ico'), $ms.ToArray())
New-Item -ItemType Directory -Force (Join-Path $root 'public') | Out-Null
[IO.File]::WriteAllBytes((Join-Path $root 'public\jarvis.png'), $pngs[-1])
"Wrote jarvis.ico ($($ms.Length) bytes, sizes $($sizes -join ',')) and public\jarvis.png"
