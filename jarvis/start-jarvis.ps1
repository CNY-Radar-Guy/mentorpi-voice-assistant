# Starts the Jarvis server (hidden, loopback only) and opens it as an app window.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$url  = 'http://127.0.0.1:8765'
$node = (Get-Command node.exe -ErrorAction SilentlyContinue).Source
if (-not $node) { $node = 'C:\Program Files\nodejs\node.exe' }
function Test-Jarvis { try { (Invoke-RestMethod "$url/api/health" -TimeoutSec 2).ok } catch { $false } }
if (-not (Test-Jarvis)) {
  New-Item -ItemType Directory -Force (Join-Path $root 'logs') | Out-Null
  Start-Process -FilePath $node -ArgumentList "`"$root\server.js`"" -WorkingDirectory $root -WindowStyle Hidden `
    -RedirectStandardError (Join-Path $root 'logs\node-stderr.log') -RedirectStandardOutput (Join-Path $root 'logs\node-stdout.log')
  for ($i = 0; $i -lt 40 -and -not (Test-Jarvis); $i++) { Start-Sleep -Milliseconds 250 }
  if (-not (Test-Jarvis)) { Add-Type -AssemblyName PresentationFramework; [System.Windows.MessageBox]::Show("Jarvis server did not start. See $root\logs", 'Jarvis') | Out-Null; exit 1 }
}
# Warm the model in the background so the first answer is quicker.
# Uses warmup-model.ps1: a real /api/chat with num_ctx 8192, num_predict 1, think false, keep_alive 30m
# (matching server.js chat). Logs success/failure to logs\warmup.log. Timeout is 10 minutes because a
# cold load from the USB HDD on D: can take ~4 minutes. Do not use -Command with an inline JSON body;
# Start-Process quoting previously stripped quotes and Ollama returned HTTP 400.
$warmupPs1 = Join-Path $root 'warmup-model.ps1'
Start-Process powershell.exe -WindowStyle Hidden -ArgumentList '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $warmupPs1
if ($args -contains '-NoBrowser') { exit 0 }
$edge = @("${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe", "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe") | Where-Object { Test-Path $_ } | Select-Object -First 1
$brave = @("$env:ProgramFiles\BraveSoftware\Brave-Browser\Application\brave.exe", "$env:LOCALAPPDATA\BraveSoftware\Brave-Browser\Application\brave.exe") | Where-Object { Test-Path $_ } | Select-Object -First 1
$browser = if ($edge) { $edge } elseif ($brave) { $brave } else { $null }
if ($browser) { Start-Process $browser -ArgumentList "--app=$url", '--window-size=1840,900' } else { Start-Process $url }
