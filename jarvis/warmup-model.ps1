# Preload qwen3:14b with the same runner Jarvis chat uses: num_ctx 8192, think false, keep_alive 30m.
# Invoked in the background by start-jarvis.ps1. Logs to logs\warmup.log.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$logDir = Join-Path $root 'logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$log = Join-Path $logDir 'warmup.log'

function Write-WarmupLog([string]$msg) {
  $line = '{0:yyyy-MM-dd HH:mm:ss zzz} {1}' -f (Get-Date), $msg
  Add-Content -LiteralPath $log -Value $line -Encoding utf8
}

$uri = 'http://127.0.0.1:11434/api/chat'
$body = '{"model":"qwen3:14b","messages":[{"role":"user","content":"ok"}],"stream":false,"think":false,"keep_alive":"30m","options":{"num_ctx":8192,"num_predict":1,"temperature":0}}'
$reqFile = Join-Path $logDir 'warmup-last-request.json'
$respFile = Join-Path $logDir 'warmup-last-response.json'
$hdrFile = Join-Path $logDir 'warmup-last-headers.txt'
[IO.File]::WriteAllText($reqFile, $body, [Text.UTF8Encoding]::new($false))

Write-WarmupLog "Warmup start: POST $uri (num_ctx=8192 num_predict=1 think=false keep_alive=30m)"
$sw = [Diagnostics.Stopwatch]::StartNew()
try {
  # 10-minute cap: a cold load of the 8.64 GiB blob from the USB HDD can exceed 3 minutes.
  $code = & curl.exe -sS -D $hdrFile -o $respFile -w '%{http_code}' --max-time 600 -X POST $uri -H 'Content-Type: application/json' --data-binary "@$reqFile"
  $sw.Stop()
  $secs = [math]::Round($sw.Elapsed.TotalSeconds, 1)
  $resp = ''
  if (Test-Path -LiteralPath $respFile) { $resp = (Get-Content -LiteralPath $respFile -Raw -ErrorAction SilentlyContinue) }
  $snip = if ($resp) { ($resp.Trim() -replace '\s+', ' ').Substring(0, [Math]::Min(400, $resp.Trim().Length)) } else { '(empty)' }
  if ($code -eq '200') {
    Write-WarmupLog "Warmup SUCCESS HTTP $code in ${secs}s body=$snip"
    exit 0
  }
  Write-WarmupLog "Warmup FAILED HTTP $code in ${secs}s body=$snip"
  exit 1
} catch {
  $sw.Stop()
  $secs = [math]::Round($sw.Elapsed.TotalSeconds, 1)
  Write-WarmupLog "Warmup FAILED after ${secs}s: $($_.Exception.Message)"
  exit 1
}