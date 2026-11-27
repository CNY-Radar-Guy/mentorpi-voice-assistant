# Stops the Jarvis server. Does not touch Ollama or loaded models.
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$procs = Get-CimInstance Win32_Process -Filter "Name='node.exe'" | Where-Object { $_.CommandLine -like '*AI-Models\jarvis\server.js*' }
foreach ($p in $procs) { Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue; "Stopped Jarvis server PID $($p.ProcessId)" }
if (-not $procs) { 'Jarvis server was not running.' }
Remove-Item (Join-Path $root 'jarvis.pid') -ErrorAction SilentlyContinue
