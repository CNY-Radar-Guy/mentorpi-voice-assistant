JARVIS v1 - fully local assistant on ALPHA-ONE
==============================================
Built Nov 23, 2026. Everything lives in D:\AI-Models\jarvis.

START / STOP
  Desktop shortcut "Jarvis"  -> runs start-jarvis.ps1 (hidden), opens an app window.
  start-jarvis.ps1           -> starts the server if it isn't running, warms up qwen3:14b,
                                opens http://127.0.0.1:8765 as an Edge app window (no tabs/address bar;
                                falls back to Brave, then the default browser).
                                Add -NoBrowser to only start the server.
  stop-jarvis.ps1            -> stops the Jarvis server only (Ollama and models are untouched).
  URL: http://127.0.0.1:8765  (bound to 127.0.0.1 only; not reachable from the LAN)

HOW IT WORKS
  server.js   - small Node.js server (uses the Node 20 already installed at C:\Program Files\nodejs;
                no npm packages, nothing installed). Talks to Ollama at 127.0.0.1:11434 with
                model qwen3:14b, think:false, streaming replies, and (when the model supports it)
                a function-calling loop for the shell agent (max 3 tool rounds per turn; models
                without tools fall back to normal chat).
  public\     - the app page (index.html) and icon image.
  config.json          - persona prompt, model, voice, accent colour, panels, quick commands, web settings.
  custom-commands.json - extra buttons; each sends its prompt to Jarvis.
  waiting.json         - the "Waiting on you" list (local only).
  bots.json            - local inventory of John's bots (Grok Bot, New Bot, Home robots) and local
                         Ollama models with what each is good for. Used by the Bots tab and Bot coach.
  shell-agent.js       - guarded local shell-access agent (Node built-ins only). Loaded by server.js.
  shell-policy.json    - editable allowlists, vendor domains, and timeouts. Restart Jarvis after edits.
  downloads\           - default folder for approved HTTPS downloads (also the user Downloads folder).
  backups\             - timestamped copies made before every approved Program change; history.json logs them.
  brave-profile\       - throwaway Brave profile used ONLY for headless page reads (never your own profile).
  logs\server.log      - server log (web approvals/denials, program changes, errors).
  logs\shell-agent-audit.jsonl - append-only audit of every proposed, blocked, approved, denied, and
                                 executed shell-agent action.
  jarvis.ico / make-icon.ps1 - the icon and the script that generated it.

SHELL AGENT (guarded, localhost only)
  Bound to the same 127.0.0.1:8765 server. Jarvis (qwen3:14b) can call these tools from chat:
    run_safe_command  - allowlisted read-only checks (hostname, whoami, Get-Date, Get-ComputerInfo,
                        Get-Process, Get-Service, Get-Volume, Get-PSDrive, Get-ChildItem, Test-Path,
                        where.exe, winget list/search, nvidia-smi, ollama list, ipconfig,
                        Test-NetConnection, Get-ItemProperty on uninstall keys). Runs immediately
                        when the validator passes. 15 s timeout, 16 KB output cap. No command
                        chaining, backticks, subexpressions or redirects. Safe trailing pipes only:
                        Select-Object, Format-Table, Format-List, Measure-Object.
    search_software   - winget search <query>, read-only, runs immediately.
    install_software  - propose winget install --id <exact_id> -e (or an official HTTPS installer
                        from an allowlisted vendor domain). NEVER runs until you click Approve & Run.
    download_file     - propose an HTTPS download into Downloads or D:\AI-Models\jarvis\downloads.
                        NEVER runs until you click Approve & Run.
    echo / Write-Output - simple Tier 1 text output (no chaining, redirects, or variable expansion).
    run_openscad      - render a .scad file in Desktop / Documents / Downloads / D:\AI-Models / TEMP
                        to .stl or .png via openscad.com (execFile argv, 180 s, Tier 1).
    run_bambu_studio  - Tier 1, no approval: launch Bambu Studio GUI detached (optional .stl/.3mf) or
                        run CLI --info / --slice / --export-3mf / --export-slicedata on safe-dir files.
                        Official exe only: C:\Program Files\Bambu Studio\bambu-studio.exe.

  Guardrail tiers:
    Tier 1  - read-only allowlist, plus echo/Write-Output, run_openscad, and run_bambu_studio
              (CLI and detached GUI launch): execute immediately, feed verified stdout back to Ollama.
              Bambu Studio is never gated by Approve & Run.
    Tier 2  - installs / downloads: in-memory approval (10 minute expiry, one-time token),
              Approve & Run / Deny card in chat. POST /api/shell-agent/approve or /deny.
    Hard-blocked - destructive wipes, piped remote execution (irm | iex), Invoke-Expression,
              EncodedCommand, credential/secret paths, shutdown/reboot, Defender/firewall
              tampering. Never allowed, even with approval.
  Policy file: D:\AI-Models\jarvis\shell-policy.json (edit allowed domains / commands, then restart).
  Audit log:   D:\AI-Models\jarvis\logs\shell-agent-audit.jsonl
  The header shows "Shell Agent: Guarded (localhost)"; click it to open the audit viewer.
  Chat messages show a compact Tool Activity panel (Executed / Blocked by guardrail / Awaiting approval).

THE SCREEN
  Systems (left): live, refreshed every 5 s from this PC
    - Ollama up/down, version, which model is loaded and its VRAM
    - GPU: utilization, temperature, VRAM, power (nvidia-smi)
    - CPU %, memory used
    - C: and D: free space
    - Wally (MentorPi, 192.168.50.125): ping + whether TCP port 22 accepts a connection.
      Read-only: the connection is closed immediately, nothing is sent, no SSH, no commands.
      Checked at most every 30 s.
    - Tailscale: Windows service state + login state (e.g. "needs login")
    - Sparklines: GPU utilization (blue) and VRAM (purple) under the GPU row, memory under Memory;
      last 30 samples (~2.5 min), kept only while the window is open.
  Bots tab (left, next to Systems): the bots.json inventory and local models. Local only -
    Jarvis cannot contact Grok Bot, New Bot or Home robots; it's for planning and coaching.
  Core (centre): glows; pulses while speaking, spins faster while thinking.
    The answer is typed out under it and spoken aloud.
  Quick commands: Morning brief, Status, What's waiting, Focus mode, Bot coach + your custom commands (dashed).
    Bot coach: qwen3:14b reads bots.json + live status + waiting list and suggests routines to
    create, tasks to delegate, a role for New Bot, and which local model fits which job.
  Ctrl+K (Cmd+K): command palette. Fuzzy-search quick/custom commands, Bot coach, Chat/Program mode,
    Focus/Voice/Web-session toggles, Bots/Systems tabs, Undo. Up/Down + Enter runs, Esc closes;
    text that matches nothing is sent to Jarvis on Enter. Also the "Ctrl K" button in the top bar.
  Waiting on you (right): Approve / Later / Undo / remove. These only tick off locally in waiting.json -
    nothing is sent anywhere and nothing is actually approved on Wally.
  Program history (right): list of applied self-edits, with "Undo last".
  Top bar: Chat / Program mode, Shell Agent indicator (click for audit log),
    "Web: authorize for session" (default OFF), Voice, Focus.

COMMANDS (type in the bar)
  add to waiting: <text>          remove from waiting: <text>
  search: <query>                 open: https://...
  program: <change>               (or switch to Program mode)
  anything else -> Jarvis answers using a live snapshot of the Systems data, and may call the
                   guarded shell agent when you ask for a live check, a winget search, or an install.
  Esc stops speech.

PROGRAM MODE (Jarvis programs itself)
  Describe a change ("make the persona more formal", "add a command called GPU temp...",
  "hide the history panel", "slow the voice down"). Jarvis proposes edits to config.json,
  custom-commands.json, waiting.json or bots.json ONLY, shown as a diff. Nothing changes until you click
  "Approve & apply". bots.json is editable the same way (e.g. "give New Bot the job-search role").
  Each apply backs up the old files to backups\ first. "Undo last" restores them.
  Jarvis cannot edit its own code, or write outside D:\AI-Models\jarvis, via Program mode.
  Shell-agent installs and downloads also never run until you click Approve & Run in chat.
  It also refuses switching to a model that isn't already installed (no downloads).
  Manual restore of any backup: copy backups\<stamp>__config.json over config.json.

WEB (via Brave, only when you authorize)
  Jarvis asks for the web by showing an Authorize / Deny card (Brave search or a specific URL).
  Nothing is fetched until you click Authorize. On Authorize it:
    1) opens the page in your normal Brave window so you can see it, and
    2) reads the page text with Brave headless using the separate brave-profile folder
       (no cookies/logins from your real profile), falling back to a plain HTTPS fetch,
    3) has qwen3:14b summarize it.
  Read-only: no forms, logins or purchases. Local/LAN addresses are refused.
  "Web: authorize for session" auto-approves until you close/reload the window. Default off.
  Brave found at: %LOCALAPPDATA%\BraveSoftware\Brave-Browser\Application\brave.exe
  Search page used: https://search.brave.com/search?q=  (Brave may show a captcha to automated
  reads at times; Jarvis will say so instead of guessing.)

VOICE
  Output: the browser's speech synthesis. It prefers a local Windows voice (e.g. Microsoft David/Mark).
    Edge's "Online (Natural)" voices sound better but are cloud voices; choose one only by setting
    voice.voice_name in config.json (or ask in Program mode) and voice.prefer_local_voice=false.
  Input (mic button): OFF by default and asks before first use. Edge/Chrome speech recognition sends
    your audio to Microsoft/Google cloud servers - it is NOT local. Typing is fully local.

PRIVACY
  Chat, status, Program mode and the shell agent run entirely on this PC (Ollama + local server).
  The only network traffic is: Wally ping/port check on your LAN, web pages you authorize,
  approved HTTPS downloads or official installers from the vendor allowlist, and (if you turn it on)
  browser voice input. The shell agent is not reachable from the LAN.

KNOWN LIMITS (v1)
  - First answer after a cold start can take minutes: qwen3:14b (9.3 GB) loads from the USB drive D:.
    start-jarvis.ps1 warms it up and keeps it loaded for 30 min after last use.
  - It sees this PC's status snapshot, plus any verified shell-agent output it requested.
    It can't contact the cloud bots, read email, or control Wally.
  - One Jarvis server per PC; conversation memory is the last few turns in the open window only.
