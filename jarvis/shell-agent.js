'use strict';
// Jarvis guarded local shell-access agent.
// Loaded by server.js, bound to the same 127.0.0.1 server. Node built-ins only.
const fs = require('fs');
const path = require('path');
const os = require('os');
const crypto = require('crypto');
const https = require('https');
const { execFile, execFileSync, spawn } = require('child_process');

const DEFAULT_POLICY = {
  version: 1,
  bind: '127.0.0.1',
  timeouts: { read_only_ms: 15000, cad_ms: 180000, mutating_ms: 300000 },
  output_cap_bytes: 16384,
  approval_ttl_ms: 600000,
  download_dirs: ['%USERPROFILE%\\Downloads', 'D:\\AI-Models\\jarvis\\downloads'],
  safe_cad_dirs: ['%USERPROFILE%\\Desktop', '%USERPROFILE%\\Documents', '%USERPROFILE%\\Downloads', 'D:\\AI-Models', '%TEMP%'],
  allowed_domains: [
    'github.com', 'objects.githubusercontent.com', 'bambulab.com', 'download.microsoft.com',
    'aka.ms', 'ollama.com', 'python.org', 'nodejs.org', 'git-scm.com', 'mozilla.org',
    'videolan.org', '7-zip.org', 'notepad-plus-plus.org'
  ],
  tier1_commands: [
    'hostname', 'whoami', 'Get-Date', 'Get-ComputerInfo', 'Get-Process', 'Get-Service',
    'Get-Volume', 'Get-PSDrive', 'Get-ChildItem', 'dir', 'ls', 'Test-Path', 'where.exe',
    'winget list', 'winget search', 'nvidia-smi', 'ollama list', 'ipconfig',
    'Test-NetConnection', 'Get-ItemProperty', 'echo', 'Write-Output',
    'openscad', 'openscad.com', 'openscad.exe', 'bambu-studio', 'bambu-studio.exe'
  ],
  safe_pipes: ['Select-Object', 'Select', 'Format-Table', 'FT', 'Format-List', 'FL', 'Measure-Object', 'Measure'],
  uninstall_property_paths: [
    'HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall',
    'HKLM:\\SOFTWARE\\WOW6432Node\\Microsoft\\Windows\\CurrentVersion\\Uninstall',
    'HKCU:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall'
  ]
};

// Hard-blocked: never allowed, not even with approval.
const HARD_BLOCKS = [
  { re: /\brm\s+-[a-zA-Z]*r[a-zA-Z]*f\b|\brm\s+-[a-zA-Z]*f[a-zA-Z]*r\b/, why: 'recursive force delete' },
  { re: /\bRemove-Item\b[\s\S]*-Recurse\b/i, why: 'recursive Remove-Item' },
  { re: /\bdel\s+\/s\b/i, why: 'del /s' },
  { re: /\b(rd|rmdir)\s+\/s\b/i, why: 'rd /s' },
  { re: /\bformat\s+[a-zA-Z]:|\bformat\s+\/\b/i, why: 'format' },
  { re: /\bdiskpart\b/i, why: 'diskpart' },
  { re: /\bbcdedit\b/i, why: 'bcdedit' },
  { re: /\bcipher\b/i, why: 'cipher' },
  { re: /\breg(?:\.exe)?\s+delete\b/i, why: 'reg delete' },
  { re: /\bSet-MpPreference\b/i, why: 'Set-MpPreference' },
  { re: /\bStop-Service\b[\s\S]*WinDefend/i, why: 'stop Windows Defender' },
  { re: /\bnetsh\s+advfirewall\b[\s\S]*\boff\b/i, why: 'disable firewall' },
  { re: /\b(irm|iwr|Invoke-WebRequest|Invoke-RestMethod)\b[\s\S]*\|\s*(iex|Invoke-Expression)\b/i, why: 'piped remote execution' },
  { re: /\bInvoke-Expression\b/i, why: 'Invoke-Expression' },
  { re: /(?:^|[\s;&(|])iex\b/i, why: 'iex' },
  { re: /\-(?:EncodedCommand|encodedcommand|enc)\b/, why: 'EncodedCommand' },
  { re: /(?:^|[\s"'=\\/])\.ssh(?:\b|\\|\/)/i, why: 'ssh key path' },
  { re: /\bid_rsa\b/i, why: 'private key' },
  { re: /\b(?:credential|credentials|Login Data|cookies|wallet)\b/i, why: 'credential or secret material' },
  { re: /\b(?:seed[-_ ]?phrase|wallet[-_ ]?seed|mnemonic)\b/i, why: 'wallet seed' },
  { re: /\b(?:shutdown(?:\.exe)?|restart-computer|stop-computer|reboot)\b/i, why: 'shutdown or reboot' },
  { re: /\bClear-Disk\b|\bInitialize-Disk\b|\bRemove-Partition\b/i, why: 'disk wipe' }
];

const OLLAMA_TOOLS = [
  {
    type: 'function',
    function: {
      name: 'run_safe_command',
      description: 'Run ONE short allowlisted read-only command on this PC and return live stdout. Allowed commands only: hostname, whoami, Get-Date, Get-ComputerInfo, Get-Process, Get-Service, Get-Volume, Get-PSDrive, Get-ChildItem, Test-Path, where.exe, winget list, winget search, nvidia-smi, ollama list, ipconfig, Test-NetConnection, Get-ItemProperty (uninstall keys only). One command per call; no semicolons, chaining, Get-WmiObject or Get-CimInstance. Examples: "hostname" ; "Get-PSDrive -PSProvider FileSystem | Select-Object Name, Used, Free" ; "Get-Volume". Do not invent results.',
      parameters: {
        type: 'object',
        properties: {
          command: { type: 'string', description: 'The exact allowlisted read-only command to run. Safe trailing pipes are limited to Select-Object, Format-Table, Format-List, or Measure-Object.' },
          reason: { type: 'string', description: 'One short sentence on why this command is needed.' }
        },
        required: ['command']
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'search_software',
      description: 'Search available packages with winget search <query>. Read-only; runs immediately and returns package IDs and versions.',
      parameters: {
        type: 'object',
        properties: {
          query: { type: 'string', description: 'Search text, for example Bambu Studio or BambuLab.BambuStudio.' }
        },
        required: ['query']
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'install_software',
      description: 'Propose installing or upgrading a package. Never runs immediately. Returns approval_required so John can click Approve & Run in the Jarvis chat UI. Prefer a exact winget ID, or an official HTTPS installer URL on an allowlisted vendor domain.',
      parameters: {
        type: 'object',
        properties: {
          id: { type: 'string', description: 'Exact winget package ID, for example BambuLab.BambuStudio.' },
          url: { type: 'string', description: 'Optional official HTTPS installer URL on an allowlisted vendor domain.' },
          reason: { type: 'string', description: 'Why this install is being proposed.' }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'download_file',
      description: 'Propose downloading a file over HTTPS from an allowlisted vendor domain into Downloads or the Jarvis downloads folder. Never runs immediately; requires John to click Approve & Run.',
      parameters: {
        type: 'object',
        properties: {
          url: { type: 'string', description: 'HTTPS URL on an allowlisted domain.' },
          filename: { type: 'string', description: 'Optional safe filename (no path). Defaults to the URL basename.' },
          dest_dir: { type: 'string', description: 'Optional destination directory: the user Downloads folder or D:\\AI-Models\\jarvis\\downloads.' },
          reason: { type: 'string', description: 'Why this download is being proposed.' }
        },
        required: ['url']
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'run_openscad',
      description: 'Run the installed OpenSCAD CLI to render a .scad file in a safe user directory (Desktop, Documents, Downloads, D:\\AI-Models, TEMP) to a .stl or .png. Uses execFile with an argv array (no shell). Allowed flags: -o, -D key=val, --backend=Manifold, --imgsize, --camera, --autocenter, --viewall. 180 second timeout. If OpenSCAD is not installed, returns openscad_not_installed. Runs immediately (Tier 1).',
      parameters: {
        type: 'object',
        properties: {
          input: { type: 'string', description: 'Absolute path to a .scad file inside a safe user directory.' },
          output: { type: 'string', description: 'Absolute path for the .stl or .png output, also inside a safe user directory.' },
          defines: { type: 'object', description: 'Optional OpenSCAD -D assignments, e.g. { part: 1, preview: false }. Keys must be identifiers; values must be numbers, booleans, or simple tokens.' },
          backend: { type: 'string', description: 'Optional OpenSCAD backend. Only Manifold is allowed.' },
          imgsize: { type: 'string', description: 'Optional preview size as WIDTH,HEIGHT, e.g. 800,600.' },
          camera: { type: 'string', description: 'Optional --camera value (numbers and commas only).' },
          autocenter: { type: 'boolean', description: 'Pass --autocenter.' },
          viewall: { type: 'boolean', description: 'Pass --viewall.' },
          reason: { type: 'string', description: 'Why this render is being run.' }
        },
        required: ['input', 'output']
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'run_bambu_studio',
      description: 'Tier 1 (runs immediately, no approval). Run Bambu Studio at C:\\Program Files\\Bambu Studio\\bambu-studio.exe. Use action=gui (or launch_gui=true) to open the GUI, optionally with a .stl/.3mf file, spawned detached so the request does not block. Use action=info, slice, export-3mf, or export-slicedata for CLI work on .stl/.3mf/.json files in Desktop, Documents, Downloads, D:\\AI-Models, or TEMP. Allowed flags: --info, --slice, --export-3mf, --export-slicedata, --load-settings, --load-filaments, --outputdir, --arrange, --orient, --debug. execFile argv only, 180 second CLI timeout.',
      parameters: {
        type: 'object',
        properties: {
          action: { type: 'string', description: 'gui, info, slice, export-3mf, or export-slicedata. gui launches the Bambu Studio window detached.' },
          file: { type: 'string', description: 'Optional .stl or .3mf (or .json for settings) inside a safe user directory.' },
          input: { type: 'string', description: 'Alias for file.' },
          outputdir: { type: 'string', description: 'Optional --outputdir folder inside a safe user directory.' },
          load_settings: { type: 'string', description: 'Optional --load-settings .json path in a safe user directory.' },
          load_filaments: { type: 'string', description: 'Optional --load-filaments .json path in a safe user directory.' },
          arrange: { type: 'boolean', description: 'Pass --arrange.' },
          orient: { type: 'boolean', description: 'Pass --orient.' },
          debug: { type: 'string', description: 'Optional --debug value (simple token) or true for the bare flag.' },
          launch_gui: { type: 'boolean', description: 'If true, launch the Bambu Studio GUI detached (Tier 1, no approval).' },
          reason: { type: 'string', description: 'Why this is being run.' }
        }
      }
    }
  }
];

function expandEnv(s) {
  return String(s || '').replace(/%([^%]+)%/g, (_, k) => process.env[k] || '').replace(/\$\{([^}]+)\}/g, (_, k) => process.env[k] || '');
}
function nowIso() { return new Date().toISOString(); }
function rid() { return crypto.randomBytes(8).toString('hex'); }
function truncate(s, n) { s = String(s || ''); return s.length > n ? s.slice(0, n) + '\n…[truncated]' : s; }
function isSubdomainOrExact(host, domain) {
  host = String(host || '').toLowerCase(); domain = String(domain || '').toLowerCase();
  return host === domain || host.endsWith('.' + domain);
}

function createShellAgent({ root, logsDir, log = () => {} }) {
  const policyPath = path.join(root, 'shell-policy.json');
  const auditPath = path.join(logsDir, 'shell-agent-audit.jsonl');
  const localDownloads = path.join(root, 'downloads');
  fs.mkdirSync(logsDir, { recursive: true });
  fs.mkdirSync(localDownloads, { recursive: true });
  const pending = new Map();

  function loadPolicy() {
    try {
      const raw = JSON.parse(fs.readFileSync(policyPath, 'utf8'));
      return {
        ...DEFAULT_POLICY, ...raw,
        timeouts: { ...DEFAULT_POLICY.timeouts, ...(raw.timeouts || {}) },
        allowed_domains: Array.isArray(raw.allowed_domains) && raw.allowed_domains.length ? raw.allowed_domains : DEFAULT_POLICY.allowed_domains,
        tier1_commands: Array.isArray(raw.tier1_commands) && raw.tier1_commands.length ? raw.tier1_commands : DEFAULT_POLICY.tier1_commands,
        safe_pipes: Array.isArray(raw.safe_pipes) && raw.safe_pipes.length ? raw.safe_pipes : DEFAULT_POLICY.safe_pipes,
        uninstall_property_paths: Array.isArray(raw.uninstall_property_paths) && raw.uninstall_property_paths.length ? raw.uninstall_property_paths : DEFAULT_POLICY.uninstall_property_paths,
        download_dirs: Array.isArray(raw.download_dirs) && raw.download_dirs.length ? raw.download_dirs : DEFAULT_POLICY.download_dirs,
        safe_cad_dirs: Array.isArray(raw.safe_cad_dirs) && raw.safe_cad_dirs.length ? raw.safe_cad_dirs : DEFAULT_POLICY.safe_cad_dirs
      };
    } catch {
      return { ...DEFAULT_POLICY };
    }
  }

  function audit(entry) {
    const row = { ts: nowIso(), ...entry };
    try { fs.appendFileSync(auditPath, JSON.stringify(row) + '\n'); } catch (e) { log('audit write failed', e.message); }
    log('shell-agent', entry.verdict || entry.status || '', entry.tool || '', (entry.command || entry.query || entry.url || '').toString().slice(0, 180));
    return row;
  }

  function readAudit(limit = 50) {
    try {
      const lines = fs.readFileSync(auditPath, 'utf8').split('\n').filter(Boolean);
      return lines.slice(-Math.max(1, Math.min(200, limit))).map(l => { try { return JSON.parse(l); } catch { return { raw: l }; } });
    } catch { return []; }
  }

  function policySummary(p) {
    return {
      bind: p.bind || '127.0.0.1',
      tier1: 'Allowlisted read-only commands run immediately (hostname, whoami, echo/Write-Output, Get-Date, Get-ComputerInfo, Get-Process, Get-Service, Get-Volume, Get-PSDrive, Get-ChildItem, Test-Path, where.exe, winget list/search, nvidia-smi, ollama list, ipconfig, Test-NetConnection, Get-ItemProperty on uninstall keys). CAD/slicer: run_openscad and run_bambu_studio (CLI slice/info/export and detached Bambu Studio GUI launch) also run immediately in Tier 1 with execFile argv, 180 s timeout, and safe-directory path checks. Safe pipes: Select-Object, Format-Table, Format-List, Measure-Object.',
      tier2: 'install_software and download_file always return approval_required and never run until John clicks Approve & Run. Approvals expire in 10 minutes and are one-time use.',
      hard_blocked: 'Destructive wipes, piped remote execution (irm|iex), Invoke-Expression, EncodedCommand, credential/secret paths, shutdown/reboot, firewall/Defender tampering — never allowed, even with approval.',
      audit_log: auditPath,
      policy_file: policyPath
    };
  }

  function hardBlock(text) {
    const s = String(text || '');
    for (const b of HARD_BLOCKS) if (b.re.test(s)) return b.why;
    return null;
  }

  function tokenize(command) {
    const tokens = []; let cur = '', quote = null;
    for (let i = 0; i < command.length; i++) {
      const ch = command[i];
      if (quote) {
        if (ch === quote) quote = null;
        else cur += ch;
        continue;
      }
      if (ch === '"' || ch === "'") { quote = ch; continue; }
      if (/\s/.test(ch)) { if (cur) { tokens.push(cur); cur = ''; } continue; }
      cur += ch;
    }
    if (quote) throw new Error('unclosed quote');
    if (cur) tokens.push(cur);
    return tokens;
  }

  function firstToken(segment) {
    const t = tokenize(segment.trim());
    return t[0] || '';
  }

  function hasIllegalMeta(command) {
    // No chaining, substitution, redirection, or call operator. Newlines are illegal.
    if (/[\r\n]/.test(command)) return 'newlines are not allowed';
    if (/`/.test(command)) return 'backticks are not allowed';
    if (/;/.test(command)) return 'semicolons are not allowed';
    if (/&&|\|\|/.test(command)) return 'command chaining is not allowed';
    if (/\$\(|\$\{/.test(command)) return 'subexpressions are not allowed';
    if (/(^|[\s])&/.test(command) || /&$/.test(command.trim())) return 'call operator is not allowed';
    if (/[<>]/.test(command) && !/Test-NetConnection/i.test(command)) return 'redirection is not allowed';
    if (/#/.test(command)) return 'comments are not allowed';
    return null;
  }

  function allowedDestDirs(p) {
    const dirs = (p.download_dirs || []).map(expandEnv).map(d => path.resolve(d));
    const extras = [path.resolve(localDownloads), path.resolve(process.env.USERPROFILE || os.homedir(), 'Downloads')];
    const all = [...dirs, ...extras];
    const uniq = [];
    for (const d of all) if (d && !uniq.some(x => x.toLowerCase() === d.toLowerCase())) uniq.push(d);
    return uniq;
  }

  function isAllowedHost(host, p) {
    return (p.allowed_domains || []).some(d => isSubdomainOrExact(host, d));
  }

  function parseHttpsUrl(u, p) {
    let x;
    try { x = new URL(String(u || '')); } catch { return { error: 'invalid URL' }; }
    if (x.protocol !== 'https:') return { error: 'only https:// URLs are allowed' };
    if (!isAllowedHost(x.hostname, p)) return { error: `domain ${x.hostname} is not on the vendor allowlist` };
    if (x.username || x.password) return { error: 'URLs with credentials are not allowed' };
    return { url: x };
  }


  function cadRoots(p) {
    const home = process.env.USERPROFILE || os.homedir();
    const listed = (p && p.safe_cad_dirs) || [];
    const defaults = [
      path.join(home, 'Desktop'), path.join(home, 'Documents'), path.join(home, 'Downloads'),
      'D:\\AI-Models', process.env.TEMP || '', process.env.TMP || '', os.tmpdir()
    ];
    const all = [];
    for (const d of [...listed.map(expandEnv), ...defaults]) {
      if (!d) continue;
      const resolved = path.resolve(d);
      if (!all.some(x => x.toLowerCase() === resolved.toLowerCase())) all.push(resolved);
    }
    return all;
  }
  function isInsideDir(file, root) {
    const a = path.resolve(file).replace(/\//g, '\\').toLowerCase();
    const b = path.resolve(root).replace(/\//g, '\\').toLowerCase();
    return a === b || a.startsWith(b.endsWith('\\') ? b : b + '\\');
  }
  function safeUserPath(file, p, opts = {}) {
    if (file == null || file === '') return { error: 'missing path' };
    const raw = expandEnv(String(file).trim());
    if (!raw) return { error: 'empty path' };
    if (/[\0\r\n]/.test(raw)) return { error: 'invalid path characters' };
    if (/^\\\\/.test(raw)) return { error: 'UNC paths are not allowed' };
    let resolved;
    try { resolved = path.resolve(raw); } catch { return { error: 'invalid path' }; }
    if (resolved.split(/[\\/]/).includes('..')) return { error: 'path traversal is not allowed' };
    if (!cadRoots(p).some(r => isInsideDir(resolved, r))) {
      return { error: 'path must be inside Desktop, Documents, Downloads, D:\\AI-Models, or TEMP' };
    }
    if (opts.mustExist && !fs.existsSync(resolved)) return { error: 'path does not exist: ' + resolved, path: resolved };
    if (opts.exts) {
      const ext = path.extname(resolved).toLowerCase();
      if (!opts.exts.map(e => e.toLowerCase()).includes(ext)) {
        return { error: 'file extension must be ' + opts.exts.join(', '), path: resolved };
      }
    }
    return { path: resolved };
  }
  function officialBambuPath() {
    const candidates = [
      'C:\\Program Files\\Bambu Studio\\bambu-studio.exe',
      path.join(process.env.ProgramFiles || 'C:\\Program Files', 'Bambu Studio', 'bambu-studio.exe'),
      path.join(process.env['ProgramFiles(x86)'] || 'C:\\Program Files (x86)', 'Bambu Studio', 'bambu-studio.exe')
    ];
    return candidates.find(c => c && fs.existsSync(c)) || null;
  }
  function findOpenScadBin() {
    const found = [];
    const roots = [
      process.env.ProgramFiles || 'C:\\Program Files',
      process.env['ProgramFiles(x86)'] || 'C:\\Program Files (x86)',
      path.join(process.env.LOCALAPPDATA || '', 'Programs')
    ];
    for (const root of roots) {
      if (!root || !fs.existsSync(root)) continue;
      let entries = [];
      try { entries = fs.readdirSync(root); } catch { continue; }
      for (const name of entries) {
        if (!/openscad/i.test(name)) continue;
        for (const bin of ['openscad.com', 'openscad.exe']) {
          const cand = path.join(root, name, bin);
          if (fs.existsSync(cand)) found.push(cand);
        }
      }
    }
    for (const name of ['openscad.com', 'openscad.exe', 'openscad']) {
      try {
        const out = execFileSync('where.exe', [name], { windowsHide: true, encoding: 'utf8', timeout: 5000 });
        for (const line of out.split(/\r?\n/).map(s => s.trim()).filter(Boolean)) {
          if (fs.existsSync(line)) found.push(line);
        }
      } catch { }
    }
    const uniq = [...new Set(found)];
    uniq.sort((a, b) => Number(b.toLowerCase().endsWith('.com')) - Number(a.toLowerCase().endsWith('.com')));
    return uniq[0] || null;
  }
  function isOpenScadHead(name) {
    const base = path.basename(String(name || '')).toLowerCase();
    return base === 'openscad' || base === 'openscad.exe' || base === 'openscad.com';
  }
  function isBambuHead(name) {
    const base = path.basename(String(name || '')).toLowerCase();
    return base === 'bambu-studio' || base === 'bambu-studio.exe';
  }
  function isOfficialBambuHead(name) {
    if (!isBambuHead(name)) return false;
    if (!/[\\/]/.test(name)) return true;
    const official = officialBambuPath();
    return official && path.resolve(name).toLowerCase() === path.resolve(official).toLowerCase();
  }
  function parseDefine(raw) {
    const s = String(raw || '').trim();
    const m = s.match(/^([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+)$/);
    if (!m) return { error: 'invalid -D assignment: ' + s };
    const val = m[2].trim();
    if (!/^(?:-?\d+(?:\.\d+)?|true|false|[A-Za-z_][A-Za-z0-9_]*|"[A-Za-z0-9_.+-]+"|'[A-Za-z0-9_.+-]+')$/.test(val)) {
      return { error: 'unsafe -D value for ' + m[1] };
    }
    return { text: m[1] + '=' + val };
  }
  function buildOpenScadArgv(spec, p) {
    const input = safeUserPath(spec.input, p, { mustExist: true, exts: ['.scad'] });
    if (input.error) return { error: input.error };
    const output = safeUserPath(spec.output, p, { exts: ['.stl', '.png'] });
    if (output.error) return { error: output.error };
    const argv = ['-o', output.path];
    const defines = spec.defines;
    if (defines && typeof defines === 'object' && !Array.isArray(defines)) {
      for (const [k, v] of Object.entries(defines)) {
        const parsed = parseDefine(k + '=' + String(v));
        if (parsed.error) return parsed;
        argv.push('-D', parsed.text);
      }
    } else if (Array.isArray(defines)) {
      for (const d of defines) {
        const parsed = parseDefine(d);
        if (parsed.error) return parsed;
        argv.push('-D', parsed.text);
      }
    }
    const backend = String(spec.backend || 'Manifold').trim() || 'Manifold';
    if (!/^Manifold$/i.test(backend)) return { error: 'only --backend=Manifold is allowed' };
    argv.push('--backend=' + backend);
    if (spec.imgsize) {
      const img = typeof spec.imgsize === 'object' ? (spec.imgsize.width + ',' + spec.imgsize.height) : String(spec.imgsize).trim();
      if (!/^\d{2,5},\d{2,5}$/.test(img)) return { error: 'imgsize must be WIDTH,HEIGHT' };
      argv.push('--imgsize=' + img);
    }
    if (spec.camera) {
      const cam = String(spec.camera).trim();
      if (!/^[0-9.,eE+\-]+$/.test(cam) || cam.length > 120) return { error: 'invalid --camera value' };
      argv.push('--camera=' + cam);
    }
    if (spec.autocenter) argv.push('--autocenter');
    if (spec.viewall) argv.push('--viewall');
    argv.push(input.path);
    return { argv, input: input.path, output: output.path };
  }
  function parseOpenScadArgv(tokens, p) {
    const spec = { defines: [], autocenter: false, viewall: false };
    const rest = tokens.slice();
    while (rest.length) {
      const tkn = rest.shift();
      if (tkn === '-o' || tkn === '--o') {
        if (!rest.length) return { error: '-o requires an output path' };
        spec.output = rest.shift();
      } else if (tkn.startsWith('-o=') || tkn.startsWith('--o=')) {
        spec.output = tkn.split('=').slice(1).join('=');
      } else if (tkn === '-D' || tkn === '--define') {
        if (!rest.length) return { error: '-D requires an assignment' };
        spec.defines.push(rest.shift());
      } else if (tkn.startsWith('-D') && tkn.length > 2) {
        spec.defines.push(tkn.slice(2).replace(/^=/, ''));
      } else if (tkn.startsWith('--backend=')) {
        spec.backend = tkn.slice(11);
      } else if (tkn === '--backend') {
        if (!rest.length) return { error: '--backend requires a value' };
        spec.backend = rest.shift();
      } else if (tkn.startsWith('--imgsize=')) {
        spec.imgsize = tkn.slice(10);
      } else if (tkn === '--imgsize') {
        if (!rest.length) return { error: '--imgsize requires a value' };
        spec.imgsize = rest.shift();
      } else if (tkn.startsWith('--camera=')) {
        spec.camera = tkn.slice(9);
      } else if (tkn === '--camera') {
        if (!rest.length) return { error: '--camera requires a value' };
        spec.camera = rest.shift();
      } else if (tkn === '--autocenter') {
        spec.autocenter = true;
      } else if (tkn === '--viewall') {
        spec.viewall = true;
      } else if (tkn.startsWith('-') ) {
        return { error: 'OpenSCAD flag not allowed: ' + tkn };
      } else if (!spec.input) {
        spec.input = tkn;
      } else {
        return { error: 'unexpected OpenSCAD argument: ' + tkn };
      }
    }
    if (!spec.input || !spec.output) return { error: 'OpenSCAD requires an input .scad and -o output .stl or .png' };
    return buildOpenScadArgv(spec, p);
  }
  const BAMBU_FLAGS = new Set(['--info', '--slice', '--export-3mf', '--export-slicedata', '--load-settings', '--load-filaments', '--outputdir', '--arrange', '--orient', '--debug']);
  const BAMBU_VALUE_FLAGS = new Set(['--load-settings', '--load-filaments', '--outputdir', '--export-3mf', '--export-slicedata', '--debug']);
  function looksLikePath(s) { return /[\\/]|\.(stl|3mf|json|png|gcode)$/i.test(s); }
  function parseBambuArgv(tokens, p) {
    const argv = [];
    const files = [];
    let launch_gui = tokens.length === 0;
    let hasCli = false;
    const rest = tokens.slice();
    while (rest.length) {
      const tkn = rest.shift();
      if (tkn.startsWith('--')) {
        const eq = tkn.indexOf('=');
        const flag = eq === -1 ? tkn : tkn.slice(0, eq);
        const inline = eq === -1 ? null : tkn.slice(eq + 1);
        if (!BAMBU_FLAGS.has(flag)) return { error: 'Bambu Studio flag not allowed: ' + flag };
        if (flag !== '--debug' || inline != null || (rest[0] && !rest[0].startsWith('--') && !looksLikePath(rest[0]) && !/\.(stl|3mf)$/i.test(rest[0]))) {
          /* classified below */
        }
        if (['--info', '--slice', '--export-3mf', '--export-slicedata', '--load-settings', '--load-filaments', '--outputdir'].includes(flag)) hasCli = true;
        if (flag === '--arrange' || flag === '--orient' || flag === '--info') {
          argv.push(flag);
          continue;
        }
        if (flag === '--debug') {
          argv.push('--debug');
          const val = inline != null ? inline : (rest[0] && !rest[0].startsWith('--') && !looksLikePath(rest[0]) ? rest.shift() : null);
          if (val != null) {
            if (!/^[A-Za-z0-9_-]{1,20}$/.test(val)) return { error: 'invalid --debug value' };
            argv.push(val);
          }
          continue;
        }
        if (BAMBU_VALUE_FLAGS.has(flag) || flag === '--slice') {
          const val = inline != null ? inline : (rest[0] && !rest[0].startsWith('--') ? rest.shift() : null);
          if (flag === '--slice' && (val == null || val.startsWith('--'))) { argv.push('--slice'); if (val) rest.unshift(val); continue; }
          if (val == null) {
            if (flag === '--slice' || flag === '--export-3mf' || flag === '--export-slicedata') { argv.push(flag); continue; }
            return { error: flag + ' requires a value' };
          }
          if (flag === '--outputdir') {
            const dir = safeUserPath(val, p, {});
            if (dir.error) return dir;
            argv.push('--outputdir', dir.path);
          } else if (flag === '--load-settings' || flag === '--load-filaments') {
            const f = safeUserPath(val, p, { mustExist: true, exts: ['.json'] });
            if (f.error) return f;
            argv.push(flag, f.path);
          } else if (flag === '--export-3mf') {
            const f = safeUserPath(val, p, { exts: ['.3mf'] });
            if (f.error) return f;
            argv.push(flag, f.path);
          } else if (flag === '--export-slicedata') {
            const f = safeUserPath(val, p, {});
            if (f.error) return f;
            argv.push(flag, f.path);
          } else if (flag === '--slice') {
            const f = safeUserPath(val, p, { mustExist: true, exts: ['.stl', '.3mf'] });
            if (f.error) return f;
            argv.push('--slice', f.path);
            files.push(f.path);
          } else {
            return { error: 'unhandled flag ' + flag };
          }
          continue;
        }
        argv.push(flag);
      } else {
        const f = safeUserPath(tkn, p, { mustExist: true, exts: ['.stl', '.3mf', '.json'] });
        if (f.error) return f;
        argv.push(f.path);
        files.push(f.path);
      }
    }
    if (!hasCli) launch_gui = true;
    return { argv, files, launch_gui, hasCli };
  }
  function buildBambuFromSpec(spec, p) {
    const action = String(spec.action || (spec.launch_gui ? 'gui' : '')).toLowerCase().replace(/^--/, '');
    const file = spec.file || spec.input || null;
    if (spec.args && Array.isArray(spec.args)) return parseBambuArgv(spec.args.map(String), p);
    if (action === 'gui' || spec.launch_gui === true || (!action && !spec.outputdir && !spec.load_settings && !spec.load_filaments && !spec.arrange && !spec.orient && !spec.debug)) {
      const argv = [];
      if (file) {
        const f = safeUserPath(file, p, { mustExist: true, exts: ['.stl', '.3mf'] });
        if (f.error) return f;
        argv.push(f.path);
      }
      return { argv, files: argv.slice(), launch_gui: true, hasCli: false };
    }
    const tokens = [];
    if (action === 'info') tokens.push('--info');
    else if (action === 'slice') tokens.push('--slice');
    else if (action === 'export-3mf') tokens.push('--export-3mf');
    else if (action === 'export-slicedata') tokens.push('--export-slicedata');
    else if (action) return { error: 'unknown Bambu Studio action: ' + action };
    if (file) tokens.push(file);
    if (spec.outputdir) { tokens.push('--outputdir', spec.outputdir); }
    if (spec.load_settings) { tokens.push('--load-settings', spec.load_settings); }
    if (spec.load_filaments) { tokens.push('--load-filaments', spec.load_filaments); }
    if (spec.arrange) tokens.push('--arrange');
    if (spec.orient) tokens.push('--orient');
    if (spec.debug === true) tokens.push('--debug');
    else if (spec.debug) { tokens.push('--debug', String(spec.debug)); }
    return parseBambuArgv(tokens, p);
  }

  async function execOpenScad(built, p, meta, tool, reason) {
    const bin = findOpenScadBin();
    if (!bin) {
      const rec = { status: 'error', code: 'openscad_not_installed', tool, reason: 'OpenSCAD is not installed on this PC (no openscad.com or openscad.exe on PATH or under Program Files). Install OpenSCAD.OpenSCAD via winget if you want renders from Jarvis.', command: 'openscad ' + built.argv.map(a => /\s/.test(a) ? '"' + a + '"' : a).join(' ') };
      audit({ tool, verdict: 'error', status: 'error', code: 'openscad_not_installed', command: rec.command, source: meta.source });
      return rec;
    }
    const timeout = (p.timeouts && p.timeouts.cad_ms) || 180000;
    const cap = p.output_cap_bytes || 16384;
    const parent = path.dirname(built.output);
    if (!fs.existsSync(parent)) fs.mkdirSync(parent, { recursive: true });
    const command = bin + ' ' + built.argv.map(a => /\s/.test(a) ? '"' + a + '"' : a).join(' ');
    const ran = await execFileP(bin, built.argv, timeout, cap);
    let size = 0;
    try { if (fs.existsSync(built.output)) size = fs.statSync(built.output).size; } catch { }
    audit({ tool, verdict: ran.ok ? 'executed' : 'error', status: ran.ok ? 'executed' : 'error', command, exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), stderr: truncate(ran.stderr, 1000), output: built.output, bytes: size, source: meta.source });
    return {
      status: ran.ok ? 'executed' : 'error', tool, command, reason: reason || '',
      exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr, timedOut: ran.timedOut,
      output: built.output, bytes: size, guardrail: 'tier1'
    };
  }

  function launchBambuGui(exe, argv) {
    return new Promise((resolve) => {
      try {
        const child = spawn(exe, argv, { detached: true, stdio: 'ignore', windowsHide: false, cwd: path.dirname(exe) });
        child.once('error', (e) => resolve({ ok: false, exitCode: 1, stdout: '', stderr: String(e.message || e), launched: false }));
        child.unref();
        // Give the process a brief moment to fail on spawn, then report launched.
        setTimeout(() => resolve({ ok: true, exitCode: 0, stdout: 'Launched Bambu Studio GUI (detached, Tier 1, no approval).', stderr: '', launched: true, pid: child.pid }), 400);
      } catch (e) {
        resolve({ ok: false, exitCode: 1, stdout: '', stderr: String(e.message || e), launched: false });
      }
    });
  }

  async function execBambu(parsed, p, meta, tool, reason) {
    const exe = officialBambuPath();
    if (!exe) {
      const rec = { status: 'error', code: 'bambu_studio_not_installed', tool, reason: 'Bambu Studio was not found at C:\\Program Files\\Bambu Studio\\bambu-studio.exe', command: 'bambu-studio.exe' };
      audit({ tool, verdict: 'error', status: 'error', code: 'bambu_studio_not_installed', source: meta.source });
      return rec;
    }
    const command = exe + (parsed.argv.length ? ' ' + parsed.argv.map(a => /\s/.test(a) ? '"' + a + '"' : a).join(' ') : '');
    if (parsed.launch_gui && !parsed.hasCli) {
      const ran = await launchBambuGui(exe, parsed.argv);
      audit({ tool, verdict: ran.ok ? 'executed' : 'error', status: ran.ok ? 'executed' : 'error', command, exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), stderr: truncate(ran.stderr, 1000), launched: !!ran.launched, source: meta.source });
      return {
        status: ran.ok ? 'executed' : 'error', tool, command, reason: reason || '',
        exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr, launched: !!ran.launched,
        pid: ran.pid, guardrail: 'tier1'
      };
    }
    const timeout = (p.timeouts && p.timeouts.cad_ms) || 180000;
    const cap = p.output_cap_bytes || 16384;
    const ran = await execFileP(exe, parsed.argv, timeout, cap);
    audit({ tool, verdict: ran.ok ? 'executed' : 'error', status: ran.ok ? 'executed' : 'error', command, exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), stderr: truncate(ran.stderr, 1000), source: meta.source });
    return {
      status: ran.ok ? 'executed' : 'error', tool, command, reason: reason || '',
      exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr, timedOut: ran.timedOut,
      guardrail: 'tier1'
    };
  }

  async function toolOpenScad(args, p, meta) {
    const built = buildOpenScadArgv(args, p);
    if (built.error) {
      audit({ tool: 'run_openscad', verdict: 'blocked', status: 'blocked', args, reason: built.error, source: meta.source });
      return { status: 'blocked', tool: 'run_openscad', reason: built.error, guardrail: 'hard' };
    }
    return execOpenScad(built, p, meta, 'run_openscad', args.reason);
  }
  async function toolBambu(args, p, meta) {
    const parsed = buildBambuFromSpec(args, p);
    if (parsed.error) {
      audit({ tool: 'run_bambu_studio', verdict: 'blocked', status: 'blocked', args, reason: parsed.error, source: meta.source });
      return { status: 'blocked', tool: 'run_bambu_studio', reason: parsed.error, guardrail: 'hard' };
    }
    return execBambu(parsed, p, meta, 'run_bambu_studio', args.reason);
  }


    function validateCommand(command, p) {
    const raw = String(command || '').trim();
    if (!raw) return { ok: false, status: 'blocked', reason: 'empty command', tier: 'hard' };
    const why = hardBlock(raw);
    if (why) return { ok: false, status: 'blocked', reason: 'hard-blocked: ' + why, tier: 'hard' };
    const meta = hasIllegalMeta(raw);
    if (meta) return { ok: false, status: 'blocked', reason: meta, tier: 'hard' };

    const parts = raw.split('|').map(s => s.trim());
    if (parts.some(s => !s)) return { ok: false, status: 'blocked', reason: 'empty pipeline segment', tier: 'hard' };

    const safePipes = new Set((p.safe_pipes || []).map(s => s.toLowerCase()));
    let headName;
    try { headName = firstToken(parts[0]); } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
    if (!headName) return { ok: false, status: 'blocked', reason: 'missing command', tier: 'hard' };

    if (/^(echo|write-output)$/i.test(headName)) {
      if (parts.length > 1) return { ok: false, status: 'blocked', reason: 'pipes are not allowed with echo / Write-Output', tier: 'hard' };
      if (/\$|%[A-Za-z_][^%]*%/.test(raw)) return { ok: false, status: 'blocked', reason: 'environment or variable expansion is not allowed in echo', tier: 'hard' };
      return { ok: true, status: 'allowed', tier: 'tier1', route: 'echo', command: raw };
    }
    if (isOpenScadHead(headName)) {
      if (parts.length > 1) return { ok: false, status: 'blocked', reason: 'pipes are not allowed with OpenSCAD', tier: 'hard' };
      let toks;
      try { toks = tokenize(parts[0]); } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
      const built = parseOpenScadArgv(toks.slice(1), p);
      if (built.error) return { ok: false, status: 'blocked', reason: built.error, tier: 'hard' };
      return { ok: true, status: 'allowed', tier: 'tier1', route: 'openscad', command: raw, argv: built.argv, output: built.output, input: built.input };
    }
    if (isBambuHead(headName)) {
      if (parts.length > 1) return { ok: false, status: 'blocked', reason: 'pipes are not allowed with Bambu Studio', tier: 'hard' };
      if (!isOfficialBambuHead(headName)) return { ok: false, status: 'blocked', reason: 'bambu-studio.exe may only be launched from the official install path', tier: 'hard' };
      let toks;
      try { toks = tokenize(parts[0]); } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
      const parsed = parseBambuArgv(toks.slice(1), p);
      if (parsed.error) return { ok: false, status: 'blocked', reason: parsed.error, tier: 'hard' };
      return { ok: true, status: 'allowed', tier: 'tier1', route: 'bambu', command: raw, argv: parsed.argv, launch_gui: parsed.launch_gui, hasCli: parsed.hasCli };
    }

    const tier1 = p.tier1_commands || [];
    const headOk = tier1.some(allowed => {
      const a = allowed.trim();
      if (a.toLowerCase() === headName.toLowerCase()) return true;
      // "winget list" / "winget search" / "ollama list" are two-token prefixes
      if (a.includes(' ')) {
        const prefix = a.split(/\s+/);
        try {
          const toks = tokenize(parts[0]);
          return toks.length >= prefix.length && prefix.every((t, i) => toks[i].toLowerCase() === t.toLowerCase());
        } catch { return false; }
      }
      return false;
    });
    if (!headOk) return { ok: false, status: 'blocked', reason: `"${headName}" is not on the read-only allowlist`, tier: 'hard' };

    for (let i = 1; i < parts.length; i++) {
      let name;
      try { name = firstToken(parts[i]); } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
      if (!safePipes.has(name.toLowerCase())) {
        return { ok: false, status: 'blocked', reason: `pipe to ${name} is not allowed (only Select-Object, Format-Table, Format-List, Measure-Object)`, tier: 'hard' };
      }
    }

    if (headName.toLowerCase() === 'get-itemproperty') {
      const allowedPaths = (p.uninstall_property_paths || []).map(s => s.toLowerCase());
      const pathTok = parts[0].match(/-Path\s+(['"]?)([^'"\s]+)\1/i) || parts[0].match(/\s(['"]?)(HK[LMCU]:\\[^'"\s]+)\1/i);
      const rp = pathTok ? pathTok[2] : '';
      if (!rp || !allowedPaths.some(ap => rp.toLowerCase().startsWith(ap.toLowerCase()))) {
        return { ok: false, status: 'blocked', reason: 'Get-ItemProperty is only allowed on uninstall registry keys', tier: 'hard' };
      }
    }

    if (headName.toLowerCase() === 'ollama') {
      try {
        const toks = tokenize(parts[0]);
        if (!toks[1] || toks[1].toLowerCase() !== 'list') {
          return { ok: false, status: 'blocked', reason: 'only "ollama list" is allowlisted', tier: 'hard' };
        }
      } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
    }

    if (headName.toLowerCase() === 'winget') {
      try {
        const toks = tokenize(parts[0]);
        const sub = (toks[1] || '').toLowerCase();
        if (sub !== 'list' && sub !== 'search' && sub !== 'show' && sub !== '--info') {
          return { ok: false, status: 'blocked', reason: 'only winget list/search (read-only) may run via run_safe_command; installs require install_software', tier: 'hard' };
        }
      } catch (e) { return { ok: false, status: 'blocked', reason: e.message, tier: 'hard' }; }
    }

    return { ok: true, status: 'allowed', tier: 'tier1', command: raw };
  }

  function execFileP(file, args, timeout, cap) {
    return new Promise(resolve => {
      const child = execFile(file, args, { timeout, windowsHide: true, maxBuffer: Math.max(cap * 4, 64 * 1024) }, (err, stdout, stderr) => {
        const killed = err && (err.killed || err.code === 'ERR_CHILD_PROCESS_STDIO_MAXBUFFER');
        const exitCode = err && typeof err.code === 'number' ? err.code : (err ? 1 : 0);
        resolve({
          ok: !err,
          exitCode: err && err.killed ? -1 : exitCode,
          stdout: truncate(stdout, cap),
          stderr: truncate(stderr || (err && err.message) || '', cap),
          timedOut: !!(err && err.killed),
          truncated: String(stdout || '').length > cap || String(stderr || '').length > cap,
          error: killed && err.killed ? 'timed out' : (err && !stdout && !stderr ? err.message : undefined)
        });
      });
      // Belt and suspenders in case timeout doesn't fire.
      const killer = setTimeout(() => { try { child.kill(); } catch { } }, timeout + 1000);
      child.on('exit', () => clearTimeout(killer));
    });
  }

  async function runValidatedCommand(command, p) {
    const timeout = p.timeouts.read_only_ms || 15000;
    const cap = p.output_cap_bytes || 16384;
    let head;
    try { head = firstToken(command.split('|')[0]); } catch { head = ''; }
    const direct = {
      hostname: 'hostname',
      whoami: 'whoami',
      ipconfig: 'ipconfig',
      'where.exe': 'where.exe',
      'nvidia-smi': 'nvidia-smi'
    };
    const key = head.toLowerCase();
    if (direct[key] && !command.includes('|')) {
      const args = tokenize(command).slice(1);
      return execFileP(direct[key], args, timeout, cap);
    }
    if (key === 'winget' && !command.includes('|')) {
      const args = tokenize(command).slice(1);
      return execFileP('winget', args, timeout, cap);
    }
    if (key === 'ollama' && !command.includes('|')) {
      const args = tokenize(command).slice(1);
      return execFileP('ollama', args, timeout, cap);
    }
    return execFileP('powershell.exe', ['-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-Command', command], timeout, cap);
  }

  function queueApproval(rec) {
    const approval_id = rid();
    const p = loadPolicy();
    const item = {
      ...rec,
      approval_id,
      created: Date.now(),
      expires: Date.now() + (p.approval_ttl_ms || 600000),
      used: false
    };
    pending.set(approval_id, item);
    audit({ tool: rec.tool, verdict: 'proposed', status: 'approval_required', approval_id, command: rec.preview, args: rec.args, reason: rec.reason });
    return {
      status: 'approval_required',
      approval_id,
      tool: rec.tool,
      command: rec.preview,
      reason: rec.reason || '',
      vendor: rec.vendor || null,
      domain: rec.domain || null,
      dest: rec.dest || null,
      guardrail: 'tier2',
      expires_ms: p.approval_ttl_ms || 600000,
      summary: rec.summary,
      policy: policySummary(p)
    };
  }

  function expired(item) { return !item || item.used || Date.now() > item.expires; }

  async function invoke(name, rawArgs, meta = {}) {
    const p = loadPolicy();
    const args = rawArgs && typeof rawArgs === 'object' && !Array.isArray(rawArgs) ? rawArgs : {};
    const tool = String(name || '');
    try {
      if (tool === 'run_safe_command') return await toolRunSafe(args, p, meta);
      if (tool === 'search_software') return await toolSearch(args, p, meta);
      if (tool === 'install_software') return await toolInstall(args, p, meta);
      if (tool === 'download_file') return await toolDownload(args, p, meta);
      if (tool === 'run_openscad') return await toolOpenScad(args, p, meta);
      if (tool === 'run_bambu_studio') return await toolBambu(args, p, meta);
      const blocked = { status: 'blocked', tool, reason: 'unknown tool', guardrail: 'hard' };
      audit({ tool, verdict: 'blocked', status: 'blocked', args, reason: blocked.reason, source: meta.source });
      return blocked;
    } catch (e) {
      const err = { status: 'error', tool, reason: String(e.message || e) };
      audit({ tool, verdict: 'error', status: 'error', args, reason: err.reason, source: meta.source });
      return err;
    }
  }

  async function toolRunSafe(args, p, meta) {
    const command = String(args.command || '').trim();
    const v = validateCommand(command, p);
    if (!v.ok) {
      audit({ tool: 'run_safe_command', verdict: 'blocked', status: 'blocked', command, reason: v.reason, source: meta.source });
      return { status: 'blocked', tool: 'run_safe_command', command, reason: v.reason, guardrail: v.tier || 'hard' };
    }
    if (v.route === 'openscad') {
      const result = await execOpenScad({ argv: v.argv, input: v.input, output: v.output }, p, meta, 'run_safe_command', args.reason);
      return result;
    }
    if (v.route === 'bambu') {
      const result = await execBambu({ argv: v.argv, launch_gui: v.launch_gui, hasCli: v.hasCli }, p, meta, 'run_safe_command', args.reason);
      return result;
    }
    const ran = await runValidatedCommand(v.command, p);
    audit({
      tool: 'run_safe_command', verdict: 'executed', status: 'executed', command: v.command,
      exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), stderr: truncate(ran.stderr, 1000), source: meta.source
    });
    return {
      status: 'executed', tool: 'run_safe_command', command: v.command, reason: args.reason || '',
      exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr, timedOut: ran.timedOut, truncated: ran.truncated
    };
  }

  async function toolSearch(args, p, meta) {
    const query = String(args.query || '').trim();
    if (!query) return { status: 'blocked', tool: 'search_software', reason: 'empty query', guardrail: 'hard' };
    if (!/^[A-Za-z0-9 ._+#@()[\]\-]{1,80}$/.test(query)) {
      const reason = 'query has disallowed characters or is too long';
      audit({ tool: 'search_software', verdict: 'blocked', status: 'blocked', query, reason, source: meta.source });
      return { status: 'blocked', tool: 'search_software', query, reason, guardrail: 'hard' };
    }
    const why = hardBlock(query);
    if (why) {
      audit({ tool: 'search_software', verdict: 'blocked', status: 'blocked', query, reason: why, source: meta.source });
      return { status: 'blocked', tool: 'search_software', query, reason: 'hard-blocked: ' + why, guardrail: 'hard' };
    }
    const command = `winget search ${query}`;
    // Reuse validator so the same allowlist path is audited.
    const v = validateCommand(`winget search ${JSON.stringify(query).slice(1, -1) === query ? query : '"' + query.replace(/"/g, '') + '"'}`, p);
    const ran = await execFileP('winget', ['search', query], p.timeouts.read_only_ms || 15000, p.output_cap_bytes || 16384);
    audit({ tool: 'search_software', verdict: 'executed', status: 'executed', query, command, exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), source: meta.source });
    return { status: 'executed', tool: 'search_software', query, command, exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr };
  }

  async function toolInstall(args, p, meta) {
    const id = String(args.id || '').trim();
    const url = String(args.url || '').trim();
    const reason = String(args.reason || '').trim();
    if (id && url) { /* allow id to be primary; url optional fallback not mixed in one command */ }
    if (!id && !url) return { status: 'blocked', tool: 'install_software', reason: 'provide a winget id or an official HTTPS installer URL', guardrail: 'hard' };

    if (id) {
      if (!/^[A-Za-z0-9][A-Za-z0-9._+\-]{0,80}$/.test(id)) {
        const blocked = { status: 'blocked', tool: 'install_software', reason: 'invalid winget package id', guardrail: 'hard', id };
        audit({ tool: 'install_software', verdict: 'blocked', status: 'blocked', args, reason: blocked.reason, source: meta.source });
        return blocked;
      }
      const why = hardBlock(id);
      if (why) {
        audit({ tool: 'install_software', verdict: 'blocked', status: 'blocked', id, reason: why, source: meta.source });
        return { status: 'blocked', tool: 'install_software', id, reason: 'hard-blocked: ' + why, guardrail: 'hard' };
      }
      const preview = `winget install --id ${id} -e --accept-package-agreements --accept-source-agreements`;
      return queueApproval({
        tool: 'install_software', kind: 'winget', id, args, reason, preview,
        run: { type: 'winget', id },
        vendor: 'winget', domain: null,
        summary: `Install or upgrade ${id} via winget. This will not run until you click Approve & Run. Guardrails: exact package ID only, official source agreements accepted, localhost-only agent, audited.`
      });
    }

    const parsed = parseHttpsUrl(url, p);
    if (parsed.error) {
      audit({ tool: 'install_software', verdict: 'blocked', status: 'blocked', url, reason: parsed.error, source: meta.source });
      return { status: 'blocked', tool: 'install_software', url, reason: parsed.error, guardrail: 'hard' };
    }
    const why = hardBlock(url);
    if (why) {
      audit({ tool: 'install_software', verdict: 'blocked', status: 'blocked', url, reason: why, source: meta.source });
      return { status: 'blocked', tool: 'install_software', url, reason: 'hard-blocked: ' + why, guardrail: 'hard' };
    }
    const destDir = path.resolve(localDownloads);
    const filename = safeFilename(path.posix.basename(parsed.url.pathname) || 'installer.exe');
    const dest = path.join(destDir, filename);
    const preview = `Download ${parsed.url.href} to ${dest} and start the official installer`;
    return queueApproval({
      tool: 'install_software', kind: 'url', url: parsed.url.href, args, reason, preview,
      run: { type: 'url-install', url: parsed.url.href, dest },
      vendor: parsed.url.hostname, domain: parsed.url.hostname, dest,
      summary: `Download an official installer from ${parsed.url.hostname} and run it. This will not run until you click Approve & Run. Guardrails: HTTPS + vendor allowlist, audited, localhost-only.`
    });
  }

  function safeFilename(name) {
    let n = path.basename(String(name || '').replace(/\\/g, '/')).trim();
    n = n.replace(/[^\w.\- +()[\]]/g, '_').replace(/^\.+/, '');
    if (!n || n === '.' || n === '..') n = 'download.bin';
    if (n.length > 120) {
      const ext = path.extname(n).slice(0, 12);
      n = n.slice(0, 120 - ext.length) + ext;
    }
    return n;
  }

  function resolveDestDir(requested, p) {
    const allowed = allowedDestDirs(p);
    if (!requested) return allowed[0] || path.resolve(localDownloads);
    const resolved = path.resolve(expandEnv(requested));
    if (allowed.some(d => resolved.toLowerCase() === d.toLowerCase())) return resolved;
    return { error: 'destination must be the user Downloads folder or D:\\AI-Models\\jarvis\\downloads' };
  }

  async function toolDownload(args, p, meta) {
    const parsed = parseHttpsUrl(args.url, p);
    if (parsed.error) {
      audit({ tool: 'download_file', verdict: 'blocked', status: 'blocked', url: args.url, reason: parsed.error, source: meta.source });
      return { status: 'blocked', tool: 'download_file', url: args.url, reason: parsed.error, guardrail: 'hard' };
    }
    const why = hardBlock(String(args.url || '') + ' ' + String(args.filename || '') + ' ' + String(args.dest_dir || ''));
    if (why) {
      audit({ tool: 'download_file', verdict: 'blocked', status: 'blocked', url: args.url, reason: why, source: meta.source });
      return { status: 'blocked', tool: 'download_file', reason: 'hard-blocked: ' + why, guardrail: 'hard' };
    }
    const destDir = resolveDestDir(args.dest_dir, p);
    if (destDir.error) {
      audit({ tool: 'download_file', verdict: 'blocked', status: 'blocked', url: args.url, reason: destDir.error, source: meta.source });
      return { status: 'blocked', tool: 'download_file', reason: destDir.error, guardrail: 'hard' };
    }
    const filename = safeFilename(args.filename || path.posix.basename(parsed.url.pathname) || 'download.bin');
    if (filename.includes('..') || /[\\/]/.test(args.filename || '')) {
      audit({ tool: 'download_file', verdict: 'blocked', status: 'blocked', reason: 'path traversal in filename', source: meta.source });
      return { status: 'blocked', tool: 'download_file', reason: 'path traversal in filename is not allowed', guardrail: 'hard' };
    }
    const dest = path.join(destDir, filename);
    // Confirm dest stays inside the chosen dir after resolve.
    if (!path.resolve(dest).toLowerCase().startsWith(path.resolve(destDir).toLowerCase() + path.sep) && path.resolve(dest).toLowerCase() !== path.resolve(destDir).toLowerCase()) {
      return { status: 'blocked', tool: 'download_file', reason: 'resolved path escaped the destination directory', guardrail: 'hard' };
    }
    const preview = `Download ${parsed.url.href} to ${dest}`;
    return queueApproval({
      tool: 'download_file', kind: 'download', url: parsed.url.href, args, reason: args.reason || '', preview,
      run: { type: 'download', url: parsed.url.href, dest },
      vendor: parsed.url.hostname, domain: parsed.url.hostname, dest,
      summary: `Download a file from ${parsed.url.hostname} into ${destDir}. This will not run until you click Approve & Run. Guardrails: HTTPS + vendor allowlist + safe filename, audited, localhost-only.`
    });
  }

  function followHttps(urlStr, p, timeout, maxRedirects, cb) {
    let settled = false;
    const done = (err, res, finalUrl) => { if (settled) return; settled = true; cb(err, res, finalUrl); };
    let req;
    try {
      req = https.get(urlStr, { headers: { 'User-Agent': 'JarvisShellAgent/1.0' }, timeout }, res => {
        if (res.statusCode >= 300 && res.statusCode < 400 && res.headers.location) {
          res.resume();
          if (maxRedirects <= 0) return done(new Error('too many redirects'));
          let next;
          try { next = new URL(res.headers.location, urlStr); } catch { return done(new Error('invalid redirect')); }
          if (next.protocol !== 'https:') return done(new Error('redirect dropped HTTPS'));
          if (!isAllowedHost(next.hostname, p)) return done(new Error('redirect left the vendor allowlist: ' + next.hostname));
          return followHttps(next.href, p, timeout, maxRedirects - 1, cb);
        }
        if (res.statusCode !== 200) { res.resume(); return done(new Error('HTTP ' + res.statusCode)); }
        done(null, res, urlStr);
      });
    } catch (e) { return done(e); }
    req.on('error', done);
    req.on('timeout', () => { try { req.destroy(); } catch { } done(new Error('timeout')); });
  }

  function downloadToFile(urlStr, dest, p, timeout) {
    return new Promise((resolve, reject) => {
      fs.mkdirSync(path.dirname(dest), { recursive: true });
      followHttps(urlStr, p, timeout, 5, (err, res, finalUrl) => {
        if (err) return reject(err);
        const cap = 512 * 1024 * 1024;
        let written = 0;
        const tmp = dest + '.part';
        const ws = fs.createWriteStream(tmp);
        res.on('data', chunk => {
          written += chunk.length;
          if (written > cap) { try { res.destroy(); } catch { } try { ws.destroy(); } catch { } reject(new Error('file exceeded 512 MB cap')); }
        });
        res.pipe(ws);
        ws.on('finish', () => { try { fs.renameSync(tmp, dest); } catch (e) { return reject(e); } resolve({ dest, bytes: written, finalUrl }); });
        ws.on('error', e => { try { fs.unlinkSync(tmp); } catch { } reject(e); });
        res.on('error', e => { try { fs.unlinkSync(tmp); } catch { } reject(e); });
      });
    });
  }

  async function executePending(item) {
    const p = loadPolicy();
    const timeout = p.timeouts.mutating_ms || 300000;
    const cap = p.output_cap_bytes || 16384;
    const run = item.run || {};
    if (run.type === 'winget') {
      const cmd = `winget install --id ${run.id} -e --accept-package-agreements --accept-source-agreements`;
      const ran = await execFileP('winget', ['install', '--id', run.id, '-e', '--accept-package-agreements', '--accept-source-agreements'], timeout, cap);
      return { command: cmd, ...ran };
    }
    if (run.type === 'download' || run.type === 'url-install') {
      const dl = await downloadToFile(run.url, run.dest, p, timeout);
      if (run.type === 'download') {
        return { ok: true, exitCode: 0, command: `Download ${run.url} -> ${dl.dest}`, stdout: `Saved ${dl.bytes} bytes to ${dl.dest}`, stderr: '', dest: dl.dest, bytes: dl.bytes };
      }
      const ran = await execFileP(dl.dest, [], timeout, cap);
      return { command: `Download ${run.url} -> ${dl.dest} and start installer`, dest: dl.dest, bytes: dl.bytes, ...ran };
    }
    throw new Error('unknown pending action');
  }

  async function approve(approval_id) {
    const id = String(approval_id || '');
    const item = pending.get(id);
    if (!item) return { ok: false, status: 'error', reason: 'approval not found or already handled' };
    if (item.used) return { ok: false, status: 'error', reason: 'approval already used' };
    if (Date.now() > item.expires) {
      pending.delete(id);
      audit({ tool: item.tool, verdict: 'expired', status: 'expired', approval_id: id, command: item.preview });
      return { ok: false, status: 'expired', reason: 'approval expired (10 minute limit)', approval_id: id };
    }
    item.used = true;
    pending.delete(id);
    audit({ tool: item.tool, verdict: 'approved', status: 'approved', approval_id: id, command: item.preview, args: item.args });
    try {
      const ran = await executePending(item);
      audit({
        tool: item.tool, verdict: 'executed', status: 'executed', approval_id: id, command: ran.command || item.preview,
        exitCode: ran.exitCode, stdout: truncate(ran.stdout, 2000), stderr: truncate(ran.stderr, 1000)
      });
      return { ok: true, status: 'executed', approval_id: id, tool: item.tool, command: ran.command || item.preview, exitCode: ran.exitCode, stdout: ran.stdout, stderr: ran.stderr, dest: ran.dest, bytes: ran.bytes, timedOut: ran.timedOut };
    } catch (e) {
      audit({ tool: item.tool, verdict: 'error', status: 'error', approval_id: id, command: item.preview, reason: e.message });
      return { ok: false, status: 'error', approval_id: id, tool: item.tool, command: item.preview, reason: String(e.message || e) };
    }
  }

  function deny(approval_id) {
    const id = String(approval_id || '');
    const item = pending.get(id);
    if (!item) return { ok: false, status: 'error', reason: 'approval not found or already handled' };
    item.used = true;
    pending.delete(id);
    audit({ tool: item.tool, verdict: 'denied', status: 'denied', approval_id: id, command: item.preview, args: item.args });
    return { ok: true, status: 'denied', approval_id: id, tool: item.tool, command: item.preview };
  }

  function sweep() {
    const now = Date.now();
    for (const [id, item] of pending) {
      if (item.used || now > item.expires) pending.delete(id);
    }
  }

  function status() {
    sweep();
    const p = loadPolicy();
    return {
      ok: true,
      enabled: true,
      bind: '127.0.0.1',
      label: 'Shell Agent: Guarded (localhost)',
      pending: pending.size,
      policy_file: policyPath,
      audit_log: auditPath,
      policy: policySummary(p)
    };
  }

  function systemHint() {
    return `Guarded shell agent: call tools for live facts. For hostname run run_safe_command with command hostname. For a simple echo use run_safe_command with command echo plus the text, or Write-Output. For free disk space run Get-PSDrive -PSProvider FileSystem or Get-Volume. Allowed commands include hostname, whoami, echo, Write-Output, Get-Date, Get-ComputerInfo, Get-Process, Get-Service, Get-Volume, Get-PSDrive, Get-ChildItem, Test-Path, where.exe, winget list, winget search, nvidia-smi, ollama list, ipconfig, Test-NetConnection, Get-ItemProperty on uninstall keys. One command per tool call. Never use Get-WmiObject, Get-CimInstance, semicolons, &&, or pipes other than Select-Object, Format-Table, Format-List, Measure-Object. CAD/slicer: use run_openscad to render a .scad file in a safe user folder to .stl or .png. Use run_bambu_studio to launch the Bambu Studio GUI (detached, optional .stl/.3mf) or to run --info / --slice / --export-3mf / --export-slicedata. Those CAD tools run immediately (Tier 1) with no approval card. Installs and downloads still need Approve & Run. Never invent command output. Destructive commands are hard-blocked. Do not control Wally through the shell agent; use the control_wally tool instead. Never read credentials.`;
  }

  return {
    tools: OLLAMA_TOOLS,
    invoke,
    approve,
    deny,
    status,
    audit: readAudit,
    auditPath,
    policyPath,
    systemHint,
    validateCommand: (command) => validateCommand(command, loadPolicy()),
    pendingCount: () => { sweep(); return pending.size; }
  };
}

module.exports = { createShellAgent, OLLAMA_TOOLS };
