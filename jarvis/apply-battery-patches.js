'use strict';
// One-shot patcher for Jarvis battery logging. Safe to re-run (skips applied patches).
const fs = require('fs');
const path = require('path');

const ROOT = process.argv[2] || __dirname;
const serverPath = path.join(ROOT, 'server.js');
const htmlPath = path.join(ROOT, 'public', 'index.html');

function stamp() {
  const d = new Date();
  const p = (n, w = 2) => String(n).padStart(w, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

function backup(file, tag) {
  const dir = path.dirname(file);
  const base = path.basename(file);
  const dest = path.join(dir, `${base}.bak-${tag}-${stamp()}`);
  fs.copyFileSync(file, dest);
  return dest;
}

function applyOnce(text, needle, replacement, label) {
  if (text.includes(replacement.trim()) && !needReapply(text, needle, replacement)) {
    console.log('skip already applied: ' + label);
    return text;
  }
  const count = text.split(needle).length - 1;
  if (count !== 1) {
    throw new Error(`${label}: expected exactly 1 match for needle, found ${count}`);
  }
  return text.replace(needle, replacement);
}

function needReapply() { return false; }

function nl(text) {
  return text.includes('\r\n') ? '\r\n' : '\n';
}

function patchServer(src) {
  const N = nl(src);
  let t = src;

  t = applyOnce(
    t,
    `const { createWallyDrive } = require('./wally-drive');`,
    `const { createWallyDrive } = require('./wally-drive');${N}const { createWallyBattery } = require('./wally-battery');`,
    'server require'
  );

  t = applyOnce(
    t,
    `const wallyPanelCache = { t: 0, panel: null };`,
    `const wallyPanelCache = { t: 0, panel: null };${N}let wallyBattery = null;`,
    'wallyBattery binding'
  );

  t = applyOnce(
    t,
    `  rememberWallyPanel(panel);${N}  return { result, panel };`,
    `  rememberWallyPanel(panel);${N}  try { if (wallyBattery) wallyBattery.ingest(panel); } catch (e) { log('wally-battery ingest', e.message); }${N}  return { result, panel };`,
    'ingest hook'
  );

  t = applyOnce(
    t,
    `  refreshStatus: async () => (await loadWallyPanel()).panel${N}});`,
    `  refreshStatus: async () => (await loadWallyPanel()).panel${N}});${N}wallyBattery = createWallyBattery({${N}  root: ROOT,${N}  log,${N}  loadPanel: () => loadWallyPanel(),${N}  getCachedPanel: () => wallyPanelCache${N}});`,
    'createWallyBattery'
  );

  t = applyOnce(
    t,
    `    if (req.method === 'GET' && p === '/api/wally/status') {`,
    `    if (req.method === 'GET' && p === '/api/wally/battery-health') {${N}      return send(res, 200, wallyBattery.health());${N}    }${N}    if (req.method === 'GET' && p === '/api/wally/status') {`,
    'battery-health route'
  );

  t = applyOnce(
    t,
    `server.listen(PORT, HOST, () => { log(\`Jarvis listening on http://\${HOST}:\${PORT} pid \${process.pid}\`); fs.writeFileSync(path.join(ROOT, 'jarvis.pid'), String(process.pid)); });`,
    `server.listen(PORT, HOST, () => { log(\`Jarvis listening on http://\${HOST}:\${PORT} pid \${process.pid}\`); fs.writeFileSync(path.join(ROOT, 'jarvis.pid'), String(process.pid)); try { wallyBattery.start(); } catch (e) { log('wally-battery start', e.message); } });`,
    'start logger on listen'
  );

  return t;
}

function patchHtml(src) {
  const N = nl(src);
  let t = src;

  t = applyOnce(
    t,
    `.robot-err{color:var(--bad);font-size:12.5px;margin:6px 0 0}`,
    `.robot-err{color:var(--bad);font-size:12.5px;margin:6px 0 0}${N}.wally-batt-log{margin-top:6px;font-size:11.5px;color:var(--dim);line-height:1.4}${N}.wally-batt-alert{margin:8px 0 0;padding:8px 10px;border-radius:10px;border:1px solid rgba(245,183,72,.4);background:rgba(245,183,72,.12);color:var(--warn);font-size:12.5px;line-height:1.4}${N}.wally-batt-alert b{color:var(--text);font-weight:650}`,
    'html css'
  );

  t = applyOnce(
    t,
    `        <div class="robot-err hidden" id="wallyErr"></div>${N}        <div class="wally-diag" id="wallyDiag">Loading sensors…</div>`,
    `        <div class="robot-err hidden" id="wallyErr"></div>${N}        <div class="wally-batt-alert hidden" id="wallyBattAlert" role="status" aria-live="polite"></div>${N}        <div class="wally-diag" id="wallyDiag">Loading sensors…</div>${N}        <div class="wally-batt-log" id="wallyBattLog">Battery log starting…</div>`,
    'html banner markup'
  );

  t = applyOnce(
    t,
    `const WALLY_POLL_MS=15000;`,
    `const WALLY_POLL_MS=15000;${N}let wallyBattHealth=null;${N}function renderWallyBatteryHealth(h){${N}  wallyBattHealth=h||wallyBattHealth;${N}  const el=$('#wallyBattAlert');${N}  if(el){${N}    if(wallyBattHealth && wallyBattHealth.health_check_due){${N}      el.innerHTML='<b>Battery health check due.</b> '+esc(wallyBattHealth.message||('Wall-E has reached '+(wallyBattHealth.cycles)+' cycles. Run a capacity test before the next dock run.'));${N}      el.classList.remove('hidden');${N}    }else{el.textContent='';el.classList.add('hidden')}${N}  }${N}  const line=$('#wallyBattLog');${N}  if(line && wallyBattHealth && wallyBattHealth.ok){${N}    const cyc=(wallyBattHealth.cycles!=null)?Number(wallyBattHealth.cycles).toFixed(2):'—';${N}    const cur=wallyBattHealth.current_available?'current live':'current N/A';${N}    const rows=(wallyBattHealth.logging && wallyBattHealth.logging.rows!=null)?wallyBattHealth.logging.rows+' samples':'logging';${N}    line.textContent='Battery log · '+cyc+' cycles · '+cur+' · '+rows;${N}  }${N}}`,
    'html health renderer'
  );

  t = applyOnce(
    t,
    `    const s=await api('/api/wally/status');${N}    renderWally(s, s && s.ok? '' : (s && (s.reason||s.error)) || 'Could not reach Wally');`,
    `    const [s,h]=await Promise.all([api('/api/wally/status'),api('/api/wally/battery-health').catch(()=>null)]);${N}    renderWally(s, s && s.ok? '' : (s && (s.reason||s.error)) || 'Could not reach Wally');${N}    if(h && h.ok) renderWallyBatteryHealth(h);`,
    'html refreshWally hook'
  );

  return t;
}

const backups = [];
backups.push(backup(serverPath, 'battery'));
backups.push(backup(htmlPath, 'battery'));
fs.writeFileSync(serverPath, patchServer(fs.readFileSync(serverPath, 'utf8')));
fs.writeFileSync(htmlPath, patchHtml(fs.readFileSync(htmlPath, 'utf8')));
console.log('patched ok');
backups.forEach((b) => console.log('backup ' + b));
