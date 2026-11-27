'use strict';
// Persistent Wall-E battery logger + 80-cycle health-check alert.
// Runs on ALPHA-ONE (Jarvis) so it does not add Pi SD-card wear.
// Read-only: never arms or commands motion. current_a is null until a
// current sensor appears in Wall-E telemetry (gb_lowv today publishes
// voltage, percent, and charging_hint only).

const fs = require('fs');
const path = require('path');

const LOG_INTERVAL_MS = 5000;
const MIN_WRITE_GAP_MS = 4000;
const SOC_COMMIT_EPS = 1; // percentage points; ignores sub-1% flicker
const CYCLE_ALERT_THRESHOLD = 80;
const DEFAULT_PACK_LABEL = 'stock-2S';
const CURRENT_NOTE = 'Wall-E telemetry currently exposes voltage and percent only (gb_lowv writes /home/pi/grokbot/data/battery.json from the ROS voltage topic). current_a is logged as null so it will populate automatically if a current sensor is added later (current_a, i_avg, i_raw, amps, current, or current_ma).';

function pad(n, w = 2) { return String(n).padStart(w, '0'); }

function iso8601(d = new Date()) {
  const tz = -d.getTimezoneOffset();
  const sign = tz >= 0 ? '+' : '-';
  const hh = pad(Math.floor(Math.abs(tz) / 60));
  const mm = pad(Math.abs(tz) % 60);
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())}T${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}${sign}${hh}:${mm}`;
}

function round4(n) { return Math.round(Number(n) * 10000) / 10000; }

function defaultCycleState() {
  const now = iso8601();
  return {
    pack_label: DEFAULT_PACK_LABEL,
    pack_note: 'Default label for the current ~8.4 V 2S pack. Edit pack_label in this file when swapping to a larger-capacity pack so curves stay associated with the right pack.',
    cycles: 0,
    cumulative_discharge_pct: 0,
    cumulative_charge_pct: 0,
    last_soc_pct: null,
    last_voltage: null,
    last_charge_state: null,
    charge_sessions: 0,
    discharge_sessions: 0,
    health_check_due: false,
    health_check_threshold: CYCLE_ALERT_THRESHOLD,
    health_check_alerted_at: null,
    created_at: now,
    updated_at: now
  };
}

function isPanelOnline(panel) {
  if (!panel) return false;
  if (panel.unreachable) return false;
  if (panel.battery == null && panel.voltage == null) return false;
  return true;
}

function extractCurrentA(panel) {
  const batt = (panel && panel.result && panel.result.battery) || {};
  const candidates = [
    batt.current_a, batt.i_avg, batt.i_raw, batt.amps, batt.current,
    panel && panel.current_a, panel && panel.current
  ];
  for (const c of candidates) {
    if (c == null || c === '') continue;
    const n = Number(c);
    if (Number.isFinite(n)) return n;
  }
  if (batt.current_ma != null && Number.isFinite(Number(batt.current_ma))) {
    return Number(batt.current_ma) / 1000;
  }
  return null;
}

function deriveChargeState({ charging_hint, socDelta, current_a }) {
  if (current_a != null && Number.isFinite(Number(current_a))) {
    const a = Number(current_a);
    if (a > 0.05) return 'charging';
    if (a < -0.05) return 'discharging';
  }
  if (charging_hint) return 'charging';
  if (socDelta != null && socDelta <= -SOC_COMMIT_EPS) return 'discharging';
  if (socDelta != null && socDelta >= SOC_COMMIT_EPS) return 'charging';
  return 'idle';
}

function applySampleToCycles(state, sample) {
  const next = { ...state };
  if (!sample || !sample.online || sample.battery_pct == null || !Number.isFinite(Number(sample.battery_pct))) {
    return next;
  }
  const soc = Number(sample.battery_pct);
  const prev = next.last_soc_pct;
  let socDelta = null;
  if (prev != null && Number.isFinite(Number(prev))) {
    socDelta = soc - Number(prev);
    if (socDelta <= -SOC_COMMIT_EPS) {
      next.cumulative_discharge_pct = round4(Number(next.cumulative_discharge_pct || 0) - socDelta);
      next.last_soc_pct = soc;
    } else if (socDelta >= SOC_COMMIT_EPS) {
      next.cumulative_charge_pct = round4(Number(next.cumulative_charge_pct || 0) + socDelta);
      next.last_soc_pct = soc;
    }
    // |delta| < 1% is treated as flicker; last_soc_pct stays so a later sample can still commit the drift.
  } else {
    next.last_soc_pct = soc;
  }
  if (sample.voltage != null && Number.isFinite(Number(sample.voltage))) {
    next.last_voltage = Number(sample.voltage);
  }
  const chargeState = sample.charge_state || deriveChargeState({
    charging_hint: sample.charging_hint,
    socDelta,
    current_a: sample.current_a
  });
  if (chargeState === 'charging' && next.last_charge_state !== 'charging') {
    next.charge_sessions = (next.charge_sessions || 0) + 1;
  } else if (chargeState === 'discharging' && next.last_charge_state !== 'discharging') {
    next.discharge_sessions = (next.discharge_sessions || 0) + 1;
  }
  next.last_charge_state = chargeState;
  next.cycles = round4(Number(next.cumulative_discharge_pct || 0) / 100);
  const wasDue = !!next.health_check_due;
  next.health_check_due = next.cycles >= (next.health_check_threshold || CYCLE_ALERT_THRESHOLD);
  if (next.health_check_due && !wasDue && !next.health_check_alerted_at) {
    next.health_check_alerted_at = sample.ts || iso8601();
  }
  next.updated_at = sample.ts || iso8601();
  return next;
}

function writeJSONAtomic(file, obj) {
  const tmp = file + '.tmp';
  fs.writeFileSync(tmp, JSON.stringify(obj, null, 2) + '\n');
  fs.renameSync(tmp, file);
}

function readJSON(file, def) {
  try { return JSON.parse(fs.readFileSync(file, 'utf8')); } catch { return def; }
}

function tailJsonl(file, n) {
  try {
    const text = fs.readFileSync(file, 'utf8');
    const lines = text.split(/\r?\n/).filter(Boolean);
    const out = [];
    for (let i = Math.max(0, lines.length - n); i < lines.length; i++) {
      try { out.push(JSON.parse(lines[i])); } catch { /* skip bad row */ }
    }
    return { rows: out, total: lines.length };
  } catch {
    return { rows: [], total: 0 };
  }
}

function alertPayload(state) {
  const due = !!state.health_check_due;
  const cycles = Number(state.cycles || 0);
  const pack = state.pack_label || DEFAULT_PACK_LABEL;
  const threshold = state.health_check_threshold || CYCLE_ALERT_THRESHOLD;
  return {
    alert: due,
    type: 'battery_health_check',
    robot: 'Wall-E',
    pack_label: pack,
    cycles,
    threshold,
    health_check_due: due,
    message: due
      ? `Wall-E's battery pack (${pack}) has reached ${cycles.toFixed(2)} cycles. Run a capacity test before the next dock run.`
      : `Wall-E's battery pack (${pack}) is at ${cycles.toFixed(2)} of ${threshold} cycles.`,
    alerted_at: state.health_check_alerted_at || null,
    updated_at: state.updated_at || iso8601()
  };
}

function createWallyBattery(opts) {
  const root = opts.root;
  const log = opts.log || ((...a) => console.log(...a));
  const loadPanel = opts.loadPanel;
  const getCachedPanel = opts.getCachedPanel;
  const intervalMs = opts.intervalMs || LOG_INTERVAL_MS;
  const dataDir = opts.dataDir || path.join(root, 'data');
  const logPath = path.join(dataDir, 'wally-battery-log.jsonl');
  const cyclePath = path.join(dataDir, 'wally-battery-cycles.json');
  const alertPath = path.join(dataDir, 'wally-battery-health-alert.json');

  let state = defaultCycleState();
  let timer = null;
  let inflight = false;
  let stopped = false;
  let lastWriteAt = 0;
  let lastSample = null;
  let wasOnline = null;
  let offlineNoted = false;
  let started = false;

  function ensureDir() { fs.mkdirSync(dataDir, { recursive: true }); }

  function loadState() {
    ensureDir();
    const loaded = readJSON(cyclePath, null);
    if (loaded && typeof loaded === 'object') {
      state = { ...defaultCycleState(), ...loaded };
      // Preserve a pack_label John may have edited.
      if (loaded.pack_label) state.pack_label = loaded.pack_label;
    } else {
      state = defaultCycleState();
      writeJSONAtomic(cyclePath, state);
    }
    writeJSONAtomic(alertPath, alertPayload(state));
  }

  function persistState() {
    ensureDir();
    writeJSONAtomic(cyclePath, state);
    writeJSONAtomic(alertPath, alertPayload(state));
  }

  function buildSample(panel, { online, chargeState, socDelta, current_a, note } = {}) {
    const batt = (panel && panel.result && panel.result.battery) || {};
    const pct = panel && (panel.battery != null ? panel.battery : (batt.pct != null ? batt.pct : batt.percent));
    const voltage = panel && (panel.voltage != null ? panel.voltage : (batt.v_avg != null ? batt.v_avg : batt.v_raw));
    const chargingHint = !!(panel && (panel.charging_hint || batt.charging_hint));
    return {
      ts: iso8601(),
      voltage: voltage != null && Number.isFinite(Number(voltage)) ? Number(voltage) : null,
      current_a: current_a === undefined ? extractCurrentA(panel) : current_a,
      current_available: (current_a === undefined ? extractCurrentA(panel) : current_a) != null,
      battery_pct: pct != null && Number.isFinite(Number(pct)) ? Number(pct) : null,
      armed: !!(panel && panel.armed),
      mode: (panel && panel.mode) || null,
      explore_running: !!(panel && panel.explore_running),
      charging_hint: chargingHint,
      charge_state: chargeState || 'unknown',
      online: !!online,
      note: note || undefined
    };
  }

  function appendRow(row) {
    ensureDir();
    const clean = { ...row };
    if (clean.note == null) delete clean.note;
    fs.appendFileSync(logPath, JSON.stringify(clean) + '\n');
    lastWriteAt = Date.now();
    lastSample = clean;
  }

  function ingest(panel) {
    const online = isPanelOnline(panel);
    const now = Date.now();
    if (!online) {
      if (!offlineNoted) {
        const row = {
          ts: iso8601(),
          voltage: null,
          current_a: null,
          current_available: false,
          battery_pct: null,
          armed: null,
          mode: null,
          explore_running: null,
          charging_hint: null,
          charge_state: 'unknown',
          online: false,
          note: 'offline'
        };
        appendRow(row);
        offlineNoted = true;
        wasOnline = false;
      }
      return lastSample;
    }

    const current_a = extractCurrentA(panel);
    const pct = panel.battery != null ? Number(panel.battery) : null;
    const prevSoc = state.last_soc_pct;
    const socDelta = (prevSoc != null && pct != null && Number.isFinite(pct)) ? (pct - Number(prevSoc)) : null;
    const chargingHint = !!panel.charging_hint || !!(panel.result && panel.result.battery && panel.result.battery.charging_hint);
    const chargeState = deriveChargeState({ charging_hint: chargingHint, socDelta, current_a });
    const sample = buildSample(panel, { online: true, chargeState, socDelta, current_a });

    // Dedup: skip an unchanged live row written in the last few seconds, but
    // always record a meaningful state change (armed/mode/explore/charge).
    if (lastSample && lastSample.online && (now - lastWriteAt) < MIN_WRITE_GAP_MS) {
      const same =
        lastSample.armed === sample.armed &&
        lastSample.mode === sample.mode &&
        lastSample.explore_running === sample.explore_running &&
        lastSample.charge_state === sample.charge_state &&
        lastSample.battery_pct === sample.battery_pct;
      if (same) return lastSample;
    }

    state = applySampleToCycles(state, sample);
    sample.charge_state = state.last_charge_state || sample.charge_state;
    appendRow(sample);
    persistState();
    offlineNoted = false;
    wasOnline = true;
    return sample;
  }

  async function tick() {
    if (inflight || stopped) return;
    const now = Date.now();
    if (lastWriteAt && (now - lastWriteAt) < MIN_WRITE_GAP_MS) return;
    inflight = true;
    try {
      const cache = typeof getCachedPanel === 'function' ? getCachedPanel() : null;
      if (cache && cache.panel && (now - cache.t) < MIN_WRITE_GAP_MS) {
        ingest(cache.panel);
        return;
      }
      if (typeof loadPanel === 'function') await loadPanel();
    } catch (e) {
      log('wally-battery tick', e && e.message ? e.message : e);
    } finally {
      inflight = false;
    }
  }

  function start() {
    if (started) return { ok: true, already: true };
    loadState();
    started = true;
    stopped = false;
    log('wally-battery start', 'log=' + logPath, 'cycles=' + state.cycles, 'pack=' + state.pack_label);
    tick();
    timer = setInterval(tick, intervalMs);
    if (timer && typeof timer.unref === 'function') timer.unref();
    return { ok: true, logPath, cyclePath, alertPath };
  }

  function stop() {
    stopped = true;
    if (timer) { clearInterval(timer); timer = null; }
    started = false;
  }

  function health() {
    const tail = tailJsonl(logPath, 12);
    const alert = alertPayload(state);
    return {
      ok: true,
      pack_label: state.pack_label,
      pack_note: state.pack_note,
      cycles: state.cycles,
      cumulative_discharge_pct: state.cumulative_discharge_pct,
      cumulative_charge_pct: state.cumulative_charge_pct,
      last_soc_pct: state.last_soc_pct,
      last_voltage: state.last_voltage,
      last_charge_state: state.last_charge_state,
      charge_sessions: state.charge_sessions,
      discharge_sessions: state.discharge_sessions,
      health_check_due: !!state.health_check_due,
      health_check_threshold: state.health_check_threshold || CYCLE_ALERT_THRESHOLD,
      health_check_alerted_at: state.health_check_alerted_at,
      current_available: !!(lastSample && lastSample.current_available),
      current_note: CURRENT_NOTE,
      message: alert.message,
      alert_file: alert,
      log_path: logPath,
      cycle_path: cyclePath,
      alert_path: alertPath,
      logging: {
        interval_ms: intervalMs,
        running: !!(started && timer && !stopped),
        last_sample_at: lastSample && lastSample.ts || null,
        last_online: lastSample ? !!lastSample.online : null,
        rows: tail.total
      },
      latest: lastSample || (tail.rows.length ? tail.rows[tail.rows.length - 1] : null),
      recent: tail.rows,
      existing_loggers: [
        {
          name: 'wally-session-logger',
          location: 'pi:/home/pi/grokbot/learn/session_logger.py',
          notes: 'Exploration-session only (1 Hz battery_pct/battery_v while a session is active). Left unchanged to avoid extra Pi SD-card wear. Does not track cycles or log idle/charge curves continuously.'
        }
      ]
    };
  }

  return {
    start, stop, ingest, health,
    paths: { dataDir, logPath, cyclePath, alertPath }
  };
}

module.exports = {
  createWallyBattery,
  applySampleToCycles,
  deriveChargeState,
  extractCurrentA,
  isPanelOnline,
  iso8601,
  defaultCycleState,
  alertPayload,
  LOG_INTERVAL_MS,
  SOC_COMMIT_EPS,
  CYCLE_ALERT_THRESHOLD,
  CURRENT_NOTE
};
