'use strict';
// Unit + dry-run tests for wally-battery.js. No Wall-E motion, no network.
const fs = require('fs');
const os = require('os');
const path = require('path');
const assert = require('assert');
const {
  createWallyBattery,
  applySampleToCycles,
  deriveChargeState,
  extractCurrentA,
  isPanelOnline,
  iso8601,
  defaultCycleState,
  alertPayload,
  CYCLE_ALERT_THRESHOLD,
  CURRENT_NOTE
} = require('./wally-battery');

let passed = 0;
function test(name, fn) {
  fn();
  passed += 1;
  console.log('ok  ' + name);
}

test('iso8601 is a clean offset timestamp', () => {
  const s = iso8601(new Date('2026-11-27T13:14:15.016Z'));
  assert.match(s, /^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}[+-]\d{2}:\d{2}$/);
});

test('current_a stays null when telemetry has only voltage and percent', () => {
  const panel = { ok: true, battery: 100, voltage: 8.4, charging_hint: false, result: { battery: { v_avg: 8.4, pct: 100 } } };
  assert.strictEqual(extractCurrentA(panel), null);
  assert.ok(CURRENT_NOTE.includes('current_a'));
});

test('current_a populates from current_ma or i_avg when a sensor is added', () => {
  assert.strictEqual(extractCurrentA({ result: { battery: { current_ma: 250 } } }), 0.25);
  assert.strictEqual(extractCurrentA({ result: { battery: { i_avg: -0.8 } } }), -0.8);
});

test('offline panel detection', () => {
  assert.strictEqual(isPanelOnline(null), false);
  assert.strictEqual(isPanelOnline({ unreachable: true, battery: 80, voltage: 8 }), false);
  assert.strictEqual(isPanelOnline({ ok: false }), false);
  assert.strictEqual(isPanelOnline({ ok: true, battery: 80, voltage: 8.1 }), true);
});

test('charge state: hint, SOC delta, and current sign', () => {
  assert.strictEqual(deriveChargeState({ charging_hint: true }), 'charging');
  assert.strictEqual(deriveChargeState({ socDelta: -5 }), 'discharging');
  assert.strictEqual(deriveChargeState({ socDelta: 4 }), 'charging');
  assert.strictEqual(deriveChargeState({ current_a: 0.4 }), 'charging');
  assert.strictEqual(deriveChargeState({ current_a: -0.4 }), 'discharging');
  assert.strictEqual(deriveChargeState({ socDelta: 0, charging_hint: false }), 'idle');
});

test('fractional cycle: 100% to 40% is 0.60 cycles', () => {
  let s = defaultCycleState();
  s = applySampleToCycles(s, { online: true, battery_pct: 100, ts: iso8601() });
  s = applySampleToCycles(s, { online: true, battery_pct: 40, charge_state: 'discharging', ts: iso8601() });
  assert.strictEqual(s.cycles, 0.6);
  assert.strictEqual(s.cumulative_discharge_pct, 60);
  assert.strictEqual(s.discharge_sessions, 1);
  assert.strictEqual(s.health_check_due, false);
});

test('sub-1% flicker does not count as a cycle', () => {
  let s = defaultCycleState();
  s = applySampleToCycles(s, { online: true, battery_pct: 100 });
  s = applySampleToCycles(s, { online: true, battery_pct: 99.4 });
  assert.strictEqual(s.cycles, 0);
  assert.strictEqual(s.last_soc_pct, 100);
  s = applySampleToCycles(s, { online: true, battery_pct: 98.5 });
  // 100 -> 98.5 is a 1.5 point drop, so it commits
  assert.strictEqual(s.cycles, 0.015);
});

test('charge session increments once per charging period', () => {
  let s = defaultCycleState();
  s = applySampleToCycles(s, { online: true, battery_pct: 40, charge_state: 'idle' });
  s = applySampleToCycles(s, { online: true, battery_pct: 50, charge_state: 'charging' });
  s = applySampleToCycles(s, { online: true, battery_pct: 60, charge_state: 'charging' });
  assert.strictEqual(s.charge_sessions, 1);
  assert.strictEqual(s.cumulative_charge_pct, 20);
});

test('dry-run: 80 full 100-to-0 discharges trip the health-check alert', () => {
  let s = defaultCycleState();
  s = applySampleToCycles(s, { online: true, battery_pct: 100, ts: 'start' });
  for (let i = 0; i < CYCLE_ALERT_THRESHOLD; i++) {
    s = applySampleToCycles(s, { online: true, battery_pct: 0, charge_state: 'discharging', ts: 'd' + i });
    s = applySampleToCycles(s, { online: true, battery_pct: 100, charge_state: 'charging', ts: 'c' + i });
  }
  assert.strictEqual(s.cycles, 80);
  assert.strictEqual(s.health_check_due, true);
  assert.ok(s.health_check_alerted_at);
  const alert = alertPayload(s);
  assert.strictEqual(alert.alert, true);
  assert.strictEqual(alert.type, 'battery_health_check');
  assert.match(alert.message, /80\.00 cycles/);
  assert.match(alert.message, /capacity test/);
});

test('logger writes JSONL, skips offline spam, persists cycles across restart', () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'wally-batt-'));
  const a = createWallyBattery({ root: dir, dataDir: dir, log: () => {} });
  a.ingest({ ok: true, live: true, battery: 100, voltage: 8.4, charging_hint: false, armed: false, mode: 'ground', explore_running: false, result: { battery: { v_avg: 8.4, pct: 100, charging_hint: false } } });
  a.ingest({ unreachable: true });
  a.ingest({ unreachable: true });
  a.ingest({ ok: false });
  a.ingest({ ok: true, live: true, battery: 90, voltage: 8.1, charging_hint: false, armed: false, mode: 'ground', explore_running: false, result: { battery: { v_avg: 8.1, pct: 90 } } });
  const text = fs.readFileSync(path.join(dir, 'wally-battery-log.jsonl'), 'utf8').trim().split(/\n/);
  assert.strictEqual(text.length, 3, 'expected live, one offline, live again; got ' + text.length);
  const rows = text.map((l) => JSON.parse(l));
  assert.strictEqual(rows[0].online, true);
  assert.strictEqual(rows[0].current_a, null);
  assert.strictEqual(rows[0].current_available, false);
  assert.match(rows[0].ts, /^\d{4}-\d{2}-\d{2}T/);
  assert.strictEqual(rows[1].online, false);
  assert.strictEqual(rows[1].note, 'offline');
  assert.strictEqual(rows[2].battery_pct, 90);
  assert.strictEqual(a.health().cycles, 0.1);
  assert.strictEqual(a.health().health_check_due, false);
  const cyclesDisk = JSON.parse(fs.readFileSync(path.join(dir, 'wally-battery-cycles.json'), 'utf8'));
  assert.strictEqual(cyclesDisk.cycles, 0.1);
  const alertDisk = JSON.parse(fs.readFileSync(path.join(dir, 'wally-battery-health-alert.json'), 'utf8'));
  assert.strictEqual(alertDisk.alert, false);
  // Restart: a new instance must reload the same cycle count.
  const b = createWallyBattery({ root: dir, dataDir: dir, log: () => {} });
  b.start();
  assert.strictEqual(b.health().cycles, 0.1);
  assert.strictEqual(b.health().pack_label, 'stock-2S');
  b.stop();
  fs.rmSync(dir, { recursive: true, force: true });
});

test('80-cycle dry-run via logger writes the Grok Bot alert file', () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'wally-batt-'));
  const bat = createWallyBattery({ root: dir, dataDir: dir, log: () => {}, intervalMs: 60000 });
  bat.start();
  bat.ingest({ ok: true, battery: 100, voltage: 8.4, charging_hint: false, armed: false, mode: 'ground', explore_running: false, result: { battery: { pct: 100, v_avg: 8.4 } } });
  for (let i = 0; i < CYCLE_ALERT_THRESHOLD; i++) {
    bat.ingest({ ok: true, battery: 0, voltage: 6.4, charging_hint: false, armed: false, mode: 'ground', explore_running: false, result: { battery: { pct: 0, v_avg: 6.4 } } });
    bat.ingest({ ok: true, battery: 100, voltage: 8.4, charging_hint: true, armed: false, mode: 'ground', explore_running: false, result: { battery: { pct: 100, v_avg: 8.4, charging_hint: true } } });
  }
  const h = bat.health();
  assert.ok(h.cycles >= 80, 'cycles was ' + h.cycles);
  assert.strictEqual(h.health_check_due, true);
  const alert = JSON.parse(fs.readFileSync(path.join(dir, 'wally-battery-health-alert.json'), 'utf8'));
  assert.strictEqual(alert.alert, true);
  assert.strictEqual(alert.type, 'battery_health_check');
  assert.match(alert.message, /capacity test/);
  bat.stop();
  fs.rmSync(dir, { recursive: true, force: true });
});

console.log('\n' + passed + ' tests passed');
