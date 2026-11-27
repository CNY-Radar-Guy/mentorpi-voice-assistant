'use strict';
// First-class Jarvis tool for John's robot Wally (Wall-E / MentorPi) at 192.168.50.125.
// Uses key-based SSH from ALPHA-ONE and Wally's voice HTTP API on port 8000.
// Node built-ins only. Never triggers, clears, or tests an emergency stop.

const fs = require('fs');
const path = require('path');
const { execFile } = require('child_process');

const TOOL_NAME = 'control_wally';
const WALLY_IP = '192.168.50.125';
const WALLY_USER = 'pi';
const VOICE_URL = 'http://192.168.50.125:8000';

const READ_ACTIONS = new Set(['status']);
const ALL_ACTIONS = new Set(['status', 'stop', 'say', 'ask', 'mode', 'arm', 'disarm', 'move', 'explore', 'explore_stop', 'map_start', 'map_stop', 'map_save', 'map_status']);
const MOTION_ACTIONS = new Set(['move', 'explore']);
const BLOCKED = /\b(e-?stop|estop|emergency[\s_-]*stop|unlatch|relatch|clear[_\s-]?latch)\b/i;

const OLLAMA_TOOLS = [
  {
    type: 'function',
    function: {
      name: TOOL_NAME,
      description:
        "Control John's robot Wally (also called Wall-E or MentorPi) on the home LAN over SSH and Wally's local voice HTTP API on port 8000. " +
        'Use this for live status (battery, mode, armed or safety state, services, recent logs), to halt motion and disarm, to disarm only, to speak a phrase, to send a voice or chat command, to read or set bench or ground mode, to arm or disarm him, to start or stop exploration, or to start, stop, or save LiDAR mapping. ' +
        'Call this instead of guessing Wally state. ' +
        'Start motion with action move or explore only when John has clearly asked for that in this conversation, and only while Wally is armed. Respect existing safety guards and speed caps. ' +
        'Never trigger, clear, or test an emergency stop. Do not invent a wake phrase.',
      parameters: {
        type: 'object',
        properties: {
          action: {
            type: 'string',
            description: 'One of: status, stop, say, ask, mode, arm, disarm, move, explore.',
            enum: ['status', 'stop', 'say', 'ask', 'mode', 'arm', 'disarm', 'move', 'explore', 'explore_stop', 'map_start', 'map_stop', 'map_save', 'map_status']
          },
          text: {
            type: 'string',
            description: 'Phrase to speak (say) or chat command to send (ask).'
          },
          mode: {
            type: 'string',
            description: 'For action mode: omit or use get to read the current mode; set bench or ground only when John has explicitly asked to change it.',
            enum: ['get', 'bench', 'ground']
          },
          direction: {
            type: 'string',
            description: 'For action move only: forward, back, left, or right. A short gated nudge, not a long drive.'
          },
          seconds: {
            type: 'number',
            description: 'Optional duration for move, in seconds. Capped at 2 seconds. Existing speed caps still apply.'
          },
          robot: {
            type: 'boolean',
            description: "For ask: if true (default), also speak Wally's reply on the robot speaker."
          },
          reason: {
            type: 'string',
            description: 'Short reason for the log.'
          }
        },
        required: ['action']
      }
    }
  }
];

function truthy(v) {
  return v === true || v === 'true' || v === 1 || v === '1';
}

function stripLeadIn(text) {
  return String(text || '')
    .replace(/[.!?]+$/g, '')
    .trim()
    .replace(/^(hey|ok|okay|hi)?\s*(jarvis)[,:\s]+/i, '')
    .replace(/^(please|could you|can you|would you|will you|i want you to|i need you to)\s+/i, '')
    .replace(/^(go ahead and|just|now)\s+/i, '')
    .trim();
}

function mentionsWally(s) {
  return /\b(wally|wall-?e|mentorpi)\b/i.test(s || '');
}

function hasWake(s) {
  return /\bwake up wally\b/i.test(s || '');
}

function parseUserRequest(message, _history) {
  const raw = String(message || '').trim();
  if (!raw) return null;
  const s = stripLeadIn(raw);
  if (!s || BLOCKED.test(s)) return null;
  if (!mentionsWally(s) && !hasWake(s)) return null;
  const wake = hasWake(s) || hasWake(raw);

  if (/\bdisarm\b/i.test(s)) {
    return { action: 'disarm', wake, reason: 'John asked to disarm Wally' };
  }
  if (/\b(stop|halt|freeze|abort|shut (?:him|wally|it) down)\b/i.test(s) && !/\b(estop|e-stop|emergency)\b/i.test(s)) {
    return { action: 'stop', wake, reason: 'John asked to stop Wally' };
  }
  if (/\barm\b/i.test(s) && !/\bdisarm\b/i.test(s) && !/\barm(?:ed)? status\b/i.test(s)) {
    return { action: 'arm', wake, reason: 'John asked to arm Wally' };
  }
  const modeSet = s.match(/\b(?:set|switch|put)(?:\s+(?:wally|wall-?e|him|the robot))?\s+(?:to|into|in)\s+(bench|ground)\s+mode\b/i);
  if (modeSet) {
    return { action: 'mode', mode: String(modeSet[1]).toLowerCase(), wake, reason: 'John asked to set Wally mode' };
  }
  if (/\b(what(?:'s| is) (?:wally|wall-?e)(?:'s|s)? mode|wally(?:'s|s)? mode|is (?:he|wally|wall-?e) (?:in )?(?:bench|ground))\b/i.test(s)) {
    return { action: 'mode', mode: 'get', wake, reason: 'John asked for Wally mode' };
  }
  const say = s.match(/\b(?:(?:tell|have)\s+(?:wally|wall-?e)\s+to\s+say|say(?:\s+(?:on|through|via)\s+(?:wally|wall-?e))?)\s+[:\s]+(.+)$/i) ||
    s.match(/\b(?:tell|have)\s+(?:wally|wall-?e)\s+to\s+say\s+(.+)$/i);
  if (say && say[1]) {
    return { action: 'say', text: say[1].trim().replace(/^["']|["']$/g, ''), wake, reason: 'John asked Wally to speak' };
  }
  if (/\b(explore|start exploring|go explore|begin (?:an )?explor)/i.test(s)) {
    return { action: 'explore', wake, reason: 'John asked Wally to explore' };
  }
  const mv = s.match(/\b(?:move|nudge|drive|go|turn)\s+(forward|back|backward|left|right)\b/i);
  if (mv) {
    const direction = /back/i.test(mv[1]) ? 'back' : String(mv[1]).toLowerCase();
    return { action: 'move', direction, wake, reason: 'John asked Wally to move' };
  }
  const ask = s.match(/\b(?:ask|tell|say to)\s+(?:wally|wall-?e)\s+(.+)$/i);
  if (ask && ask[1] && !/\bto say\b/i.test(s)) {
    return { action: 'ask', text: ask[1].trim().replace(/^to\s+/i, ''), robot: true, wake, reason: 'John asked to send Wally a chat command' };
  }
  if (/\b(status|how is|how's|hows|battery|armed|reachable|health|what's going on with|what is going on with)\b/i.test(s) ||
      /^(is|can you (?:check|see)|check(?: on)?)\s+(?:wally|wall-?e)\b/i.test(s)) {
    return { action: 'status', wake, reason: 'John asked for Wally status' };
  }
  return null;
}

function findSsh() {
  const candidates = [
    path.join(process.env.SystemRoot || 'C:\\Windows', 'System32', 'OpenSSH', 'ssh.exe'),
    path.join(process.env.SystemRoot || 'C:\\Windows', 'Sysnative', 'OpenSSH', 'ssh.exe')
  ];
  for (const c of candidates) {
    try { if (fs.existsSync(c)) return c; } catch { /* ignore */ }
  }
  return 'ssh';
}

function parseJsonLoose(s) {
  const t = String(s || '').trim();
  if (!t) return null;
  try { return JSON.parse(t); } catch { /* fall through */ }
  const start = t.indexOf('{');
  const end = t.lastIndexOf('}');
  if (start >= 0 && end > start) {
    try { return JSON.parse(t.slice(start, end + 1)); } catch { /* raw */ }
  }
  return null;
}

function blockedReason(action, args) {
  const blob = [action, args && args.text, args && args.reason, args && args.mode, args && args.direction].join(' ');
  if (BLOCKED.test(blob) || BLOCKED.test(String(action || ''))) {
    return 'Refused: Jarvis will not trigger, clear, or test an emergency stop on Wally. Use status, stop, disarm, say, ask, mode, arm, move, or explore instead.';
  }
  return null;
}

function createWallyTool({ root, logsDir, log = () => {} } = {}) {
  const jarvisRoot = root || path.join('D:', 'AI-Models', 'jarvis');
  const remoteScript = path.join(jarvisRoot, 'tools', 'wally', 'remote.py');
  const sshBin = findSsh();

  function handles(name) { return name === TOOL_NAME; }

  function systemHint() {
    return 'Wally robot: you control John\'s Wally (Wall-E, MentorPi at 192.168.50.125) with the control_wally tool. ' +
      'Call it instead of guessing. "Wally status" or "how is Wally" → action status. "Stop Wally" → action stop, which halts motion and disarms him. "Disarm Wally" → action disarm. "Arm Wally" → action arm, only when John clearly asks. ' +
      '"Ask Wally …" → action ask with the text (this stays dry-run, so it will not start motion). "Tell Wally to say …" → action say, which speaks on the robot. "What mode is Wally in" → action mode with mode get. Set bench or ground mode only when John explicitly asks. ' +
      'Motion: action move or explore only after John has clearly asked for it in this conversation, and only if Wally is already armed or the same request includes "wake up Wally". Respect his existing obstacle guards and speed caps. Never invent a wake phrase. ' +
      'Never trigger, clear, or test an emergency stop. The systems snapshot only has ping and SSH reachability; live battery, mode, armed state, services and logs come from this tool.';
  }

  function sshArgs() {
    return [
      '-o', 'BatchMode=yes',
      '-o', 'ConnectTimeout=8',
      '-o', 'StrictHostKeyChecking=accept-new',
      WALLY_USER + '@' + WALLY_IP,
      'python3', '-',
    ];
  }

  function runRemote(opArgs, timeoutMs) {
    return new Promise((resolve) => {
      if (!fs.existsSync(remoteScript)) {
        resolve({ ok: false, timedOut: false, exitCode: 1, stdout: '', stderr: '', parsed: null, err: new Error('Missing ' + remoteScript) });
        return;
      }
      const args = sshArgs().concat(opArgs || []);
      const child = execFile(sshBin, args, {
        windowsHide: true,
        timeout: timeoutMs || 25000,
        maxBuffer: 2 * 1024 * 1024
      }, (err, stdout, stderr) => {
        const out = String(stdout || '').trim();
        const parsed = parseJsonLoose(out);
        const exitCode = err && typeof err.code === 'number' ? err.code : (err ? 1 : 0);
        resolve({
          err,
          stdout: out,
          stderr: String(stderr || '').trim(),
          parsed,
          exitCode,
          timedOut: !!(err && err.killed)
        });
      });
      try {
        const script = fs.readFileSync(remoteScript);
        child.stdin.write(script);
        child.stdin.end();
      } catch (e) {
        try { child.kill(); } catch { /* ignore */ }
        resolve({ ok: false, timedOut: false, exitCode: 1, stdout: '', stderr: String(e.message || e), parsed: null, err: e });
      }
    });
  }

  async function invoke(name, args = {}, meta = {}) {
    if (!handles(name)) {
      return { ok: false, status: 'error', reason: 'Unknown tool ' + name };
    }
    const action = String(args.action || '').toLowerCase().trim().replace(/-/g, '_');
    const commandPreview = [TOOL_NAME, action, args.mode || '', args.direction || '', args.text ? '…' : ''].filter(Boolean).join(' ');

    const blocked = blockedReason(action, args);
    if (blocked) {
      return { ok: false, status: 'blocked', tool: TOOL_NAME, command: commandPreview, reason: blocked };
    }
    if (!action || !ALL_ACTIONS.has(action)) {
      return { ok: false, status: 'error', tool: TOOL_NAME, command: commandPreview, reason: 'Unknown or missing action. Use status, stop, say, ask, mode, arm, disarm, move, explore, explore_stop, map_start, map_stop, map_save, or map_status.' };
    }

    if (action === 'say' && !String(args.text || '').trim()) {
      return { ok: false, status: 'error', tool: TOOL_NAME, command: commandPreview, reason: 'A phrase is required for say.' };
    }
    if (action === 'ask' && !String(args.text || '').trim()) {
      return { ok: false, status: 'error', tool: TOOL_NAME, command: commandPreview, reason: 'A message is required for ask.' };
    }
    if (action === 'ask' && BLOCKED.test(String(args.text || ''))) {
      return { ok: false, status: 'blocked', tool: TOOL_NAME, command: commandPreview, reason: 'Refused: that ask looks like an emergency-stop request, which Jarvis will not send.' };
    }
    if (action === 'ask' && /\b(explore|start exploring|go explore|move forward|drive)\b/i.test(String(args.text || ''))) {
      return { ok: false, status: 'error', tool: TOOL_NAME, command: commandPreview, reason: 'Ask stays dry-run and will not start motion. If John wants Wally to move or explore, call control_wally with action move or explore after he has clearly asked and Wally is armed.' };
    }

    if (MOTION_ACTIONS.has(action) && !truthy(args.wake) && !hasWake(args.reason || '') && !hasWake(args.text || '')) {
      // Still allowed if Wally is already armed; the remote script checks ARMED and refuses if not.
    }

    const opArgs = [action === 'mode' ? 'mode' : action];
    if (action === 'mode') opArgs.push(String(args.mode || 'get').toLowerCase() || 'get');
    if (action === 'say') opArgs.push(String(args.text || '').slice(0, 500));
    if (action === 'ask') {
      opArgs.push('robot=' + (args.robot === false || args.robot === 'false' ? '0' : '1'));
      opArgs.push('dry_run=1');
      opArgs.push(String(args.text || '').slice(0, 2000));
    }
    if (action === 'move') {
      opArgs.push(String(args.direction || 'forward').toLowerCase());
      const sec = Number(args.seconds);
      opArgs.push(String(Number.isFinite(sec) ? sec : 0.8));
    }
    if (action === 'explore' && Number.isFinite(Number(args.seconds))) {
      opArgs.push(String(Number(args.seconds)));
    }
    if (action === 'map_save') {
      if (args.name) opArgs.push('name=' + String(args.name).replace(/[^A-Za-z0-9_\-]/g, '_').slice(0, 80));
      if (args.stop_after === false || args.stop_after === 'false' || args.stop_after === 0) opArgs.push('stop_after=0');
    }
    const wantDry = args.dry_run === true || args.dry_run === '1' || args.dry_run === 1 || args.dry_run === 'true';
    if (wantDry && action !== 'ask') opArgs.push('dry_run=1');
    const ACTOR_ACTIONS = new Set(['stop', 'say', 'ask', 'mode', 'arm', 'disarm', 'move', 'explore', 'explore_stop', 'map_start', 'map_stop', 'map_save']);
    if (ACTOR_ACTIONS.has(action)) opArgs.push('actor=' + (String(args.actor || 'jarvis').trim() || 'jarvis'));

    const timeoutMs = meta.timeoutMs || (action === 'ask' || action === 'say' ? 90000 : action === 'status' ? 25000 : 30000);
    const ran = await runRemote(opArgs, timeoutMs);
    const payload = ran.parsed || null;
    const ok = !!(payload && payload.ok) && !ran.timedOut;
    const status = ran.timedOut ? 'timeout' : (ok ? 'ok' : (payload && payload.error ? 'error' : 'error'));
    const reason = ran.timedOut
      ? 'The SSH command to Wally timed out. He may be offline or still booting.'
      : (ok ? '' : (payload && payload.error) || ran.stderr || (ran.err && ran.err.message) || 'The Wally command failed.');

    if (typeof log === 'function') {
      try { log('wally-tool', status, commandPreview, meta.source || '', (ran.stdout || '').slice(0, 180)); } catch { /* ignore */ }
    }

    return {
      ok,
      status,
      tool: TOOL_NAME,
      command: commandPreview,
      exitCode: ran.exitCode,
      stdout: ran.stdout,
      stderr: ran.stderr,
      result: payload,
      reason,
      unreachable: /Connection timed out|Connection refused|No route to host|Connection reset|Could not resolve|Permission denied|timed out/i.test(String(ran.stderr || '') + String(reason || ''))
    };
  }

  return {
    tools: OLLAMA_TOOLS,
    handles,
    invoke,
    systemHint,
    parseUserRequest,
    remoteScript
  };
}

module.exports = { createWallyTool, parseUserRequest, TOOL_NAME, OLLAMA_TOOLS, WALLY_IP };
