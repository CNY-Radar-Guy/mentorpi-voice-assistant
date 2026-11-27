'use strict';
// Manual-drive helper for the Jarvis Wall-E Vision panel.
// Publishes geometry_msgs/Twist to /gb/cmd_in via rosbridge so gb_gate's
// 0.5 s deadman, obstacle stops, and caps stay in the loop.
// Never publishes to /controller/cmd_vel (that would bypass the gate).
// Node built-ins only. Does not arm Wall-E or start missions.

const net = require('net');
const crypto = require('crypto');

const HARD_MAX_LIN = 0.288; // Jarvis manual-drive hard cap (3x previous 0.096). Gate stop distances are unchanged.
const HARD_MAX_ANG = 0.48;  // conservative, under explore HARD_MAX_ANG 0.64
const MIN_LIN = 0.02;        // slider creep floor (commanded speeds may be 0 for stop)
const DEFAULT_LIN = 0.08;
const DEFAULT_ANG = 0.30;
const ROSBRIDGE_PORT = 9090;
const CMD_IN = '/gb/cmd_in';
const TWIST_TYPE = 'geometry_msgs/Twist';
const STALE_MS = 8000;

const DIRS = {
  forward: { lin: DEFAULT_LIN, ang: 0 },
  back: { lin: -DEFAULT_LIN, ang: 0 },
  backward: { lin: -DEFAULT_LIN, ang: 0 },
  left: { lin: 0, ang: DEFAULT_ANG },
  right: { lin: 0, ang: -DEFAULT_ANG },
  'forward-left': { lin: DEFAULT_LIN, ang: DEFAULT_ANG },
  forward_left: { lin: DEFAULT_LIN, ang: DEFAULT_ANG },
  'fwd-left': { lin: DEFAULT_LIN, ang: DEFAULT_ANG },
  'forward-right': { lin: DEFAULT_LIN, ang: -DEFAULT_ANG },
  forward_right: { lin: DEFAULT_LIN, ang: -DEFAULT_ANG },
  'fwd-right': { lin: DEFAULT_LIN, ang: -DEFAULT_ANG },
  'back-left': { lin: -DEFAULT_LIN, ang: DEFAULT_ANG },
  back_left: { lin: -DEFAULT_LIN, ang: DEFAULT_ANG },
  'backward-left': { lin: -DEFAULT_LIN, ang: DEFAULT_ANG },
  'back-right': { lin: -DEFAULT_LIN, ang: -DEFAULT_ANG },
  back_right: { lin: -DEFAULT_LIN, ang: -DEFAULT_ANG },
  'backward-right': { lin: -DEFAULT_LIN, ang: -DEFAULT_ANG },
  stop: { lin: 0, ang: 0 }
};

function clamp(v, max) {
  const n = Number(v);
  if (!Number.isFinite(n)) return 0;
  if (n > max) return max;
  if (n < -max) return -max;
  return n;
}

function almostZero(lin, ang) {
  return Math.abs(lin) < 1e-6 && Math.abs(ang) < 1e-6;
}

function permit(panel) {
  if (!panel || panel.ok === false || panel.unreachable) {
    return { allowed: false, code: 'unknown', badge: 'Status unknown — manual disabled', reason: (panel && (panel.error || panel.reason)) || 'Wally status is unavailable.' };
  }
  if (panel.estop_latched) {
    return { allowed: false, code: 'estop', badge: 'E-stop latched — manual disabled', reason: 'An emergency-stop latch is present. Jarvis will not clear it or send drive commands.' };
  }
  if (panel.explore_running) {
    return { allowed: false, code: 'explore', badge: 'Explore active — manual disabled', reason: 'An autonomous explore mission is running. Stop explore before using the D-pad.' };
  }
  if (!panel.armed) {
    return { allowed: false, code: 'disarmed', badge: 'Disarmed', reason: 'Wally is not armed. Arm him before using manual drive.' };
  }
  return { allowed: true, code: 'ready', badge: 'Armed — manual ready', reason: '' };
}

// Custom {lin, ang} bodies (including simultaneous linear + angular) are accepted
// and independently clamped to HARD_MAX_LIN / HARD_MAX_ANG. Direction aliases
// include the four diagonal combined-axis names used by the Jarvis D-pad.
function parseDriveBody(body) {
  const b = body || {};
  let lin = 0;
  let ang = 0;
  const stopFlag = b.stop === true || b.stop === 1 || b.stop === '1' || b.stop === 'true' || String(b.action || '').toLowerCase() === 'stop';
  const dir = String(b.direction || b.dir || '').toLowerCase().trim();
  if (stopFlag || dir === 'stop') {
    lin = 0;
    ang = 0;
  } else if (dir && DIRS[dir]) {
    lin = DIRS[dir].lin;
    ang = DIRS[dir].ang;
  } else {
    lin = b.lin != null ? b.lin : (b.linear != null ? b.linear : 0);
    ang = b.ang != null ? b.ang : (b.angular != null ? b.angular : 0);
  }
  lin = clamp(lin, HARD_MAX_LIN);
  ang = clamp(ang, HARD_MAX_ANG);
  return { lin, ang, stop: almostZero(lin, ang), direction: dir || (almostZero(lin, ang) ? 'stop' : 'custom') };
}

function encodeWsFrame(data, opcode) {
  const payload = Buffer.from(data);
  const len = payload.length;
  let header;
  if (len < 126) {
    header = Buffer.alloc(2);
    header[0] = 0x80 | (opcode || 1);
    header[1] = 0x80 | len;
  } else if (len < 65536) {
    header = Buffer.alloc(4);
    header[0] = 0x80 | (opcode || 1);
    header[1] = 0x80 | 126;
    header.writeUInt16BE(len, 2);
  } else {
    header = Buffer.alloc(10);
    header[0] = 0x80 | (opcode || 1);
    header[1] = 0x80 | 127;
    header.writeUInt32BE(0, 2);
    header.writeUInt32BE(len, 6);
  }
  const mask = crypto.randomBytes(4);
  const masked = Buffer.alloc(len);
  for (let i = 0; i < len; i++) masked[i] = payload[i] ^ mask[i % 4];
  return Buffer.concat([header, mask, masked]);
}

function tryDecodeWs(buf) {
  if (buf.length < 2) return null;
  const opcode = buf[0] & 0x0f;
  const masked = (buf[1] & 0x80) !== 0;
  let len = buf[1] & 0x7f;
  let off = 2;
  if (len === 126) {
    if (buf.length < 4) return null;
    len = buf.readUInt16BE(2);
    off = 4;
  } else if (len === 127) {
    if (buf.length < 10) return null;
    const hi = buf.readUInt32BE(2);
    const lo = buf.readUInt32BE(6);
    if (hi !== 0 || lo > 8 * 1024 * 1024) throw new Error('websocket frame too large');
    len = lo;
    off = 10;
  }
  let mask = null;
  if (masked) {
    if (buf.length < off + 4) return null;
    mask = buf.slice(off, off + 4);
    off += 4;
  }
  if (buf.length < off + len) return null;
  let payload = buf.slice(off, off + len);
  if (mask) {
    const p = Buffer.alloc(len);
    for (let i = 0; i < len; i++) p[i] = payload[i] ^ mask[i % 4];
    payload = p;
  }
  return { opcode, payload, rest: buf.slice(off + len) };
}

function twistMsg(lin, ang) {
  return {
    linear: { x: lin, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: ang }
  };
}

function createWallyDrive({ host, port, log = () => {}, getStatus, refreshStatus } = {}) {
  const rbHost = host || '192.168.50.125';
  const rbPort = port || ROSBRIDGE_PORT;
  let sock = null;
  let ready = false;
  let advertised = false;
  let connecting = null;
  let buf = Buffer.alloc(0);
  let latest = null; // { lin, ang } latest-wins slot
  let sending = false;

  function closeSock() {
    ready = false;
    advertised = false;
    const s = sock;
    sock = null;
    if (s) {
      try { s.destroy(); } catch { /* ignore */ }
    }
  }

  function sendJson(obj) {
    if (!sock || !ready) throw new Error('rosbridge not connected');
    sock.write(encodeWsFrame(JSON.stringify(obj), 1));
  }

  function onBytes(chunk) {
    buf = Buffer.concat([buf, chunk]);
    while (buf.length >= 2) {
      let frame;
      try { frame = tryDecodeWs(buf); } catch (e) {
        log('wally-drive', 'ws decode error', e.message || e);
        closeSock();
        return;
      }
      if (!frame) break;
      buf = frame.rest;
      if (frame.opcode === 8) { closeSock(); return; }
      if (frame.opcode === 9 && sock && ready) {
        try { sock.write(encodeWsFrame(frame.payload, 10)); } catch { /* ignore */ }
      }
    }
  }

  function connect() {
    if (ready && sock) return Promise.resolve();
    if (connecting) return connecting;
    connecting = new Promise((resolve, reject) => {
      const key = crypto.randomBytes(16).toString('base64');
      const expect = crypto.createHash('sha1').update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
      const s = net.connect({ host: rbHost, port: rbPort });
      let headerBuf = Buffer.alloc(0);
      let upgraded = false;
      let settled = false;
      const timer = setTimeout(() => {
        if (!settled) {
          settled = true;
          try { s.destroy(); } catch { /* ignore */ }
          connecting = null;
          reject(new Error('rosbridge connection timed out'));
        }
      }, 5000);
      s.setNoDelay(true);
      s.on('error', (e) => {
        if (!settled) {
          settled = true;
          clearTimeout(timer);
          connecting = null;
          reject(e);
        } else {
          closeSock();
        }
      });
      s.on('close', () => {
        if (sock === s) closeSock();
        if (!settled) {
          settled = true;
          clearTimeout(timer);
          connecting = null;
          reject(new Error('rosbridge closed before upgrade'));
        }
      });
      s.once('connect', () => {
        s.write(
          'GET / HTTP/1.1\r\n' +
          'Host: ' + rbHost + ':' + rbPort + '\r\n' +
          'Upgrade: websocket\r\n' +
          'Connection: Upgrade\r\n' +
          'Sec-WebSocket-Key: ' + key + '\r\n' +
          'Sec-WebSocket-Version: 13\r\n' +
          '\r\n'
        );
      });
      s.on('data', (d) => {
        if (!upgraded) {
          headerBuf = Buffer.concat([headerBuf, d]);
          const idx = headerBuf.indexOf('\r\n\r\n');
          if (idx < 0) return;
          const head = headerBuf.slice(0, idx).toString('utf8');
          const rest = headerBuf.slice(idx + 4);
          if (!/101/.test(head.split('\r\n')[0] || '') || !head.toLowerCase().includes('upgrade: websocket')) {
            if (!settled) {
              settled = true;
              clearTimeout(timer);
              connecting = null;
              try { s.destroy(); } catch { /* ignore */ }
              reject(new Error('rosbridge websocket upgrade failed'));
            }
            return;
          }
          const acc = (head.match(/Sec-WebSocket-Accept:\s*(\S+)/i) || [])[1];
          if (acc && acc !== expect) {
            if (!settled) {
              settled = true;
              clearTimeout(timer);
              connecting = null;
              try { s.destroy(); } catch { /* ignore */ }
              reject(new Error('rosbridge accept mismatch'));
            }
            return;
          }
          upgraded = true;
          sock = s;
          ready = true;
          buf = rest;
          s.removeAllListeners('data');
          s.on('data', onBytes);
          if (buf.length) onBytes(Buffer.alloc(0));
          try {
            sendJson({ op: 'advertise', topic: CMD_IN, type: TWIST_TYPE });
            advertised = true;
          } catch (e) {
            if (!settled) {
              settled = true;
              clearTimeout(timer);
              connecting = null;
              closeSock();
              reject(e);
            }
            return;
          }
          if (!settled) {
            settled = true;
            clearTimeout(timer);
            connecting = null;
            resolve();
          }
          return;
        }
      });
    });
    return connecting;
  }

  async function publishTwist(lin, ang) {
    await connect();
    if (!advertised) {
      sendJson({ op: 'advertise', topic: CMD_IN, type: TWIST_TYPE });
      advertised = true;
    }
    sendJson({ op: 'publish', topic: CMD_IN, msg: twistMsg(lin, ang) });
  }

  async function flush() {
    if (sending) return;
    sending = true;
    try {
      while (latest) {
        const job = latest;
        latest = null;
        await publishTwist(job.lin, job.ang);
      }
    } finally {
      sending = false;
    }
  }

  async function sendVelocity(lin, ang) {
    latest = { lin, ang };
    await flush();
  }

  async function handle(body, { status, statusAgeMs } = {}) {
    const cmd = parseDriveBody(body);
    let panel = status;
    let age = statusAgeMs == null ? Infinity : statusAgeMs;
    if (!cmd.stop && (age > STALE_MS || !panel) && typeof refreshStatus === 'function') {
      try {
        panel = await refreshStatus();
        age = 0;
      } catch (e) {
        const p = permit(panel);
        return {
          http: 503,
          body: {
            ok: false,
            published: false,
            lin: cmd.lin,
            ang: cmd.ang,
            stop: cmd.stop,
            error: 'Could not refresh Wally status before a drive command: ' + (e.message || e),
            badge: p.badge || 'Status unknown — manual disabled',
            code: 'unknown',
            reason: 'Manual drive is refused when live status cannot be confirmed.'
          }
        };
      }
    }
    if (!panel && typeof getStatus === 'function') {
      const cached = getStatus();
      if (cached && cached.panel) {
        panel = cached.panel;
        age = Date.now() - (cached.t || 0);
      }
    }
    const p = permit(panel);
    if (!cmd.stop && !p.allowed) {
      return {
        http: 409,
        body: {
          ok: false,
          published: false,
          lin: cmd.lin,
          ang: cmd.ang,
          stop: false,
          error: p.reason || p.badge,
          badge: p.badge,
          code: p.code,
          reason: p.reason
        }
      };
    }
    // Never inject a zero onto /gb/cmd_in during explore: that would finish the mission stream.
    if (cmd.stop && p.code === 'explore') {
      return {
        http: 200,
        body: {
          ok: true,
          published: false,
          skipped: true,
          lin: 0,
          ang: 0,
          stop: true,
          badge: p.badge,
          code: p.code,
          reason: p.reason
        }
      };
    }
    try {
      await sendVelocity(cmd.lin, cmd.ang);
    } catch (e) {
      return {
        http: 502,
        body: {
          ok: false,
          published: false,
          lin: cmd.lin,
          ang: cmd.ang,
          stop: cmd.stop,
          error: 'Could not reach Wall-E motion gate (rosbridge): ' + (e.message || e),
          badge: p.badge,
          code: p.code,
          reason: 'Rosbridge publish to /gb/cmd_in failed.'
        }
      };
    }
    return {
      http: 200,
      body: {
        ok: true,
        published: true,
        lin: cmd.lin,
        ang: cmd.ang,
        stop: cmd.stop,
        badge: p.badge,
        code: p.code,
        reason: cmd.stop ? 'Stop sent on the gated /gb/cmd_in stream.' : 'Drive pulse sent on the gated /gb/cmd_in stream.',
        caps: { hard_max_lin: HARD_MAX_LIN, hard_max_ang: HARD_MAX_ANG }
      }
    };
  }

  return {
    handle,
    permit,
    parseDriveBody,
    caps: { hard_max_lin: HARD_MAX_LIN, hard_max_ang: HARD_MAX_ANG, min_lin: MIN_LIN, default_lin: DEFAULT_LIN, default_ang: DEFAULT_ANG },
    close: closeSock
  };
}

module.exports = {
  createWallyDrive,
  permit,
  parseDriveBody,
  clamp,
  HARD_MAX_LIN,
  HARD_MAX_ANG,
  MIN_LIN,
  DEFAULT_LIN,
  DEFAULT_ANG
};
