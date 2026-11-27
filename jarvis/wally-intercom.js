'use strict';
// Wall-E two-way intercom bridge (half duplex). Browser <-> WebSocket (/api/intercom/ws) <-> TCP <-> gb-intercom on the Pi.
// Audio only: never arms, disarms or drives Wall-E. Node built-ins only (minimal RFC 6455 server, no npm).
const net = require('net'), crypto = require('crypto');

function createWallyIntercom(opts = {}) {
  const PI_HOST = opts.host || '192.168.50.125', PI_PORT = opts.port || 8791;
  const log = opts.log || (() => {});
  const ORIGIN_RE = /^http:\/\/(127\.0\.0\.1|localhost):8765$/;
  const HOST_OK = h => h === '127.0.0.1:8765' || h === 'localhost:8765';
  const CLIENT_IDLE_MS = 8000;
  let cur = null;               // active session {ws, pi, mode, ...}
  const hist = [];               // last few session summaries
  let lastError = '';

  // ---------- Pi framing ----------
  const frame = (kind, payload) => { const b = Buffer.isBuffer(payload) ? payload : Buffer.from(JSON.stringify(payload)); const h = Buffer.alloc(5); h.write(kind, 0, 'latin1'); h.writeUInt32BE(b.length, 1); return Buffer.concat([h, b]); };
  function piReader(onFrame) {
    let buf = Buffer.alloc(0);
    return chunk => {
      buf = buf.length ? Buffer.concat([buf, chunk]) : chunk;
      while (buf.length >= 5) {
        const n = buf.readUInt32BE(1); if (buf.length < 5 + n) break;
        const kind = buf.toString('latin1', 0, 1), body = buf.subarray(5, 5 + n); buf = buf.subarray(5 + n);
        onFrame(kind, body);
      }
    };
  }

  // ---------- WebSocket (server side) ----------
  function wsSend(sock, op, data) {
    if (!sock || sock.destroyed) return;
    const len = data.length; let h;
    if (len < 126) { h = Buffer.from([0x80 | op, len]); }
    else if (len < 65536) { h = Buffer.alloc(4); h[0] = 0x80 | op; h[1] = 126; h.writeUInt16BE(len, 2); }
    else { h = Buffer.alloc(10); h[0] = 0x80 | op; h[1] = 127; h.writeBigUInt64BE(BigInt(len), 2); }
    try { sock.write(Buffer.concat([h, data])); } catch { }
  }
  const wsJSON = (sock, o) => wsSend(sock, 1, Buffer.from(JSON.stringify(o)));
  function wsReader(onMsg, onClose) {
    let buf = Buffer.alloc(0), frag = null, fragOp = 0;
    return chunk => {
      buf = buf.length ? Buffer.concat([buf, chunk]) : chunk;
      for (;;) {
        if (buf.length < 2) return;
        const fin = !!(buf[0] & 0x80), op = buf[0] & 0x0f, masked = !!(buf[1] & 0x80); let len = buf[1] & 0x7f, off = 2;
        if (len === 126) { if (buf.length < 4) return; len = buf.readUInt16BE(2); off = 4; }
        else if (len === 127) { if (buf.length < 10) return; len = Number(buf.readBigUInt64BE(2)); off = 10; }
        if (len > (1 << 20)) return onClose('frame too large');
        if (!masked) return onClose('unmasked client frame');
        if (buf.length < off + 4 + len) return;
        const mask = buf.subarray(off, off + 4); const data = Buffer.from(buf.subarray(off + 4, off + 4 + len));
        for (let i = 0; i < data.length; i++) data[i] ^= mask[i & 3];
        buf = buf.subarray(off + 4 + len);
        if (op === 8) return onClose('client closed');
        if (op === 9) { onMsg(10, data, true); continue; }
        if (op === 10) continue;
        if (op === 0) { if (!frag) continue; frag.push(data); if (fin) { onMsg(fragOp, Buffer.concat(frag)); frag = null; } continue; }
        if (!fin) { frag = [data]; fragOp = op; continue; }
        onMsg(op, data);
      }
    };
  }

  function burstAdd(s, d) { const b = s.burst || (s.burst = { n: 0, sq: 0, peak: 0, t0: Date.now() }); for (let i = 0; i + 1 < d.length; i += 2) { const v = d.readInt16LE(i); b.sq += v * v; if (Math.abs(v) > b.peak) b.peak = Math.abs(v); } b.n += d.length >> 1; }
  function burstEnd(s) { const b = s.burst; s.burst = null; if (!b || !b.n) { log('intercom talk burst', s.id, 'NO audio from browser'); return; } const rms = Math.sqrt(b.sq / b.n), dbfs = +(20 * Math.log10(rms / 32768 + 1e-9)).toFixed(1); s.lastTalk = { seconds: +(b.n / 16000).toFixed(2), rms_dbfs: dbfs, peak: b.peak, at: new Date().toISOString() }; log('intercom talk burst', s.id, JSON.stringify(s.lastTalk)); }
  function endSession(s, why) {
    if (!s || s.ended) return;
    if (s.mode === 'talk') burstEnd(s);
    s.ended = true; clearInterval(s.timer);
    try { if (s.pi && !s.pi.destroyed) { s.pi.write(frame('J', { op: 'stop' })); s.pi.end(); setTimeout(() => { try { s.pi.destroy(); } catch { } }, 300); } } catch { }
    try { wsJSON(s.ws, { op: 'state', state: 'off', why }); wsSend(s.ws, 8, Buffer.alloc(0)); s.ws.end(); setTimeout(() => { try { s.ws.destroy(); } catch { } }, 300); } catch { }
    const sum = { id: s.id, started: new Date(s.t0).toISOString(), ended: new Date().toISOString(), why, bytes_to_walle: s.up, bytes_from_walle: s.down, talk_switches: s.switches, last_talk: s.lastTalk || null };
    hist.unshift(sum); hist.length = Math.min(hist.length, 10);
    if (cur === s) cur = null;
    log('intercom end', s.id, why, `up=${s.up} down=${s.down}`);
  }

  function handleUpgrade(req, sock) {
    const u = new URL(req.url, 'http://127.0.0.1:8765');
    if (u.pathname !== '/api/intercom/ws') return false;
    const host = (req.headers.host || '').toLowerCase(), origin = req.headers.origin || '';
    const key = req.headers['sec-websocket-key'];
    if (!HOST_OK(host) || !ORIGIN_RE.test(origin) || !key || String(req.headers.upgrade || '').toLowerCase() !== 'websocket') {
      sock.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n'); return true;
    }
    const accept = crypto.createHash('sha1').update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
    sock.write(`HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ${accept}\r\n\r\n`);
    sock.setNoDelay(true);
    if (cur) endSession(cur, 'replaced by a new Jarvis intercom client');
    const s = { id: crypto.randomBytes(4).toString('hex'), ws: sock, pi: null, mode: 'connecting', t0: Date.now(), lastRx: Date.now(), up: 0, down: 0, switches: 0, ended: false };
    cur = s;
    log('intercom start', s.id);
    wsJSON(sock, { op: 'state', state: 'connecting', session: s.id });
    const pi = net.connect({ host: PI_HOST, port: PI_PORT });
    s.pi = pi; pi.setNoDelay(true); pi.setTimeout(4000);
    pi.on('connect', () => { pi.setTimeout(0); pi.write(frame('J', { op: 'hello', client: 'jarvis', session: s.id })); });
    pi.on('timeout', () => { lastError = 'Pi intercom connect timeout'; endSession(s, lastError); });
    pi.on('error', e => { lastError = 'Pi intercom: ' + e.message; endSession(s, lastError); });
    pi.on('close', () => endSession(s, 'Pi intercom closed'));
    pi.on('data', piReader((kind, body) => {
      if (kind === 'A') { s.down += body.length; return wsSend(sock, 2, body); }
      let m = {}; try { m = JSON.parse(body.toString()); } catch { return; }
      if (m.op === 'ready') { s.mode = 'listen'; lastError = ''; wsJSON(sock, { op: 'state', state: 'listen', session: s.id, rate: m.rate || 16000 }); }
      else if (m.op === 'mode') { s.mode = m.mode; wsJSON(sock, { op: 'state', state: m.mode }); }
      else if (m.op === 'warn') wsJSON(sock, { op: 'warn', msg: m.msg });
      else if (m.op === 'bye') endSession(s, 'Pi: ' + (m.why || 'bye'));
    }));
    sock.on('data', wsReader((op, data, isPing) => {
      s.lastRx = Date.now();
      if (isPing) return wsSend(sock, 10, data);
      if (op === 2) { if (s.mode === 'talk' && !pi.destroyed && pi.writable) { s.up += data.length; burstAdd(s, data); pi.write(frame('A', data)); } return; }
      let m = {}; try { m = JSON.parse(data.toString()); } catch { return; }
      if (m.op === 'mode' && (m.mode === 'talk' || m.mode === 'listen')) {
        if (m.mode !== s.mode) s.switches++;
        if (m.mode === 'listen' && s.mode === 'talk') burstEnd(s);
        s.mode = m.mode;                          // optimistic: lets the first audio frames through immediately
        if (pi.writable) pi.write(frame('J', { op: 'mode', mode: m.mode }));
      } else if (m.op === 'ping') { if (pi.writable) pi.write(frame('J', { op: 'ping' })); wsJSON(sock, { op: 'pong', t: m.t }); }
      else if (m.op === 'stop') endSession(s, 'user turned intercom off');
    }, why => endSession(s, why)));
    sock.on('close', () => endSession(s, 'browser disconnected'));
    sock.on('end', () => endSession(s, 'browser disconnected'));   // http upgrade sockets are allowHalfOpen: FIN arrives as 'end'
    sock.on('error', () => endSession(s, 'browser socket error'));
    s.timer = setInterval(() => { if (Date.now() - s.lastRx > CLIENT_IDLE_MS) endSession(s, 'watchdog: browser silent 8s'); }, 1000);
    return true;
  }

  function probePi(ms = 2500) {
    return new Promise(resolve => {
      const c = net.connect({ host: PI_HOST, port: PI_PORT }); let done = false;
      const fin = r => { if (done) return; done = true; try { c.destroy(); } catch { } resolve(r); };
      c.setTimeout(ms, () => fin({ ok: false, error: 'timeout' }));
      c.on('error', e => fin({ ok: false, error: e.message }));
      c.on('connect', () => c.write(frame('J', { op: 'status' })));
      c.on('data', piReader((k, b) => { try { fin(Object.assign({ ok: true }, JSON.parse(b.toString()))); } catch (e) { fin({ ok: false, error: 'bad reply' }); } }));
    });
  }

  function status() {
    return { ok: true, active: !!cur, mode: cur ? cur.mode : 'off', session: cur ? cur.id : null, since: cur ? new Date(cur.t0).toISOString() : null,
      bytes_to_walle: cur ? cur.up : 0, bytes_from_walle: cur ? cur.down : 0, last_talk: cur ? (cur.lastTalk || null) : null, pi: `${PI_HOST}:${PI_PORT}`, last_error: lastError, recent: hist.slice(0, 5) };
  }
  function stop(why = 'stopped via API') { if (cur) { endSession(cur, why); return { ok: true, stopped: true }; } return { ok: true, stopped: false }; }
  return { handleUpgrade, status, stop, probePi };
}
module.exports = { createWallyIntercom };
