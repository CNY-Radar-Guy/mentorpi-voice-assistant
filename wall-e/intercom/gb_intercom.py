#!/usr/bin/env python3
"""Wall-E two-way intercom daemon (half duplex) for Jarvis on ALPHA-ONE.

Audio only. Never arms, disarms or publishes motion; imports nothing from the gate/obstacle/explore code.

TCP :8791 (allow-listed peers only). Frames: 1 byte type + 4 byte big-endian length + payload.
  'J' = UTF-8 JSON control, 'A' = raw PCM S16_LE 16 kHz mono.
Client -> Pi:  {"op":"hello"} starts a session (LISTEN), {"op":"mode","mode":"talk"|"listen"},
               {"op":"ping"}, {"op":"stop"}, {"op":"status"} (probe; no session), 'A' = John's voice (TALK only).
Pi -> client:  {"op":"ready"|"mode"|"state"|"warn"|"bye"|"status"} and 'A' = Wall-E mic audio (LISTEN only).

Coexistence with grokbot:
  * mic:     arecord on the shared dsnoop PCM "gbmic" (same as grokbot), so nobody locks anybody out.
  * speaker: aplay on plughw:CARD=Device only while John is talking; data/speaking.json is marked like any
             other player so grokbot's echo masks see it.
  * hold:    data/intercom_hold.json {pid, mode, until} heartbeated every 0.5 s while a session runs.
             grokbot pauses wake/conversation (and the voice kill-switch while TALK or stationary) only while
             the heartbeat is fresh and this pid is alive, so a crash or kill fails open to normal listening.
Session ends (and everything is released + the hold cleared) on: stop op, socket close, 4 s without any
frame from the client (watchdog), new session taking over, SIGTERM.
"""
import json, os, signal, socket, struct, subprocess, sys, threading, time

PORT = int(os.environ.get("INTERCOM_PORT", "8791"))
ALLOW = {a.strip() for a in os.environ.get("INTERCOM_ALLOW", "192.168.50.30,127.0.0.1").split(",") if a.strip()}
DATA = os.environ.get("GB_DATA", "/home/pi/grokbot/data")
HOLD = os.path.join(DATA, "intercom_hold.json")
SPEAKING = os.path.join(DATA, "speaking.json")
MIC_DEV = os.environ.get("INTERCOM_MIC", "gbmic")
SPK_DEV = os.environ.get("INTERCOM_SPK", "plughw:CARD=Device,DEV=0")
RATE = 16000
CHUNK = 1280                     # 40 ms of S16 mono @ 16 kHz
WATCHDOG_S = float(os.environ.get("INTERCOM_WATCHDOG_S", "4.0"))
ECHO_TAIL_S = 0.35               # drop Wall-E mic audio this long after John's voice stops (speaker tail)
MAX_FRAME = 1 << 20


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def atomic_json(path, obj):
    tmp = path + ".intercom.tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


def speaking(on):
    try:
        if on:
            atomic_json(SPEAKING, {"pid": os.getpid(), "start": time.time(), "until": 0, "text": "", "src": "intercom"})
        else:
            d = {}
            try:
                d = json.load(open(SPEAKING))
            except Exception:
                pass
            if d.get("pid") == os.getpid():          # never clobber grokbot's own flag
                atomic_json(SPEAKING, {"pid": os.getpid(), "until": time.time() + 0.3, "text": "", "src": "intercom"})
    except Exception as ex:
        log("speaking flag:", ex)


def rms16(b):
    import array, math
    a = array.array("h"); a.frombytes(b[: len(b) // 2 * 2])
    return math.sqrt(sum(x * x for x in a) / len(a)) if a else 0.0


class Session:
    def __init__(self, sock, peer):
        self.sock, self.peer = sock, peer
        self.lock = threading.Lock()           # socket send lock
        self.mode = "listen"
        self.alive = True
        self.last_rx = time.time()
        self.talk_end = 0.0
        self.mic = None
        self.spk = None
        self.spk_retry = 0.0
        self.last_audio = 0.0
        self.stats = {"rx_audio": 0, "tx_audio": 0, "spk_busy": 0}
        self.burst = {"bytes": 0, "sumsq": 0.0, "n": 0, "peak": 0}
        self.started = time.time()
        self.id = "%x" % int(self.started * 1000)

    # ---- io
    def send(self, kind, payload):
        if isinstance(payload, (dict, list)):
            payload = json.dumps(payload).encode()
        with self.lock:
            self.sock.sendall(kind + struct.pack(">I", len(payload)) + payload)

    def sendj(self, **kw):
        try:
            self.send(b"J", kw)
        except Exception:
            pass

    # ---- hold heartbeat
    def hold(self):
        try:
            atomic_json(HOLD, {"pid": os.getpid(), "mode": self.mode, "session": self.id, "peer": self.peer,
                               "start": self.started, "until": time.time() + 2.0})
        except Exception as ex:
            log("hold write failed:", ex)

    # ---- speaker (John -> Wall-E)
    def spk_open(self):
        if self.spk and self.spk.poll() is None:
            return True
        if time.time() < self.spk_retry:
            return False
        self.spk = subprocess.Popen(["aplay", "-q", "-D", SPK_DEV, "-t", "raw", "-f", "S16_LE", "-r", str(RATE),
                                     "-c", "1", "--buffer-time=200000", "--period-time=20000"],
                                    stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(0.03)
        if self.spk.poll() is not None:            # grokbot (or another player) owns the hw device right now
            err = (self.spk.stderr.read() or b"").decode(errors="replace").strip()[:160]
            self.spk = None; self.spk_retry = time.time() + 0.3; self.stats["spk_busy"] += 1
            self.sendj(op="warn", msg="Wall-E speaker busy (grokbot speaking?) - retrying", detail=err)
            return False
        speaking(True)
        return True

    def spk_close(self, drain=True):
        p, self.spk = self.spk, None
        if p is None:
            return
        try:
            p.stdin.close()
        except Exception:
            pass
        try:
            p.wait(timeout=0.6 if drain else 0.05)
        except Exception:
            p.kill()
            try: p.wait(timeout=1)
            except Exception: pass
        try:
            err = (p.stderr.read() or b"").decode(errors="replace").strip()
            log(f"aplay exit={p.returncode}" + (f" stderr={err[:200]!r}" if err else " (clean)"))
        except Exception:
            pass
        speaking(False)

    def play(self, pcm):
        if self.mode != "talk" or not pcm:
            return
        self.last_audio = time.time()
        if not self.spk_open():
            return
        try:
            self.spk.stdin.write(pcm); self.spk.stdin.flush()
            self.stats["rx_audio"] += len(pcm)
            import array
            a = array.array("h"); a.frombytes(pcm[: len(pcm) // 2 * 2])
            b = self.burst; b["bytes"] += len(pcm); b["n"] += len(a)
            b["sumsq"] += float(sum(x * x for x in a)); b["peak"] = max(b["peak"], max((abs(x) for x in a), default=0))
        except Exception:
            self.spk_close(drain=False)

    # ---- mic (Wall-E -> John)
    def mic_loop(self):
        while self.alive:
            try:
                self.mic = subprocess.Popen(["arecord", "-q", "-D", MIC_DEV, "-f", "S16_LE", "-r", str(RATE), "-c", "1",
                                             "-t", "raw"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                while self.alive:
                    b = self.mic.stdout.read(CHUNK)
                    if not b:
                        raise IOError("arecord ended")
                    if self.mode == "listen" and time.time() - self.talk_end > ECHO_TAIL_S:
                        self.send(b"A", b); self.stats["tx_audio"] += len(b)
            except Exception as ex:
                if self.alive:
                    log("mic:", ex); time.sleep(0.5)
            finally:
                self.mic_close()

    def mic_close(self):
        p, self.mic = self.mic, None
        if p:
            try: p.terminate(); p.wait(timeout=1)
            except Exception:
                try: p.kill()
                except Exception: pass

    def set_mode(self, m):
        m = "talk" if m == "talk" else "listen"
        if m == self.mode:
            return
        self.mode = m
        if m == "listen":
            self.spk_close(drain=True); self.talk_end = time.time()
            b = self.burst
            if b["n"]:
                import math
                r = math.sqrt(b["sumsq"] / b["n"])
                log(f"session {self.id}: talk burst played {b['bytes'] / 32000.0:.2f}s rms={r:.0f} ({20 * math.log10(r / 32768 + 1e-9):.1f} dBFS) peak={b['peak']}")
            else:
                log(f"session {self.id}: talk burst had NO audio")
            self.burst = {"bytes": 0, "sumsq": 0.0, "n": 0, "peak": 0}
        self.hold()
        self.sendj(op="mode", mode=m)
        log(f"session {self.id}: mode -> {m}")

    def close(self, why):
        if not self.alive:
            return
        self.alive = False
        self.sendj(op="bye", why=why)
        self.spk_close(drain=(self.mode == "talk")); self.mic_close()   # stop mid-sentence: finish playing what arrived
        try:
            d = json.load(open(HOLD))
            if d.get("session") == self.id:
                os.unlink(HOLD)
        except Exception:
            pass
        try: self.sock.shutdown(socket.SHUT_RDWR)
        except Exception: pass
        try: self.sock.close()
        except Exception: pass
        log(f"session {self.id} closed ({why}); mic+speaker released, grokbot hold cleared; stats {self.stats}")


CURRENT = {"s": None}
CUR_LOCK = threading.Lock()


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        b = sock.recv(n - len(buf))
        if not b:
            raise ConnectionError("peer closed")
        buf.extend(b)
    return bytes(buf)


def status_obj():
    s = CURRENT["s"]
    return {"op": "status", "ok": True, "pid": os.getpid(), "active": bool(s and s.alive),
            "mode": s.mode if s and s.alive else "off", "peer": s.peer if s and s.alive else None,
            "stats": s.stats if s and s.alive else None, "mic": MIC_DEV, "speaker": SPK_DEV, "rate": RATE,
            "hold_file": os.path.exists(HOLD)}


def handle(sock, peer):
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(1.0)
    sess = None
    try:
        while True:
            try:
                hdr = recv_exact(sock, 5)
            except socket.timeout:
                if sess and time.time() - sess.last_rx > WATCHDOG_S:
                    sess.close("watchdog: client silent %.0fs" % WATCHDOG_S); return
                if sess and not sess.alive:
                    return
                continue
            kind, n = hdr[:1], struct.unpack(">I", hdr[1:])[0]
            if n > MAX_FRAME:
                raise ValueError("frame too large")
            sock.settimeout(5.0); payload = recv_exact(sock, n); sock.settimeout(1.0)
            if sess:
                if not sess.alive:
                    return
                sess.last_rx = time.time()
            if kind == b"A":
                if sess: sess.play(payload)
                continue
            msg = json.loads(payload.decode() or "{}")
            op = msg.get("op")
            if op == "status":
                body = json.dumps(status_obj()).encode()
                sock.sendall(b"J" + struct.pack(">I", len(body)) + body)
                if not sess: return
            elif op == "hello":
                with CUR_LOCK:
                    old = CURRENT["s"]
                    if old and old.alive:
                        old.close("replaced by a new intercom session")
                    sess = Session(sock, peer); CURRENT["s"] = sess
                sess.hold()
                threading.Thread(target=sess.mic_loop, daemon=True).start()
                sess.sendj(op="ready", rate=RATE, session=sess.id, mode=sess.mode)
                log(f"session {sess.id} started from {peer} (listen)")
            elif op == "mode" and sess:
                sess.set_mode(msg.get("mode"))
            elif op == "ping" and sess:
                sess.hold(); sess.sendj(op="pong", t=msg.get("t"), mode=sess.mode)
            elif op == "stop":
                if sess: sess.close("client stop")
                return
    except Exception as ex:
        if sess and sess.alive:
            sess.close("connection lost: %s" % ex)
    finally:
        if sess and sess.alive:
            sess.close("handler exit")
        try: sock.close()
        except Exception: pass


def heartbeat():
    while True:
        s = CURRENT["s"]
        if s and s.alive:
            s.hold()
            if s.spk is not None and time.time() - s.last_audio > 0.6:
                s.spk_close(drain=True)           # no audio from John -> release the speaker (silent when idle)
        time.sleep(0.5)


def main():
    try:
        os.unlink(HOLD)                       # stale hold from a previous run
        log("cleared stale hold file")
    except FileNotFoundError:
        pass
    def _term(*_):
        s = CURRENT["s"]
        if s: s.close("service stopping")
        sys.exit(0)
    signal.signal(signal.SIGTERM, _term); signal.signal(signal.SIGINT, _term)
    threading.Thread(target=heartbeat, daemon=True).start()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT)); srv.listen(4)
    log(f"gb-intercom listening on :{PORT} allow={sorted(ALLOW)} mic={MIC_DEV} spk={SPK_DEV}")
    while True:
        c, a = srv.accept()
        if a[0] not in ALLOW:
            log("refused", a[0]); c.close(); continue
        threading.Thread(target=handle, args=(c, a[0]), daemon=True).start()


if __name__ == "__main__":
    main()
