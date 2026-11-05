#!/usr/bin/env python3
"""gb_clap.py - always-on double-clap e-stop (systemd gb-clap, Restart=always, Nice -15; independent of grokbot + network).

Reads the USB mic continuously through the shared ALSA dsnoop PCM 'gbmic' (grokbot's arecord uses the same PCM).
Detector: 4th-order high-pass ~1.5 kHz -> 1 ms energy envelope (dB) -> onset = rise >= RISE dB within <= 10 ms AND peak
>= floor + MARGIN dB, confirmed by decay >= DECAY dB within 100 ms -> two onsets 150..600 ms apart = DOUBLE CLAP.
Adaptive noise floor = slow 20th-percentile of the envelope over the last ~2 s (track noise raises it, so it can't fire
on steady motor noise).  Config: /home/pi/grokbot/data/clap.json {"sensitivity": low|med|high|max|<margin dB>,
"test_until": epoch}  (re-read every second; voice: 'clap sensitivity higher/lower', 'test clap').
On a double clap: ESTOP files (host + container mirror), mission abort file, zero via rosbridge (best effort),
log to data/clap.log; grokbot's estop reactor kills missions/features and speaks 'Emergency stop' (if grokbot is down,
we play data/estop.wav ourselves).  Test mode: writes data/clap_event.json only, no latch.
Usage: gb_clap.py               (daemon)     gb_clap.py --wav f.wav [--sens high]   (offline test, prints events)
"""
import json, math, os, subprocess, sys, time
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

GB = "/home/pi/grokbot"
CFG = os.path.join(GB, "data", "clap.json")
ESTOP = os.path.join(GB, "data", "ESTOP")
SHARED_ESTOP = "/home/pi/docker/tmp/gb_safety/ESTOP"
ABORT = "/home/pi/docker/tmp/grokbot/explore/abort"
LOGF = os.path.join(GB, "data", "clap.log")
EVENT = os.path.join(GB, "data", "clap_event.json")
WAV = os.path.join(GB, "data", "estop.wav")
PCM = os.environ.get("CLAP_PCM", "gbmic")
SR = 48000
PRESETS = {"low": (18.0, 24.0), "med": (13.0, 17.0), "high": (10.0, 12.0), "max": (7.0, 8.0)}   # (rise dB, margin dB)
# absolute peak (1 ms, >1.5 kHz, dBFS) an onset must reach: a hand clap within a few metres is far louder than room
# clicks/footsteps (the 2026-11-04 22:18 false trigger peaked at -40/-50 dBFS over a -62 dB floor).
ABS_MIN = {"low": -22.0, "med": -27.0, "high": -32.0, "max": -38.0}
# while the robot's own speaker plays (data/speaking.json or any ALSA playback stream, +0.5 s tail) the detector is NOT
# deaf: it raises rise/margin by SPEAK_BOOST dB, needs a loud absolute peak and a sharp broadband transient (energy
# above 11 kHz rising with it; TTS is 22-24 kHz sampled so it has ~nothing up there, plosives are band-limited).
SPEAK_BOOST = float(os.environ.get("CLAP_SPEAK_BOOST", "13"))
SPEAK_ABS = float(os.environ.get("CLAP_SPEAK_ABS", "-20"))
SPEAK_HF_RISE = float(os.environ.get("CLAP_SPEAK_HF_RISE", "12"))
SPEAK_FLAG = os.path.join(GB, "data", "speaking.json")
SPEAK_TAIL = 0.5

def log(*a):
    line = time.strftime("%Y-%m-%d %H:%M:%S") + " [clap] " + " ".join(str(x) for x in a)
    print(line, flush=True)
    try:
        with open(LOGF, "a") as f: f.write(line + "\n")
    except Exception: pass

def sens_params(s):
    s = str(s if s is not None else "high").lower().strip()
    if s in PRESETS: return PRESETS[s] + (s,)
    try:
        m = float(s.replace("db", "")); return (max(6.0, m * 0.75), m, "%gdB" % m)
    except ValueError:
        return PRESETS["high"] + ("high",)

class Detector:
    def __init__(self, sr=SR, sens="high"):
        self.sr = sr; self.hop = sr // 1000                    # 1 ms envelope
        self.sos = butter(4, 1500, "highpass", fs=sr, output="sos"); self.zi = sosfilt_zi(self.sos) * 0.0
        self.sos_hi = butter(4, min(11000, sr * 0.45), "highpass", fs=sr, output="sos"); self.zi_hi = sosfilt_zi(self.sos_hi) * 0.0
        self.env_hi = []; self.rem_hi = np.zeros(0, dtype=np.float32); self.speaking = False; self.debug = None
        self.set_sens(sens)
        self.env = []                                         # recent dB values (1 ms), ~2.2 s
        self.floor = -90.0; self.n = 0; self.rem = np.zeros(0, dtype=np.float32)
        self.pending = None                                   # onset awaiting decay confirmation
        self.onsets = []; self.last_onset = -1.0; self.events = []
        self.fhist = []
    def set_sens(self, sens):
        self.rise, self.margin, self.sens_name = sens_params(sens)
        self.decay = max(8.0, self.rise * 0.7)
        self.abs_min = ABS_MIN.get(self.sens_name, -32.0 if not self.sens_name.endswith("dB") else -20.0 - self.margin)
    def process(self, x):
        """x: float32 mono samples (-1..1). Returns list of double-clap events (dict)."""
        y, self.zi = sosfilt(self.sos, x, zi=self.zi)
        h, self.zi_hi = sosfilt(self.sos_hi, x, zi=self.zi_hi)
        y = np.concatenate([self.rem, y.astype(np.float32)]); h = np.concatenate([self.rem_hi, h.astype(np.float32)])
        k = len(y) // self.hop
        self.rem = y[k * self.hop:]; self.rem_hi = h[k * self.hop:]
        out = []
        if k == 0: return out
        e = y[:k * self.hop].reshape(k, self.hop); eh = h[:k * self.hop].reshape(k, self.hop)
        db = 10 * np.log10(np.mean(e * e, axis=1) + 1e-12); dbh = 10 * np.log10(np.mean(eh * eh, axis=1) + 1e-12)
        for v, vh in zip(db, dbh):
            self.env_hi.append(float(vh))
            if len(self.env_hi) > 64: self.env_hi.pop(0)
            ev = self._step(float(v))
            if ev: out.append(ev)
        return out
    def _step(self, v):
        self.n += 1; t = self.n / 1000.0
        self.env.append(v)
        if len(self.env) > 2200: self.env.pop(0)
        if self.n % 50 == 0 and len(self.env) > 200:          # adaptive floor: 30th percentile of the last ~2 s
            self.floor = float(np.percentile(self.env, 30))
        ev = None
        if self.pending:
            p = self.pending; dt = t - p["t"]
            if dt <= 0.012: p["peak"] = max(p["peak"], v)
            if dt <= 0.004: p["hf_peak"] = max(p["hf_peak"], self.env_hi[-1])
            p["tail"].append(v)
            if dt >= 0.100:                                   # decay judged on the 60..100 ms tail (power mean)
                tail = p["tail"][-40:]
                tail_db = 10 * math.log10(sum(10 ** (x / 10) for x in tail) / len(tail) + 1e-12)
                self.pending = None
                p["tail_db"] = tail_db; p["hf_rise"] = p["hf_peak"] - p["hf_base"]
                ok = (tail_db <= p["peak"] - self.decay or tail_db <= self.floor + 4.0) and p["peak"] >= p["abs_min"]
                if ok and p["spk"]:
                    ok = p["hf_rise"] >= SPEAK_HF_RISE
                if self.debug is not None:
                    self.debug.append({k: (round(v, 2) if isinstance(v, float) else v) for k, v in p.items() if k != "tail"} | {"ok": ok})
                if ok:
                    self._onset(p); ev = self._double()
        elif self.n > 600 and t - self.last_onset > 0.08 and len(self.env) > 20:   # 0.6 s warm-up for the floor
            pre = self.env[-16:-4]                             # 4..15 ms ago (power mean)
            base = 10 * math.log10(sum(10 ** (x / 10) for x in pre) / len(pre) + 1e-12)
            boost = SPEAK_BOOST if self.speaking else 0.0
            if v - base >= self.rise + boost and v - self.floor >= self.margin + boost:
                ph = self.env_hi[-16:-4] or [-120.0]
                hb = 10 * math.log10(sum(10 ** (x / 10) for x in ph) / len(ph) + 1e-12)
                self.pending = {"t": t, "peak": v, "rise": v - base, "floor": self.floor, "tail": [], "spk": self.speaking,
                                "abs_min": max(self.abs_min, SPEAK_ABS) if self.speaking else self.abs_min,
                                "hf_base": hb, "hf_peak": self.env_hi[-1]}
        return ev
    def _onset(self, p):
        self.last_onset = p["t"]; self.onsets.append(p)
        self.onsets = [o for o in self.onsets if p["t"] - o["t"] <= 0.65]
    def _double(self):
        last = self.onsets[-1]
        for o in self.onsets[:-1]:
            dt = last["t"] - o["t"]
            if 0.150 <= dt <= 0.600:
                self.onsets = []
                return {"t": round(last["t"], 3), "gap_ms": round(dt * 1000), "rise_db": [round(o["rise"], 1), round(last["rise"], 1)],
                        "peak_db": [round(o["peak"], 1), round(last["peak"], 1)], "floor_db": round(last["floor"], 1),
                        "sens": self.sens_name, "speaking": [o["spk"], last["spk"]],
                        "hf_rise": [round(o.get("hf_rise", 0), 1), round(last.get("hf_rise", 0), 1)]}
        return None

def _pid_alive(pid):
    try: os.kill(int(pid), 0); return True
    except Exception: return False

def speaker_busy(state):
    """True while the robot's speaker plays anything (+SPEAK_TAIL): grokbot's flag or any ALSA playback substream."""
    now = time.time(); on = False
    try:
        d = json.load(open(SPEAK_FLAG))
        on = (d.get("until", 1) == 0 and _pid_alive(d.get("pid", 0)) and now - d.get("start", 0) < 120) or d.get("until", 0) > now
    except Exception: pass
    if not on:
        for st in state.setdefault("pcm", []) or []:
            try:
                if "RUNNING" in open(st).read(200): on = True; break
            except Exception: pass
    if on: state["last"] = now
    return now - state.get("last", 0.0) < SPEAK_TAIL

def read_cfg():
    try: return json.load(open(CFG))
    except Exception: return {"sensitivity": "high"}

def fire(ev, test):
    ev = dict(ev, ts=time.strftime("%Y-%m-%d %H:%M:%S"), test=bool(test))
    if test:
        log("TEST double clap (no latch):", json.dumps(ev))
        with open(EVENT + ".tmp", "w") as f: json.dump(ev, f)
        os.replace(EVENT + ".tmp", EVENT); return
    d = {"t": time.time(), "ts": ev["ts"], "src": "clap", "why": "double clap gap=%sms rise=%s" % (ev["gap_ms"], ev["rise_db"])}
    for p in (ESTOP, SHARED_ESTOP):
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w") as f: json.dump(d, f)
        except Exception as ex: log("estop write failed", p, ex)
    try:
        os.makedirs(os.path.dirname(ABORT), exist_ok=True); open(ABORT, "w").write("ESTOP clap")
    except Exception: pass
    log("DOUBLE CLAP -> ESTOP LATCHED", json.dumps(ev))
    with open(EVENT + ".tmp", "w") as f: json.dump(ev, f)
    os.replace(EVENT + ".tmp", EVENT)
    try:                                                     # zero via rosbridge (best effort; gate+watchdog act on the file)
        if "/home/pi/grokbot/venv/lib/python3.11/site-packages" not in sys.path:
            sys.path.append("/home/pi/grokbot/venv/lib/python3.11/site-packages")
        import websocket
        ws = websocket.create_connection("ws://127.0.0.1:9090", timeout=1.5)
        z = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        ws.send(json.dumps({"op": "advertise", "topic": "/gb/motion_req", "type": "std_msgs/String"}))
        ws.send(json.dumps({"op": "advertise", "topic": "/controller/cmd_vel", "type": "geometry_msgs/Twist"}))
        time.sleep(0.2)
        for _ in range(5):
            ws.send(json.dumps({"op": "publish", "topic": "/gb/motion_req", "msg": {"data": json.dumps({"op": "stop", "src": "clap"})}}))
            ws.send(json.dumps({"op": "publish", "topic": "/controller/cmd_vel", "msg": z})); time.sleep(0.05)
        ws.close()
    except Exception as ex:
        log("rosbridge zero skipped:", ex)
    try:
        if subprocess.run(["systemctl", "is-active", "--quiet", "grokbot"]).returncode != 0 and os.path.exists(WAV):
            subprocess.run(["aplay", "-q", "-D", os.environ.get("CLAP_OUT", "plughw:CARD=Device,DEV=0"), WAV], timeout=5)
    except Exception: pass

def daemon():
    cfg = read_cfg(); det = Detector(SR, cfg.get("sensitivity", "high")); t_cfg = time.time()
    log("up: pcm=%s sens=%s (rise %.0f dB, margin %.0f dB, decay %.0f dB)" % (PCM, det.sens_name, det.rise, det.margin, det.decay))
    p = subprocess.Popen(["arecord", "-q", "-D", PCM, "-f", "S16_LE", "-r", str(SR), "-c", "1", "-t", "raw", "--buffer-time=100000"],
                         stdout=subprocess.PIPE)
    last_fire = 0.0; last_stat = time.time()
    import glob
    spk = {"pcm": glob.glob("/proc/asound/card*/pcm*p/sub*/status")}; n_spk = 0
    while True:
        b = p.stdout.read(SR // 100 * 2)                    # 10 ms
        if not b:
            log("arecord ended (rc=%s) -> exit for systemd restart" % p.poll()); sys.exit(1)
        x = np.frombuffer(b, dtype=np.int16).astype(np.float32) / 32768.0
        det.speaking = speaker_busy(spk)
        for ev in det.process(x):
            test = float(cfg.get("test_until", 0) or 0) > time.time()
            if test or time.time() - last_fire > 4.0:              # 4 s hold-off: never re-fire on our own 'Emergency stop'
                last_fire = time.time(); fire(ev, test)
        now = time.time()
        if now - t_cfg > 1.0:
            t_cfg = now; c2 = read_cfg()
            if str(c2.get("sensitivity")) != str(cfg.get("sensitivity")):
                det.set_sens(c2.get("sensitivity", "high")); log("sensitivity ->", det.sens_name, "rise", det.rise, "margin", det.margin)
            cfg = c2
        if now - last_stat > 600:
            last_stat = now; log("alive: floor %.1f dB sens %s" % (det.floor, det.sens_name))

def wav_test(path, sens, speaking=None, debug=None):
    """speaking: None = never, True = always, or list of (start_s, end_s) windows (end gets +SPEAK_TAIL)."""
    import wave
    w = wave.open(path); sr = w.getframerate(); ch = w.getnchannels()
    a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768.0
    if ch > 1: a = a.reshape(-1, ch).mean(axis=1)
    det = Detector(sr, sens); evs = []; det.debug = debug
    for i in range(0, len(a), sr // 100):
        t = i / sr
        det.speaking = bool(speaking) if not isinstance(speaking, list) else any(s <= t <= e + SPEAK_TAIL for s, e in speaking)
        for ev in det.process(a[i:i + sr // 100]):
            ev["at_s"] = round(t, 2); evs.append(ev)
    return evs

if __name__ == "__main__":
    if "--wav" in sys.argv:
        f = sys.argv[sys.argv.index("--wav") + 1]
        s = sys.argv[sys.argv.index("--sens") + 1] if "--sens" in sys.argv else "high"
        sp = True if "--speaking" in sys.argv else None
        print(json.dumps({"wav": os.path.basename(f), "sens": s, "speaking": bool(sp), "events": wav_test(f, s, sp)}))
    else:
        daemon()
