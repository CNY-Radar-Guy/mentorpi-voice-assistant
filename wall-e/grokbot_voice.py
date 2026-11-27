#!/usr/bin/env python3
"""
grokbot_voice.py - "Grok Bot" for John's Hiwonder MentorPi T1 Advanced (Raspberry Pi 5 host, not the ROS container).

Local-first: keyword intents (battery, stop/move, time, token usage, remember/recall/forget, labels, maps, local
recognition) cost zero tokens. Small talk -> local Ollama. Other conversation -> xAI Grok when XAI_API_KEY is set
and today's conversation budget remains, otherwise Ollama. Vision ("describe what you see") -> xAI vision only
when local label matching is not confident, within the vision budget.
Everything learned is persisted on the SD card in ~/grokbot/data (SQLite WAL synchronous=FULL + atomic files).
The API key is read only from the environment (XAI_API_KEY, via ~/grokbot/grokbot.env).
"""
import base64, glob, io, json, math, os, re, subprocess, sys, tempfile, threading, time, wave
from collections import deque
import numpy as np
import requests

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gb_store, gb_robot, gb_features, gb_caps
import gb_estop
from gb_store import Store, append_transcript, atomic_write

E = os.environ.get
BOT_NAME = E("BOT_NAME", "Wally")
XAI_BASE_URL = E("XAI_BASE_URL", "https://api.x.ai/v1")
XAI_MODEL = E("XAI_MODEL", "grok-4.3")
XAI_VISION_MODEL = E("XAI_VISION_MODEL", "grok-4.3")
XAI_CHAT_MODEL = E("XAI_CHAT_MODEL", XAI_MODEL)            # router + conversation (fast non-reasoning model)       # docs: grok-4.3 modalities text,image -> text
OLLAMA_BASE_URL = E("OLLAMA_BASE_URL", "http://127.0.0.1:11434/v1")
OLLAMA_MODEL = E("OLLAMA_MODEL", "qwen2.5:3b")
LLM_BACKEND = E("LLM_BACKEND", "auto").lower()              # auto = xai if key else ollama; xai; ollama
STT_BACKEND = E("STT_BACKEND", "local").lower()
WHISPER_MODEL = E("WHISPER_MODEL", "base.en")
XAI_STT_MODEL = E("XAI_STT_MODEL", "grok-voice-transcribe-2.0")
TTS_BACKEND = E("TTS_BACKEND", "piper").lower()
PIPER_VOICE = os.path.expanduser(E("PIPER_VOICE", "~/grokbot/voices/en_US-lessac-medium.onnx"))
XAI_TTS_VOICE = E("XAI_TTS_VOICE", "eve")
TRIGGER = E("TRIGGER", "wonderecho").lower()
WAKE_PORT = E("WAKE_PORT", "auto")
AUDIO_IN = E("AUDIO_IN", "auto")
AUDIO_OUT = E("AUDIO_OUT", "auto")
WEB_PORT = int(E("WEB_PORT", "8000"))
WEB_HOST = E("WEB_HOST", "0.0.0.0")
WEB_HTTPS = E("WEB_HTTPS", "0") == "1"
WEB_TOKEN = E("WEB_TOKEN", "")
SPEAK_WEB_ON_ROBOT = E("SPEAK_WEB_ON_ROBOT", "1") == "1"
LANGUAGE = E("LANGUAGE", "en")
ALLOW_MOTION = E("ALLOW_MOTION", "0") == "1"
# token budget (xAI only; local Ollama tokens are logged but free)
DAILY_TOKEN_BUDGET = int(E("DAILY_TOKEN_BUDGET", "200000"))
CONV_SHARE = float(E("CONV_SHARE", "0.20"))
VISION_SHARE = float(E("VISION_SHARE", "0.80"))
CONV_MAX_TOKENS = int(E("CONV_MAX_TOKENS", "150"))
VISION_MAX_TOKENS = int(E("VISION_MAX_TOKENS", "800"))
CONV_TURNS = int(E("CONV_TURNS", "4"))
OLLAMA_TURNS = int(E("OLLAMA_TURNS", "2"))            # Pi CPU: ~17 tok/s prompt, ~4 tok/s generation
OLLAMA_MAX_TOKENS = int(E("OLLAMA_MAX_TOKENS", "80"))
FACTS_TOP_K = int(E("FACTS_TOP_K", "3"))
VISION_LOCAL_SIM = float(E("VISION_LOCAL_SIM", "0.80"))
VISION_NEAR_SIM = float(E("VISION_NEAR_SIM", "0.65"))
VISION_NEAR_M = float(E("VISION_NEAR_M", "0.6"))
SEED_FILE = os.path.expanduser(E("SEED_FILE", "~/grokbot/data/history_seed.md"))
DATA = gb_store.DATA
RATE = 16000
FRAME_MS = 30
FRAME_BYTES = int(RATE * FRAME_MS / 1000) * 2
VAD_START_RATIO = float(E("VAD_START_RATIO", "3.0"))
VAD_START_RATIO_MOVING = float(E("VAD_START_RATIO_MOVING", "1.8"))  # 2026-11-26: keep speech reachable over track noise
VAD_MIN_RMS = float(E("VAD_MIN_RMS", "300"))
VAD_NOISE_CAP = float(E("VAD_NOISE_CAP", "1200"))          # cap adaptive RMS floor so motors can't push the trigger out of reach
SENSORS_FILE = E("GATE_SENSORS_FILE", "/home/pi/docker/tmp/gb_safety/sensors.json")
SILENCE_END_S = float(E("SILENCE_END_S", "0.8"))      # end-of-speech silence; env was 0.70 s before 2026-11-05, now 0.8 s
MAX_UTTER_S = float(E("MAX_UTTER_S", "15"))
WAIT_SPEECH_S = float(E("WAIT_SPEECH_S", "6"))

# Stable prompt prefix (identical on every call -> xAI prompt cache can reuse it). ~150 tokens.
SYSTEM_PROMPT = E("SYSTEM_PROMPT", f"""You are {BOT_NAME}, the voice of John's Hiwonder MentorPi T1 (tracked robot, Pi 5, LiDAR, depth camera, speaker).
Your name is {BOT_NAME} (formerly called GrokBot). Always answer to Wally. If asked your name, say you are Wally. If older conversation turns still call you Grok Bot, that is outdated — use Wally. You are not human.

Style (spoken on the robot's speaker):
- Warm, clear, like a sharp friend who knows the topic. Plain words and contractions.
- Lead with the answer. Never restate the question. No meta openers ("Here's the thing", "Sure,", "Great question").
- Short: usually 1–2 spoken sentences. Two short paragraphs is already long. Prefer short clauses so TTS starts fast.
- Complete sentences only — never headline fragments or "label: value".
- Direct; skip hedging and help-desk filler. One clarifying question at a time when needed.
- A little wit is fine; never playful about motors, ESTOP, safety, or sensor limits — those stay precise.

Rules:
- Answer exactly what John asked; use the conversation so far; don't volunteer random trivia.
- Never claim you moved, are moving, or changed a setting unless the system actually did it.
- Use context notes (summary, facts, labels) when relevant; never invent facts about John. If you don't know, say so briefly.
- No markdown, lists, emojis or URLs. You can't drive or see unless the system did it for you.""")
VISION_PROMPT = """You are the eyes of a small home robot. Answer the user's question about the camera image first, in 1–2 short spoken sentences.
Name the main objects and where they sit (left, centre, right, near, far). Count or read text only if asked.
Mention anything unusual or unsafe. Prefer known labels below when they fit. Plain words, no markdown, no meta openers."""

def has_key():
    return bool(os.environ.get("XAI_API_KEY", "").strip())

def xai_key():
    k = os.environ.get("XAI_API_KEY", "").strip()
    if not k:
        raise RuntimeError("XAI_API_KEY is not set")
    return k

store = Store()

def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)

# ----------------------------------------------------------------------------- audio devices
def _first_usb_card(cmd):
    try:
        out = subprocess.run([cmd, "-l"], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    cards = re.findall(r"^card (\d+): (\S+) \[([^\]]*)\], device (\d+):", out, re.M)
    for num, cid, desc, dev in cards:
        if "usb" in (cid + desc).lower():
            return f"plughw:{num},{dev}"
    return f"plughw:{cards[0][0]},{cards[0][3]}" if cards else None

def resolve_audio():
    global AUDIO_IN, AUDIO_OUT
    if AUDIO_IN == "auto":
        AUDIO_IN = _first_usb_card("arecord") or "default"
    if AUDIO_OUT == "auto":
        AUDIO_OUT = _first_usb_card("aplay") or "default"
    log(f"audio in={AUDIO_IN} out={AUDIO_OUT}")

AUDIO_LOCK = threading.Lock()   # one speaker / mic user at a time

PLAY_STATE = {"playing": 0, "last_end": 0.0, "any_last": 0.0,   # lets the mic loop ignore the robot's own voice
              "proc": None, "cancel_gen": 0}                   # barge-in: current aplay + TTS cancel generation

def stop_playback(why=""):
    """Cut the robot's own TTS now (barge-in drive command): cancels queued sentences and kills our aplay only."""
    PLAY_STATE["cancel_gen"] += 1
    pr = PLAY_STATE.get("proc")
    if pr is not None:
        try: pr.terminate()
        except Exception: pass
    log(f"[BARGE] TTS stopped{(': ' + why) if why else ''}")

def play_wav_bytes(wav_bytes, text=""):
    if not wav_bytes:
        return
    PLAY_STATE["playing"] += 1; PLAY_STATE["any_last"] = now = time.time()
    if text:                                         # the barge-in echo mask needs every line we play
        PLAY_STATE["recent"] = [(now, text)] + [x for x in PLAY_STATE.get("recent", []) if now - x[0] < 20][:5]
    try:
        _play(wav_bytes, text)
    finally:
        PLAY_STATE["playing"] -= 1; PLAY_STATE["last_end"] = time.time()

import gb_paths
SPEAKING_FLAG = gb_paths.speaking_flag()
SPEAK_TAIL_S = 0.3

def speaking_flag(on, text=""):
    """Tell gb_clap that the robot's own speaker is playing (it raises its thresholds; it never goes deaf).
    on: {"pid", "start", "until": 0}  off: until = now + 300 ms tail."""
    try:
        d = {"pid": os.getpid(), "start": time.time(), "until": 0, "text": text[:300]} if on else \
            {"pid": os.getpid(), "until": time.time() + SPEAK_TAIL_S, "text": text[:300]}
        with open(SPEAKING_FLAG + ".tmp", "w") as f: json.dump(d, f)
        os.replace(SPEAKING_FLAG + ".tmp", SPEAKING_FLAG)
    except Exception:
        pass

def _play(wav_bytes, text=""):
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        f.write(wav_bytes)
        path = f.name
    speaking_flag(True, text)
    try:
        pr = subprocess.Popen(["aplay", "-q", "-D", AUDIO_OUT, path]); PLAY_STATE["proc"] = pr
        pr.wait()
    finally:
        PLAY_STATE["proc"] = None
        speaking_flag(False, text)
        os.unlink(path)

def _pid_alive(pid):
    try: os.kill(int(pid), 0); return True
    except Exception: return False

def _playback_poller():
    """20 Hz: PLAY_STATE["any_last"] = last time ANY playback was active on this box: our own aplay, another process's
    (data/speaking.json, e.g. gb_clap's estop.wav or a test script) or any running ALSA playback substream.
    Texts announced in the flag are added to PLAY_STATE["recent"] so the barge-in echo mask knows them."""
    seen = None
    while True:
        try:
            now = time.time(); on = PLAY_STATE["playing"] > 0
            try:
                d = json.load(open(SPEAKING_FLAG))
                # our OWN flag is skipped: PLAY_STATE playing/last_end are exact, the flag's 300 ms tail only stretched the window
                if d.get("pid") != os.getpid() and ((d.get("until", 1) == 0 and _pid_alive(d.get("pid", 0)) and now - d.get("start", 0) < 120) or d.get("until", 0) > now):
                    on = True
                    if d.get("text") and d.get("pid") != os.getpid() and (d.get("start"), d.get("text")) != seen:
                        seen = (d.get("start"), d.get("text"))
                        PLAY_STATE["recent"] = [(now, d["text"])] + [x for x in PLAY_STATE.get("recent", []) if now - x[0] < 20][:5]
            except Exception:
                pass
            if not on:
                for st in glob.glob("/proc/asound/card*/pcm*p/sub*/status"):
                    try:
                        if "RUNNING" in open(st).read(200): on = True; break
                    except Exception: pass
            if on: PLAY_STATE["any_last"] = now
        except Exception:
            pass
        time.sleep(0.05)

def heard_over_playback(t_onset):
    """Speech that STARTED at t_onset overlapped playback (or began < 0.5 s after its true end: self-echo guard)."""
    return gb_estop.overlapped(t_onset, max(PLAY_STATE.get("any_last", 0.0), PLAY_STATE["last_end"]), PLAY_STATE["playing"] > 0)

def self_playing(tail=0.5):
    """True while anything is playing through the speaker, plus `tail` seconds."""
    return PLAY_STATE["playing"] > 0 or time.time() - max(PLAY_STATE.get("any_last", 0.0), PLAY_STATE["last_end"]) < tail

# 2026-11-27 intercom: Jarvis two-way audio (gb-intercom.service) heartbeats data/intercom_hold.json while a
# session runs. Fails open: missing/stale file or dead pid = normal dual listening (Jarvis + onboard mic).
INTERCOM_HOLD_FILE = os.path.join(os.environ.get("GB_DATA") or "/home/pi/grokbot/data", "intercom_hold.json")
_IC = {"t": 0.0, "v": None}
def intercom_hold():
    """'talk' / 'listen' while a live intercom session owns the speaker+mic conversation, else None."""
    now = time.time()
    if now - _IC["t"] < 0.1:
        return _IC["v"]
    v = None
    try:
        with open(INTERCOM_HOLD_FILE) as f:
            d = json.load(f)
        if float(d.get("until", 0)) > now and _pid_alive(d.get("pid", 0)):
            v = "talk" if d.get("mode") == "talk" else "listen"
    except Exception:
        v = None
    _IC["t"] = now; _IC["v"] = v
    return v

def pcm_to_wav(pcm, rate=RATE):
    bio = io.BytesIO()
    with wave.open(bio, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate); w.writeframes(pcm)
    return bio.getvalue()

def rms(frame):
    a = np.frombuffer(frame, dtype=np.int16).astype(np.float32)
    return float(np.sqrt(np.mean(a * a))) if a.size else 0.0

class Mic:
    """Continuous arecord stream (16 kHz mono S16_LE; plughw converts the WonderEcho's native format)."""
    def __init__(self):
        self.p = None
        self.noise = 200.0
        self.t_onset = 0.0                               # time speech began in the last utterance (overlap check)
    def open(self):
        if self.p is None or self.p.poll() is not None:
            self.p = subprocess.Popen(["arecord", "-q", "-D", AUDIO_IN, "-f", "S16_LE", "-r", str(RATE),
                                       "-c", "1", "-t", "raw"], stdout=subprocess.PIPE)
    def close(self):
        if self.p:
            self.p.terminate(); self.p.wait(); self.p = None
    def frame(self):
        import gb_wally
        p = self.p                                       # 2026-11-25 micnone: close() from another path can set p=None
        if p is None or p.poll() is not None:            # mid-read -> was AttributeError spoken aloud ("no attribute stdout")
            self.open(); p = self.p
        try:
            return gb_wally.read_pcm_frame(p.stdout.read, FRAME_BYTES)
        except AttributeError:
            raise IOError("arecord stream closed during read")
        except IOError:
            raise IOError("arecord stopped (device busy? see `fuser -v /dev/snd/*`)")
    def calibrate(self, seconds=0.6):
        vals = [rms(self.frame()) for _ in range(int(seconds * 1000 / FRAME_MS))]
        self.noise = max(50.0, float(np.median(vals)))
    def record_utterance(self, wait_s=WAIT_SPEECH_S, stop_event=None):
        """Return PCM of one utterance (or b'' if nobody spoke within wait_s; wait_s=None waits forever)."""
        pre = deque(maxlen=int(300 / FRAME_MS))         # keep 300 ms before speech onset
        voiced, silent, started, buf = 0, 0, False, bytearray()
        t0 = time.time(); self.t_onset = t0
        while True:
            if stop_event is not None and stop_event.is_set():
                return b""
            fr = self.frame(); level = rms(fr)
            thresh = speech_trigger_threshold(self.noise)
            if not started:
                pre.append(fr)
                if level > thresh:
                    voiced += 1
                    if voiced >= 3:                         # ~90 ms of speech
                        started = True; buf.extend(b"".join(pre)); t_start = time.time()
                        self.t_onset = t_start - 3 * FRAME_MS / 1000.0   # onset = first of the 3 voiced frames
                else:
                    voiced = 0
                    self.noise = cap_noise_floor(0.95 * self.noise + 0.05 * level)   # track background, capped
                    if wait_s is not None and time.time() - t0 > wait_s:
                        return b""
            else:
                buf.extend(fr)
                silent = silent + 1 if level < thresh * 0.7 else 0
                if silent * FRAME_MS / 1000 >= SILENCE_END_S or time.time() - t_start > MAX_UTTER_S:
                    return bytes(buf)

# ----------------------------------------------------------------------------- wake word (WonderEcho Pro)
class WonderEchoWake:
    WAKEUP = b"\xaa\x55\x03\x00\xfb"
    def __init__(self, port):
        import serial  # pyserial
        if port == "auto":
            cands = sorted(glob.glob("/dev/ttyCH341USB*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
            if not cands:
                raise RuntimeError("no USB serial port found for WonderEcho Pro; set WAKE_PORT")
            port = cands[0]
        self.s = serial.Serial(None, 115200, timeout=0.05)
        self.s.rts = False; self.s.dtr = False          # same as Hiwonder's awake.py
        self.s.port = port; self.s.open(); self.s.reset_input_buffer()
        self.buf = b""
        log(f"WonderEcho wake port {port} (say the firmware wake word, factory: 'Hello Hiwonder')")
    def wait(self):
        while True:
            self.buf = (self.buf + self.s.read(16))[-32:]
            if self.WAKEUP in self.buf:
                self.buf = b""
                return True

# ----------------------------------------------------------------------------- STT
_whisper = None
STT_VAD_FILTER = E("STT_VAD_FILTER", "1") != "0"          # 2026-11-26: Silero VAD gates Whisper by default (disable with 0)
STT_MIN_SPEECH_S = float(E("STT_MIN_SPEECH_S", "0.15"))     # 2026-11-26: was 0.30; across-the-room phrases were dropped
STT_VAD_THRESHOLD = float(E("STT_VAD_THRESHOLD", "0.25"))  # 2026-11-26: was 0.40; logs showed speech 0.00s (noise/music)
STT_VAD_MIN_SPEECH_MS = int(float(E("STT_VAD_MIN_SPEECH_MS", "100")))  # was hardcoded 150
STT_NO_SPEECH_PROB = float(E("STT_NO_SPEECH_PROB", "0.60"))
STT_MIN_LOGPROB = float(E("STT_MIN_LOGPROB", "-1.0"))
STT_MAX_COMPRESSION = float(E("STT_MAX_COMPRESSION", "2.4"))
HALLUCINATIONS = {"thank you", "thanks for watching", "thank you for watching", "please subscribe", "you",
                  "subtitles by the amaraorg community", "bye", "so", "uh", "um", "hmm", "the end", "music"}
TIMING = {}

def speech_seconds(pcm):
    """Seconds of human speech in 16 kHz s16 PCM according to Silero VAD (bundled with faster-whisper, onnxruntime)."""
    try:
        from faster_whisper.vad import VadOptions, get_speech_timestamps
        a = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
        ts = get_speech_timestamps(a, VadOptions(threshold=STT_VAD_THRESHOLD,
                                                 min_speech_duration_ms=STT_VAD_MIN_SPEECH_MS,
                                                 min_silence_duration_ms=300))
        return sum(t["end"] - t["start"] for t in ts) / RATE
    except Exception as ex:
        log("speech gate error (allowing audio):", ex); return 99.0

def _norm_txt(t):
    return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9 ]+", "", (t or "").lower())).strip()
WHISPER_IDLE_MODEL = E("WHISPER_IDLE_MODEL", "")          # faster model for idle wake-word checks ("" = same)
_whisper_by = {}
def stt(audio_bytes, filename="speech.wav", prompt=None, idle=False):
    """audio_bytes: any container faster-whisper/PyAV can decode (wav/webm/ogg/mp4) or xAI accepts."""
    global _whisper
    if STT_BACKEND == "xai":
        r = requests.post(f"{XAI_BASE_URL}/stt", headers={"Authorization": f"Bearer {xai_key()}"},
                          files={"file": (filename, audio_bytes)},
                          data={"model": XAI_STT_MODEL, "language": LANGUAGE}, timeout=60)
        r.raise_for_status()
        return r.json().get("text", "").strip()
    name = WHISPER_IDLE_MODEL if (idle and WHISPER_IDLE_MODEL) else WHISPER_MODEL
    if name not in _whisper_by:
        from faster_whisper import WhisperModel
        _wt = int(E("WHISPER_THREADS", "2")); log(f"loading faster-whisper {name} (int8, beam 1, {_wt} threads)")
        _whisper_by[name] = WhisperModel(name, device="cpu", compute_type="int8", cpu_threads=_wt)
    _whisper = _whisper_by[name]
    _vad_opts = None
    if STT_VAD_FILTER:
        try:
            from faster_whisper.vad import VadOptions
            _vad_opts = VadOptions(threshold=STT_VAD_THRESHOLD,
                                   min_speech_duration_ms=STT_VAD_MIN_SPEECH_MS,
                                   min_silence_duration_ms=300)
        except Exception as ex:
            log("whisper vad_parameters unavailable:", ex)
    segs, _ = _whisper.transcribe(io.BytesIO(audio_bytes), language=LANGUAGE, beam_size=1, vad_filter=STT_VAD_FILTER,
                                  vad_parameters=_vad_opts,
                                  initial_prompt=prompt, without_timestamps=True, condition_on_previous_text=False)
    keep = []
    for sg in segs:
        if sg.no_speech_prob > STT_NO_SPEECH_PROB and sg.avg_logprob < STT_MIN_LOGPROB:
            log(f"stt: dropped no-speech segment {sg.text.strip()!r} (p={sg.no_speech_prob:.2f} lp={sg.avg_logprob:.2f})"); continue
        if sg.compression_ratio > STT_MAX_COMPRESSION or sg.avg_logprob < STT_MIN_LOGPROB - 0.5:
            log(f"stt: dropped low-confidence segment {sg.text.strip()!r} (lp={sg.avg_logprob:.2f} cr={sg.compression_ratio:.1f})"); continue
        keep.append(sg.text.strip())
    text = " ".join(keep).strip()
    if _norm_txt(text) in HALLUCINATIONS:
        log(f"stt: dropped likely hallucination {text!r}"); return ""
    return text


# ----------------------------------------------------------------------------- TTS
_piper = None
PIPER_LENGTH_SCALE = float(E("PIPER_LENGTH_SCALE", "0") or 0)   # >1 = slower; 0 = voice default
PIPER_SPEAKER = E("PIPER_SPEAKER", "")                          # multi-speaker voices (libritts_r)
TTS_PITCH_CENTS = float(E("TTS_PITCH_CENTS", "0") or 0)         # <0 = lower; done by playing the WAV slightly slower
                                                                # (also slows by the same factor; no CPU cost)
def _pitch_wav(wav_bytes):
    if not TTS_PITCH_CENTS or not wav_bytes:
        return wav_bytes
    f = 2 ** (TTS_PITCH_CENTS / 1200.0)
    src = io.BytesIO(wav_bytes)
    with wave.open(src, "rb") as r:
        params = r.getparams(); frames = r.readframes(r.getnframes())
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(params.nchannels); w.setsampwidth(params.sampwidth)
        w.setframerate(max(8000, int(round(params.framerate * f)))); w.writeframes(frames)
    return out.getvalue()
def tts(text):
    """Return WAV bytes (or b'' if TTS disabled)."""
    global _piper
    if not text or TTS_BACKEND == "none":
        return b""
    if TTS_BACKEND == "xai":
        r = requests.post(f"{XAI_BASE_URL}/tts", headers={"Authorization": f"Bearer {xai_key()}"},
                          json={"text": text, "voice_id": XAI_TTS_VOICE, "language": LANGUAGE,
                                "output_format": {"codec": "wav", "sample_rate": 24000}}, timeout=60)
        r.raise_for_status()
        return r.content
    if _piper is None:
        from piper import PiperVoice
        _piper = PiperVoice.load(PIPER_VOICE)
    cfg = None
    scale = PIPER_LENGTH_SCALE
    if not scale:
        try:
            import gb_cadence
            scale = gb_cadence.current_scale() or 0
        except Exception:
            scale = 0
    if scale or PIPER_SPEAKER:
        from piper import SynthesisConfig
        cfg = SynthesisConfig(length_scale=scale or None,
                              speaker_id=int(PIPER_SPEAKER) if PIPER_SPEAKER else None)
    bio = io.BytesIO()
    with wave.open(bio, "wb") as w:
        _piper.synthesize_wav(text, w, syn_config=cfg)
    return _pitch_wav(bio.getvalue())

def split_sentences(text, min_len=12):
    """Split for spoken playback. Prefer a short first clause (<= ~12 words) so Piper starts quickly."""
    text = (text or "").strip()
    if not text: return []
    parts = [p.strip() for p in re.split(r"(?<=[.!?;:])\s+", text) if p.strip()]
    # peel a short opening clause off a long first sentence (comma / em-dash / "and" / "so")
    if parts:
        first = parts[0]
        m = re.match(r"^(.{12,90}?)(?:,\s+|\s+[—–-]\s+|\s+and\s+|\s+so\s+|\s+but\s+)(.+)$", first, re.I | re.S)
        if m and len(m.group(1).split()) <= 12 and len(m.group(2).split()) >= 3:
            parts = [m.group(1).rstrip(",;:") + ".", m.group(2)[0].upper() + m.group(2)[1:]] + parts[1:]
    out = []
    for p in parts:
        if out and len(out[-1]) < min_len: out[-1] += " " + p
        else: out.append(p)
    return out


PAGE = r"""<!doctype html><html><head><meta name=viewport content="width=device-width,initial-scale=1">
<title>__NAME__</title><style>
body{font-family:system-ui,sans-serif;max-width:640px;margin:auto;padding:12px;background:#111;color:#eee}
#log div{margin:6px 0;padding:8px 10px;border-radius:10px}.u{background:#234}.b{background:#333}
button,input{font-size:18px;padding:12px;border-radius:10px;border:0}input{width:70%}
#talk{width:100%;margin-top:10px;background:#1d6;color:#000;font-weight:bold}#talk.rec{background:#e33}
small{color:#999}</style></head><body><h2>__NAME__</h2><div id=log></div>
<form id=f><input id=t placeholder="Type a message..." autocomplete=off><button>Send</button></form>
<button id=talk>Hold to talk</button>
<label><input type=checkbox id=robot checked style="width:auto"> also speak on robot</label><br>
<small id=hint></small>
<script>
const tok=new URLSearchParams(location.search).get('token')||'';const H={'X-Token':tok};
const log=document.getElementById('log');function add(c,t){const d=document.createElement('div');d.className=c;d.textContent=t;log.appendChild(d);d.scrollIntoView()}
function handle(j){if(j.error){add('b','Error: '+j.error);return}if(j.user)add('u','You: '+j.user);add('b','__NAME__: '+j.reply);
 if(j.audio_b64&&!document.getElementById('robot').checked){new Audio('data:audio/wav;base64,'+j.audio_b64).play()}}
document.getElementById('f').onsubmit=async e=>{e.preventDefault();const t=document.getElementById('t');if(!t.value)return;add('u','You: '+t.value);
 const r=await fetch('api/chat',{method:'POST',headers:{...H,'Content-Type':'application/json'},body:JSON.stringify({text:t.value,robot:document.getElementById('robot').checked,dry_run:false})});
 t.value='';const j=await r.json();delete j.user;handle(j)};
const btn=document.getElementById('talk');let rec,chunks=[];
if(!(navigator.mediaDevices&&window.MediaRecorder)){btn.disabled=true;document.getElementById('hint').textContent='Mic needs HTTPS or localhost (see guide). Typing works.'}
async function start(e){e.preventDefault();if(rec&&rec.state=='recording')return;const s=await navigator.mediaDevices.getUserMedia({audio:true});
 rec=new MediaRecorder(s);chunks=[];rec.ondataavailable=e=>chunks.push(e.data);rec.onstop=async()=>{s.getTracks().forEach(t=>t.stop());
 const b=new Blob(chunks,{type:rec.mimeType});const fd=new FormData();fd.append('audio',b,'speech.'+(rec.mimeType.includes('mp4')?'mp4':'webm'));
 fd.append('robot',document.getElementById('robot').checked?'1':'0');add('b','...');const r=await fetch('api/voice',{method:'POST',headers:H,body:fd});
 log.lastChild.remove();handle(await r.json())};rec.start();btn.classList.add('rec');btn.textContent='Listening... release to send'}
function stop(e){e.preventDefault();if(rec&&rec.state=='recording'){rec.stop()}btn.classList.remove('rec');btn.textContent='Hold to talk'}
btn.addEventListener('pointerdown',start);btn.addEventListener('pointerup',stop);btn.addEventListener('pointerleave',stop);
</script></body></html>"""


# ============================================================================= LLM calls + usage logging
_http = requests.Session()          # keep-alive to xAI / ollama (reconnects on error)
def chat_call(backend, messages, max_tokens, category):
    if backend == "xai":
        url = f"{XAI_BASE_URL}/chat/completions"; model = XAI_CHAT_MODEL
        headers = {"Authorization": f"Bearer {xai_key()}", "x-grok-conv-id": "mentorpi-grokbot"}
        # Non-reasoning Grok models reject a reasoning field. Omit it.
        body = {"model": model, "messages": messages, "max_tokens": max_tokens, "temperature": 0.6}
    else:
        # 2026-11-25: native /api/chat to ALPHA-ONE Ollama; think off for short voice replies
        base = OLLAMA_BASE_URL.replace("/v1", "").rstrip("/")
        url = f"{base}/api/chat"; model = OLLAMA_MODEL; headers = {}
        body = {"model": model, "messages": messages, "stream": False, "think": False,
                "keep_alive": E("OLLAMA_KEEP_ALIVE", "30m"),
                "options": {"num_predict": max_tokens, "temperature": 0.6}}
        ollama_timeout = (float(E("OLLAMA_CONNECT_TIMEOUT", "2")), float(E("OLLAMA_TIMEOUT", "20")))
        r = _http.post(url, headers=headers, json=body, timeout=ollama_timeout)
        r.raise_for_status(); j = r.json()
        usage = {"prompt_tokens": j.get("prompt_eval_count") or 0,
                 "completion_tokens": j.get("eval_count") or 0}
        usage["total_tokens"] = usage["prompt_tokens"] + usage["completion_tokens"]
        tokens = store.log_usage("ollama", model, category, usage)
        text = (j.get("message") or {}).get("content") or ""
        text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
        return re.sub(r"[*_#`>]+", "", text).strip(), tokens, model
    r = _http.post(url, headers=headers, json=body, timeout=180)
    r.raise_for_status(); j = r.json()
    tokens = store.log_usage(backend, model, category, j.get("usage"))
    text = (j["choices"][0]["message"].get("content") or "")
    text = re.sub(r"<think>.*?</think>", "", text, flags=re.S)
    return re.sub(r"[*_#`>]+", "", text).strip(), tokens, model

def vision_call(jpeg, question, label_lines):
    b64 = base64.b64encode(jpeg).decode()
    prompt = VISION_PROMPT + ("\nKnown labels (name, map x,y, note):\n" + "\n".join(label_lines) if label_lines else "")
    body = {"model": XAI_VISION_MODEL, "max_output_tokens": VISION_MAX_TOKENS,
            "input": [{"role": "system", "content": prompt},
                      {"role": "user", "content": [
                          {"type": "input_image", "image_url": f"data:image/jpeg;base64,{b64}", "detail": "high"},
                          {"type": "input_text", "text": question}]}]}
    r = _http.post(f"{XAI_BASE_URL}/responses", json=body, timeout=120,
                   headers={"Authorization": f"Bearer {xai_key()}", "x-grok-conv-id": "mentorpi-grokbot-vision"})
    r.raise_for_status(); j = r.json()
    tokens = store.log_usage("xai", XAI_VISION_MODEL, "vision", j.get("usage"))
    text = j.get("output_text") or ""
    if not text:
        for item in j.get("output", []):
            for c in item.get("content", []) or []:
                if c.get("type") in ("output_text", "text"):
                    text += c.get("text", "")
    return re.sub(r"[*_#`>]+", "", text).strip(), tokens

def budget_left(category):
    share = CONV_SHARE if category == "conversation" else VISION_SHARE
    return int(DAILY_TOKEN_BUDGET * share) - store.xai_today(category)

def ollama_state():
    """'ready' | 'no_model' (server up, model still downloading/missing) | 'down'"""
    try:
        r = requests.get(OLLAMA_BASE_URL.replace("/v1", "") + "/api/tags", timeout=2)
        names = [m.get("name", "") for m in r.json().get("models", [])]
        base = OLLAMA_MODEL if ":" in OLLAMA_MODEL else OLLAMA_MODEL + ":latest"
        return "ready" if base in names else "no_model"
    except Exception:
        return "down"

def ollama_up():
    return ollama_state() == "ready"

SENSORS_FILE = os.path.join(os.path.dirname(gb_paths.shared_estop()), "sensors.json")   # navstd: written by gb_gate at 2 Hz
SENSORS_STALE_S = 3.0

def _cm(x, none="unknown"):
    return none if x is None else ("%d cm" % round(float(x) * 100) if float(x) < 9.5 else "clear")

def sensor_suite(path=None, now=None):
    """navstd: the gate's FULL fused sensor picture, or None if missing / stale (never guessed)."""
    try:
        d = json.load(open(path or SENSORS_FILE))
    except Exception:
        return None
    if (now or time.time()) - float(d.get("t") or 0) > SENSORS_STALE_S:
        return None
    return d

def sensor_suite_note(d):
    """navstd: compact, factual telemetry line for the AI context / status answer (LiDAR sectors + sides + dropouts +
    memories, depth + depth memory, IMU tilt/gyro, RGB advisory, faults, ESTOP)."""
    if not d:
        return "Live sensors: no fresh gate telemetry (do not guess distances)."
    S = d.get("sensors") or {}; L = S.get("lidar") or {}; D = S.get("depth") or {}; I = S.get("imu") or {}
    R = S.get("rgb") or {}; fr = S.get("fresh") or {}; F = d.get("faults") or {}
    stale = [k for k in ("lidar", "depth", "imu", "odom") if not fr.get(k)]
    parts = ["Live sensors (gate, fused): front %s via %s, rear %s, turn room %s" % (
                 _cm(d.get("front")), S.get("front_src") or "?", _cm(d.get("rear")), _cm(d.get("sweep"))),
             "LiDAR sides: left gap %s, right gap %s; dropouts ahead %d%%, behind %d%%, %s near rays filled in" % (
                 _cm(L.get("left_gap"), "nothing within 60 cm"), _cm(L.get("right_gap"), "nothing within 60 cm"), round(100 * float(L.get("blind_f") or 0)),
                 round(100 * float(L.get("blind_r") or 0)), L.get("imputed", 0))]
    if L.get("clearest_deg") is not None:
        parts.append("most open direction %d deg (%s)" % (L["clearest_deg"], _cm(L.get("clearest_m"))))
    parts.append("depth camera ahead %s (valid %s), low-obstacle memory %s, LiDAR memory ahead %s / behind %s" % (
        _cm(D.get("fwd"), "nothing seen"), D.get("valid"), _cm(D.get("mem"), "none"), _cm(L.get("mem_fwd"), "none"),
        _cm(L.get("mem_rev"), "none")))
    if I.get("tilt_deg") is not None:
        parts.append("IMU tilt %.0f deg" % float(I["tilt_deg"]))
    if R.get("hazard"):
        parts.append("camera sees %s in the path" % (R.get("label") or "something"))
    if F.get("recent"):
        parts.append("%s collision faults in the last %ss (ESTOP at %s)" % (F.get("recent"), int(F.get("win_s") or 0), F.get("limit")))
    if stale:
        parts.append("stale: " + ", ".join(stale))
    if d.get("estop"):
        parts.append("EMERGENCY STOP is latched")
    return "; ".join(parts) + "."

def status_with_sensors():
    rep = gb_features.status_report(say_battery())
    sn = sensor_suite()
    return (rep + " " + sensor_suite_note(sn)) if sn else rep

def context_note(user_text):
    facts = store.search_facts(user_text, FACTS_TOP_K)
    summ = store.get_summary()
    parts = []
    if re.search(r"\b(you|your|wally|robot|see|around|near|obstacle|wall|stuck|bump|sensor|lidar|camera|front|behind|left|right|clear|space|room)\b",
                 user_text or "", re.I):
        parts.append(sensor_suite_note(sensor_suite()))   # navstd: the AI reasons from the full fused sensor suite
    if summ: parts.append("Conversation summary: " + summ)
    if facts: parts.append("Relevant facts: " + " | ".join(facts))
    return "\n".join(parts)

OLLAMA_FALLBACK = E("OLLAMA_FALLBACK", "0") == "1"    # with LLM_BACKEND=xai: use local qwen if Grok is unreachable

def converse(user_text, smalltalk=False):
    """returns (reply, tokens, category, backend)"""
    category = "smalltalk" if smalltalk else "conversation"
    if LLM_BACKEND == "ollama":
        # 2026-11-25: local Ollama only for chat; do not fall back to xAI / cloud tokens
        backends = ["ollama"]
        if not ollama_up():
            return "I can't reach my local brain right now.", 0, "local", "none"
    elif LLM_BACKEND == "xai":
        backends = ["xai"] + (["ollama"] if OLLAMA_FALLBACK else [])
        if not has_key():
            return "I don't have my Grok key yet, so I can only do built-in commands.", 0, "local", "none"
        if budget_left("conversation") < CONV_MAX_TOKENS + 900:
            if not OLLAMA_FALLBACK:
                return "I've used today's Grok conversation budget. Built-in commands still work.", 0, "local", "none"
            backends = ["ollama"]
    else:                                                  # auto / ollama: previous local-first behaviour
        want_xai = LLM_BACKEND == "auto" and has_key() and not smalltalk and \
                   budget_left("conversation") >= CONV_MAX_TOKENS + 900
        backends = ["xai"] if want_xai else ["ollama"]
        if backends == ["ollama"] and not ollama_up():
            if has_key() and budget_left("conversation") > 1000:
                backends = ["xai"]
            else:
                return ("I'm still setting up my local brain, it's downloading. Until then I can do built-in commands "
                        "like battery, time, remember and recall."), 0, "local", "none"
    note = context_note(user_text)
    for backend in backends:
        if backend == "ollama" and not ollama_up():
            continue
        msgs = [{"role": "system", "content": SYSTEM_PROMPT}]
        msgs += store.last_turns((CONV_TURNS if backend == "xai" else OLLAMA_TURNS) * 2)
        msgs.append({"role": "user", "content": (f"[context]\n{note}\n[/context]\n" if note else "") + user_text})
        try:
            reply, tokens, model = chat_call(backend, msgs, CONV_MAX_TOKENS if backend == "xai" else OLLAMA_MAX_TOKENS,
                                             category)
            return reply or "Hmm, I drew a blank there.", tokens, category, backend
        except Exception as ex:
            body = getattr(getattr(ex, "response", None), "text", "") or ""
            log(f"{backend} chat failed: {type(ex).__name__}: {str(ex)[:160]} {body[:160]}")
            if backend == "xai" and ("credits" in body or "spending limit" in body):
                last_err = "credits"
    if "xai" in backends:
        if locals().get("last_err") == "credits":
            return "I can't reach Grok right now, the xAI account is out of credits.", 0, "local", "none"
        return "I can't reach Grok right now.", 0, "local", "none"
    return "My local brain isn't answering right now.", 0, "local", "none"

def maybe_summarize():
    """rolling summary: fold turns older than the last CONV_TURNS turns into one short summary (local model preferred)"""
    if os.environ.get("SUMMARIES", "1") == "0":
        return
    old = store.unsummarized_old(keep=CONV_TURNS * 2)
    if len(old) < 8:
        return
    convo = "\n".join(f"{r['role']}: {r['text']}" for r in old)[-6000:]
    prev = store.get_summary()
    msgs = [{"role": "system", "content": "Update the running summary of a conversation between John and his robot. "
             "Keep durable details (names, preferences, plans, open questions). Max 80 words, plain text."},
            {"role": "user", "content": f"Previous summary: {prev or '(none)'}\n\nNew turns:\n{convo}"}]
    try:
        if ollama_up():
            text, _, _ = chat_call("ollama", msgs, 160, "summary")
        elif has_key() and budget_left("conversation") > 2000:
            text, _, _ = chat_call("xai", msgs, 160, "summary")
        else:
            text = (prev + " " + " ".join(r["text"] for r in old if r["role"] == "user"))[-500:]
        store.set_summary(text.strip(), old[-1]["id"])
    except Exception as ex:
        log("summary failed:", ex)

# ============================================================================= local intents
NUM_WORDS = {"one": 1, "two": 2, "three": 3, "a": 1, "half a": 0.5}

def fmt_usage(d):
    tot = sum(v[0] for v in d.values())
    if not tot:
        return "none"
    parts = [f"{v[0]:,} {k}" for k, v in sorted(d.items(), key=lambda kv: -kv[1][0])]
    return f"{tot:,} tokens ({', '.join(parts)})"

def usage_report():
    today = time.strftime("%Y-%m-%d"); week = time.time() - 7 * 86400
    xs = [("today", store.usage_sum(day=today, backend="xai")), ("this week", store.usage_sum(since_ts=week, backend="xai")),
          ("in total", store.usage_sum(backend="xai"))]
    loc = store.usage_sum(day=today, backend="ollama")
    s = "This is my own robot API usage, not your Grok Bot app plan. Cloud Grok: " + \
        "; ".join(f"{n} {fmt_usage(d)}" for n, d in xs) + ". "
    s += (f"Budget left today: {max(0, budget_left('conversation')):,} conversation and "
          f"{max(0, budget_left('vision')):,} vision tokens. ")
    s += f"Local model today: {fmt_usage(loc)}, which is free."
    return s

def say_battery():
    b = gb_robot.read_battery()
    if not b:
        return "I couldn't read the battery right now."
    v = b.get("v_avg") or b["v_raw"]
    s = f"The battery is at {v:.2f} volts, about {b['pct']} percent."
    if b.get("charging_hint"):
        s += " It looks like it's charging, so the reading runs a bit high."
    if v < 6.9:
        s += " That's low. Please plug in the charger."
    return s

def do_label(name):
    blk = gb_caps.vision_block()
    if blk:
        return blk
    try:
        jpeg = gb_robot.snapshot()
    except Exception as ex:
        jpeg = None; log("snapshot failed:", ex)
    pose = None
    try:
        pose = gb_robot.get_pose()
    except Exception as ex:
        log("pose failed:", ex)
    if pose: pose["map_id"] = store.current_map_id() or pose["frame"]
    emb, snap = None, None
    if jpeg:
        snap = os.path.join(DATA, "snapshots", f"{time.strftime('%Y%m%d_%H%M%S')}_{re.sub(r'[^a-z0-9]+', '_', name.lower())}.jpg")
        atomic_write(snap, jpeg)
        try:
            import gb_vision; emb = gb_vision.embed(jpeg)
        except Exception as ex:
            log("embed failed:", ex)
    store.add_label(name, emb, pose, snap)
    where = f" at x {pose['x']:.1f}, y {pose['y']:.1f} in the {pose['frame']} frame" if pose else ""
    seen = "with a camera fingerprint" if emb is not None else "without a picture (camera unavailable)"
    return (f"Got it, this is the {name}. I saved it {seen}{where}. I can match it again later by look and place, "
            f"but I can't do full object recognition on my own.")

def label_matches(jpeg, pose):
    import gb_vision
    q = gb_vision.embed(jpeg); out = []
    for r in store.labels():
        if r["embedding"] is None:
            continue
        sim = float(q @ gb_vision.from_blob(r["embedding"]))
        dist = None
        if pose and r["map_x"] is not None and (r["frame"] == pose["frame"]):
            dist = math.hypot(r["map_x"] - pose["x"], r["map_y"] - pose["y"])
        out.append((sim, dist, r))
    out.sort(key=lambda t: -t[0]); return out

def label_line(r):
    xy = f"{r['map_x']:.2f},{r['map_y']:.2f}" if r["map_x"] is not None else "?,?"
    return f"{r['name']}\t{xy}\t{r['frame'] or ''} seen{r['count']}"

def do_vision(question, detailed):
    blk = gb_caps.vision_block()
    if blk:
        return blk, 0, "local", "local"
    try:
        jpeg = gb_robot.snapshot()
    except Exception as ex:
        return f"I can't get a camera picture right now: {ex}", 0, "local", "none"
    pose = None
    try: pose = gb_robot.get_pose(timeout=2)
    except Exception: pass
    matches = []
    try: matches = label_matches(jpeg, pose)
    except Exception as ex: log("label match failed:", ex)
    if matches and not detailed:
        sim, dist, r = matches[0]
        if sim >= VISION_LOCAL_SIM or (sim >= VISION_NEAR_SIM and dist is not None and dist <= VISION_NEAR_M):
            store.touch_label(r["id"])
            how = f"It looks {int(sim * 100)} percent like what you showed me" + (
                f", and we're {dist:.1f} meters from where you named it" if dist is not None else "")
            return f"That's the {r['name']}, I think. {how}.", 0, "local", "local"
    if not has_key():
        guess = f" My best guess from my own memory is the {matches[0][2]['name']}, but I'm not sure." if matches else ""
        return "I don't recognize it from my saved labels, and describing new things needs the xAI key." + guess, 0, "local", "none"
    if budget_left("vision") < VISION_MAX_TOKENS + 2500:
        return "I've used today's vision budget, so I can't look closely until tomorrow.", 0, "local", "none"
    import gb_vision
    lines = [label_line(m[2]) for m in matches[:3] if m[0] > 0.4]
    try:
        text, tokens = vision_call(gb_vision.jpeg_for_api(jpeg), question, lines)
    except Exception as ex:
        body = getattr(getattr(ex, "response", None), "text", "") or ""
        log(f"vision failed: {type(ex).__name__}: {str(ex)[:160]} {body[:160]}")
        why = ", the xAI account is out of credits" if ("credits" in body or "spending limit" in body) else ""
        return f"I can't reach Grok right now{why}.", 0, "local", "none"
    return text or "I looked but couldn't make it out.", tokens, "vision", "xai"

PENDING = {"forget_all": 0.0}

# ============================================================================= Hiwonder stock features (gb_features)
def _explore():
    import gb_explore
    return gb_explore

def _announce(dry):
    def f(text):
        log("[mission] " + text)
        try: append_transcript({"ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "source": "mission", "user": "",
                                "robot": text, "tokens": 0, "category": "mission", "backend": "local", "secs": 0})
        except Exception: pass
        if not dry or E("EXPLORE_SPEAK_DRY", "0") == "1":
            speak(text)
    return f


# ------------------------------------------------------------------ 2026-11-26 voicefix: wake-to-mission handoff helpers
WAKE_STATE = {"until": 0.0, "followup": False, "session": False, "last_mode": None}
PENDING_CMD = {"item": None}


def bare_wake_prompt():
    """Spoken reply after a bare wake. Keeps John's WAKE_ACK and clearly asks for the next command."""
    ack = (WAKE_ACK or "").strip().strip('"')
    if re.search(r"what (would you like|should i|can i do)|i'?m listening", ack, re.I):
        return ack
    if not ack:
        return "I'm listening. What should I do?"
    return ack + " I'm listening. What should I do?"


def mark_wake_window(until, followup=None):
    WAKE_STATE["until"] = float(until or 0.0)
    if followup is not None:
        WAKE_STATE["followup"] = bool(followup)
    return until



def floor_operating_mode():
    """Current bench/ground operating mode from gb_floor (data/learn/mode.json)."""
    try:
        import gb_floor
        return gb_floor.mode()
    except Exception:
        return "unknown"


def bench_mode_entered(previous_mode, current_mode):
    """True when operating mode changes into bench (plug-in / return to the stand).

    A first observation of bench (previous unset/unknown) is not a fresh plug-in, so a
    session opened while already in bench (bench testing) stays awake until a later
    re-entry, or until the process exits on power-off.
    """
    prev = "" if previous_mode is None else str(previous_mode).strip().lower()
    cur = "" if current_mode is None else str(current_mode).strip().lower()
    return cur == "bench" and prev not in ("", "bench", "unknown")


def session_awake():
    """True once a wake phrase has opened a stay-awake session that has not been ended."""
    return bool(WAKE_STATE.get("session"))


def reset_awake_state():
    """Test helper: clear the stay-awake session and wake window (boot-equivalent)."""
    WAKE_STATE["until"] = 0.0
    WAKE_STATE["followup"] = False
    WAKE_STATE["session"] = False
    WAKE_STATE["last_mode"] = None
    return WAKE_STATE


def enter_awake_session(current_mode=None):
    """Open a stay-awake session after a successful wake. Lasts until bench entry or process exit."""
    if current_mode is None:
        current_mode = floor_operating_mode()
    WAKE_STATE["session"] = True
    if WAKE_STATE.get("last_mode") is None:
        WAKE_STATE["last_mode"] = current_mode
    return True


def end_awake_session(reason=""):
    """Return to sleep (needs a fresh wake word). Used for bench-mode entry."""
    WAKE_STATE["session"] = False
    mark_wake_window(0.0, followup=False)
    return reason or "ended"


def apply_bench_sleep_if_needed(current_mode=None, previous_mode=None):
    """End the stay-awake session on a transition into bench mode. Returns True if we went to sleep."""
    cur = floor_operating_mode() if current_mode is None else current_mode
    prev = WAKE_STATE.get("last_mode") if previous_mode is None else previous_mode
    should = bool(session_awake() and bench_mode_entered(prev, cur))
    WAKE_STATE["last_mode"] = cur
    if should:
        end_awake_session("bench")
        return True
    return False


def stay_listening(session=None, mission_running=False, awake_until=0.0, now=None):
    """True when commands should be accepted without a fresh wake word.

    A stay-awake session (or an active mission) ignores follow-up / idle expiry.
    Idle time after a mission ends does not put him back to sleep.
    """
    if session is None:
        session = session_awake()
    if session or mission_running:
        return True
    now = time.time() if now is None else float(now)
    try:
        return bool(awake_until and now < float(awake_until))
    except (TypeError, ValueError):
        return False


def _wake_window_open(now=None):
    now = time.time() if now is None else float(now)
    try:
        if stay_listening(now=now, mission_running=mission_followup_open(),
                          awake_until=WAKE_STATE.get("until") or 0.0):
            return True
    except Exception:
        pass
    return bool(WAKE_STATE.get("until") and now < float(WAKE_STATE["until"]))


def normalize_explore_utterance(text):
    """Apply explore near-miss normalization when gb_explore is available."""
    try:
        return _explore().normalize_explore_command(text)
    except Exception:
        return text or ""


def extract_wake_command(text):
    """Split a transcript into (wake_matched, remainder)."""
    try:
        matched, rest = match_wake(text)
        if matched:
            return True, rest or ""
    except Exception:
        pass
    return False, text or ""


def looks_like_explore_command(text):
    t = normalize_explore_utterance(text)
    try:
        plan = _explore().parse_plan(t)
    except Exception:
        plan = None
    return plan if (plan and plan.get("mission")) else None


def consider_handoff_command(text, spoken_texts=(), awake=False):
    """Decide whether a transcript should be dispatched as a post-wake / over-TTS command.

    Returns ('wake_only', ''), ('mission', plan), ('command', cmd), or None if it should be ignored.
    Never starts motion itself. Rejects speaker-echo of recent TTS.
    """
    if not (text or "").strip():
        return None
    try:
        if gb_estop.speaker_echo(text, spoken_texts or _recent_lines()) or gb_estop.announcement_echo(text, spoken_texts or _recent_lines()):
            return None
    except Exception:
        pass
    matched, rest = extract_wake_command(text)
    cmd = (rest if matched else (text if awake or _wake_window_open() or recently_prompted_for_command(spoken_texts) else "")) or ""
    cmd = cmd.strip()
    if matched and len(cmd) < 2:
        return ("wake_only", "")
    if not cmd:
        return None
    plan = looks_like_explore_command(cmd) or (looks_like_explore_command(text) if matched else None)
    if plan:
        return ("mission", plan)
    # A non-echo command in the wake window (or a wake+command utterance) should be dispatched.
    if matched or awake or _wake_window_open() or recently_prompted_for_command(spoken_texts):
        if _norm_txt(cmd) in FOLLOWUP_FILLERS:
            return None
        return ("command", cmd)
    return None


def recently_prompted_for_command(spoken_texts=None):
    blob = " ".join(x if isinstance(x, str) else (x[1] if isinstance(x, (tuple, list)) and len(x) > 1 else str(x))
                    for x in (spoken_texts if spoken_texts is not None else _recent_lines()))
    return bool(re.search(r"i'?m listening|what should i do|what would you like|ready to rock", blob, re.I))


def queue_pending_command(text, src="", awake=False, spoken_texts=None):
    """Queue a non-echo command heard while TTS is playing. Returns True if queued."""
    decision = consider_handoff_command(text, spoken_texts=spoken_texts if spoken_texts is not None else (PLAY_STATE.get("recent") or _recent_lines()), awake=awake)
    if not decision or decision[0] != "mission":  # only queue explicit missions during playback (prevents echo-chatter loops)
        return False
    PENDING_CMD["item"] = {"kind": decision[0], "payload": decision[1], "text": text, "src": src, "t": time.time()}
    return True


def take_pending_command(max_age=30.0):
    item = PENDING_CMD.get("item")
    PENDING_CMD["item"] = None
    if not item:
        return None
    if time.time() - float(item.get("t") or 0) > max_age:
        return None
    return item


def _dispatch_pending_command(item):
    """Run a queued post-wake command on the main loop. Does not itself arm or move except via answer()/start_mission."""
    if not item:
        return None
    log(f"[handoff] dispatching queued {item.get('kind')} from {item.get('src')}: {item.get('text')!r}")
    kind, payload = item.get("kind"), item.get("payload")
    try:
        import gb_arm
        gb_arm.maybe_arm(item.get("text") or "", "voice")
    except Exception as ex:
        log("pending maybe_arm failed:", ex)
    import gb_wakegate as WG
    WG.set_context(True, item.get("text") or "")
    try:
        if kind == "mission" and payload:
            reply = start_mission(payload)
        else:
            gb_features.set_dry(False)
            reply, _ = answer(item.get("text") if not payload else payload, "voice")
    finally:
        WG.clear_context()
    if reply:
        speak(reply)
    return reply


def start_mission(plan):
    import gb_wakegate as WG                      # 2026-11-24 wakegate: no motion without the wake phrase
    _wg = WG.block("mission", plan)
    if _wg: return _wg
    if gb_estop.is_set():
        return "I'm emergency-latched, so I won't start a mission. Say resume first."
    dry = gb_features.is_dry()
    log(f"[mission] plan {json.dumps(plan)} dry={dry}")
    return _explore().request(plan, dry=dry, store=store, announce=_announce(dry))

# ============================================================================= kill switch (gb_estop)
def _moving():
    try:
        if gb_estop.recently_moving() or _explore().running(): return True
    except Exception: pass
    try:
        if gb_features.DRIVE.get("busy") or gb_features.ACTIVE: return True
    except Exception: pass
    return False

def mission_followup_open():
    """True while an explore/motion mission is running: keep the follow-up window from closing."""
    try:
        if _explore().running():
            return True
    except Exception:
        pass
    return False

def tracks_are_moving(sensors=None, moving=None, sensors_path=None):
    """Motion state for VAD / wake thresholds: explicit flag, live mission, or sensors.json / gate status."""
    if moving is not None:
        return bool(moving)
    try:
        if _moving():
            return True
    except Exception:
        pass
    if sensors is None:
        path = sensors_path or SENSORS_FILE
        try:
            with open(path) as f:
                sensors = json.load(f)
        except Exception:
            sensors = None
    if not isinstance(sensors, dict):
        return False
    if sensors.get("motion_guards"):
        return True
    if str(sensors.get("mode") or "") in ("timed", "turn", "stream", "bounded"):
        return True
    try:
        if abs(float(sensors.get("lin") or 0)) > 1e-4 or abs(float(sensors.get("ang") or 0)) > 1e-4:
            return True
    except (TypeError, ValueError):
        pass
    odom = (sensors.get("sensors") or {}).get("odom") or {}
    try:
        if abs(float(odom.get("v") or 0)) > 1e-4 or abs(float(odom.get("w") or 0)) > 1e-4:
            return True
    except (TypeError, ValueError):
        pass
    return False

def cap_noise_floor(noise, cap=None):
    cap = VAD_NOISE_CAP if cap is None else cap
    try:
        n = float(noise)
    except (TypeError, ValueError):
        n = float(VAD_MIN_RMS)
    return min(max(50.0, n), float(cap))

def vad_start_ratio(moving=None, sensors=None):
    if tracks_are_moving(sensors=sensors, moving=moving):
        return VAD_START_RATIO_MOVING
    return VAD_START_RATIO

def speech_trigger_threshold(noise, moving=None, sensors=None):
    """RMS onset threshold. While moving the ratio is lowered and the noise floor is capped
    so track noise cannot push the speech trigger out of reach."""
    return max(VAD_MIN_RMS, cap_noise_floor(noise) * vad_start_ratio(moving=moving, sensors=sensors))

def reset_listener_noise(was_moving, moving, base=300.0):
    """When motion starts, drop a stale/inflated idle noise floor so barge-in can hear 'stop'."""
    if moving and not was_moving:
        return float(base)
    return None

def followup_still_open(awake_until, now=None, mission_running=False):
    """The 20 s FOLLOWUP_S window stays open for the whole time a mission is running."""
    now = time.time() if now is None else now
    if mission_running:
        return True
    try:
        return bool(awake_until and now < float(awake_until))
    except (TypeError, ValueError):
        return False

def is_mission_free_command(text):
    """Commands honored during an active mission without a fresh wake phrase."""
    t = _norm_txt(text)
    return bool(re.search(
        r"\b(halt|freeze|abort|cancel|disarm|whoa|woah|return|come back|come here|go home|head home|go back)\b", t))

def stt_is_hallucination(text):
    return _norm_txt(text) in HALLUCINATIONS

def _stop_track():
    import gb_perceive; gb_perceive.stop_track()

def _nav2_stop():
    ex = _explore()
    if ex.nav2_running(): ex.nav2_stop()

ESTOP_ACTIONS = {"explore": lambda: _explore().abort("ESTOP"), "features": lambda: gb_features.emergency_stop(),
                 "track": _stop_track, "nav2": _nav2_stop}

def do_estop(why="", src="voice"):
    """Persistent emergency lock. Used for a deliberate 'emergency stop', not for a plain halt."""
    try:
        import gb_wally
        gb_wally.drop_goals("estop")
    except Exception as ex:
        log("drop goals failed:", ex)
    floor = gb_estop.floor_mode()                    # 2026-11-25 noestop: floor -> normal stop, nothing latched
    gb_estop.estop(src, why, actions=ESTOP_ACTIONS)
    if floor:
        return "Stopped. Motors are off and I'm disarmed. Say wake up Wally when you want to drive again."
    return "Emergency stop. Motors are latched off. Say unlock or resume when it's safe. I will not continue the old trip."

def do_halt(why=""):
    """Cancel the current motion without writing an ESTOP file. Does not clear a fault or a clap stop."""
    try:
        import gb_wally
        gb_wally.drop_goals("halt")
    except Exception as ex:
        log("drop goals failed:", ex)
    try:
        _explore().abort("halt")
    except Exception as ex:
        log("explore abort failed:", ex)
    try:
        gb_features.emergency_stop()
    except Exception as ex:
        log("feature halt failed:", ex)
    try:
        gb_estop.gate_request({"op": "stop", "src": "halt"})
    except Exception as ex:
        log("gate halt failed:", ex)
    try:
        gb_estop.zero_now("halt")
    except Exception as ex:
        log("zero halt failed:", ex)
    return ""   # 2026-11-17: plain stop speaks nothing (John)

def _recent_lines():
    return [x for _t, x in PLAY_STATE.get("recent", [])]


def respond_stop(text, why, moving=False):
    action = gb_estop.stop_action(text, moving=moving, spoken_texts=_recent_lines())
    if action in ("latch", "halt"):                   # 2026-11-25 arm: a real stop disarms
        import gb_arm; gb_arm.disarm("stop phrase: " + str(text)[:60], actor="voice")
    if action == "latch":
        return do_estop(why)
    if action == "halt":
        return do_halt(why)
    return None


def consider_stop(text, why, moving=False):
    """Run a real stop. Do not speak again for our own announcement or a second time inside 8 s."""
    spoken = _recent_lines()
    if gb_estop.announcement_echo(text, spoken) or gb_estop.future_stop(text):
        log(f"ignored own announcement or a later stop condition: {text!r}")
        return None
    reply = respond_stop(text, why, moving=moving)
    if not reply:
        return None
    if not gb_estop.claim_stop_speech():
        return ""
    return reply

def _floor_release(src):
    """2026-11-25 noestop: on the floor no ESTOP/MAINT file may block John. Archive (never delete) leftovers, cause logged."""
    if not gb_estop.floor_mode():
        return False
    try:
        import gb_arm
        return bool(gb_arm._floor_release(src))
    except Exception as ex:
        log("floor release failed:", ex); return False

def do_resume():
    """Unlock clears a normal stop only. It does not resume a goal, a follow, or a mission.
    2026-11-25 noestop: on the floor nothing is latched; leftovers are archived and unlock always succeeds."""
    if gb_estop.floor_mode():
        _floor_release("voice unlock")
        try:
            import gb_wally
            gb_wally.drop_goals("unlock")
        except Exception as ex:
            log("drop goals failed:", ex)
        return "Nothing is locked. Say wake up Wally when you want to drive."
    try:
        import gb_maint
        if gb_maint.is_set():
            return "The maintenance lock is still on. Resume does not clear it, and I am not moving."
    except Exception as ex:
        log("maintenance check failed:", ex)
        return "I couldn't check the maintenance lock, so I'm not moving."
    try:
        import gb_wally
        gb_wally.drop_goals("unlock")
    except Exception as ex:
        log("drop goals failed:", ex)
    if gb_estop.clear("voice", only_normal=True):
        return "Brake off. Ready to drive."
    if gb_estop.is_set():
        return "That stop is a safety fault. A drive command will not clear it."
    return "I wasn't latched. I'm not moving."

def do_timed(p):
    import gb_wakegate as WG                      # 2026-11-24 wakegate
    _wg = WG.block("timed", p)
    if _wg: return _wg
    """Timed drive / turn: ONE new command. A normal stop can unlock. A safety fault cannot."""
    import gb_unlock
    prov = gb_unlock.fresh_provenance(p.get("requested") or "", source="voice")
    _floor_release("drive command")                   # 2026-11-25 noestop: floor latch leftovers never block
    latch = gb_estop.info() if gb_estop.is_set() else None
    try:
        import gb_maint
        maint = gb_maint.is_set()
    except Exception:
        maint = True
    gates = {"maint": maint, "allow_motion": bool(ALLOW_MOTION), "autonomous": False}
    if latch and latch.get("kind") == "safety":
        gates["physical"] = latch.get("src") in ("joystick", "physical", "hardware")
        if latch.get("src") in ("gate-tilt", "tilt"):
            gates["tip_hard"] = True
        if latch.get("src") == "battery":
            gates["battery_shutdown"] = True
        if latch.get("src") == "watchdog":
            gates["watchdog"] = True
    decision = gb_unlock.decide(p.get("requested") or "", prov, latch, gates, motion=p)
    if not decision["allow"]:
        return decision["say"]
    if decision["unlock"]:
        if not gb_estop.clear("move-intent", only_normal=True):
            return "That stop is a safety fault. A drive command will not clear it."
        try:
            import gb_wally
            gb_wally.drop_goals("move-intent")
        except Exception as ex:
            log("drop goals failed:", ex)
    if gb_estop.is_set():
        return "I'm emergency-latched, so I won't move. Say resume first."
    if not ALLOW_MOTION:
        return "Driving by voice is switched off. Set ALLOW_MOTION to 1 in my settings to allow it."
    blk = gb_caps.motion_block("drive")
    if blk: return blk
    desc = gb_estop.describe(p)
    if gb_features.is_dry():
        log(f"[timed] DRY {p}"); return f"Dry run: I would {desc}. No command was sent."
    try:
        rid = gb_estop.gate_request(dict(p, src="voice"))
        log(f"[timed] request id={rid} {p}")
        cap = (p.get("dur") or 3.0) if p["op"] == "timed" else gb_estop.TIMED_CAP
        # belt and braces: if the gate somehow kept this id alive, cancel it just after its deadline
        threading.Timer(min(gb_estop.TIMED_CAP, cap) + 0.7,
                        lambda: gb_estop.gate_request({"op": "stop", "only_id": rid, "src": "timed-backup"})).start()
    except Exception as ex:
        log("[timed] gate request failed:", ex)
        try: gb_estop.zero_now("timed request failed")
        finally: pass
        return f"I couldn't reach my motion gate, so I'm not moving ({str(ex)[:60]})."
    return f"I sent the gate a request to {desc}. I have not seen the wheels move."

def estop_intent(text):
    """Runs FIRST for every voice and chat command. Returns a reply or None."""
    hit = respond_stop(text, "command: " + text[:120], moving=_moving())
    if hit is not None:
        return hit
    if gb_estop.resume_phrase(text):
        return do_resume()
    t = gb_estop._norm(text)
    m = re.search(r"\bclap (?:sensitivity|detector|detection)\b.*?\b(higher|up|more|increase|raise|lower|down|less|decrease|reduce|"
                  r"low|medium|med|high|max|maximum)\b", t) or re.search(r"\b(higher|lower|more|less) clap sensitivity\b", t)
    if m:
        w = m.group(1)
        if w in ("higher", "up", "more", "increase", "raise"): lv = gb_estop.clap_step(+1)
        elif w in ("lower", "down", "less", "decrease", "reduce"): lv = gb_estop.clap_step(-1)
        else: lv = {"medium": "med", "maximum": "max"}.get(w, w); gb_estop.clap_set(sensitivity=lv)
        return f"Clap sensitivity is now {lv}."
    if re.search(r"\b(test (?:the )?clap|clap test|test (?:the )?double clap)\b", t):
        gb_estop.clap_set(test_until=time.time() + 15)
        return "Clap test is on for 15 seconds. Clap twice and I'll tell you if I heard it. I won't latch the brake during the test."
    if re.search(r"\b(is|are) (?:the )?(?:e ?stop|emergency stop|brake) (?:on|set|latched|active)\b|\bam i latched\b", t):
        if gb_estop.floor_mode():
            return "No. On the floor I never latch an emergency stop. A stop just halts me and disarms me."
        return ("Yes, the emergency brake is latched (%s). Say resume to release it." % (gb_estop.info() or {}).get("src", "file")
                if gb_estop.is_set() else "No, the emergency brake is not latched.")
    try:
        k = gb_caps.speed_mult()
    except Exception:
        k = 1.0
    pm = gb_estop.parse_motion(text, k)
    if pm:
        return do_timed(pm)
    return None

def do_stop():
    import gb_arm; gb_arm.disarm("stop intent", actor="voice")      # 2026-11-25 arm
    return do_halt("stop intent")

def _old_do_stop():
    try: aborted = _explore().abort("stop command")
    except Exception as ex: aborted = False; log("explore abort failed:", ex)
    try:
        started = gb_features.emergency_stop()
    except Exception as ex:
        try: gb_robot.stop()
        except Exception: pass
        return f"I sent stop, but something failed: {ex}"
    return "Stopping." + (f" I ended {', '.join(gb_features.NAMES[n] for n in started)}." if started else "")

def map_start():
    import gb_mapping
    if not gb_caps.get()["lidar"]:
        return gb_caps.MSG_LIDAR_OFF
    if not gb_features.lidar_alive():
        return ("I can't start mapping, it needs the LiDAR and the LiDAR isn't publishing scans right now. "
                "Mapping and navigation will work once the LiDAR is fixed.")
    return gb_mapping.start()[1]

_COLOR_RX = r"(red|green|blue|black|white|dark green|yellow|orange|purple|pink|grey|gray)"
_LIGHT_RX = r"(red|green|blue|white|yellow|purple|orange|pink|cyan)"

def feature_intent(t, text):
    """Fast local intents for Hiwonder features. Returns (reply, category) or None."""
    if re.search(r"\b(lights?|leds?|lamps?)\b", t) and re.search(r"\b(turn|switch|set|make|change|lights? (on|off|to)|leds? (on|off|to))\b", t):
        if re.search(r"\b(off|out)\b", t):
            return gb_features.set_lights("off")[1], "local"
        m = re.search(r"\b" + _LIGHT_RX + r"\b", t)
        if m:
            return gb_features.set_lights(m.group(1))[1], "local"
        if re.search(r"\bon\b", t):
            return gb_features.set_lights("white")[1], "local"
    m = re.search(r"\b(beep|honk|buzz)\b(?:.*?\b(\d|one|two|three|four|five|twice|thrice)\b)?", t)
    if m:
        n = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "twice": 2, "thrice": 3}.get(m.group(2), m.group(2) or 1)
        return gb_features.beep(int(n))[1], "local"
    if re.search(r"\b(status|system check|health check|diagnostics|status report)\b", t):
        return status_with_sensors(), "local"         # navstd: + fused sensor suite
    if re.search(r"\bline\b", t) and re.search(r"\b(follow|following|track|tracking|patrol|go along|drive along|ride along|along the)\b", t):
        m = re.search(_COLOR_RX, t)
        return gb_features.start_feature("line", m.group(1) if m else "black")[1], "local"
    if re.search(r"\b(track|tracking|follow|chase)\b", t) and re.search(_COLOR_RX, t) and not re.search(r"\bline\b", t):
        return gb_features.start_feature("color", re.search(_COLOR_RX, t).group(1))[1], "local"
    if re.search(r"\b(color|colour) track", t):
        return gb_features.start_feature("color", None)[1], "local"
    if re.search(r"\bgesture", t) and not re.search(r"\b(stop|end|exit|quit)\b", t):
        return gb_features.start_feature("gesture")[1], "local"
    if re.search(r"\bobstacle avoid|\bavoid(ing)? obstacles\b", t):
        return gb_features.start_feature("lidar", mode="avoid")[1], "local"
    if re.search(r"\blidar follow|\bfollow me\b|\bdrive toward me\b|\bcome to me\b", t):
        return None
    if re.search(r"\b(guard mode|lidar guard|guarding|stand guard)\b", t):
        return gb_features.start_feature("lidar", mode="guard")[1], "local"
    m = re.search(r"\b(?:navigate|go|drive|head|take me|take yourself|get yourself|make your way)(?: over| back| on)? to (?:the |my |our )?(.+)$", t)
    if m and not re.search(r"\b(sleep|bed|left|right|forward|back|backward|front|rear|middle|centre|center)\b", m.group(1)):
        return gb_features.navigate(re.sub(r"[ ,]*\b(please|now|for me|right now)$", "", m.group(1)).strip(), store)[1], "local"
    return None

ROUTER_PROMPT = SYSTEM_PROMPT.replace("You can't drive the robot or see unless the system did it for you.", "") + """
You control the robot by answering with ONE JSON object only (no prose, no code fences):
{"intent": "<name>", ...arguments..., "say": "<short spoken reply>"}
Intents and arguments:
- drive: "steps": [[x, z, t], ...] x = linear m/s (-0.16..0.16, negative = backward), z = angular rad/s (-0.8..0.8, positive = left/counterclockwise), t = seconds. Total time at most 5 s. Defaults x 0.12, z 0.64, t 1.5. Turning in place: x 0. "A quarter turn" is about z 0.64 for 2.5 s.
- stop: halt all motion and features.
- line_follow: "color" (black default; red, green, blue, black, white).
- color_track: "color" (red, green, blue, black, white).
- gesture: start hand-gesture driving.
- lidar_mode: "mode" avoid | follow | guard.
- lights: "color" (red, green, blue, white, yellow, purple, orange, pink, cyan, off).
- beep: "times" 1-5.
- battery, time, date.
- status: any question about the robot's own health, sensors, camera, LiDAR, motors or what is running. Never guess these yourself.
- describe: the user wants you to look with the camera (what do you see, is anything on the floor, read text, count things); "question" = what to look for.
- identify: what is this thing in front of the camera.
- map_start, map_stop, map_save ("name" optional), where ("target": a labelled thing).
- mission: a multi-step autonomous drive. "mission": "explore" (drive around by itself and map the room, then return; "duration_s" optional), "distance" (drive forward "distance_m" meters, convert feet x0.3048), or "wander" (drive around avoiding obstacles for "duration_s" seconds). "return": true if the user wants it to come back/return/park where it started. E.g. "go about ten feet and then come home" -> {"intent":"mission","mission":"distance","distance_m":3.05,"return":true}. Prefer mission over drive/map_start whenever a distance in feet/meters, a wander time in minutes, autonomous mapping, or coming back is requested.
- navigate: any request to go, drive, move or take itself to a named place or thing ("target"), e.g. "head over to the couch".
- sensor: turn a sensor on or off. "sensor": "lidar" | "camera" | "both", "on": true/false.
- speed: change driving speed. "change": "up" | "down" | "normal".
- local: objects, faces, people, follow, come to me, where things are, joystick, model updates, hit ratio, safety limits. Put ONE short canonical command in "text", e.g. "what is this", "this is a mug", "where is the mug", "what's around here", "list objects", "forget mug", "this is me", "this is Mark", "who is this", "who do you know", "follow Mark", "come to me", "go to the mug", "set up the joystick", "update your models".
- chat: anything else; put the answer in "say" (1-2 short spoken sentences, lead with the result, short first clause).
Use "chat" if unsure. For robot actions keep "say" under 12 words.
MOTION HONESTY (hard rule): a "chat" reply never moves the robot. In "chat", never say or imply that you are driving,
moving, following, coming, heading, turning, backing up, rolling or on your way (no "I'm driving toward you", "I'm
following you", "heading back now"). If the user wants you to move, follow or come, use the matching intent instead
(local "follow me" / "come to me", drive, mission, navigate). If no intent fits, say plainly that you are not moving."""

# 2026-11-23 fix 6: chat replies must never claim motion that is not happening (11:11 "I'm driving slowly toward you now"
# came back as intent "chat", so nothing moved). Applied ONLY to conversation replies, never to executed motion intents.
_MOTION_CLAIM = re.compile(
    r"\b(?:i['\u2019]?m|i am|i['\u2019]?ll|i will|i['\u2019]?ve|let me|here i)\s+(?:(?:now|still|already|just|slowly|gonna|going to|about to|"
    r"right|be|been|start|starting|keep|keeping)\s+)*(?:driv(?:e|ing)|follow(?:ing)?|mov(?:e|ing)|com(?:e|ing)|"
    r"head(?:ing)?|roll(?:ing)?|turn(?:ing)?|back(?:ing)? up|revers(?:e|ing)|navigat(?:e|ing)|go(?:ing)? (?:toward|over|back|"
    r"to (?:you|the|my|your|where))|on my way|steer(?:ing)?|approach(?:ing)?)\b|\bon my way\b|\bhere i come\b|\bfollowing you\b", re.I)  # 2026-11-25 navfix: also curly apostrophes (I’ll drive...) from the Ollama router
MOTION_HONEST_REPLY = "I'm not moving right now. Say follow me or come to me if you want me to drive."

def honest_motion(reply):
    """Return reply, or an honest replacement if a CHAT reply claims the robot is moving."""
    if reply and _MOTION_CLAIM.search(reply):
        log(f"motion-honesty: replaced chat reply {reply[:120]!r}")
        return MOTION_HONEST_REPLY
    return reply

def grok_route(user_text):
    """Ask Grok for a JSON intent. Returns dict or None."""
    if LLM_BACKEND != "xai" or not has_key() or budget_left("conversation") < 1200:
        return None
    _t = time.time(); note = context_note(user_text); _tn = time.time() - _t
    msgs = [{"role": "system", "content": ROUTER_PROMPT}] + store.last_turns(int(E("ROUTER_TURNS", "2")) * 2)
    msgs.append({"role": "user", "content": (f"[context]\n{note}\n[/context]\n" if note else "") + user_text})
    try:
        _t = time.time()
        r = _http.post(f"{XAI_BASE_URL}/chat/completions", timeout=60,
                         headers={"Authorization": f"Bearer {xai_key()}", "x-grok-conv-id": "mentorpi-grokbot-router"},
                         json={"model": XAI_CHAT_MODEL, "messages": msgs, "max_tokens": int(E("ROUTER_MAX_TOKENS", "400")),
                               "temperature": 0.2})
        r.raise_for_status(); j = r.json()
        tokens = store.log_usage("xai", XAI_CHAT_MODEL, "conversation", j.get("usage"))
        raw = (j["choices"][0]["message"].get("content") or "").strip()
        m = re.search(r"\{.*\}", raw, re.S)
        try: d = json.loads(m.group(0)) if m else None
        except Exception: d = None
        if (not isinstance(d, dict) or "intent" not in d) and raw and "{" not in raw:
            d = {"intent": "chat", "say": raw}               # plain-text answer = chat reply (no second LLM call)
        if not isinstance(d, dict) or "intent" not in d:
            log(f"router: unparsable {raw[:160]!r}"); return None
        d["_tokens"] = tokens; log(f"router ({XAI_CHAT_MODEL}, context {_tn:.2f}s, xai {time.time() - _t:.2f}s): {json.dumps(d)[:240]}")
        return d
    except Exception as ex:
        body = getattr(getattr(ex, "response", None), "text", "") or ""
        log(f"router failed: {type(ex).__name__}: {str(ex)[:160]} {body[:160]}"); return None

def dispatch(d, user_text):
    """Map a Grok JSON intent onto the same local handlers. Returns (reply, category) or a vision sentinel."""
    it = str(d.get("intent", "chat")).lower(); say = (d.get("say") or "").strip()
    import gb_wakegate as WG                          # 2026-11-24 wakegate: router motion intents need the wake phrase
    if it in WG.MOTION_INTENTS:
        _wg = WG.block("intent:" + it, user_text)
        if _wg: return _wg, "local"
    if it == "stop":
        return do_stop(), "local"
    if it in ("local", "identify"):
        import gb_perceive
        r = gb_perceive.intent(d.get("text") or ("what is this" if it == "identify" else user_text),
                               {"explore": _explore(), "announce": _announce(False), "store": store})
        return (r or say or "I'm not sure how to do that."), "local"
    if it in ("drive", "mission"):
        _pm = gb_estop.parse_motion(user_text, gb_caps.speed_mult())
        if _pm and (it == "drive" or re.search(r"\b(second|sec|seconds)\b", user_text.lower())):
            return do_timed(_pm), "local"
    if it == "drive":
        steps = []
        for st in d.get("steps") or []:
            try: steps.append((float(st[0]), float(st[1]), float(st[2])))
            except Exception: pass
        ok, msg, _ = gb_features.drive(steps)
        return (msg if not ok or gb_features.is_dry() else (say or msg)), "local"
    if it == "line_follow":
        ok, msg = gb_features.start_feature("line", d.get("color") or "black"); return msg, "local"
    if it == "color_track":
        ok, msg = gb_features.start_feature("color", d.get("color")); return msg, "local"
    if it == "gesture":
        return gb_features.start_feature("gesture")[1], "local"
    if it == "lidar_mode":
        return gb_features.start_feature("lidar", mode=d.get("mode") or "avoid")[1], "local"
    if it == "lights":
        return gb_features.set_lights(d.get("color") or "white")[1], "local"
    if it == "beep":
        try: n = int(d.get("times") or 1)
        except Exception: n = 1
        return gb_features.beep(n)[1], "local"
    if it == "battery":
        return say_battery(), "local"
    if it == "status":
        return status_with_sensors(), "local"         # navstd: + fused sensor suite
    if it == "time":
        return time.strftime("It's %-I:%M %p."), "local"
    if it == "date":
        return time.strftime("Today is %A, %B %-d, %Y."), "local"
    if it == "describe":
        return ("__VISION_DETAILED__", "vision")
    if it == "identify":
        return ("__VISION_LOCAL_FIRST__", "vision")
    if it == "map_start":
        return map_start(), "local"
    if it == "map_stop":
        import gb_mapping; return gb_mapping.stop()[1], "local"
    if it == "map_save":
        import gb_mapping
        try: return gb_mapping.save((d.get("name") or "").strip() or None, stop_after=True, store=store)[1], "local"
        except Exception as ex: return f"I couldn't save the map: {ex}.", "local"
    if it == "mission":
        plan = _explore().plan_from_router(d)
        if plan:
            return start_mission(plan), "local"
        return "I couldn't work out the distance or time for that. Try: drive two meters and come back.", "local"
    if it == "navigate":
        return gb_features.navigate(d.get("target") or "", store)[1], "local"
    if it == "sensor":
        sn = str(d.get("sensor") or "").lower(); on = bool(d.get("on", True))
        words = {"lidar": "lidar", "camera": "camera", "both": "both sensors"}.get(sn)
        if words:
            return sensor_speed_intent(f"turn {words} {'on' if on else 'off'}")
    if it == "speed":
        ch = str(d.get("change") or "").lower()
        phrase = {"up": "increase speed", "down": "decrease speed", "normal": "normal speed"}.get(ch)
        if phrase:
            return sensor_speed_intent(phrase)
    if it == "where":
        r = intent("where is " + (d.get("target") or "it"))
        if r: return r
    return None

_SENS_RX = re.compile(r"\b(lidar|lidars|lie ?dar|laser|camera|cameras|both sensors|sensors)\b")
_NOT_CMD = re.compile(r"^(is|are|was|were|what|why|how|does|do|did|can you see|where)\b")

def sensor_speed_intent(t):
    """Local (no-token) intents: LiDAR / camera on-off gates and the driving speed setting."""
    if _SENS_RX.search(t) and not _NOT_CMD.match(t) and len(t.split()) <= 10 and \
       re.search(r"\b(off|on|disable|enable|deactivate|activate|shut (?:it )?down|power (?:it )?down|stop using|start using)\b", t):
        off = bool(re.search(r"\b(off|disable|deactivate|shut (?:it )?down|power (?:it )?down|stop using)\b", t))
        names = []
        if re.search(r"\b(lidar|lidars|lie ?dar|laser|both sensors|sensors)\b", t): names.append("lidar")
        if re.search(r"\b(camera|cameras|both sensors|sensors)\b", t): names.append("camera")
        for n in names: gb_caps.set_sensor(n, not off)
        if off: gb_features.emergency_stop()               # stop anything that relied on the gated sensor
        d = gb_caps.get()
        if not d["lidar"] and not d["camera"]: tail = " Both sensors are off, so I won't move."
        elif not d["lidar"]: tail = " I can only do short camera-guarded drives now; no mapping or exploring."
        elif not d["camera"]: tail = " I'll rely on the LiDAR and won't look at anything."
        else: tail = " All sensors are on."
        nice = " and ".join("LiDAR" if n == "lidar" else "camera" for n in names)
        return f"Okay, my {nice} {'is' if len(names) == 1 else 'are'} {'off' if off else 'on'}.{tail}", "local"
    if re.search(r"\b(voice|talk|talking|speak|speaking|volume|say|saying)\b", t):
        return None
    cur = gb_caps.get()["speed_pct"]
    # 2026-11-26 speedcap: while an explore mission is running, "pick up speed"
    # ramps the autonomous cruise toward 1.0 m/s (the node still applies the
    # sensor-reaction ceiling). A new mission always resets to default cruise.
    _exploring = False
    try:
        import gb_explore as _EX
        _exploring = bool(_EX.running())
    except Exception:
        _exploring = False
    if re.search(r"\b(normal|default|full|regular|reset(?: the| your)?)(?: driving)? speed\b|\breset (?:the |your )?speed\b", t):
        if _exploring:
            v = _EX.reset_explore_speed_target()
            return f"Explore speed is back to the default cruise, {v:.3f} meters per second.", "local"
        p = gb_caps.set_speed_pct(100)["speed_pct"]
        return f"Driving speed is back to normal, {p} percent.", "local"
    if re.search(r"\b(increase|raise|bump up|turn up|pick up|up) (?:the |your |my )?(?:driving )?speed\b|\bspeed (?:it )?up\b|^(?:go |drive |move |a (?:little|bit) )?faster\b|\b(?:drive|go|move) faster\b", t):
        if _exploring:
            v = _EX.request_pick_up_speed()
            return (f"Okay, picking up explore speed toward {_EX.HARD_MAX_LIN:.1f} meters per second. "
                    f"Target is now {v:.3f} meters per second, still limited by how far I can see."), "local"
        p = gb_caps.set_speed_pct(cur + 10)["speed_pct"]
        return (f"Driving speed is now {p} percent." + (" That's my maximum." if p == cur else "")), "local"
    if re.search(r"\b(decrease|lower|reduce|turn down|drop) (?:the |your |my )?(?:driving )?speed\b|\bslow (?:it )?down\b|^(?:go |drive |move |a (?:little|bit) )?slower\b|\b(?:drive|go|move) slower\b", t):
        if _exploring:
            v = _EX.request_slow_down_speed()
            return f"Okay, easing explore speed to {v:.3f} meters per second.", "local"
        p = gb_caps.set_speed_pct(cur - 10)["speed_pct"]
        return (f"Driving speed is now {p} percent." + (" That's my minimum." if p == cur else "")), "local"
    return None

def intent(text):
    """Return (reply, category) for local intents, or None to fall through to the LLM."""
    t = text.lower().strip().rstrip(".!?")
    t = re.sub(r"^(hey |ok |okay )?(grok ?bot|grok|robot)[, ]+", "", t)
    t = (gb_estop.strip_leadins_raw(t) or t).lower().strip().rstrip(".!?")   # natural: "can you / I want you to / please"
    _k = estop_intent(text)
    if _k is not None:   # 2026-11-17: "" = silent halt, handled locally
        return _k, "local"
    import gb_wakegate as WG                          # 2026-11-24 wakegate: follow / come / go-to / explore need the wake phrase
    if WG.looks_like_motion(t):
        _wg = WG.block("local-motion", text)
        if _wg: return _wg, "local"
    try:
        import gb_wally
        _wy = gb_wally.handle(text)
    except Exception as ex:
        _wy = None
        log("wally intent failed:", ex)
    if _wy:
        return _wy, "local"
    try:
        import gb_perceive
        _pr = gb_perceive.intent(t, {"explore": _explore(), "announce": _announce(False), "store": store})
    except Exception as ex:
        _pr = f"I ran into a problem: {str(ex)[:100]}"; log("perceive failed:", ex)
    if _pr:
        return _pr, "local"
    _ss = sensor_speed_intent(re.sub(r"^(?:can you|could you|please|would you)\s+", "", t))
    if _ss:
        return _ss
    try:
        _plan = _explore().parse_plan(normalize_explore_utterance(t))
    except Exception as ex:
        _plan = None; log("mission parse failed:", ex)
    if _plan:
        return start_mission(_plan), "local"
    if re.search(r"\b(halt|freeze|stand still|cancel that|abort|disarm|whoa|woah)\b", t) and len(t.split()) <= 6 and not re.search(r"\b(map|mapping|slam)\b", t):
        return do_stop(), "local"
    if re.search(r"\b(mission|exploring|exploration)\b", t) and re.search(r"\b(status|going|progress|how far|doing)\b", t):
        return _explore().status_text(), "local"
    if re.search(r"\b(battery|voltage|volts|charge level|power level)\b", t):
        return say_battery(), "local"
    if re.search(r"\btoken(s)? (usage|use|used|budget)|how many tokens|my usage\b", t):
        return usage_report(), "local"
    if re.search(r"\bwhat(?:'s| is) the time\b|\bwhat time is it\b", t):
        return time.strftime("It's %-I:%M %p."), "local"
    if re.search(r"\bwhat(?:'s| is) (the )?(date|day)( today)?\b|\bwhat day is it\b", t):
        return time.strftime("Today is %A, %B %-d, %Y."), "local"
    m = re.match(r"^(?:please )?(?:remember|note|learn)(?: that)? (.+)", t)
    if m:
        fact = text.strip().rstrip(".")
        fact = re.sub(r"^(?:.*?\b)?(?:remember|note|learn)(?: that)?\s+", "", fact, flags=re.I)
        store.add_fact(fact[0].upper() + fact[1:] if fact else fact)
        return f"Okay, I'll remember that {fact}.", "local"
    if re.match(r"^(yes,? )?forget everything$", t) and time.time() - PENDING["forget_all"] < 30:
        n = store.forget_all(); PENDING["forget_all"] = 0
        return f"Done. I forgot {n} things you taught me.", "local"
    m = re.match(r"^(?:please )?forget(?: about| that)? (.+)", t)
    if m:
        labs = [] if m.group(1).strip() in ("everything", "all", "all facts", "everything you know") else store.find_labels(re.sub(r"^(the|my|our|a|an) ", "", m.group(1).strip()))
        for r in labs:
            store.tx("DELETE FROM labels WHERE id=?", (r["id"],))
        if labs:
            store.write_labels_tsv()
        res = store.forget(m.group(1))
        if labs and not res:
            return f"Forgotten the label {', '.join(r['name'] for r in labs)}.", "local"
        if res is None:
            PENDING["forget_all"] = time.time()
            return "Are you sure? Say 'yes forget everything' within 30 seconds.", "local"
        return (f"Forgotten: {'; '.join(res)}." if res else f"I didn't have anything saved about {m.group(1)}."), "local"
    m = re.match(r"^what (?:do|did) you (?:know|remember|learn)(?: about (.+))?$", t)
    if m:
        if not m.group(1):
            n = store.fact_count(); labs = len(store.labels())
            return f"I know {n} facts and {labs} labeled things. Ask me about a topic.", "local"
        facts = store.search_facts(m.group(1), 3)
        labs = store.find_labels(m.group(1))
        out = facts + [f"the {r['name']} is at x {r['map_x']:.1f}, y {r['map_y']:.1f}" for r in labs[:2] if r["map_x"] is not None]
        return ("Here's what I know: " + ". ".join(out) + ".") if out else f"I don't know anything about {m.group(1)} yet.", "local"
    m = re.match(r"^(?:this|that|here) is (?:the |a |an |my |our )?(.+)$", t)
    if m and len(m.group(1).split()) <= 5:
        return do_label(m.group(1)), "local"
    m = re.match(r"^where(?:'s| is| are) (?:the |my |our )?(.+)$", t)
    if m:
        labs = store.find_labels(m.group(1))
        if labs:
            r = labs[-1]
            if r["map_x"] is not None:
                return f"You showed me the {r['name']} at x {r['map_x']:.1f}, y {r['map_y']:.1f} meters in the {r['frame']} frame.", "local"
            return f"I know the {r['name']} but not where it is.", "local"
        facts = store.search_facts(m.group(1), 2)
        if facts:
            return "From what you told me: " + ". ".join(facts) + ".", "local"
        return f"I don't know where the {m.group(1)} is yet. Point my camera at it and say 'this is the {m.group(1)}'.", "local"
    if re.match(r"^(?:please )?(?:start|begin) (?:the )?(?:mapping|making a map|map making|a new map|slam)\b", t) or \
       re.match(r"^(?:please )?(?:make|build|create) a map\b", t):
        return map_start(), "local"
    if re.match(r"^(?:please )?(?:stop|cancel|end|quit) (?:the )?(?:mapping|map making|slam)\b", t):
        import gb_mapping
        if _explore().abort("stop mapping"):
            gb_features.emergency_stop()
            return "Stopping the mapping mission. I'll save what I have and stop the motors.", "local"
        return gb_mapping.stop()[1], "local"
    m = re.match(r"^(?:please )?save (?:the |this |my )?map(?: as| called| named)? ?(.*)$", t)
    if m:
        import gb_mapping
        try:
            return gb_mapping.save(m.group(1).strip() or None, stop_after=True, store=store)[1], "local"
        except Exception as ex:
            return f"I couldn't save the map: {ex}.", "local"
    r = feature_intent(t, text)
    if r:
        return r
    m = re.search(r"\b(?:go|move|drive|back up|reverse)\s+(?:the robot\s+|a (?:little|bit)\s+)?(forward|forwards|ahead|back|backward|backwards)\b|\bturn (left|right)\b|\b(spin) around\b", t)
    if m:
        if not ALLOW_MOTION:
            return "Driving by voice is switched off. Set ALLOW_MOTION to 1 in my settings to allow it.", "local"
        d = m.group(1) or m.group(2) or m.group(3)
        _w = {"one":1,"two":2,"three":3,"four":4,"five":5}
        _dm = re.search(r"(\d+(?:\.\d+)?|one|two|three|four|five) second", t)
        _dur = min(5.0, float(_w.get(_dm.group(1), _dm.group(1)))) if _dm else (1.5 if d != "spin" else 3.0)
        lin, ang = {"forward": (0.12, 0), "ahead": (0.12, 0), "back": (-0.096, 0), "backward": (-0.096, 0),
                    "backwards": (-0.096, 0), "forwards": (0.12, 0), "left": (0, 0.64), "right": (0, -0.64), "spin": (0, 0.8)}[d]
        return gb_features.drive([(lin, ang, _dur)])[1], "local"
    if re.search(r"\b(describe|what do you see|what can you see|look around|identify|how many|count the|read (the|this))\b", t):
        return ("__VISION_DETAILED__", "vision")
    if re.search(r"\bwhat(?:'s| is) (this|that)\b|\bwhat am i (pointing at|holding|showing)\b|\bdo you (know|recognize) (this|that|what this is)\b", t):
        return ("__VISION_LOCAL_FIRST__", "vision")
    return None

SMALLTALK = re.compile(r"^(hi|hello|hey|good (morning|afternoon|evening|night)|thanks|thank you|how are you|"
                       r"what'?s up|nice|cool|ok(ay)?|bye|goodbye|see you|tell me a joke|you'?re (funny|great|awesome))\b", re.I)

def answer(user_text, source="voice"):
    """Single entry point for voice and web. Returns (reply, meta dict)."""
    t0 = time.time(); tokens = 0; backend = "local"; TIMING["answer_t0"] = t0
    try:
        r = intent(user_text)
        if r and r[0] in ("__VISION_DETAILED__", "__VISION_LOCAL_FIRST__"):
            reply, tokens, category, backend = do_vision(user_text, detailed=r[0] == "__VISION_DETAILED__")
        elif r:
            reply, category = r
        else:
            small = bool(SMALLTALK.match(user_text.strip()))
            d = None if small else grok_route(user_text)
            rr = dispatch(d, user_text) if d and str(d.get("intent", "chat")).lower() != "chat" else None
            if d: tokens = d.get("_tokens", 0) or 0
            if rr and rr[0] in ("__VISION_DETAILED__", "__VISION_LOCAL_FIRST__"):
                q = (d.get("question") or user_text) if d else user_text
                reply, vt, category, backend = do_vision(q, detailed=rr[0] == "__VISION_DETAILED__")
                tokens += vt; backend = "xai"
            elif rr:
                reply, category = rr; backend = "xai-intent"
            elif d and (d.get("say") or "").strip():
                reply = re.sub(r"[*_#`>]+", "", d["say"]).strip(); category = "conversation"; backend = "xai"
                reply = honest_motion(reply)                                    # 2026-11-23 fix 6
            else:
                reply, ct, category, backend = converse(user_text, smalltalk=small); tokens += ct
                if category in ("conversation", "smalltalk"): reply = honest_motion(reply)   # 2026-11-23 fix 6
    except Exception as ex:
        log("answer error:", repr(ex)); reply, category = f"I ran into a problem: {str(ex)[:140] or type(ex).__name__}.", "error"
    store.add_turn("user", user_text, category); store.add_turn("assistant", reply, category)
    append_transcript({"ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "source": source, "user": user_text, "robot": reply,
                       "tokens": tokens, "category": category, "backend": backend, "secs": round(time.time() - t0, 2)})
    log(f"[{category}/{backend} {tokens}t {time.time() - t0:.1f}s] {BOT_NAME}: {reply}")
    threading.Thread(target=maybe_summarize, daemon=True).start()
    return reply, {"category": category, "tokens": tokens, "backend": backend}

def seed_watcher():
    placeholder = ("# Grok Bot knowledge seed\n<!-- PLACEHOLDER: paste knowledge here. Each '- ' bullet or paragraph becomes "
                   "one fact. Headings and comments are ignored. The file is re-imported automatically when it changes. -->\n")
    if not os.path.exists(SEED_FILE):
        atomic_write(SEED_FILE, placeholder)
    last = None
    while True:
        try:
            mt = os.path.getmtime(SEED_FILE)
            if mt != last:
                n = store.import_seed(SEED_FILE); last = mt
                if n is not None:
                    log(f"imported {n} facts from {SEED_FILE}"); store.event("seed_import", str(n))
        except Exception as ex:
            log("seed import error:", ex)
        time.sleep(15)

# ============================================================================= wake port: WonderEcho serial = tty on the same USB hub as its sound card
def find_wonderecho_port():
    hubs = []
    for c in glob.glob("/sys/class/sound/card*"):
        try:
            cid = open(c + "/id").read().strip()
            name = open(f"/proc/asound/card{c.rsplit('card', 1)[1]}/usbid").read() if os.path.exists(
                f"/proc/asound/card{c.rsplit('card', 1)[1]}/usbid") else ""
        except Exception:
            continue
        dev = os.path.realpath(c + "/device")
        if "usb" in dev and (AUDIO_IN == "auto" or cid in AUDIO_IN or c.rsplit("card", 1)[1] in AUDIO_IN):
            hubs.append(os.path.dirname(os.path.dirname(dev)))      # .../3-2/3-2.2/3-2.2:1.0 -> .../3-2
    for tty in sorted(glob.glob("/sys/class/tty/ttyUSB*") + glob.glob("/sys/class/tty/ttyCH341USB*")):
        dev = os.path.realpath(tty + "/device")
        if any(dev.startswith(h + "/") for h in hubs):
            return "/dev/" + os.path.basename(tty)
    return None

# ============================================================================= local voice loop
# Two wake paths, both active:
#  * software phrase: exact "wake up Wally" (see gb_wally.classify_wake). Hello/Hey Wally and a
#    bare "wally" are not wakes. Legacy G-Bot phrases stay exact-or-fuzzy only when the
#    utterance does not mention wally.
#  * hardware: WonderEcho Pro factory firmware still sends "Hello Hiwonder" (AA 55 03 00 FB).
#    That chip is not reflashed by this software.
# Everything heard is logged to ~/grokbot/data/heard/YYYY-MM-DD.jsonl (text, matched, wake source).
import difflib
WAKE_WORDS = [w.strip().lower() for w in E("WAKE_WORDS", "wake up wally,hey wally,wally wake up,"
              "wake up g-bot,wake up gbot,wake up g bot,wake up gee bot,"
              "wake up jibot,wake up the bot,wake up gbot,wake up jeebot,wake up g. bot,hey grok,hey grock,a grok,hey groc,hey grog").split(",") if w.strip()]
WAKE_WORDS = [w for w in WAKE_WORDS if w not in ("hello wally", "hi wally", "wally")]  # keep "hey wally"
WAKE_ANYWHERE = set(_x.strip().lower() for _x in os.environ.get("WAKE_ANYWHERE","gbot,grok,grock,groc,grog,jibot,jeebot").split(",") if _x.strip())
WAKE_ANYWHERE |= {"grokbot", "grockbot", "grogbot", "grocbot", "grokbots", "grockbots", "grokbott", "groqbot", "grokebot",
                  "gbots", "geebot", "jeebot", "rockbot", "grok", "groq"}
WAKE_ANYWHERE.discard("wally")
def _env_float(name, default):
    """A blank or junk env value uses the default. It must not kill service import."""
    raw = E(name, "")
    text = str(raw).strip() if raw is not None else ""
    if not text:
        text = str(default)
    try:
        return float(text)
    except (TypeError, ValueError):
        return float(default)

WAKE_TOKEN_FUZZ = _env_float("WAKE_TOKEN_FUZZ", "0.80")      # single word ~ "grokbot" (e.g. grogbot, grakbot)
WAKE_FUZZ = _env_float("WAKE_FUZZ", "1.0")                   # blank or unset is an exact match, not a crash
HW_WAKE = E("HW_WAKE", "1") == "1"
LOG_HEARD = E("LOG_HEARD", "1") == "1"
WAKE_ACK = E("WAKE_ACK", "Hello John, are you ready to rock and roll?")
FOLLOWUP_FILLERS = {"bye", "all right", "alright", "okay", "ok", "so", "yeah", "yes", "no", "um", "uh", "hmm", "thank you",
                    "thanks", "you", "right", "cool", "nice", "wow", "oh", "huh", "what", "goodbye", "okay okay"}
WAKE_ECHO = re.compile(r"^(?:\W*(?:wait|okay|ok|hey|hi|hello|hallo)?[ ,]*(?:hi|he|high|hey)[ -]?(?:wonder|wander|wonders)\W*)+", re.I)

def _norm(t):
    return re.sub(r"\s+", " ", re.sub(r"[^a-z0-9 ]+", "", t.lower().replace("-", " "))).strip()

WAKE_ANY_MAXPOS = int(E("WAKE_ANY_MAXPOS", "8"))    # wake word deeper in long chatter = overheard, ignore

def match_wake(text):
    """Software wake is an accepted phrase (wake up wally / hey wally / wally wake up). A serial event is not a match."""
    import gb_wally
    if not gb_wally.software_wake(text, hardware_event=False):
        return False, ""
    return gb_wally.classify_wake(text)

def _match_wake_raw(text):
    """Return (matched, remainder_command). Exact variants or fuzzy 2-3 word windows within the first 6 words.
    Single-word variants only count at the start. The remainder keeps the original wording."""
    orig = [t for t in re.split(r"[\s,]+", text.replace("-", " ")) if t]
    toks = [_norm(t) for t in orig]
    keep = [(o, t) for o, t in zip(orig, toks) if t]
    orig = [o for o, _ in keep]; words = [t for _, t in keep]
    rest = lambda k: " ".join(orig[k:]).strip(" ,.!?")
    for w in sorted(WAKE_WORDS, key=len, reverse=True):
        ww = _norm(w).split(); L = len(ww)
        for i in range(0, min(6, len(words)) if L > 1 else 1):
            if words[i:i + L] == ww:
                return True, rest(i + L)
    targets = [_norm(w) for w in WAKE_WORDS if len(_norm(w).split()) >= 3 and "wally" not in _norm(w).split()]
    for i in range(min(6, len(words))):
        for L in (4, 3, 2):
            win = " ".join(words[i:i + L])
            if len(words[i:i + L]) == L and any(difflib.SequenceMatcher(None, win, t).ratio() >= WAKE_FUZZ for t in targets):
                return True, rest(i + L)
    if words in (["wake", "up"], ["wake", "up", "please"], ["wakey", "wakey"]):
        return True, ""                                     # bare "wake up" (G-Bot part lost by Whisper)
    for i,t in enumerate(words[:WAKE_ANY_MAXPOS]):
        if t in WAKE_ANYWHERE or (t in ("gee","g","ji") and i+1<len(words) and words[i+1]=="bot"):
            k = i + 2 if t in ("gee", "g", "ji") else i + 1
            if k < len(words) and words[k] in ("bot", "bots", "but") and t not in ("gee", "g", "ji"):
                k += 1                                      # "grok bot" / "grock bot"
            return True, rest(k)
    for i, t in enumerate(words[:6]):
        if len(t) >= 6 and difflib.SequenceMatcher(None, t, "grokbot").ratio() >= WAKE_TOKEN_FUZZ:
            return True, rest(i + 1)
    for i in range(len(words)-1):
        if words[i]=="the" and words[i+1]=="rock":
            return True, rest(i+2)
    return False, ""

def log_heard(text, matched, source, secs):
    if not LOG_HEARD:
        return
    try:
        d = os.path.join(DATA, "heard"); os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, time.strftime("%Y-%m-%d") + ".jsonl"), "a") as f:
            f.write(json.dumps({"ts": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "heard": text, "matched": matched,
                                "source": source, "secs": round(secs, 2)}) + "\n"); f.flush(); os.fsync(f.fileno())
    except Exception as ex:
        log("heard log failed:", ex)

HW_EVENT = threading.Event()
def hw_wake_thread(port):
    while True:
        try:
            w = WonderEchoWake(port)
            while True:
                w.wait(); log("hardware wake (WonderEcho 'Hello Hiwonder')"); HW_EVENT.set()
        except Exception as ex:
            log("WonderEcho wake port error:", ex); time.sleep(5)

def speak(text):
    """Speak sentence by sentence: synthesis of sentence n+1 overlaps playback of sentence n.
    Returns seconds from call to first audio (None if nothing to say)."""
    t0 = time.time(); sents = split_sentences(text); gen0 = PLAY_STATE["cancel_gen"]
    PLAY_STATE["recent"] = [(t0, text)] + [x for x in PLAY_STATE.get("recent", []) if t0 - x[0] < 20][:5]
    if not sents:
        return None
    if len(sents) == 1:
        wav = tts(sents[0]); first = time.time() - t0
        with AUDIO_LOCK:
            if PLAY_STATE["cancel_gen"] == gen0: play_wav_bytes(wav, sents[0])
        TIMING["tts_first"] = first; return first
    import queue
    q = queue.Queue(maxsize=3)
    def _put(x):
        while True:
            try: q.put(x, timeout=0.5); return True
            except queue.Full:
                if PLAY_STATE["cancel_gen"] != gen0: return False
    def prod():
        try:
            for snt in sents:
                if PLAY_STATE["cancel_gen"] != gen0 or not _put((tts(snt), snt)): break
        except Exception as ex:
            log("tts error:", ex)
        finally:
            _put(None)
    threading.Thread(target=prod, daemon=True).start()
    first = None
    with AUDIO_LOCK:
        PLAY_STATE["playing"] += 1                    # keep the mic gate closed across sentence gaps
        try:
            while True:
                item = q.get()
                if item is None or PLAY_STATE["cancel_gen"] != gen0: break   # barge-in cut our speech
                if first is None: first = time.time() - t0
                play_wav_bytes(item[0], item[1])
        finally:
            PLAY_STATE["playing"] -= 1; PLAY_STATE["last_end"] = time.time()
    TIMING["tts_first"] = first; return first

def barge_kill(tx):
    """Kill check for audio that overlapped our own speech: drop words we were saying ourselves (echo), then any
    stop word left wins."""
    words = gb_estop._norm(tx).split()
    spoken = set()
    for _t, said in PLAY_STATE.get("recent", []):
        spoken |= set(gb_estop._norm(said).split())
    left = " ".join(w for w in words if w not in spoken)
    return bool(left) and gb_estop.kill_phrase(left, moving=True)

def barge_decide(tx):
    """Overlapped transcript -> gb_estop.bargein_decision with the texts we were saying (echo mask)."""
    try: k = gb_caps.speed_mult()
    except Exception: k = 1.0
    return gb_estop.bargein_decision(tx, [s for _t, s in PLAY_STATE.get("recent", [])], k)

BARGE = {"t": 0.0, "cmd": ""}; BARGE_LOCK = threading.Lock()

def barge_drive(dec, src):
    """John gave a drive command over our speech: cut the TTS, then run it exactly like an awake command:
    answer() -> intent -> estop_intent (kill check, latch check) -> do_timed (ALLOW_MOTION, caps block, gate with
    3 s default / 10 s cap, backup stop timer). Returns the reply to speak, or None for a duplicate."""
    cmd = dec["cmd"]
    import gb_wakegate as WG                          # 2026-11-24 wakegate: speech over playback never starts motion
    WG.set_context(False, cmd)
    _wg = WG.block("barge-drive", cmd); WG.clear_context()
    if _wg: return _wg
    with BARGE_LOCK:                                  # stop listener + main mic can both catch the same words
        if time.time() - BARGE["t"] < 4.0 and BARGE["cmd"] == cmd:
            log(f"[BARGE] duplicate drive ignored ({src}): {cmd!r}"); return None
        BARGE["t"] = time.time(); BARGE["cmd"] = cmd
    log(f"[BARGE] drive command over playback ({src}): {cmd!r} action={dec['action']} parse={dec['motion']}")
    stop_playback(src)
    gb_features.set_dry(False)
    reply, _ = answer(cmd, "voice")
    return reply

RESUME_LOCK = threading.Lock()

def barge_resume(tx, src):
    """'resume' over our own speech while latched (bargein_decision already dropped our own 'say resume' echo).
    Stop listener + main mic can both catch it: the first clears, the second sees no latch and stays quiet."""
    with RESUME_LOCK:
        if not gb_estop.is_set():
            log(f"[ESTOP] resume over playback ignored ({src}): not latched"); return None
        log(f"[ESTOP] resume over playback ({src}): {tx!r}")
        stop_playback("resume")
        return do_resume()

def _resume_run(tx, src):
    try:
        reply = barge_resume(tx, src)
        if reply: speak(reply)
    except Exception as ex:
        log("barge resume failed:", repr(ex))

def relatch_allowed(tx, t_onset, src):
    """Stop phrase while ALREADY latched: True = re-latch (restart auto-clear). Self-voice filter (timing + own-line
    text). Only for the re-latch path; the first latch is never filtered here."""
    ok, why = gb_estop.relatch_decision(tx, t_onset, max(PLAY_STATE.get("any_last", 0.0), PLAY_STATE["last_end"]),
                                        PLAY_STATE["playing"] > 0, [x for _t, x in PLAY_STATE.get("recent", [])])
    if not ok:
        log(f"[ESTOP] re-latch ignored ({src}: {why}): {tx!r}")
    return ok

def _barge_run(dec, src):
    try:
        reply = barge_drive(dec, src)
        if reply: speak(reply)
    except Exception as ex:
        log("barge drive failed:", repr(ex))

def stop_listener():
    """Barge-in: while the robot is talking, thinking or moving, a second mic stream (ALSA dsnoop) listens ONLY for
    the kill phrase. Short VAD segments (<= 2.5 s), its own whisper instance so it never waits for the main loop."""
    if E("STOP_LISTENER", "1") != "1":
        return
    try:
        from faster_whisper import WhisperModel
        model = WhisperModel(E("STOP_WHISPER_MODEL", WHISPER_MODEL), device="cpu", compute_type="int8",
                             cpu_threads=int(E("STOP_WHISPER_THREADS", "2")))
    except Exception as ex:
        log("stop listener: whisper load failed:", ex); return
    log(f"stop listener up on {AUDIO_IN} (barge-in kill switch)")
    p = None; noise = 300.0; p_t0 = 0.0; nfr = 0       # p_t0 + nfr frames = audio clock (immune to pipe backlog)
    was_moving = False
    while True:
        try:
            moving_now = _moving()
            reset = reset_listener_noise(was_moving, moving_now, base=300.0)
            if reset is not None:
                noise = reset
                log("stop listener: noise floor reset on motion start")
            was_moving = moving_now
            active = self_playing(0.5) or moving_now or MAIN_BUSY["busy"]
            _ic = intercom_hold()
            if _ic == "talk" or (_ic and not moving_now):   # 2026-11-27 intercom: deaf to John's voice on our speaker
                active = False
            if not active:
                if p: p.terminate(); p.wait(); p = None
                time.sleep(0.1); continue
            if p is None or p.poll() is not None:
                p = subprocess.Popen(["arecord", "-q", "-D", AUDIO_IN, "-f", "S16_LE", "-r", str(RATE), "-c", "1", "-t", "raw"],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
                p_t0 = time.time(); nfr = 0
                if moving_now:
                    noise = 300.0
            buf = bytearray(); voiced = 0; started = False; t_s = 0.0; quiet = 0
            while True:
                fr = p.stdout.read(FRAME_BYTES)
                if not fr or len(fr) < FRAME_BYTES: raise IOError("stop listener arecord ended")
                nfr += 1
                lv = rms(fr); th = speech_trigger_threshold(noise, moving=moving_now or _moving())
                if not started:
                    if lv > th:
                        voiced += 1; buf.extend(fr)
                        if voiced >= 3:
                            started = True; t_s = time.time()
                            t_a = min(t_s, p_t0 + nfr * FRAME_MS / 1000.0)   # when this audio was captured (backlog-safe)
                    else:
                        voiced = 0; buf = bytearray(); noise = cap_noise_floor(0.97 * noise + 0.03 * lv)
                        if not (self_playing(0.5) or _moving() or MAIN_BUSY["busy"]): break
                        if intercom_hold() == "talk": break
                else:
                    buf.extend(fr); quiet = quiet + 1 if lv < th * 0.7 else 0
                    if quiet * FRAME_MS >= 300 or time.time() - t_s > 2.5: break
            if not started: continue
            segs, _ = model.transcribe(io.BytesIO(pcm_to_wav(bytes(buf))), language=LANGUAGE, beam_size=1, vad_filter=False,
                                       without_timestamps=True, condition_on_previous_text=False,
                                       initial_prompt="Hey Wally.")
            tx = " ".join(sg.text.strip() for sg in segs).strip()
            if not tx: continue
            _ic = intercom_hold()
            if _ic and (_ic == "talk" or not _moving() or heard_over_playback(t_s - 3 * FRAME_MS / 1000.0)):
                log(f"stop listener ignored intercom audio: {tx!r}"); continue
            if (gb_estop.announcement_echo(tx, _recent_lines()) or gb_estop.future_stop(tx)
                    or gb_estop.speaker_echo(tx, PLAY_STATE.get("recent") or _recent_lines())):
                log(f"stop listener ignored own speaker echo or later stop condition: {tx!r}")
                continue
            echo = self_playing(0.8) or heard_over_playback(t_s - 3 * FRAME_MS / 1000.0)   # playback + 0.8 s acoustic tail
            dec = barge_decide(tx) if echo else None
            moving_now = _moving()
            take = (dec["action"] == "kill") if echo else bool(gb_estop.stop_action(tx, moving=moving_now, spoken_texts=_recent_lines()))
            if take:
                if not gb_estop.is_set():
                    log(f"[ESTOP] barge-in kill phrase: {tx!r}")
                    reply = consider_stop(tx, "voice (barge-in): " + tx, moving=moving_now or echo)
                    if reply:
                        threading.Thread(target=speak, args=(reply,), daemon=True).start()
                elif relatch_allowed(tx, t_a - 3 * FRAME_MS / 1000.0, "stop listener"):   # latched: re-latch only if not our own voice
                    log(f"[ESTOP] barge-in kill phrase while latched (re-latch): {tx!r}")
                    consider_stop(tx, "voice (barge-in): " + tx, moving=True)
            elif dec and dec["action"] in ("drive", "latched"):   # drive command over our speech: cut TTS, run it
                threading.Thread(target=_barge_run, args=(dec, "stop listener"), daemon=True).start()
            elif dec and dec["action"] == "resume":              # 'resume' over our speech while latched (echo-filtered)
                threading.Thread(target=_resume_run, args=(tx, "stop listener"), daemon=True).start()
            else:
                if queue_pending_command(tx, src="stop listener", awake=_wake_window_open()):
                    log(f"stop listener queued command during playback: {tx!r}")
                else:
                    log(f"stop listener heard (no kill): {tx!r}")
        except Exception as ex:
            import gb_health
            dead = p is not None and p.poll() is not None
            kind = gb_health.audio_io_kind(ex, cancelling=dead or bool(PLAY_STATE.get("cancel_gen")))
            if kind == "cancel":
                log("arecord interrupted during barge-in cancel")
            else:
                log("audio failure (stop listener):", repr(ex))
            try:
                if p: p.terminate(); p.wait()
            except Exception: pass
            p = None; time.sleep(0.2 if kind == "cancel" else 1.0)

MAIN_BUSY = {"busy": False}

def estop_reactor():
    """Reacts to the ESTOP file whoever wrote it (SSH touch, clap daemon, joystick button): kill everything, speak.
    Also reports clap-test detections."""
    was = gb_estop.is_set(); last_clap = 0.0
    try:
        import gb_floor; floor_seen = os.path.getmtime(gb_floor.stop_event_path())
    except Exception:
        floor_seen = 0.0
    GUARD_EV = "/home/pi/docker/tmp/gb_safety/guard_event.json"; last_guard_say = 0.0
    try: guard_seen = os.path.getmtime(GUARD_EV)
    except OSError: guard_seen = 0.0
    ev = os.path.join(DATA, "clap_event.json")
    try: last_clap = os.path.getmtime(ev)
    except OSError: pass
    while True:
        try:
            now_set = gb_estop.is_set()
            try:                                             # 2026-11-25 noestop: floor normal stop from gate / clap / host
                import gb_floor
                fe = gb_floor.stop_event_path(); fm = os.path.getmtime(fe) if os.path.exists(fe) else 0.0
                if fm and fm > floor_seen:
                    floor_seen = fm
                    try: fd = json.load(open(fe))
                    except (OSError, ValueError): fd = {}
                    if str(fd.get("src", "")) not in ("voice", "chat", "do_estop") and time.time() - float(fd.get("t", 0) or 0) < 5.0:
                        log(f"[FLOOR STOP] {fd.get('src')}: {fd.get('why')} -> ending missions/features/tracker (normal stop, no latch)")
                        for name, fn in ESTOP_ACTIONS.items():
                            threading.Thread(target=fn, daemon=True).start()
                        threading.Thread(target=speak, args=("Stopped. Say wake up Wally to drive again.",), daemon=True).start()
            except Exception as ex:
                log("floor stop watch error:", ex)
            if now_set and not was:
                inf = gb_estop.info() or {}
                if inf.get("src") not in ("voice", "chat"):
                    log(f"[ESTOP] file latched by {inf.get('src', 'file')} -> killing missions/features/tracker")
                    for name, fn in ESTOP_ACTIONS.items():
                        threading.Thread(target=fn, daemon=True).start()
                    threading.Thread(target=gb_estop.zero_now, args=("reactor",), daemon=True).start()
                    threading.Thread(target=speak, args=(("Stopped." if gb_estop.floor_mode() else "Emergency stop."),), daemon=True).start()
            if was and not now_set:                          # 2026-11-17: spoken confirmation for a remote clear
                try:
                    rc = json.load(open(os.path.join(DATA, "remote_clear.json")))
                    if time.time() - float(rc.get("t", 0)) < 5.0:
                        threading.Thread(target=speak, args=("Brake off. Ready to drive.",), daemon=True).start()
                except (OSError, ValueError): pass
            was = now_set
            try:                                             # gb_gate guard trips -> spoken (throttled)
                gm = os.path.getmtime(GUARD_EV)
                if gm > guard_seen:
                    guard_seen = gm; g = json.load(open(GUARD_EV))
                    if time.time() - last_guard_say > 5.0 and time.time() - g.get("t", 0) < 3.0:
                        last_guard_say = time.time()
                        w = str(g.get("why", "obstacle")).split(" (")[0].replace("m ", " meters ")
                        threading.Thread(target=speak, args=(f"Guard: {w}. I held still.",), daemon=True).start()
            except (OSError, ValueError): pass
            try:
                mt = os.path.getmtime(ev)
                if mt > last_clap:
                    last_clap = mt; d = json.load(open(ev))
                    if d.get("test"):
                        threading.Thread(target=speak, args=(f"Double clap detected. Peaks {d.get('peak_db', '?')} dB. Test only, not latched.",),
                                         daemon=True).start()
            except (OSError, ValueError): pass
        except Exception as ex:
            log("estop reactor error:", ex)
        time.sleep(0.1)

AUTOCLEAR_SAY = "Brake off. Ready to drive."         # no stop words: our own echo can never re-latch it

def estop_autoclear():
    """ESTOP_AUTOCLEAR_S (15 s) after a latch, release it via gb_estop.clear('auto'). Latch only: never resumes motion
    or a cancelled mission. Own thread with no audio/LLM/lock dependency, so it fires even while grokbot is busy;
    the spoken line runs on a separate thread through speak() (marks speaking.json)."""
    ac = gb_estop.AutoClear(on_clear=lambda: threading.Thread(target=speak, args=(AUTOCLEAR_SAY,), daemon=True).start())
    if ac.timeout <= 0:
        log("estop auto-clear disabled (ESTOP_AUTOCLEAR_S=0)"); return
    log(f"estop auto-clear up: latch releases after {ac.timeout:g}s (motion is never resumed)")
    while True:
        try: ac.step()
        except Exception as ex: log("estop auto-clear error:", ex)
        time.sleep(0.25)

def voice_loop():
    resolve_audio()
    if TRIGGER in ("wonderecho", "hybrid", "phrase") and HW_WAKE:
        port = WAKE_PORT if WAKE_PORT != "auto" else find_wonderecho_port()
        if port:
            threading.Thread(target=hw_wake_thread, args=(port,), daemon=True).start()
            log(f"hardware wake on {port}")
        else:
            log("WonderEcho serial port not found; software wake phrase only")
    phrase_on = TRIGGER in ("wonderecho", "hybrid", "phrase", "vad")
    log(f"software wake phrases: {WAKE_WORDS[:4]}... fuzz={WAKE_FUZZ}")
    try:
        stt(pcm_to_wav(b"\0" * RATE)); stt(pcm_to_wav(b"\0" * RATE), idle=True)   # load both whisper models
    except Exception as ex:
        log("whisper load failed:", ex)
    greet = E("STARTUP_GREETING", "")   # live file is empty; emptiness alone is not the suppress switch
    if "{period}" in greet:   # time-of-day greeting; exempt from SUPPRESS_STARTUP_SPEECH (that still mutes health/warm-up speech)
        _h = time.localtime().tm_hour
        _per = "morning" if 5 <= _h < 12 else "afternoon" if 12 <= _h < 17 else "evening" if 17 <= _h < 21 else "night"
        greet = greet.replace("{period}", _per)
    if greet.strip(): log("startup greeting:", greet); speak(greet)
    mic = Mic(); awake_until = 0.0; followup = False
    prompt = E("WAKE_PROMPT", "Wake up Wally.")
    try:
        import gb_arm; gb_arm.disarm("voice service start (default disarmed)", actor="safety")   # 2026-11-25 arm
    except Exception as ex:
        log("arm reset failed:", ex)
    ic_was = False
    while True:
        try:
            MAIN_BUSY["busy"] = False
            if apply_bench_sleep_if_needed():            # 2026-11-27 stay-awake: only bench-mode entry returns to sleep
                awake_until = 0.0; followup = False
                log("wake session ended: entered bench mode")
            if intercom_hold():                          # 2026-11-27 intercom: Jarvis owns speaker+mic conversation
                if not ic_was:
                    log("intercom active: main mic released, wake phrase + voice commands paused")
                    ic_was = True
                if mic.p is not None:
                    mic.close()
                if take_pending_command():
                    log("intercom active: dropped a queued voice command")
                if awake_until:
                    awake_until = mark_wake_window(0.0, followup=False)
                followup = False; HW_EVENT.clear()
                time.sleep(0.1); continue
            if ic_was:
                ic_was = False; mic.noise = 200.0          # recalibrate the noise floor on the next pass
                log("intercom ended: normal dual listening restored (onboard mic + Jarvis)")
            pending = take_pending_command()
            if pending:
                try:
                    _dispatch_pending_command(pending)
                except Exception as ex:
                    log("pending command dispatch failed:", ex)
            with AUDIO_LOCK:
                mic.open()
            if mic.noise == 200.0:
                mic.calibrate(0.6)
            if HW_EVENT.is_set():
                HW_EVENT.clear()
                log("WonderEcho event ignored; software wake is a spoken phrase ('wake up Wally', 'hey Wally', or 'Wally wake up')")
            if followup and awake_until and PLAY_STATE["last_end"]:   # follow-up window runs from the END of our playback
                awake_until = max(awake_until, gb_estop.followup_deadline(PLAY_STATE["last_end"]))
            # listen in short slices so a hardware wake can interrupt idle listening
            t_listen = time.time()
            pcm = mic.record_utterance(wait_s=1.5)
            if intercom_hold():                          # intercom started mid-utterance: never act on it
                continue
            if HW_EVENT.is_set() and not pcm:
                continue
            if not pcm or len(pcm) < RATE * 2 * 0.35:
                if apply_bench_sleep_if_needed():
                    awake_until = 0.0; followup = False
                    log("wake session ended: entered bench mode")
                elif (not session_awake() and not mission_followup_open()
                      and awake_until and time.time() > awake_until):
                    awake_until = mark_wake_window(0.0, followup=False); log("wake window closed")
                continue
            if heard_over_playback(mic.t_onset):   # speech STARTED during playback or < 0.5 s after its true end (was: t_listen)
                # overlapped our own speech: never act on it EXCEPT the kill switch (was: silently dropped)
                try:
                    mic.close()
                    tx = (stt(pcm_to_wav(pcm[-RATE * 2 * 8:]), idle=True) or "").strip()
                    dec = barge_decide(tx) if tx else None
                    if dec and dec["action"] == "kill":
                        if not gb_estop.is_set() or relatch_allowed(tx, mic.t_onset, "main mic, overlapped"):   # first latch: unfiltered
                            log(f"[ESTOP] kill phrase during playback: {tx!r}")
                            reply = consider_stop(tx, "voice (during speech): " + tx, moving=True)
                            if reply:
                                speak(reply)
                    elif dec and dec["action"] in ("drive", "latched"):   # drive over our speech: no wake word needed
                        reply = barge_drive(dec, "main mic")
                        if reply:
                            speak(reply); followup = True
                            awake_until = gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"]))
                    elif dec and dec["action"] == "resume":   # resume over our speech while latched (own echo filtered)
                        reply = barge_resume(tx, "main mic")
                        if reply:
                            speak(reply); followup = True
                            awake_until = gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"]))
                    elif tx and queue_pending_command(tx, src="main mic overlapped", awake=_wake_window_open()):
                        pe = max(PLAY_STATE.get("any_last", 0.0), PLAY_STATE["last_end"])
                        log(f"queued command over playback (not echo): {tx!r} [onset {mic.t_onset - pe:+.2f}s vs playback end]")
                    elif tx:
                        pe = max(PLAY_STATE.get("any_last", 0.0), PLAY_STATE["last_end"])
                        log(f"ignored (overlapped playback): {tx!r} [onset {mic.t_onset - pe:+.2f}s vs playback end]")
                except Exception as ex:
                    log("playback kill check failed:", ex)
                continue
            mic.close(); MAIN_BUSY["busy"] = True            # don't buffer our own speech while thinking/talking
            t_eos = time.time()                           # end of speech + SILENCE_END_S tail
            sp = speech_seconds(pcm); t_gate = time.time() - t_eos
            if sp < STT_MIN_SPEECH_S:
                log(f"gate: dropped {len(pcm) / (2 * RATE):.1f}s of audio, speech {sp:.2f}s (noise/music)"); continue
            t0 = time.time()
            idle_now = not stay_listening(mission_running=mission_followup_open(),
                                          awake_until=awake_until, now=t_listen)
            peek = int(float(E("IDLE_PEEK_S", "5")) * RATE) * 2
            if idle_now and len(pcm) > peek + 2 * RATE * 2:
                # long idle chatter: only check the first seconds for a wake phrase (keeps the mic deaf for less time)
                text = (stt(pcm_to_wav(pcm[:peek]), prompt=prompt, idle=True) or "").strip()
                if text and match_wake(text)[0]:
                    text = (stt(pcm_to_wav(pcm), prompt=prompt) or "").strip()
            else:
                text = (stt(pcm_to_wav(pcm), prompt=None if awake_until else prompt, idle=idle_now) or "").strip()
            t_stt = time.time() - t0
            try:
                import gb_cadence
                gb_cadence.observe(len(pcm) / float(2 * RATE), text or "")
            except Exception:
                pass
            awake = stay_listening(mission_running=mission_followup_open(),
                                   awake_until=awake_until, now=t_listen)  # stay awake after wake until bench/power-off
            matched, rest = match_wake(text) if text else (False, "")
            if text and not matched:
                import gb_wally
                if gb_wally.wake_prefix(text):
                    more = mic.record_utterance(wait_s=1.5)
                    more_text = ""
                    if more and len(more) >= RATE * 2 * 0.2:
                        more_text = (stt(pcm_to_wav(more), prompt=prompt, idle=True) or "").strip()
                    joined = gb_wally.join_wake_text(text, more_text)
                    log(f"wake prefix {text!r} plus {more_text!r}")
                    text = joined
                    matched, rest = match_wake(text) if text else (False, "")
            if matched:                                   # 2026-11-25 arm: exact 'wake up Wally' arms motion
                import gb_arm; gb_arm.maybe_arm(text, "voice")
                enter_awake_session()                     # 2026-11-27 stay-awake: remain listening until bench / power-off
            log_heard(text, matched, "awake" if awake else "idle", time.time() - t0)
            log(f"heard: {text!r} wake={matched} awake={awake}")
            if not text:
                continue
            # KILL SWITCH FIRST: every transcript, wake word or not, awake or not
            if gb_estop.announcement_echo(text, _recent_lines()) or gb_estop.future_stop(text):
                log(f"ignored own announcement or a later stop condition: {text!r}")
                continue
            if gb_estop.stop_action(text, moving=_moving(), spoken_texts=_recent_lines()):
                log(f"[ESTOP] stop heard (no wake word needed): {text!r}")
                if gb_estop.is_set() and not relatch_allowed(text, mic.t_onset, "main mic"):   # first latch: unfiltered
                    continue
                reply = consider_stop(text, "voice: " + text, moving=_moving())
                if reply:
                    speak(reply)
                awake_until = 0.0; continue
            if gb_estop.is_set() and gb_estop.resume_phrase(text):
                speak(do_resume()); continue
            try:
                if mission_followup_open() and is_mission_free_command(text):
                    if re.search(r"\b(halt|freeze|abort|cancel|disarm|whoa|woah)\b", _norm_txt(text)):
                        log("[mission] halt heard without wake word -> abort")
                        speak(do_stop()); continue
                    log("[mission] return/come-back heard without wake word -> dispatch")
                    gb_features.set_dry(False)
                    reply, _ = answer(text, "voice")
                    if reply:
                        speak(reply)
                    followup = True
                    awake_until = max(time.time() + 30.0, gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"])))
                    continue
            except Exception as ex:
                log("mission command check failed:", ex)
            if awake:
                cmd = WAKE_ECHO.sub("", text).strip(" ,.!?")
                if matched:
                    cmd = rest
                elif followup and _norm_txt(cmd) in FOLLOWUP_FILLERS:
                    log(f"follow-up filler ignored: {cmd!r}"); continue
                if not cmd or len(cmd) < 2:               # only the wake phrase itself was heard
                    HW_EVENT.clear(); speak(bare_wake_prompt())
                    # keep followup=False so the first command in this window may start motion
                    awake_until = mark_wake_window(gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"])), followup=False)
                    followup = False
                    continue
            elif matched or (phrase_on and TRIGGER == "vad"):
                cmd = rest if matched else text
                if not cmd or len(cmd) < 2:
                    speak(bare_wake_prompt())
                    awake_until = mark_wake_window(gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"])), followup=False)
                    followup = False
                    continue
            else:
                continue                                  # not addressed to us
            import gb_wakegate as WG                      # 2026-11-24 wakegate: decided BEFORE followup is reset
            WG.set_context(WG.motion_allowed(matched, awake, followup) or mission_followup_open() or session_awake(), text)
            HW_EVENT.clear()
            keep_awake = mission_followup_open() or session_awake()
            if not keep_awake:
                awake_until = 0.0; followup = False
            if not cmd or len(cmd) < 2:
                WG.clear_context(); continue
            gb_features.set_dry(False)
            try:
                t_a = time.time(); reply, _ = answer(cmd, "voice"); t_ans = time.time() - t_a
            finally:
                WG.clear_context()
            first = speak(reply)
            log(f"[timing] eos_tail={SILENCE_END_S:.2f}s gate={t_gate:.2f}s stt={t_stt:.2f}s answer={t_ans:.2f}s "
                f"tts_first={(first or 0):.2f}s => end-of-speech to first audio "
                f"{SILENCE_END_S + t_gate + t_stt + t_ans + (first or 0):.2f}s")
            # follow-up window, no wake word needed: FOLLOWUP_S from END of playback.
            # While a mission is running the window stays open so stop/return work without a fresh wake.
            followup = True
            awake_until = mark_wake_window(gb_estop.followup_deadline(max(time.time(), PLAY_STATE["last_end"])), followup=True)
            if mission_followup_open() or session_awake():
                awake_until = mark_wake_window(max(awake_until, time.time() + 86400.0), followup=True)
        except KeyboardInterrupt:
            break
        except Exception as ex:
            import gb_health
            kind = gb_health.audio_io_kind(ex, cancelling=bool(PLAY_STATE.get("cancel_gen")))
            if kind == "cancel":
                log("arecord interrupted during barge-in cancel")
            else:
                log("audio failure:" if kind != "failed" else "voice loop error:", repr(ex))
            mic.close()
            try:
                if not isinstance(ex, IOError): speak(f"I ran into a problem: {str(ex)[:120] or type(ex).__name__}.")
            except Exception: pass
            time.sleep(2)

# ============================================================================= web
def make_app():
    from flask import Flask, jsonify, request, abort
    app = Flask(__name__)
    LOCAL = ("127.0.0.1", "::1")

    @app.before_request
    def _auth():
        if request.path.startswith("/internal/"):
            if request.remote_addr not in LOCAL: abort(403)
            return
        if WEB_TOKEN and request.args.get("token") != WEB_TOKEN and request.headers.get("X-Token") != WEB_TOKEN:
            abort(401)

    def respond(user_text, robot, source, dry=False):
        gb_features.set_dry(dry)
        matched, rest = match_wake(user_text)            # typed/web text may start with the wake phrase too
        if matched and rest:
            user_text = rest
        t_ans = time.time(); reply, meta = answer(user_text, source); t_ans = time.time() - t_ans
        wav = b""; first = None
        if robot and SPEAK_WEB_ON_ROBOT:
            first = speak(reply)
        else:
            wav = tts(reply)
        log(f"[timing] web answer={t_ans:.2f}s tts_first_audio={first if first is None else round(first, 2)}s")
        return {"user": user_text, "reply": reply, **meta, "timing": {"answer_s": round(t_ans, 2),
                "tts_first_s": None if first is None else round(first, 2)},
                "audio_b64": base64.b64encode(wav).decode() if (wav and not robot) else ""}

    @app.get("/")
    def index(): return PAGE.replace("__NAME__", BOT_NAME)

    @app.get("/health")
    def health():
        return {"ok": True, "key": has_key(), "ollama": ollama_up(), "facts": store.fact_count(),
                "labels": len(store.labels()), "xai_model": XAI_MODEL, "ollama_model": OLLAMA_MODEL}

    @app.get("/api/usage")
    def usage(): return {"report": usage_report()}

    @app.post("/api/chat")
    def chat():
        j = request.get_json(force=True, silent=True) or {}
        robot = bool(j.get("robot", True))
        dry = bool(j.get("dry_run", not robot))            # robot:false => no physical motion unless dry_run:false
        try: return jsonify({**respond(str(j.get("text", ""))[:2000], robot, "web", dry), "dry_run": dry})
        except Exception as ex: return jsonify({"error": str(ex)}), 500

    @app.post("/api/voice")
    def voice():
        f = request.files.get("audio")
        if not f: return jsonify({"error": "no audio"}), 400
        try:
            text = stt(f.read(), f.filename or "speech.webm")
            if not text: return jsonify({"error": "didn't catch that"}), 200
            return jsonify(respond(text, request.form.get("robot", "1") == "1", "web-voice",
                                   request.form.get("dry_run", "0") == "1"))
        except Exception as ex: return jsonify({"error": str(ex)}), 500

    @app.post("/internal/say")          # used by the low-voltage monitor (localhost only)
    def say():
        text = (request.get_json(force=True, silent=True) or {}).get("text", "")
        wav = tts(text)
        if AUDIO_LOCK.acquire(timeout=15):
            try: play_wav_bytes(wav, text)
            finally: AUDIO_LOCK.release()
            return {"ok": True}
        return {"ok": False}, 503

    @app.post("/internal/flush")
    def flush():
        store.checkpoint(); return {"ok": True}
    return app

def run_web():
    make_app().run(WEB_HOST, WEB_PORT, threaded=True, ssl_context="adhoc" if WEB_HTTPS else None)

def _health_monitor():
    """Rosbridge freshness only. Does not open /dev/ldlidar or restart drivers.

    Depth counts only after resolve_depth_scale agrees on a unit. Spoken lines
    stay off unless HEALTH_SPEAK=1, so a sanity boot does not talk.
    """
    import gb_wally, gb_health
    h = gb_wally.health()
    speak_ok = E("HEALTH_SPEAK", "0") == "1" and E("SUPPRESS_STARTUP_SPEECH", "0") != "1"
    while True:
        lidar = {"fresh": False, "valid_frac": 0.0, "driver_ok": False, "fault": "no sample"}
        depth = {"fresh": False, "valid_frac": 0.0, "unit_ok": False, "fault": "no sample"}
        rgb = {"fresh": False}
        ref_m = None
        try:
            scans = gb_robot.subscribe_once("/scan_raw", "sensor_msgs/LaserScan", timeout=0.8, n=1)
            if scans:
                msg = scans[0]
                frame = (msg.get("header") or {}).get("frame_id") or msg.get("frame_id")
                lidar = gb_health.lidar_status(msg.get("ranges") or [], frame)
                ref_m = lidar.get("ref_m")
        except Exception:
            pass
        try:
            frames = gb_robot.subscribe_once("/ascamera/camera_publisher/depth0/image_raw", "sensor_msgs/Image", timeout=0.8, n=1)
            if frames:
                depth = gb_health.depth_status(frames[0], ref_m)
        except Exception:
            pass
        try:
            rgb_msgs = gb_robot.subscribe_once("/ascamera/camera_publisher/rgb0/image", "sensor_msgs/Image", timeout=0.4, n=1)
            if rgb_msgs:
                rgb = {"fresh": True}
        except Exception:
            pass
        try:
            phrases = h.update(lidar, depth, rgb)
            gb_health.write_shared(h)
            for line in phrases:
                log("health:", line)
                if line == gb_health.NO_RANGE:
                    try:
                        gb_wally.drop_goals("no_range")
                    except Exception:
                        pass
                if speak_ok:
                    threading.Thread(target=speak, args=(line,), daemon=True).start()
        except Exception as ex:
            log("health monitor:", ex)
        time.sleep(1.0)

def _install_wally_runtime():
    """Teaching capture uses the existing snapshot URL. A follow step, if allowed, goes through the gate.

    No follow loop is started. A pointed mission is still only a spoken decision here;
    it is not ticked onto /gb/motion_req. Autonomy stays off unless the env says otherwise.
    """
    try:
        import gb_wally
    except Exception as ex:
        log("wally runtime not installed:", ex)
        return

    def capture():
        import io
        from PIL import Image
        import gb_robot
        jpeg = gb_robot.snapshot()
        im = Image.open(io.BytesIO(jpeg)).convert("L").resize((9, 8))
        px = list(im.getdata())
        return [px[i * 9:(i + 1) * 9] for i in range(8)], None

    def publish(cmd):
        try:
            import gb_maint
            if gb_maint.is_set():
                return gb_estop.gate_request({"op": "stop", "src": "wally"})
        except Exception:
            return None
        if not cmd.get("allow") or cmd.get("op") == "stop" or gb_estop.is_set() or not (cmd.get("lin") or cmd.get("ang")):
            return gb_estop.gate_request({"op": "stop", "src": "wally"})
        return gb_estop.gate_request({
            "op": "timed", "lin": float(cmd.get("lin") or 0), "ang": float(cmd.get("ang") or 0),
            "dur": 0.3, "src": "wally", "id": (cmd.get("provenance") or {}).get("id")})

    gb_wally.install_runtime(capture=capture, publisher=publish, start_thread=True)


def main():
    _install_wally_runtime()
    log(f"{BOT_NAME}: backend={LLM_BACKEND} key={'set' if has_key() else 'missing'} ollama={OLLAMA_MODEL} "
        f"STT={STT_BACKEND} TTS={TTS_BACKEND} trigger={TRIGGER} web=:{WEB_PORT} data={DATA}")
    store.write_labels_tsv()
    try:
        st = gb_caps.sync()
        log("capabilities (fresh user gate, not a health fault): %s" % st)
        log("banner: %s" % gb_caps.startup_banner(None, st))
    except Exception as ex: log("caps sync failed:", ex)
    threading.Thread(target=seed_watcher, daemon=True).start()
    if E("TTS_WARMUP", "0") == "1" and E("SUPPRESS_STARTUP_SPEECH", "0") != "1":
        threading.Thread(target=lambda: tts("warm up"), daemon=True).start()
    threading.Thread(target=_playback_poller, daemon=True, name="playback_poller").start()
    threading.Thread(target=estop_reactor, daemon=True, name="estop_reactor").start()
    threading.Thread(target=estop_autoclear, daemon=True, name="estop_autoclear").start()
    threading.Thread(target=_health_monitor, daemon=True, name="health_monitor").start()
    threading.Thread(target=lambda: (resolve_audio(), stop_listener()), daemon=True, name="stop_listener").start()
    threading.Thread(target=gb_estop.warm_up, daemon=True).start()
    if TRIGGER == "none":
        resolve_audio(); run_web(); return
    if TRIGGER == "enter":
        if WEB_PORT: threading.Thread(target=run_web, daemon=True).start()
        voice_loop(); return
    threading.Thread(target=voice_loop, daemon=True).start()
    run_web()

if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--ask":         # grokbot_voice.py --ask "what's the battery"
        print(answer(" ".join(sys.argv[2:]), "cli")[0]); sys.exit(0)
    if len(sys.argv) > 2 and sys.argv[1] == "--say":
        resolve_audio(); play_wav_bytes(tts(" ".join(sys.argv[2:])), " ".join(sys.argv[2:])); sys.exit(0)
    main()
