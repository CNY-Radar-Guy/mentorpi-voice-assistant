#!/usr/bin/env python3
"""echo_collect.py OUTDIR ROUNDS - play the robot's own TTS through its speaker (grokbot's _play -> speaking flag) while
recording the shared mic, to calibrate gb_clap against self-echo. Also plays double claps mixed into TTS (labelled)."""
import io, json, os, subprocess, sys, time, wave
import numpy as np
sys.path.insert(0, "/home/pi/grokbot"); os.chdir("/home/pi/grokbot")
import grokbot_voice as g
g.AUDIO_OUT = "plughw:CARD=Device,DEV=0"
out = sys.argv[1]; rounds = int(sys.argv[2]); os.makedirs(out, exist_ok=True)
TEXTS = [g.WAKE_ACK, "I am Grok Bot, so woke up. Chat at me.", "Emergency stop.",
         "Sure. I can see a chair about one meter ahead of me, a little to the left, and a person standing near the door.",
         "Okay, driving forward for four seconds at a gentle speed. Clap twice any time to end it.",
         "The battery is at seven point six volts, which is about seventy percent, so we have plenty of time for a test run today."]
def wav_arr(b):
    w = wave.open(io.BytesIO(b)); sr = w.getframerate(); a = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32)
    if w.getnchannels() > 1: a = a.reshape(-1, w.getnchannels()).mean(axis=1)
    if sr != 48000: a = np.interp(np.arange(int(len(a) * 48000 / sr)) * sr / 48000, np.arange(len(a)), a)
    return a / 32768.0
def arr_wav(a):
    bio = io.BytesIO(); w = wave.open(bio, "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(48000)
    w.writeframes((np.clip(a, -1, 1) * 32767).astype(np.int16).tobytes()); w.close(); return bio.getvalue()
cache = {t: g.tts(t) for t in TEXTS}
rec = subprocess.Popen(["arecord", "-q", "-D", "gbmic", "-f", "S16_LE", "-r", "48000", "-c", "1", os.path.join(out, "mic.wav")])
t0 = time.time(); labels = []
time.sleep(2.0)
for r in range(rounds):
    for t in TEXTS:
        a = time.time(); g.play_wav_bytes(cache[t]); labels.append({"kind": "tts", "round": r, "text": t[:40], "s": a - t0, "e": time.time() - t0})
        time.sleep(0.4)
clap = wave.open("/home/pi/grokbot/gb_safety/wavs/double_clap_quiet.wav") if os.path.exists("/home/pi/grokbot/gb_safety/wavs/double_clap_quiet.wav") else None
if len(sys.argv) > 3:
    cw = np.frombuffer(open(sys.argv[3], "rb").read()[44:], dtype=np.int16).astype(np.float32) / 32768.0
    for k, t in enumerate(TEXTS[3:6]):
        sp = wav_arr(cache[t]); mix = np.copy(sp); at = int(1.2 * 48000)
        seg = cw[:max(0, min(len(cw), len(mix) - at))]; mix[at:at + len(seg)] += seg
        a = time.time(); g.play_wav_bytes(arr_wav(mix)); labels.append({"kind": "clap_in_tts", "text": t[:40], "s": a - t0, "e": time.time() - t0, "clap_at": a - t0 + 1.2})
        time.sleep(1.0)
    a = time.time(); g.play_wav_bytes(arr_wav(cw)); labels.append({"kind": "clap_alone", "s": a - t0, "e": time.time() - t0})
time.sleep(1.5); rec.terminate(); rec.wait()
json.dump(labels, open(os.path.join(out, "labels.json"), "w"), indent=1); print("done", len(labels), "segments", round(time.time() - t0, 1), "s")
