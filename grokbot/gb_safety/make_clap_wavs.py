#!/usr/bin/env python3
"""Synthetic test audio for gb_clap: claps (decaying broadband bursts), speech-like babble, music, track/motor noise."""
import numpy as np, wave, os, sys
SR = 48000; rng = np.random.default_rng(7); OUT = sys.argv[1] if len(sys.argv) > 1 else "."
def save(name, x):
    x = np.clip(x, -1, 1); w = wave.open(os.path.join(OUT, name), "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
    w.writeframes((x * 32767).astype(np.int16).tobytes()); w.close()
def clap(amp=0.6, tau=0.012):
    n = int(0.12 * SR); t = np.arange(n) / SR
    b = rng.standard_normal(n) * np.exp(-t / tau) * np.minimum(1, t / 0.0005)
    b += 0.5 * rng.standard_normal(n) * np.exp(-np.maximum(0, t - 0.004) / (tau * 0.6)) * (t > 0.004)
    return amp * b / np.max(np.abs(b))
def put(x, s, at):
    i = int(at * SR); x[i:i + len(s)] += s[:max(0, len(x) - i)]
def track_noise(n, lvl):
    t = np.arange(n) / SR
    hum = sum(np.sin(2 * np.pi * f * t + rng.uniform(0, 6)) / (k + 1) for k, f in enumerate([180, 360, 540, 1250, 2600, 4100]))
    br = np.convolve(rng.standard_normal(n), np.ones(8) / 8, "same")
    grind = rng.standard_normal(n) * (0.6 + 0.4 * np.sin(2 * np.pi * 23 * t))           # track links rattling at ~23 Hz
    return lvl * (0.25 * hum / 3 + 0.5 * br + 0.35 * grind)
def babble(n, lvl):
    t = np.arange(n) / SR; x = np.zeros(n)
    for v in range(4):
        f0 = rng.uniform(100, 220); env = (np.sin(2 * np.pi * rng.uniform(3, 6) * t + v) > 0.2).astype(float)
        env = np.convolve(env, np.ones(int(0.03 * SR)) / int(0.03 * SR), "same")
        for h in range(1, 18): x += env * np.sin(2 * np.pi * f0 * h * t) / h
        x += 0.15 * env * rng.standard_normal(n)                                       # fricatives
    return lvl * x / np.max(np.abs(x))
def music(n, lvl, drums=False):
    t = np.arange(n) / SR; x = np.zeros(n)
    for k, ch in enumerate([[261.6, 329.6, 392], [220, 261.6, 329.6], [174.6, 220, 261.6], [196, 246.9, 293.7]] * 4):
        s, e = int(k * 0.5 * SR), int((k + 1) * 0.5 * SR)
        if s >= n: break
        tt = t[s:e] - t[s]; env = np.minimum(1, tt / 0.03) * np.exp(-tt / 0.6)
        for f in ch:
            for h in range(1, 8): x[s:e] += env * np.sin(2 * np.pi * f * h * tt) / h ** 1.5
    x = x / np.max(np.abs(x))
    if drums:
        for k in range(int(n / SR / 0.5)):
            put(x, 0.5 * clap(1.0, 0.03) * (1 if k % 2 else 0.3), k * 0.5)             # hi-hat/snare on every beat (0.5 s)
    return lvl * x
N = int(4 * SR)
for name, bg in [("quiet", lambda: 0.002 * rng.standard_normal(N)), ("track", lambda: track_noise(N, 0.12)),
                 ("loudtrack", lambda: track_noise(N, 0.30)), ("speech", lambda: babble(N, 0.3))]:
    x = bg(); put(x, clap(0.7), 1.0); put(x, clap(0.6), 1.3); save(f"double_clap_{name}.wav", x)        # 300 ms
x = 0.002 * rng.standard_normal(N); put(x, clap(0.25), 1.0); put(x, clap(0.22), 1.55); save("double_clap_soft_far_550ms.wav", x)
x = track_noise(N, 0.12); put(x, clap(0.35), 1.0); put(x, clap(0.35), 1.18); save("double_clap_track_fast_180ms.wav", x)
x = 0.002 * rng.standard_normal(N); put(x, clap(0.7), 1.0); save("single_clap.wav", x)
x = 0.002 * rng.standard_normal(N); put(x, clap(0.7), 1.0); put(x, clap(0.6), 1.9); save("two_claps_900ms_apart.wav", x)
save("noise_track.wav", track_noise(N * 2, 0.30))
save("noise_speech.wav", babble(N * 2, 0.5))
save("noise_music.wav", music(N * 2, 0.5))
save("noise_music_drums.wav", music(N * 2, 0.5, drums=True))
x = np.zeros(N * 2); t = np.arange(N * 2) / SR; x += 0.3 * np.sin(2 * np.pi * 1000 * t) * (np.sin(2 * np.pi * 0.5 * t) > 0); save("noise_beeps.wav", x)
print("ok", sorted(os.listdir(OUT)))
