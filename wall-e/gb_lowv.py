#!/usr/bin/env python3
"""gb_lowv.py - low-voltage safety monitor for the MentorPi (2S Li-ion 7.4 V 2200 mAh).
Polls every LOWV_POLL_S (5 s), keeps a LOWV_AVG_S (30 s) moving average, writes ~/grokbot/data/battery.json atomically
(the voice assistant reads it, so only this process touches the serial port regularly).
3 consecutive smoothed readings < LOWV_WARN_V -> stop motors, speak warning, countdown LOWV_SHUTDOWN_DELAY_S (60 s)
with reminders at 30 s and 10 s, flush SQLite + sync, then `sudo shutdown -h now`.
Smoothed < LOWV_HARD_V (2 consecutive) -> immediate stop + flush + shutdown.
Countdown cancels if the smoothed voltage recovers >= LOWV_WARN_V + LOWV_CANCEL_MARGIN.
LOWV_DRY_RUN=1 logs/speaks but never shuts down. LOWV_FAKE_V=<volts> forces readings (testing)."""
import json, os, subprocess, sys, time
from collections import deque
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import requests, gb_robot, gb_store
from gb_store import atomic_write

E = os.environ.get
WARN = float(E("LOWV_WARN_V", "6.8")); HARD = float(E("LOWV_HARD_V", "6.5"))
MARGIN = float(E("LOWV_CANCEL_MARGIN", "0.2")); DELAY = float(E("LOWV_SHUTDOWN_DELAY_S", "60"))
POLL = float(E("LOWV_POLL_S", "5")); AVG_S = float(E("LOWV_AVG_S", "30")); NEED = int(E("LOWV_CONSECUTIVE", "3"))
DRY = E("LOWV_DRY_RUN", "0") == "1"; FAKE = E("LOWV_FAKE_V", "")
STALE_S = float(E("LOWV_STALE_S", "10"))   # 2026-11-24 topiconly: no reading this long -> 'battery data stale' alert (never a low trigger)
SAY_URL = f"http://127.0.0.1:{E('WEB_PORT', '8000')}/internal/say"
FLUSH_URL = f"http://127.0.0.1:{E('WEB_PORT', '8000')}/internal/flush"
DATA = gb_store.DATA; LOG = os.path.join(DATA, "lowv.log"); BJSON = gb_robot.BATTERY_JSON

def log(msg):
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n"); f.flush(); os.fsync(f.fileno())

store = gb_store.Store()
def event(kind, detail):
    log(f"{kind}: {detail}")
    try: store.event("lowv_" + kind, detail)
    except Exception as ex: print("db event failed", ex)

def say(text):
    if DRY: text = "Test only. " + text
    try:
        if requests.post(SAY_URL, json={"text": text}, timeout=25).ok: return
    except Exception: pass
    try:                                    # fallback: piper CLI from the venv + aplay
        voice = os.path.expanduser(E("PIPER_VOICE", "~/grokbot/voices/en_US-lessac-medium.onnx"))
        piper = os.path.join(os.path.dirname(sys.executable), "piper")
        dev = E("AUDIO_OUT", "auto"); dev = "plughw:CARD=Device,DEV=0" if dev == "auto" else dev
        subprocess.run(f'echo {json.dumps(text)} | "{piper}" -m "{voice}" -f /tmp/lowv.wav && aplay -q -D {dev} /tmp/lowv.wav',
                       shell=True, timeout=30)
    except Exception as ex: log(f"speak failed: {ex}")

def stop_motors():
    try: gb_robot.stop()
    except Exception as ex: log(f"stop via rosbridge failed: {ex}")

def flush():
    try: requests.post(FLUSH_URL, timeout=10)
    except Exception: pass
    try: store.checkpoint()
    except Exception as ex: log(f"checkpoint failed: {ex}")
    os.sync()

def shutdown(reason):
    event("shutdown", reason); stop_motors(); flush()
    if DRY:
        log("DRY RUN: would run sudo shutdown -h now"); return
    subprocess.run(["sudo", "-n", "/sbin/shutdown", "-h", "now"])

def read_mv():
    if FAKE:                                # number, or path of a file holding a number (for tests)
        return float(open(FAKE).read() if os.path.isfile(FAKE) else FAKE) * 1000, "fake"
    # 2026-11-24 topiconly: ROS topic /ros_robot_controller/battery (UInt16 mV) ONLY. The direct /dev/ttyACM0 read
    # (battery_serial_mV) was removed: a second reader on the controller's serial port kills its IMU/battery thread.
    try:
        mv = gb_robot.battery_topic_mV(timeout=1.5)
        if mv: return mv, "ros_topic"
    except Exception as ex:
        log(f"battery topic read failed: {ex}")
    return None, None

def stale_update(st, mv, now):
    """Track topic freshness. Returns 'stale' once when data has been missing > STALE_S, 'fresh' once on recovery,
    else None. Stale data never feeds the low-voltage logic (no false trigger), and the serial port is never opened."""
    if mv:
        was = st.get("stale"); st["last_ok"] = now; st["stale"] = False
        return "fresh" if was else None
    if not st.get("stale") and now - st.setdefault("last_ok", now) > STALE_S:
        st["stale"] = True; return "stale"
    return None

def main():
    win = deque(maxlen=max(1, int(AVG_S / POLL)))
    below = 0; hard_below = 0; deadline = None; warned = set(); prev = None
    log(f"start: warn<{WARN} hard<{HARD} cancel>={WARN + MARGIN} delay={DELAY}s poll={POLL}s avg={AVG_S}s n={NEED} dry={DRY} "
        f"source=ros_topic only stale>{STALE_S}s")
    st = {}
    while True:
        t0 = time.time()
        mv, src = read_mv()
        s_ev = stale_update(st, mv, time.time())
        if s_ev == "stale":
            event("stale", f"battery data stale: no /ros_robot_controller/battery reading for >{STALE_S:.0f}s "
                           f"(not treated as low; serial port not opened)")
        elif s_ev == "fresh":
            event("fresh", "battery data fresh again from ROS topic")
        if mv:
            v = mv / 1000.0; win.append(v); avg = sum(win) / len(win)
            trend = None if prev is None else avg - prev; prev = avg
            atomic_write(BJSON, json.dumps({"ts": time.time(), "v_raw": round(v, 3), "v_avg": round(avg, 3),
                         "pct": gb_robot.battery_pct(avg), "source": src, "samples": len(win),
                         "state": "countdown" if deadline else "ok",
                         "charging_hint": bool(trend is not None and trend > 0.004 and len(win) == win.maxlen)}))
            full = len(win) >= min(3, win.maxlen)
            hard_below = hard_below + 1 if (full and avg < HARD) else 0
            below = below + 1 if (full and avg < WARN) else 0
            if hard_below >= 2:
                say("Battery critically low. Shutting down now."); shutdown(f"hard: avg {avg:.2f} V < {HARD}")
                if not DRY: return
                hard_below = 0
            if deadline is None and below >= NEED:
                deadline = time.time() + DELAY; warned = set()
                event("warn", f"avg {avg:.2f} V < {WARN} for {below} readings; shutdown in {DELAY:.0f}s")
                stop_motors(); say("Battery low, shutting down in one minute. Plug in the charger to cancel.")
            elif deadline is not None:
                if avg >= WARN + MARGIN:
                    event("cancel", f"recovered to {avg:.2f} V"); say("Battery recovered. Shutdown cancelled.")
                    deadline = None; below = 0
                else:
                    left = deadline - time.time()
                    for mark in (30, 10):
                        if left <= mark and mark not in warned:
                            warned.add(mark); say(f"Battery low. Shutting down in {mark} seconds.")
                    if left <= 0:
                        shutdown(f"warn countdown expired at {avg:.2f} V"); deadline = None
                        if not DRY: return
        time.sleep(max(0.2, POLL - (time.time() - t0)))

if __name__ == "__main__":
    main()
